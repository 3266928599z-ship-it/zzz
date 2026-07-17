#pragma warning(disable: 4005)
#pragma warning(disable: 4828)
#include "ModelMatchWorker.h"

#include <pcl/memory.h> // 必须包含，用于 pcl::make_shared
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/ppf.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <limits>

ModelMatchWorker::ModelMatchWorker(QObject* parent) : QObject(parent)
{
    m_cadDs.reset(new PointCloudNT());
    m_scanDs.reset(new PointCloudNT());
}

ModelMatchWorker::~ModelMatchWorker() {}

void ModelMatchWorker::setInput(CloudRGB::Ptr cadCloud, CloudRGB::Ptr scanCloud,
    std::vector<CloudRGB::Ptr> seamPointsList,
    std::vector<QString> seamNames,
    const MatchConfig& cfg)
{
    m_cadCloud = cadCloud;
    m_scanCloud = scanCloud;
    m_seamPoints = seamPointsList;
    m_seamNames = seamNames;
    m_cfg = cfg;
}

// ==========================================
// 主控流水线
// ==========================================
void ModelMatchWorker::run()
{
    MatchResult result;
    try {
        if (!m_cadCloud || !m_scanCloud || m_cadCloud->size() < 100 || m_scanCloud->size() < 100) {
            result.errorMessage = QString::fromUtf8("CAD或扫描点云有效点数不足（至少需要100点）。");
            emit finished(result);
            return;
        }

        emit progress(QString::fromUtf8("预处理阶段 (降采样与特征提取)..."));
        preprocess();

        if (m_scanToCadSizeRatio > 1.08f) {
            emit progress(QString::fromUtf8(
                "扫描/CAD尺寸比为 %1，自动模式将通过CAD引导裁剪排除夹具与圆环。")
                .arg(m_scanToCadSizeRatio, 0, 'f', 3));
        }

        // 后续阶段只使用降采样点云，及时释放为线程安全而创建的百万级原始副本。
        m_cadCloud.reset();
        m_scanCloud.reset();

        emit progress(QString::fromUtf8("粗配准阶段 (全局特征匹配)..."));
        std::vector<Eigen::Matrix4f> coarseTransforms;
        std::vector<double> coarseFitnesses;

        if (!coarseRegistration(coarseTransforms, coarseFitnesses)) {
            result.success = false;
            result.errorMessage = QString::fromUtf8(
                "粗配准未生成有效候选（CAD降采样 %1 点，扫描工件候选 %2 点，体素 %3）。"
                "请重点检查扫描工件是否被底板过滤掉，以及CAD与扫描单位是否一致。")
                .arg(m_cadDs ? m_cadDs->size() : 0)
                .arg(m_scanDs ? m_scanDs->size() : 0)
                .arg(m_leafSize, 0, 'f', 3);
            emit finished(result);
            return;
        }

        emit progress(QString::fromUtf8("精配准阶段A (点到点ICP校准)..."));
        struct RefinedCandidate {
            Eigen::Matrix4f transform;
            double icpFitness;
            float score;
            float overlap;
            float reverseOverlap;
            float normalConsistency;
            float featureOverlap;
            float spatialCoverage;
            float guidedRatio;
            float rmse;
            float p95;
            PointCloudNT::Ptr guidedScan;
        };
        std::vector<RefinedCandidate> refinedCandidates;

        auto qualityFromMetrics = [&](float overlap, float reverseOverlap,
            float normalConsistency, float featureOverlap, float spatialCoverage,
            float guidedRatio, float rmse) {
                if (!std::isfinite(rmse)) return std::numeric_limits<float>::infinity();
                return rmse + (1.0f - overlap) * m_leafSize * 2.0f +
                    (1.0f - featureOverlap) * m_leafSize * 3.0f +
                    (1.0f - spatialCoverage) * m_leafSize * 2.0f +
                    (1.0f - std::min(guidedRatio / 0.65f, 1.0f)) * m_leafSize * 1.5f +
                    (1.0f - normalConsistency) * m_leafSize * 0.50f -
                    std::min(reverseOverlap, 0.40f) * m_leafSize * 0.75f;
            };

        const size_t originalScanPointCount = m_scanDs->size();
        const size_t minimumGuidedPoints = std::max<size_t>(250,
            static_cast<size_t>(originalScanPointCount * 0.10f));
        for (size_t i = 0; i < coarseTransforms.size(); ++i) {
            emit progress(QString::fromUtf8("精配准A：复核刚体姿态 %1/%2...")
                .arg(i + 1).arg(coarseTransforms.size()));

            PointCloudNT::Ptr guidedWide = extractCadGuidedScan(coarseTransforms[i], 6.0f);
            if (guidedWide->size() < minimumGuidedPoints) continue;
            double icpFitness = 0.0;
            Eigen::Matrix4f refinedTransform =
                fineRegistrationA(coarseTransforms[i], guidedWide, icpFitness);

            PointCloudNT::Ptr guidedTight = extractCadGuidedScan(refinedTransform, 3.5f);
            if (guidedTight->size() < minimumGuidedPoints) continue;
            refinedTransform = fineRegistrationA(refinedTransform, guidedTight, icpFitness);

            PointCloudNT::Ptr guidedFinal = extractCadGuidedScan(refinedTransform, 2.5f);
            if (guidedFinal->size() < minimumGuidedPoints) continue;
            refinedTransform = fineRegistrationA(refinedTransform, guidedFinal, icpFitness);

            float overlap = 0.0f, reverseOverlap = 0.0f, normalConsistency = 0.0f;
            float rmse = 0.0f, p95 = 0.0f;
            evaluateAlignmentForCloud(refinedTransform, guidedFinal, overlap, reverseOverlap,
                normalConsistency, rmse, p95);
            const float featureOverlap = evaluateFeatureOverlap(refinedTransform);
            const float spatialCoverage =
                evaluateSpatialCoverageForCloud(refinedTransform, guidedFinal);
            const float guidedRatio = static_cast<float>(guidedFinal->size()) /
                static_cast<float>(originalScanPointCount);
            const float score = qualityFromMetrics(overlap, reverseOverlap,
                normalConsistency, featureOverlap, spatialCoverage, guidedRatio, rmse);
            if (!std::isfinite(score)) continue;

            refinedCandidates.push_back({ refinedTransform, icpFitness, score,
                overlap, reverseOverlap, normalConsistency, featureOverlap,
                spatialCoverage, guidedRatio, rmse, p95, guidedFinal });
            m_coarseCandidateReport += QString::fromUtf8(
                "自动裁剪候选%1：质量=%2，保留=%3点(%4%)，扫描覆盖=%5%，关键特征=%6%，空间结构=%7%，CAD支持=%8%\n")
                .arg(i + 1).arg(score, 0, 'f', 3)
                .arg(guidedFinal->size()).arg(guidedRatio * 100.0f, 0, 'f', 1)
                .arg(overlap * 100.0f, 0, 'f', 1)
                .arg(featureOverlap * 100.0f, 0, 'f', 1)
                .arg(spatialCoverage * 100.0f, 0, 'f', 1)
                .arg(reverseOverlap * 100.0f, 0, 'f', 1);
        }

        if (refinedCandidates.empty()) {
            result.success = false;
            result.errorMessage = QString::fromUtf8("所有粗配准姿态在ICP复核后均无有效对应。请检查主体点云和CAD是否为同一零件及同一尺度。");
            emit finished(result);
            return;
        }
        std::sort(refinedCandidates.begin(), refinedCandidates.end(),
            [](const RefinedCandidate& a, const RefinedCandidate& b) { return a.score < b.score; });

        const RefinedCandidate& bestRefined = refinedCandidates.front();
        Eigen::Matrix4f T_A = bestRefined.transform;
        const double fitnessA = bestRefined.icpFitness;
        result.fitnessStageA = static_cast<float>(fitnessA);

        // 后续精配准只使用CAD自动提取出的主体工件，并重新计算其关键特征。
        m_scanDs = bestRefined.guidedScan;
        extractEdgeFeatures(m_scanDs, m_leafSize, m_scanEdgeSubset, m_scanEdgeFeatures);
        CloudRGB::Ptr guidedPreview(new CloudRGB());
        guidedPreview->reserve(m_scanDs->size());
        for (const auto& point : m_scanDs->points) {
            pcl::PointXYZRGB colored;
            colored.x = point.x; colored.y = point.y; colored.z = point.z;
            colored.r = 80; colored.g = 220; colored.b = 255;
            guidedPreview->push_back(colored);
        }
        emit processedScanReady(guidedPreview);
        emit progress(QString::fromUtf8(
            "CAD引导自动裁剪完成：从 %1 点保留工件主体 %2 点（%3%）。")
            .arg(originalScanPointCount).arg(m_scanDs->size())
            .arg(bestRefined.guidedRatio * 100.0f, 0, 'f', 1));
        if (refinedCandidates.size() > 1) {
            const float secondRatio = refinedCandidates[1].score /
                std::max(bestRefined.score, 1e-6f);
            result.wasAmbiguous = secondRatio < m_cfg.ambiguityRatio;
            result.ambiguityRatio = secondRatio;
            result.T_candidate2 = refinedCandidates[1].transform;
        }

        emit progress(QString::fromUtf8("精配准阶段B (鲁棒点到面迭代)..."));
        std::vector<float> weights(m_cadDs->size(), 1.0f);
        int iters = 0;
        bool conv = false;
        float finalW = m_cfg.seamWeightMax;
        double fitnessB = 0;

        // 内部始终求解“扫描 -> CAD”，最终取逆得到 CAD -> 扫描。
        Eigen::Matrix4f scanToCad = fineRegistrationB(T_A, weights, fitnessA, iters, conv, finalW, fitnessB);

        float overlap = 0.0f;
        float reverseOverlap = 0.0f;
        float normalConsistency = 0.0f;
        float featureOverlap = 0.0f;
        float spatialCoverage = 0.0f;
        float rmse = 0.0f;
        float p95 = 0.0f;
        evaluateAlignment(scanToCad, overlap, reverseOverlap, normalConsistency, rmse, p95);
        featureOverlap = evaluateFeatureOverlap(scanToCad);
        spatialCoverage = evaluateSpatialCoverage(scanToCad);

        const float finalQuality = qualityFromMetrics(overlap, reverseOverlap,
            normalConsistency, featureOverlap, spatialCoverage,
            bestRefined.guidedRatio, rmse);
        if (!std::isfinite(finalQuality) ||
            finalQuality > bestRefined.score + m_leafSize * 0.10f) {
            emit progress(QString::fromUtf8(
                "精配准B使整体质量下降，已回退到多候选ICP复核的最佳姿态。"));
            scanToCad = bestRefined.transform;
            overlap = bestRefined.overlap;
            reverseOverlap = bestRefined.reverseOverlap;
            normalConsistency = bestRefined.normalConsistency;
            featureOverlap = bestRefined.featureOverlap;
            spatialCoverage = bestRefined.spatialCoverage;
            rmse = bestRefined.rmse;
            p95 = bestRefined.p95;
            conv = false;
        }

        result.fitnessStageB = rmse;
        result.overlapRatio = overlap;
        result.reverseOverlapRatio = reverseOverlap;
        result.normalConsistency = normalConsistency;
        result.featureOverlapRatio = featureOverlap;
        result.spatialCoverageRatio = spatialCoverage;
        result.rmse = rmse;
        result.p95Residual = p95;
        result.iterationsStageB = iters;
        result.convergedStageB = conv;
        result.finalSeamWeight = finalW;
        result.T_final = scanToCad.inverse();
        result.coarseCandidateReport = m_coarseCandidateReport;
        result.seamResults.clear();

        result.success = true;
        result.verdict = classifyResult(result);
        if (result.verdict == MatchVerdict::Pass) {
            result.verdictMessage = QString::fromUtf8("匹配可信");
        }
        else if (result.verdict == MatchVerdict::NeedsReview) {
            result.verdictMessage = QString::fromUtf8("匹配存在歧义，建议人工确认");
        }
        else {
            result.verdictMessage = QString::fromUtf8("匹配质量不足");
        }

    }
    catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = QString::fromUtf8(e.what());
    }

    emit finished(result);
}

// ==========================================
// 辅助函数：提取高曲率边缘子集及 FPFH
// ==========================================
void ModelMatchWorker::extractEdgeFeatures(PointCloudNT::Ptr cloud, float leafSize, PointCloudNT::Ptr& outEdgeCloud, FeatureCloudT::Ptr& outFeatures)
{
    const float featureRadius = leafSize * 6.0f;
    pcl::PrincipalCurvaturesEstimation<PointNT, PointNT, pcl::PrincipalCurvatures> estimator;
    estimator.setInputCloud(cloud);
    estimator.setInputNormals(cloud);
    pcl::search::KdTree<PointNT>::Ptr curvatureTree(new pcl::search::KdTree<PointNT>);
    estimator.setSearchMethod(curvatureTree);
    estimator.setRadiusSearch(leafSize * 4.0f);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures(
        new pcl::PointCloud<pcl::PrincipalCurvatures>());
    estimator.compute(*curvatures);

    struct RankedPoint { float saliency; size_t index; };
    std::vector<RankedPoint> ranked;
    ranked.reserve(std::min(cloud->size(), curvatures->size()));
    for (size_t i = 0; i < cloud->size() && i < curvatures->size(); ++i) {
        const auto& curvature = curvatures->points[i];
        const float saliency = std::abs(curvature.pc1) + std::abs(curvature.pc2) +
            std::abs(cloud->points[i].curvature);
        if (std::isfinite(saliency)) ranked.push_back({ saliency, i });
    }
    std::sort(ranked.begin(), ranked.end(), [](const RankedPoint& a, const RankedPoint& b) {
        return a.saliency > b.saliency;
        });

    const size_t requested = static_cast<size_t>(ranked.size() *
        std::clamp(m_cfg.edgePercentile, 0.15f, 0.45f));
    const size_t minimum = std::min<size_t>(400, ranked.size());
    const size_t keepCount = std::min<size_t>(3000, std::max(minimum, requested));
    outEdgeCloud.reset(new PointCloudNT());
    outEdgeCloud->reserve(keepCount);
    for (size_t i = 0; i < keepCount; ++i) {
        outEdgeCloud->push_back(cloud->points[ranked[i].index]);
    }

    auto computeValidDescriptors = [&](PointCloudNT::Ptr keypoints,
        PointCloudNT::Ptr& validPoints, FeatureCloudT::Ptr& validFeatures) {
            pcl::search::KdTree<PointNT>::Ptr tree(new pcl::search::KdTree<PointNT>);
            pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> fpfh;
            fpfh.setInputCloud(keypoints);
            // 邻域索引来自完整搜索表面，法向必须与完整表面一一对应。
            fpfh.setInputNormals(cloud);
            fpfh.setSearchSurface(cloud);
            fpfh.setSearchMethod(tree);
            fpfh.setRadiusSearch(featureRadius);
            FeatureCloudT::Ptr rawFeatures(new FeatureCloudT());
            fpfh.compute(*rawFeatures);

            validPoints.reset(new PointCloudNT());
            validFeatures.reset(new FeatureCloudT());
            validPoints->reserve(keypoints->size());
            validFeatures->reserve(rawFeatures->size());
            for (size_t i = 0; i < keypoints->size() && i < rawFeatures->size(); ++i) {
                bool featureFinite = true;
                for (float value : rawFeatures->points[i].histogram) {
                    if (!std::isfinite(value)) {
                        featureFinite = false;
                        break;
                    }
                }
                if (featureFinite) {
                    validPoints->push_back(keypoints->points[i]);
                    validFeatures->push_back(rawFeatures->points[i]);
                }
            }
        };

    PointCloudNT::Ptr validPoints;
    FeatureCloudT::Ptr validFeatures;
    computeValidDescriptors(outEdgeCloud, validPoints, validFeatures);

    if (validPoints->size() < 100) {
        PointCloudNT::Ptr uniformKeypoints(new PointCloudNT());
        const size_t targetCount = std::min<size_t>(2000, cloud->size());
        const double step = static_cast<double>(cloud->size()) /
            static_cast<double>(std::max<size_t>(targetCount, 1));
        uniformKeypoints->reserve(targetCount);
        for (size_t i = 0; i < targetCount; ++i) {
            const size_t index = std::min(cloud->size() - 1,
                static_cast<size_t>(std::floor(i * step)));
            uniformKeypoints->push_back(cloud->points[index]);
        }
        computeValidDescriptors(uniformKeypoints, validPoints, validFeatures);
    }

    if (validPoints->size() < 100) {
        computeValidDescriptors(cloud, validPoints, validFeatures);
    }

    outEdgeCloud = validPoints;
    outFeatures = validFeatures;
}

// ==========================================
// 阶段 0：预处理 (高速优化版)
// ==========================================
void ModelMatchWorker::preprocess()
{
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*m_cadCloud, minPt, maxPt);
    float diagonal = (maxPt.getVector3fMap() - minPt.getVector3fMap()).norm();
    // 动态计算叶尺寸，相对 CAD 尺寸的百分比
    m_leafSize = std::max(diagonal * m_cfg.leafSizeFactor, 0.5f);

    // 定义一个 Lambda 预处理流水线
    auto downsampleAndNormals = [&](CloudRGB::Ptr input, PointCloudNT::Ptr& output, bool applySor) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*input, *xyz);

        // 1. 【极度关键】：绝对优先进行体素降采样！
        // 瞬间把几百万的点云降阶到几万级别，大幅卸载 CPU 负担
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setLeafSize(m_leafSize, m_leafSize, m_leafSize);
        grid.setInputCloud(xyz);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>());
        grid.filter(*ds);

        // 扫描数据中占比很大的工作台/底板会淹没工件特征，先剔除最大平面。
        if (applySor && ds->size() > 200) {
            pcl::SACSegmentation<pcl::PointXYZ> segmentation;
            segmentation.setOptimizeCoefficients(true);
            segmentation.setModelType(pcl::SACMODEL_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setMaxIterations(500);
            segmentation.setDistanceThreshold(m_leafSize * 1.25f);
            segmentation.setInputCloud(ds);

            pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients());
            segmentation.segment(*planeIndices, *planeCoefficients);

            const float planeRatio = static_cast<float>(planeIndices->indices.size()) /
                static_cast<float>(ds->size());
            if (planeRatio > 0.15f && ds->size() - planeIndices->indices.size() >= 100) {
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(ds);
                extract.setIndices(planeIndices);
                extract.setNegative(true);
                pcl::PointCloud<pcl::PointXYZ>::Ptr withoutPlane(new pcl::PointCloud<pcl::PointXYZ>());
                extract.filter(*withoutPlane);
                ds = withoutPlane;
            }
        }

        // 2. 降采样后再做 SOR 滤波去飞点，速度从几分钟变成几毫秒
        if (applySor && ds->size() > 50) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(ds);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            pcl::PointCloud<pcl::PointXYZ>::Ptr ds_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            sor.filter(*ds_filtered);
            ds = ds_filtered;
        }

        // 点云中可能同时存在工件、定位环和夹具残块。按空间连通性聚类，并优先保留
        // 尺寸接近 CAD 的最大主体，避免无关物体污染 FPFH 和 ICP。
        if (applySor && ds->size() > 200) {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr clusterTree(
                new pcl::search::KdTree<pcl::PointXYZ>());
            clusterTree->setInputCloud(ds);

            std::vector<pcl::PointIndices> clusters;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
            const size_t pointsBeforeClustering = ds->size();
            extraction.setClusterTolerance(m_leafSize * 2.0f);
            extraction.setMinClusterSize(40);
            extraction.setMaxClusterSize(static_cast<int>(ds->size()));
            extraction.setSearchMethod(clusterTree);
            extraction.setInputCloud(ds);
            extraction.extract(clusters);

            int selectedCluster = -1;
            float selectedScore = -std::numeric_limits<float>::infinity();
            float selectedDiagonal = 0.0f;
            for (size_t i = 0; i < clusters.size(); ++i) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
                cluster->reserve(clusters[i].indices.size());
                for (int index : clusters[i].indices) cluster->push_back(ds->points[index]);

                pcl::PointXYZ clusterMin, clusterMax;
                pcl::getMinMax3D(*cluster, clusterMin, clusterMax);
                const float clusterDiagonal =
                    (clusterMax.getVector3fMap() - clusterMin.getVector3fMap()).norm();
                const float ratio = clusterDiagonal / std::max(diagonal, 1e-6f);
                const bool plausibleSize = ratio >= 0.45f && ratio <= 1.35f;
                const float sizePenalty = std::abs(std::log(std::max(ratio, 1e-3f)));
                const float score = std::log1p(static_cast<float>(cluster->size())) -
                    sizePenalty * 2.0f + (plausibleSize ? 3.0f : 0.0f);
                if (score > selectedScore) {
                    selectedScore = score;
                    selectedCluster = static_cast<int>(i);
                    selectedDiagonal = clusterDiagonal;
                }
            }

            if (selectedCluster >= 0 && clusters[selectedCluster].indices.size() >= 100) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr workpiece(new pcl::PointCloud<pcl::PointXYZ>());
                workpiece->reserve(clusters[selectedCluster].indices.size());
                for (int index : clusters[selectedCluster].indices) {
                    workpiece->push_back(ds->points[index]);
                }
                ds = workpiece;
                const float sizeRatio = selectedDiagonal / std::max(diagonal, 1e-6f);
                emit progress(QString::fromUtf8(
                    "工件聚类：共 %1 个连通区域，从 %2 点保留主体 %3 点，对角线 %4，尺寸比 %5")
                    .arg(clusters.size()).arg(pointsBeforeClustering).arg(ds->size())
                    .arg(selectedDiagonal, 0, 'f', 1).arg(sizeRatio, 0, 'f', 3));
                if (sizeRatio > 1.08f) {
                    emit progress(QString::fromUtf8(
                        "尺寸警告：扫描主体对角线大于CAD超过8%，仍可能包含夹具/圆环，或点云标定尺度不正确。"));
                }
            }
        }

        // 3. 在轻量级点云上计算法向 (OpenMP 多核全开)
        output.reset(new PointCloudNT());
        pcl::NormalEstimationOMP<pcl::PointXYZ, PointNT> ne;
        ne.setInputCloud(ds);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(m_leafSize * 3.0f);
        ne.compute(*output);

        // 拼接 XYZ 和 法向
        pcl::copyPointCloud(*ds, *output);

        PointCloudNT::Ptr valid(new PointCloudNT());
        valid->reserve(output->size());
        for (auto point : output->points) {
            const Eigen::Vector3f normal = point.getNormalVector3fMap();
            if (!pcl::isFinite(point) || !normal.allFinite() || normal.squaredNorm() < 0.25f) continue;
            point.getNormalVector3fMap().normalize();
            valid->push_back(point);
        }
        output = valid;
        };

    // 执行 CAD 的预处理 (CAD 本身纯净，不需要 SOR 滤波)
    downsampleAndNormals(m_cadCloud, m_cadDs, false);

    // 执行扫描点云的预处理 (需要 SOR 去除散乱飞点)
    downsampleAndNormals(m_scanCloud, m_scanDs, true);

    if (m_scanDs && !m_scanDs->empty()) {
        PointNT scanMin, scanMax;
        pcl::getMinMax3D(*m_scanDs, scanMin, scanMax);
        const float scanDiagonal =
            (scanMax.getVector3fMap() - scanMin.getVector3fMap()).norm();
        m_scanToCadSizeRatio = scanDiagonal / std::max(diagonal, 1e-6f);
    }
    else {
        m_scanToCadSizeRatio = std::numeric_limits<float>::infinity();
    }

    // 提取高曲率特征边缘与 FPFH 描述子
    extractEdgeFeatures(m_cadDs, m_leafSize, m_cadEdgeSubset, m_cadEdgeFeatures);
    extractEdgeFeatures(m_scanDs, m_leafSize, m_scanEdgeSubset, m_scanEdgeFeatures);

    CloudRGB::Ptr preview(new CloudRGB());
    preview->reserve(m_scanDs->size());
    for (const auto& point : m_scanDs->points) {
        pcl::PointXYZRGB colored;
        colored.x = point.x; colored.y = point.y; colored.z = point.z;
        colored.r = 80; colored.g = 220; colored.b = 255;
        preview->push_back(colored);
    }
    emit processedScanReady(preview);

    emit progress(QString::fromUtf8(
        "预处理完成：CAD %1 点/关键点 %2，扫描 %3 点/关键点 %4，体素尺寸 %5")
        .arg(m_cadDs->size()).arg(m_cadEdgeSubset->size())
        .arg(m_scanDs->size()).arg(m_scanEdgeSubset->size()).arg(m_leafSize, 0, 'f', 3));
}
// ==========================================
// 阶段 1：粗配准
// ==========================================
float ModelMatchWorker::evaluateFeatureOverlap(const Eigen::Matrix4f& scanToCad) const
{
    if (!m_scanEdgeSubset || m_scanEdgeSubset->empty() ||
        !m_cadEdgeSubset || m_cadEdgeSubset->empty()) return 0.0f;

    PointCloudNT::Ptr aligned(new PointCloudNT());
    pcl::transformPointCloudWithNormals(*m_scanEdgeSubset, *aligned, scanToCad);
    pcl::KdTreeFLANN<PointNT> tree;
    tree.setInputCloud(m_cadEdgeSubset);
    const float maxDistanceSquared = std::pow(m_leafSize * 3.0f, 2.0f);
    size_t supported = 0;
    for (const auto& point : aligned->points) {
        std::vector<int> index(1);
        std::vector<float> distance(1);
        if (tree.nearestKSearch(point, 1, index, distance) <= 0 ||
            distance[0] > maxDistanceSquared) continue;
        const float normalAgreement = std::abs(point.getNormalVector3fMap().dot(
            m_cadEdgeSubset->points[index[0]].getNormalVector3fMap()));
        if (normalAgreement >= 0.45f) ++supported;
    }
    return static_cast<float>(supported) / static_cast<float>(aligned->size());
}

float ModelMatchWorker::evaluateSpatialCoverage(const Eigen::Matrix4f& scanToCad) const
{
    return evaluateSpatialCoverageForCloud(scanToCad, m_scanDs);
}

float ModelMatchWorker::evaluateSpatialCoverageForCloud(
    const Eigen::Matrix4f& scanToCad, PointCloudNT::Ptr scanCloud) const
{
    if (!scanCloud || scanCloud->empty() || !m_cadDs || m_cadDs->empty()) return 0.0f;

    PointNT minPoint, maxPoint;
    pcl::getMinMax3D(*m_cadDs, minPoint, maxPoint);
    const Eigen::Vector3f minimum = minPoint.getVector3fMap();
    const Eigen::Vector3f extent = (maxPoint.getVector3fMap() - minimum)
        .cwiseMax(Eigen::Vector3f::Constant(1e-6f));

    auto cellIndex = [&](const PointNT& point) {
        const Eigen::Vector3f normalized =
            ((point.getVector3fMap() - minimum).cwiseQuotient(extent) * 3.0f)
            .cwiseMax(Eigen::Vector3f::Zero())
            .cwiseMin(Eigen::Vector3f::Constant(2.999f));
        const int x = static_cast<int>(normalized.x());
        const int y = static_cast<int>(normalized.y());
        const int z = static_cast<int>(normalized.z());
        return x + 3 * y + 9 * z;
        };
    auto normalBin = [](const PointNT& point) {
        const Eigen::Vector3f normal = point.getNormalVector3fMap().cwiseAbs();
        Eigen::Index axis = 0;
        normal.maxCoeff(&axis);
        return static_cast<int>(axis);
        };

    bool cadCells[27] = {};
    bool matchedCells[27] = {};
    bool cadNormalBins[3] = {};
    bool matchedNormalBins[3] = {};
    for (const auto& point : m_cadDs->points) {
        cadCells[cellIndex(point)] = true;
        cadNormalBins[normalBin(point)] = true;
    }

    PointCloudNT::Ptr aligned(new PointCloudNT());
    pcl::transformPointCloudWithNormals(*scanCloud, *aligned, scanToCad);
    pcl::KdTreeFLANN<PointNT> tree;
    tree.setInputCloud(m_cadDs);
    const float maximumDistanceSquared = std::pow(m_leafSize * 4.0f, 2.0f);
    for (const auto& point : aligned->points) {
        std::vector<int> index(1);
        std::vector<float> distance(1);
        if (tree.nearestKSearch(point, 1, index, distance) <= 0 ||
            distance[0] > maximumDistanceSquared) continue;
        const PointNT& target = m_cadDs->points[index[0]];
        matchedCells[cellIndex(target)] = true;
        matchedNormalBins[normalBin(target)] = true;
    }

    int cadCellCount = 0, matchedCellCount = 0;
    int cadNormalCount = 0, matchedNormalCount = 0;
    for (int i = 0; i < 27; ++i) {
        if (cadCells[i]) ++cadCellCount;
        if (cadCells[i] && matchedCells[i]) ++matchedCellCount;
    }
    for (int i = 0; i < 3; ++i) {
        if (cadNormalBins[i]) ++cadNormalCount;
        if (cadNormalBins[i] && matchedNormalBins[i]) ++matchedNormalCount;
    }
    const float cellCoverage = static_cast<float>(matchedCellCount) /
        static_cast<float>(std::max(cadCellCount, 1));
    const float normalCoverage = static_cast<float>(matchedNormalCount) /
        static_cast<float>(std::max(cadNormalCount, 1));
    return cellCoverage * 0.65f + normalCoverage * 0.35f;
}

bool ModelMatchWorker::coarseRegistration(std::vector<Eigen::Matrix4f>& transforms,
    std::vector<double>& fitnesses)
{
    struct Candidate {
        Eigen::Matrix4f T;
        float score;
        float scanOverlap;
        float cadOverlap;
        float normalConsistency;
        float featureOverlap;
        float spatialCoverage;
        float rmse;
    };
    std::vector<Candidate> candidates;

    pcl::KdTreeFLANN<PointNT> cadTree;
    cadTree.setInputCloud(m_cadDs);
    const bool distinctiveFeaturesAvailable = m_scanEdgeSubset && m_cadEdgeSubset &&
        m_scanEdgeSubset->size() >= 100 && m_cadEdgeSubset->size() >= 100;

    auto scoreCandidate = [&](const Eigen::Matrix4f& transform, Candidate& candidate) -> bool {
        PointCloudNT::Ptr alignedScan(new PointCloudNT());
        pcl::transformPointCloudWithNormals(*m_scanDs, *alignedScan, transform);

        std::vector<float> residuals;
        residuals.reserve(alignedScan->size());
        const float maxDistSqr = std::pow(m_leafSize * 4.0f, 2.0f);
        double normalSum = 0.0;
        for (const auto& point : alignedScan->points) {
            std::vector<int> index(1);
            std::vector<float> distance(1);
            if (cadTree.nearestKSearch(point, 1, index, distance) > 0 && distance[0] <= maxDistSqr) {
                residuals.push_back(distance[0]);
                const Eigen::Vector3f sourceNormal = point.getNormalVector3fMap();
                const Eigen::Vector3f targetNormal = m_cadDs->points[index[0]].getNormalVector3fMap();
                normalSum += std::abs(sourceNormal.dot(targetNormal));
            }
        }
        const float scanOverlap = alignedScan->empty() ? 0.0f :
            static_cast<float>(residuals.size()) / static_cast<float>(alignedScan->size());
        if (scanOverlap < 0.12f || residuals.empty()) return false;

        pcl::KdTreeFLANN<PointNT> scanTree;
        scanTree.setInputCloud(alignedScan);
        size_t cadSupported = 0;
        for (const auto& point : m_cadDs->points) {
            std::vector<int> index(1);
            std::vector<float> distance(1);
            if (scanTree.nearestKSearch(point, 1, index, distance) > 0 && distance[0] <= maxDistSqr) {
                ++cadSupported;
            }
        }
        const float cadOverlap = static_cast<float>(cadSupported) /
            static_cast<float>(m_cadDs->size());
        const float normalConsistency = static_cast<float>(normalSum / residuals.size());
        const float featureOverlap = distinctiveFeaturesAvailable
            ? evaluateFeatureOverlap(transform) : 0.0f;
        if (distinctiveFeaturesAvailable && featureOverlap < 0.08f) return false;
        const float spatialCoverage = evaluateSpatialCoverage(transform);
        if (spatialCoverage < 0.12f) return false;

        const size_t keepCount = std::max<size_t>(1, static_cast<size_t>(residuals.size() * 0.7f));
        std::nth_element(residuals.begin(), residuals.begin() + keepCount - 1, residuals.end());
        const double sum = std::accumulate(residuals.begin(), residuals.begin() + keepCount, 0.0);
        const float trimmedRmse = std::sqrt(static_cast<float>(sum / keepCount));
        const float featurePenalty = distinctiveFeaturesAvailable
            ? (1.0f - featureOverlap) * m_leafSize * 2.5f : 0.0f;
        const float score = trimmedRmse +
            (1.0f - scanOverlap) * m_leafSize * 2.0f +
            featurePenalty +
            (1.0f - spatialCoverage) * m_leafSize * 1.5f +
            (1.0f - normalConsistency) * m_leafSize * 0.75f -
            std::min(cadOverlap, 0.40f) * m_leafSize * 0.75f;
        candidate = { transform, score,
            scanOverlap, cadOverlap, normalConsistency, featureOverlap,
            spatialCoverage, trimmedRmse };
        return true;
    };

    // 粗配准仅接受 det(R)=+1 的旋转和平移变换。
    if (distinctiveFeaturesAvailable && m_scanEdgeFeatures && m_cadEdgeFeatures &&
        m_scanEdgeFeatures->size() == m_scanEdgeSubset->size() &&
        m_cadEdgeFeatures->size() == m_cadEdgeSubset->size()) {
        for (int run = 0; run < std::max(8, m_cfg.sacRuns); ++run) {
            pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> sac;
            sac.setInputSource(m_scanEdgeSubset);
            sac.setSourceFeatures(m_scanEdgeFeatures);
            sac.setInputTarget(m_cadEdgeSubset);
            sac.setTargetFeatures(m_cadEdgeFeatures);
            sac.setMaximumIterations(25000);
            sac.setNumberOfSamples(3);
            sac.setCorrespondenceRandomness(18);
            sac.setSimilarityThreshold(0.75f);
            sac.setMaxCorrespondenceDistance(m_leafSize * 4.0f);
            sac.setInlierFraction(std::clamp(m_cfg.sacInlierFrac, 0.08f, 0.30f));

            PointCloudNT alignedEdges;
            sac.align(alignedEdges);
            if (!sac.hasConverged()) continue;

            const Eigen::Matrix4f transform = sac.getFinalTransformation();
            if (transform.block<3, 3>(0, 0).determinant() < 0.0f) continue;
            Candidate candidate;
            if (scoreCandidate(transform, candidate)) candidates.push_back(candidate);
        }
    }

    // PPF 针对完整CAD到局部场景进行点对投票。限制模型点数以控制 O(N^2) 特征规模。
    auto uniformSubset = [](PointCloudNT::Ptr input, size_t maximumPoints) {
        PointCloudNT::Ptr output(new PointCloudNT());
        const size_t count = std::min(maximumPoints, input->size());
        output->reserve(count);
        const double step = static_cast<double>(input->size()) /
            static_cast<double>(std::max<size_t>(count, 1));
        for (size_t i = 0; i < count; ++i) {
            const size_t index = std::min(input->size() - 1,
                static_cast<size_t>(std::floor(i * step)));
            output->push_back(input->points[index]);
        }
        return output;
        };

    try {
        PointCloudNT::Ptr ppfModel = uniformSubset(m_cadDs, 500);
        PointCloudNT::Ptr ppfSceneBase = uniformSubset(m_scanDs, 1400);
        Eigen::Vector4f modelCentroid;
        pcl::compute3DCentroid(*ppfModel, modelCentroid);
        for (auto& point : ppfModel->points) {
            const Eigen::Vector3f radial = point.getVector3fMap() - modelCentroid.head<3>();
            if (point.getNormalVector3fMap().dot(radial) < 0.0f) {
                point.getNormalVector3fMap() *= -1.0f;
            }
        }

        pcl::PointCloud<pcl::PPFSignature>::Ptr modelFeatures(
            new pcl::PointCloud<pcl::PPFSignature>());
        pcl::PPFEstimation<PointNT, PointNT, pcl::PPFSignature> estimator;
        estimator.setInputCloud(ppfModel);
        estimator.setInputNormals(ppfModel);
        estimator.compute(*modelFeatures);

        pcl::PPFHashMapSearch::Ptr searchMethod(new pcl::PPFHashMapSearch(
            12.0f * static_cast<float>(M_PI) / 180.0f, m_leafSize * 1.5f));
        searchMethod->setInputFeatureCloud(modelFeatures);

        for (int normalPass = 0; normalPass < 2; ++normalPass) {
            PointCloudNT::Ptr ppfScene(new PointCloudNT(*ppfSceneBase));
            if (normalPass == 1) {
                for (auto& point : ppfScene->points) point.getNormalVector3fMap() *= -1.0f;
            }

            pcl::PPFRegistration<PointNT, PointNT> registration;
            registration.setInputSource(ppfModel);
            registration.setInputTarget(ppfScene);
            registration.setSearchMethod(searchMethod);
            registration.setSceneReferencePointSamplingRate(8);
            registration.setPositionClusteringThreshold(m_leafSize * 4.0f);
            registration.setRotationClusteringThreshold(
                15.0f * static_cast<float>(M_PI) / 180.0f);
            PointCloudNT alignedModel;
            registration.align(alignedModel);

            const auto poses = registration.getBestPoseCandidates();
            const size_t poseCount = std::min<size_t>(5, poses.size());
            for (size_t i = 0; i < poseCount; ++i) {
                const Eigen::Matrix4f cadToScan = poses[i].pose.matrix();
                if (!cadToScan.allFinite() ||
                    cadToScan.block<3, 3>(0, 0).determinant() < 0.0f) continue;
                Candidate candidate;
                if (scoreCandidate(cadToScan.inverse(), candidate)) {
                    candidates.push_back(candidate);
                }
            }
        }
    }
    catch (const std::exception& exception) {
        emit progress(QString::fromUtf8("PPF候选生成失败，继续使用FPFH/PCA：%1")
            .arg(QString::fromUtf8(exception.what())));
    }

    // FPFH 对局部缺失和重复平面较敏感。补充 24 个右手系 PCA 姿态作为确定性候选，
    // 它们只包含旋转和平移，不包含反射。
    auto principalFrame = [](PointCloudNT::Ptr cloud, Eigen::Vector3f& center,
        Eigen::Matrix3f& basis) {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud, centroid);
            center = centroid.head<3>();
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            basis.col(0) = solver.eigenvectors().col(2).normalized();
            basis.col(1) = solver.eigenvectors().col(1).normalized();
            basis.col(2) = basis.col(0).cross(basis.col(1)).normalized();
        };

    Eigen::Vector3f scanCenter, cadCenter;
    Eigen::Matrix3f scanBasis, cadBasis;
    principalFrame(m_scanDs, scanCenter, scanBasis);
    principalFrame(m_cadDs, cadCenter, cadBasis);
    const int permutations[6][3] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2},
        {1, 2, 0}, {2, 0, 1}, {2, 1, 0}
    };
    for (const auto& permutation : permutations) {
        for (int sx : { -1, 1 }) {
            for (int sy : { -1, 1 }) {
                for (int sz : { -1, 1 }) {
                    Eigen::Matrix3f sourceFrame;
                    sourceFrame.col(0) = static_cast<float>(sx) * scanBasis.col(permutation[0]);
                    sourceFrame.col(1) = static_cast<float>(sy) * scanBasis.col(permutation[1]);
                    sourceFrame.col(2) = static_cast<float>(sz) * scanBasis.col(permutation[2]);
                    const Eigen::Matrix3f rotation = cadBasis * sourceFrame.transpose();
                    if (rotation.determinant() < 0.99f) continue;

                    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                    transform.block<3, 3>(0, 0) = rotation;
                    transform.block<3, 1>(0, 3) = cadCenter - rotation * scanCenter;
                    Candidate candidate;
                    if (scoreCandidate(transform, candidate)) candidates.push_back(candidate);
                }
            }
        }
    }

    if (candidates.empty()) return false;

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.score < b.score;
        });

    std::vector<Candidate> distinctCandidates;
    for (const auto& candidate : candidates) {
        bool duplicate = false;
        for (const auto& selected : distinctCandidates) {
            const Eigen::Matrix3f relativeRotation = candidate.T.block<3, 3>(0, 0) *
                selected.T.block<3, 3>(0, 0).transpose();
            const float cosAngle = std::clamp((relativeRotation.trace() - 1.0f) * 0.5f, -1.0f, 1.0f);
            const float angle = std::acos(cosAngle);
            const float translation = (candidate.T.block<3, 1>(0, 3) -
                selected.T.block<3, 1>(0, 3)).norm();
            if (angle < 8.0f * static_cast<float>(M_PI) / 180.0f &&
                translation < m_leafSize * 3.0f) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) distinctCandidates.push_back(candidate);
        if (distinctCandidates.size() >= 6) break;
    }
    if (distinctCandidates.empty()) return false;

    m_coarseCandidateReport.clear();
    const size_t reportCount = distinctCandidates.size();
    for (size_t i = 0; i < reportCount; ++i) {
        const auto& candidate = distinctCandidates[i];
        m_coarseCandidateReport += QString::fromUtf8(
            "刚体姿态候选%1：分数=%2，扫描覆盖=%3%，关键特征=%4%，空间结构=%5%，CAD支持=%6%，法向=%7%\n")
            .arg(i + 1).arg(candidate.score, 0, 'f', 3)
            .arg(candidate.scanOverlap * 100.0f, 0, 'f', 1)
            .arg(candidate.featureOverlap * 100.0f, 0, 'f', 1)
            .arg(candidate.spatialCoverage * 100.0f, 0, 'f', 1)
            .arg(candidate.cadOverlap * 100.0f, 0, 'f', 1)
            .arg(candidate.normalConsistency * 100.0f, 0, 'f', 1);
    }

    transforms.clear();
    fitnesses.clear();
    for (const auto& candidate : distinctCandidates) {
        transforms.push_back(candidate.T);
        fitnesses.push_back(candidate.score);
    }
    return true;
}

PointCloudNT::Ptr ModelMatchWorker::extractCadGuidedScan(
    const Eigen::Matrix4f& scanToCad, float distanceFactor) const
{
    PointCloudNT::Ptr extracted(new PointCloudNT());
    if (!m_scanDs || m_scanDs->empty() || !m_cadDs || m_cadDs->empty()) return extracted;

    PointCloudNT::Ptr aligned(new PointCloudNT());
    pcl::transformPointCloudWithNormals(*m_scanDs, *aligned, scanToCad);
    pcl::KdTreeFLANN<PointNT> tree;
    tree.setInputCloud(m_cadDs);
    const float maximumDistanceSquared =
        std::pow(m_leafSize * distanceFactor, 2.0f);
    extracted->reserve(m_scanDs->size());

    for (size_t i = 0; i < aligned->size(); ++i) {
        std::vector<int> index(1);
        std::vector<float> distance(1);
        if (tree.nearestKSearch(aligned->points[i], 1, index, distance) <= 0 ||
            distance[0] > maximumDistanceSquared) continue;
        const float normalAgreement = std::abs(
            aligned->points[i].getNormalVector3fMap().dot(
                m_cadDs->points[index[0]].getNormalVector3fMap()));
        if (normalAgreement < 0.15f) continue;
        extracted->push_back(m_scanDs->points[i]);
    }
    return extracted;
}

// ==========================================
// 阶段 2A：点到点 ICP
// ==========================================
Eigen::Matrix4f ModelMatchWorker::fineRegistrationA(const Eigen::Matrix4f& initT,
    PointCloudNT::Ptr sourceCloud, double& fitnessOut)
{
    if (!sourceCloud || sourceCloud->size() < 100) {
        fitnessOut = std::numeric_limits<double>::infinity();
        return initT;
    }
    PointCloudNT::Ptr scanAligned(new PointCloudNT());
    pcl::transformPointCloudWithNormals(*sourceCloud, *scanAligned, initT);

    pcl::IterativeClosestPoint<PointNT, PointNT> icpA;
    icpA.setInputSource(scanAligned);
    icpA.setInputTarget(m_cadDs);
    icpA.setMaximumIterations(m_cfg.icpMaxIterA);
    icpA.setMaxCorrespondenceDistance(m_leafSize * 3.0f);
    icpA.setRANSACOutlierRejectionThreshold(m_leafSize * 1.5f);
    icpA.setTransformationEpsilon(1e-7);
    icpA.setEuclideanFitnessEpsilon(1e-6);

    auto distRej = pcl::make_shared<pcl::registration::CorrespondenceRejectorDistance>();
    distRej->setMaximumDistance(m_leafSize * 2.0f);
    icpA.addCorrespondenceRejector(distRej);

    PointCloudNT::Ptr resultA(new PointCloudNT());
    emit progress(QString::fromUtf8("精配准A：正在执行鲁棒ICP..."));
    icpA.align(*resultA);
    if (!icpA.hasConverged() || !icpA.getFinalTransformation().allFinite()) {
        emit progress(QString::fromUtf8("精配准A：ICP未收敛，保留粗配准姿态。"));
        fitnessOut = std::numeric_limits<double>::infinity();
        return initT;
    }
    fitnessOut = icpA.getFitnessScore();
    emit progress(QString::fromUtf8("精配准A完成：适应度 %1").arg(fitnessOut, 0, 'f', 4));

    return icpA.getFinalTransformation() * initT;
}

// ==========================================
// 辅助函数：构建焊缝引力场
// ==========================================
std::vector<float> ModelMatchWorker::buildSeamWeights(PointCloudNT::Ptr cadDs)
{
    std::vector<float> weights(cadDs->size(), 1.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr allSeams(new pcl::PointCloud<pcl::PointXYZ>());

    for (auto& cloudRGB : m_seamPoints) {
        if (!cloudRGB) continue;
        for (const auto& pt : cloudRGB->points) {
            allSeams->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }
    }

    if (allSeams->empty()) return weights;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(allSeams);

    float sigma = m_leafSize * m_cfg.seamSigmaFactor;
    std::vector<int> idx(1);
    std::vector<float> dist_sqr(1);

    for (size_t i = 0; i < cadDs->size(); ++i) {
        pcl::PointXYZ q(cadDs->points[i].x, cadDs->points[i].y, cadDs->points[i].z);
        if (kdtree.nearestKSearch(q, 1, idx, dist_sqr) > 0) {
            float d2 = dist_sqr[0];
            weights[i] = 1.0f + (m_cfg.seamWeightMax - 1.0f) * std::exp(-d2 / (2.0f * sigma * sigma));
        }
    }
    return weights;
}

// ==========================================
// 阶段 2B：自定义加权点到面 ICP
// ==========================================
Eigen::Matrix4f ModelMatchWorker::fineRegistrationB(const Eigen::Matrix4f& T_A, const std::vector<float>& weights,
    double fitnessA, int& iterOut, bool& convergedOut,
    float& finalWeightOut, double& fitnessOut)
{
    Eigen::Matrix4f T_B = Eigen::Matrix4f::Identity();
    PointCloudNT::Ptr scanAligned(new PointCloudNT());
    pcl::transformPointCloudWithNormals(*m_scanDs, *scanAligned, T_A);

    pcl::KdTreeFLANN<PointNT> kdtreeTarget;
    kdtreeTarget.setInputCloud(m_cadDs);

    convergedOut = false;
    float currentWMax = m_cfg.seamWeightMax;
    float maxAngleStep = 0.05f;

    for (iterOut = 0; iterOut < m_cfg.icpMaxIterB; ++iterOut) {
        Eigen::Matrix<float, 6, 6> ATA = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> ATb = Eigen::Matrix<float, 6, 1>::Zero();
        int validPairs = 0;
        double currentTotalError = 0;

        for (size_t i = 0; i < scanAligned->size(); ++i) {
            std::vector<int> nn_idx(1);
            std::vector<float> nn_dist_sqr(1);
            if (kdtreeTarget.nearestKSearch(scanAligned->points[i], 1, nn_idx, nn_dist_sqr) > 0) {
                if (nn_dist_sqr[0] > (m_leafSize * 2.0f) * (m_leafSize * 2.0f)) continue;

                int tgt_idx = nn_idx[0];
                Eigen::Vector3f p = scanAligned->points[i].getVector3fMap();
                Eigen::Vector3f n_src = scanAligned->points[i].getNormalVector3fMap();
                Eigen::Vector3f q = m_cadDs->points[tgt_idx].getVector3fMap();
                Eigen::Vector3f n = m_cadDs->points[tgt_idx].getNormalVector3fMap();

                float cos_theta = n_src.dot(n);
                if (cos_theta < 0.0f) {
                    n = -n;
                    cos_theta = -cos_theta;
                }
                if (cos_theta < 0.1f) continue;

                const float distance = std::sqrt(nn_dist_sqr[0]);
                const float huberDelta = m_leafSize;
                const float robustWeight = distance <= huberDelta ? 1.0f : huberDelta / distance;
                float w = robustWeight * cos_theta;

                Eigen::Vector3f pxn = p.cross(n);
                Eigen::Matrix<float, 1, 6> J;
                J << pxn(0), pxn(1), pxn(2), n(0), n(1), n(2);
                float r = -n.dot(p - q);

                ATA += w * J.transpose() * J;
                ATb += w * J.transpose() * r;
                validPairs++;
                currentTotalError += nn_dist_sqr[0];
            }
        }

        if (validPairs < 10) break;
        fitnessOut = currentTotalError / validPairs;

        Eigen::Matrix<float, 6, 1> x = ATA.ldlt().solve(ATb);
        if (!x.allFinite()) break;
        float angle = x.head<3>().norm();
        if (angle > maxAngleStep) x *= (maxAngleStep / angle);

        Eigen::Matrix4f dT = Eigen::Matrix4f::Identity();
        if (angle > 1e-6) dT.block<3, 3>(0, 0) = Eigen::AngleAxisf(x.head<3>().norm(), x.head<3>().normalized()).toRotationMatrix();
        dT.block<3, 1>(0, 3) = x.tail<3>();

        T_B = dT * T_B;
        pcl::transformPointCloudWithNormals(*scanAligned, *scanAligned, dT);

        if (x.head<3>().norm() < 1e-4f && x.tail<3>().norm() < 1e-4f) {
            convergedOut = true;
            break;
        }
    }

    finalWeightOut = currentWMax;
    return T_B * T_A;
}

void ModelMatchWorker::evaluateAlignment(const Eigen::Matrix4f& scanToCad,
    float& overlap, float& reverseOverlap, float& normalConsistency,
    float& rmse, float& p95Residual) const
{
    evaluateAlignmentForCloud(scanToCad, m_scanDs, overlap, reverseOverlap,
        normalConsistency, rmse, p95Residual);
}

void ModelMatchWorker::evaluateAlignmentForCloud(const Eigen::Matrix4f& scanToCad,
    PointCloudNT::Ptr scanCloud, float& overlap, float& reverseOverlap,
    float& normalConsistency, float& rmse, float& p95Residual) const
{
    overlap = 0.0f;
    reverseOverlap = 0.0f;
    normalConsistency = 0.0f;
    rmse = std::numeric_limits<float>::infinity();
    p95Residual = std::numeric_limits<float>::infinity();
    if (!scanCloud || scanCloud->empty() || !m_cadDs || m_cadDs->empty()) return;

    PointCloudNT::Ptr aligned(new PointCloudNT());
    pcl::transformPointCloudWithNormals(*scanCloud, *aligned, scanToCad);
    pcl::KdTreeFLANN<PointNT> tree;
    tree.setInputCloud(m_cadDs);

    const float maxDistSqr = std::pow(m_leafSize * 2.5f, 2.0f);
    std::vector<float> residuals;
    residuals.reserve(aligned->size());
    double squaredSum = 0.0;
    double normalSum = 0.0;
    for (const auto& point : aligned->points) {
        std::vector<int> index(1);
        std::vector<float> distance(1);
        if (tree.nearestKSearch(point, 1, index, distance) > 0 && distance[0] <= maxDistSqr) {
            residuals.push_back(std::sqrt(distance[0]));
            squaredSum += distance[0];
            normalSum += std::abs(point.getNormalVector3fMap().dot(
                m_cadDs->points[index[0]].getNormalVector3fMap()));
        }
    }

    overlap = static_cast<float>(residuals.size()) / static_cast<float>(aligned->size());
    if (residuals.empty()) return;
    rmse = std::sqrt(static_cast<float>(squaredSum / residuals.size()));
    normalConsistency = static_cast<float>(normalSum / residuals.size());

    pcl::KdTreeFLANN<PointNT> scanTree;
    scanTree.setInputCloud(aligned);
    size_t supportedCad = 0;
    for (const auto& point : m_cadDs->points) {
        std::vector<int> index(1);
        std::vector<float> distance(1);
        if (scanTree.nearestKSearch(point, 1, index, distance) > 0 && distance[0] <= maxDistSqr) {
            ++supportedCad;
        }
    }
    reverseOverlap = static_cast<float>(supportedCad) / static_cast<float>(m_cadDs->size());
    std::sort(residuals.begin(), residuals.end());
    const size_t p95Index = std::min(residuals.size() - 1,
        static_cast<size_t>(std::floor((residuals.size() - 1) * 0.95f)));
    p95Residual = residuals[p95Index];
}

// ==========================================
// 焊缝投影与评估
// ==========================================
std::vector<SeamMatchResult> ModelMatchWorker::projectSeams(const Eigen::Matrix4f& T_final)
{
    std::vector<SeamMatchResult> results;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtreeScan;
    if (!m_scanCloud->empty()) kdtreeScan.setInputCloud(m_scanCloud);

    float confThreshSqr = (m_leafSize * m_cfg.confThreshFactor) * (m_leafSize * m_cfg.confThreshFactor);

    for (size_t si = 0; si < m_seamPoints.size(); ++si) {
        SeamMatchResult sr;
        sr.seamId = si;
        sr.seamName = m_seamNames[si];
        sr.meanResidual = 0;
        sr.maxResidual = 0;
        int validPts = 0;

        auto seamCloud = m_seamPoints[si];
        if (!seamCloud || seamCloud->empty() || m_scanCloud->empty()) {
            sr.confidence = SeamConfidence::Low;
            results.push_back(sr);
            continue;
        }

        CloudRGB::Ptr transSeam(new CloudRGB());
        pcl::transformPointCloud(*seamCloud, *transSeam, T_final);

        for (const auto& pt : transSeam->points) {
            std::vector<int> idx(1);
            std::vector<float> dist_sqr(1);
            WeldPathPoint wp;
            wp.normal = Eigen::Vector3f(0, 0, 1); // 简化处理，真实法向应由CAD推导

            if (kdtreeScan.nearestKSearch(pt, 1, idx, dist_sqr) > 0 && dist_sqr[0] < confThreshSqr) {
                wp.position = m_scanCloud->points[idx[0]].getVector3fMap();
                wp.fromScan = true;
                validPts++;
                float dist = std::sqrt(dist_sqr[0]);
                sr.meanResidual += dist;
                if (dist > sr.maxResidual) sr.maxResidual = dist;
            }
            else {
                wp.position = pt.getVector3fMap();
                wp.fromScan = false;
            }
            sr.pathPoints.push_back(wp);
        }

        sr.coverageRatio = (float)validPts / transSeam->size();
        if (validPts > 0) sr.meanResidual /= validPts;

        if (sr.coverageRatio < 0.5f) sr.confidence = SeamConfidence::Low;
        else if (sr.meanResidual > (m_leafSize * m_cfg.confThreshFactor) * 0.7f) sr.confidence = SeamConfidence::Medium;
        else sr.confidence = SeamConfidence::High;

        results.push_back(sr);
    }
    return results;
}

MatchVerdict ModelMatchWorker::classifyResult(const MatchResult& r) const
{
    if (!std::isfinite(r.rmse) || r.overlapRatio < 0.30f ||
        r.reverseOverlapRatio < 0.10f || r.featureOverlapRatio < 0.15f ||
        r.spatialCoverageRatio < 0.18f || r.normalConsistency < 0.45f) {
        return MatchVerdict::Fail;
    }
    if (r.rmse > m_leafSize * 1.25f || r.p95Residual > m_leafSize * 2.0f) {
        return MatchVerdict::Fail;
    }
    if (r.wasAmbiguous) return MatchVerdict::NeedsReview;
    if (!r.convergedStageB || r.overlapRatio < 0.55f ||
        r.reverseOverlapRatio < 0.25f || r.featureOverlapRatio < 0.35f ||
        r.spatialCoverageRatio < 0.40f || r.normalConsistency < 0.70f) {
        return MatchVerdict::NeedsReview;
    }
    return MatchVerdict::Pass;
}
