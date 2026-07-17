#pragma once

// 1. 【极度重要：防范 Windows 宏污染】
#ifndef NOMINMAX
#define NOMINMAX
#endif

// 2. 【极度重要：先包含 PCL/Eigen/Boost】
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// 3. 【最后包含 Qt 头文件】
#include <QObject>
#include <QThread>
#include <vector>
#include <QString>
#include <QMetaType>

// ===================================================================
// 匹配参数与结果数据结构
// ===================================================================

// ---- 可调参数 ----
struct MatchConfig {
    float leafSizeFactor = 0.01f;
    float edgePercentile = 0.25f;
    int   sacRuns = 8;
    float sacInlierFrac = 0.15f;
    float ambiguityRatio = 1.10f;
    float seamSigmaFactor = 7.0f;
    float seamWeightMax = 8.0f;
    float confThreshFactor = 2.0f;
    int   icpMaxIterA = 50;
    int   icpMaxIterB = 50;
};

// ---- 焊缝路径点 ----
struct WeldPathPoint {
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    bool fromScan;   // false = 扫描点云缺失,该点为CAD推算
};

enum class SeamConfidence { High, Medium, Low };
enum class MatchVerdict { Pass, NeedsReview, Fail };

// ---- 逐焊缝结果 ----
struct SeamMatchResult {
    int seamId;
    QString seamName;
    float coverageRatio;
    float meanResidual;
    float maxResidual;
    SeamConfidence confidence;
    std::vector<WeldPathPoint> pathPoints;
};

// ---- 整体匹配结果 ----
struct MatchResult {
    // 【终极修复核心】：接管内存分配，保障 16 字节对齐，防止 Qt 跨线程注册崩溃
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool success = false;
    Eigen::Matrix4f T_final = Eigen::Matrix4f::Identity();
    float fitnessStageA = 0.f;
    float fitnessStageB = 0.f;
    float overlapRatio = 0.f;
    float reverseOverlapRatio = 0.f;
    float normalConsistency = 0.f;
    float featureOverlapRatio = 0.f;
    float spatialCoverageRatio = 0.f;
    float rmse = 0.f;
    float p95Residual = 0.f;
    QString coarseCandidateReport;
    int   iterationsStageB = 0;
    bool  convergedStageB = false;
    float finalSeamWeight = 1.f;
    bool  wasAmbiguous = false;
    float ambiguityRatio = 0.f;
    Eigen::Matrix4f T_candidate2 = Eigen::Matrix4f::Identity();
    std::vector<SeamMatchResult> seamResults;
    MatchVerdict verdict = MatchVerdict::Fail;
    QString verdictMessage;
    QString errorMessage;
};


// ===================================================================
// ModelMatchWorker 类定义
// ===================================================================

// 定义内部使用的类型别名
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

class ModelMatchWorker : public QObject
{
    Q_OBJECT
public:
    using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;
    explicit ModelMatchWorker(QObject* parent = nullptr);
    ~ModelMatchWorker();

    // 接收主线程传来的数据
    void setInput(CloudRGB::Ptr cadCloud,
        CloudRGB::Ptr scanCloud,
        std::vector<CloudRGB::Ptr> seamPointsList,
        std::vector<QString> seamNames,
        const MatchConfig& cfg);

public slots:
    void run(); // 子线程执行的入口

signals:
    void progress(QString stage);
    void processedScanReady(CloudRGB::Ptr cloud);
    void finished(MatchResult result);

private:
    // ---- 阶段实现 ----
    void preprocess();

    bool coarseRegistration(std::vector<Eigen::Matrix4f>& transforms,
        std::vector<double>& fitnesses);

    Eigen::Matrix4f fineRegistrationA(const Eigen::Matrix4f& initT,
        PointCloudNT::Ptr sourceCloud,
        double& fitnessOut);

    std::vector<float> buildSeamWeights(PointCloudNT::Ptr cadDs);

    Eigen::Matrix4f fineRegistrationB(const Eigen::Matrix4f& T_A,
        const std::vector<float>& weights,
        double fitnessA,
        int& iterOut,
        bool& convergedOut,
        float& finalWeightOut,
        double& fitnessOut);

    std::vector<SeamMatchResult> projectSeams(const Eigen::Matrix4f& T_final);

    MatchVerdict classifyResult(const MatchResult& r) const;

    // ---- 内部辅助函数 ----
    void extractEdgeFeatures(PointCloudNT::Ptr cloud, float leafSize, PointCloudNT::Ptr& outEdgeCloud, FeatureCloudT::Ptr& outFeatures);

    void evaluateAlignment(const Eigen::Matrix4f& scanToCad,
        float& overlap,
        float& reverseOverlap,
        float& normalConsistency,
        float& rmse,
        float& p95Residual) const;

    void evaluateAlignmentForCloud(const Eigen::Matrix4f& scanToCad,
        PointCloudNT::Ptr scanCloud,
        float& overlap,
        float& reverseOverlap,
        float& normalConsistency,
        float& rmse,
        float& p95Residual) const;

    float evaluateFeatureOverlap(const Eigen::Matrix4f& scanToCad) const;
    float evaluateSpatialCoverage(const Eigen::Matrix4f& scanToCad) const;
    float evaluateSpatialCoverageForCloud(const Eigen::Matrix4f& scanToCad,
        PointCloudNT::Ptr scanCloud) const;
    PointCloudNT::Ptr extractCadGuidedScan(const Eigen::Matrix4f& scanToCad,
        float distanceFactor) const;

private:
    CloudRGB::Ptr m_cadCloud;
    CloudRGB::Ptr m_scanCloud;
    std::vector<CloudRGB::Ptr> m_seamPoints;
    std::vector<QString> m_seamNames;
    MatchConfig m_cfg;

    float m_leafSize = 1.0f;
    float m_scanToCadSizeRatio = 1.0f;

    // 预处理后的缓存
    PointCloudNT::Ptr m_cadDs;
    PointCloudNT::Ptr m_scanDs;
    PointCloudNT::Ptr m_cadEdgeSubset;
    PointCloudNT::Ptr m_scanEdgeSubset;
    FeatureCloudT::Ptr m_cadEdgeFeatures;
    FeatureCloudT::Ptr m_scanEdgeFeatures;

    QString m_coarseCandidateReport;
};

// ===================================================================
// 极其重要：向 Qt 注册自定义类型，允许其在多线程信号槽中传递
// ===================================================================
Q_DECLARE_METATYPE(MatchResult);
