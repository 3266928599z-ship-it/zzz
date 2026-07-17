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

#include "MatchTypes.h"


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
