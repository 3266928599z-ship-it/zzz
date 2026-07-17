#pragma once

#include <Eigen/Core>
#include <QString>
#include <QMetaType>
#include <vector>

struct MatchConfig {
    float leafSizeFactor = 0.01f;
    float edgePercentile = 0.25f;
    int sacRuns = 8;
    float sacInlierFrac = 0.15f;
    float ambiguityRatio = 1.10f;
    float seamSigmaFactor = 7.0f;
    float seamWeightMax = 8.0f;
    float confThreshFactor = 2.0f;
    int icpMaxIterA = 50;
    int icpMaxIterB = 50;
};

struct WeldPathPoint {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
    bool fromScan = false;
};

enum class SeamConfidence { High, Medium, Low };
enum class MatchVerdict { Pass, NeedsReview, Fail };

struct SeamMatchResult {
    int seamId = -1;
    QString seamName;
    float coverageRatio = 0.0f;
    float meanResidual = 0.0f;
    float maxResidual = 0.0f;
    SeamConfidence confidence = SeamConfidence::Low;
    std::vector<WeldPathPoint> pathPoints;
};

struct MatchResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool success = false;
    Eigen::Matrix4f T_final = Eigen::Matrix4f::Identity();
    float fitnessStageA = 0.0f;
    float fitnessStageB = 0.0f;
    float overlapRatio = 0.0f;
    float reverseOverlapRatio = 0.0f;
    float normalConsistency = 0.0f;
    float featureOverlapRatio = 0.0f;
    float spatialCoverageRatio = 0.0f;
    float rmse = 0.0f;
    float p95Residual = 0.0f;
    QString coarseCandidateReport;
    int iterationsStageB = 0;
    bool convergedStageB = false;
    float finalSeamWeight = 1.0f;
    bool wasAmbiguous = false;
    float ambiguityRatio = 0.0f;
    Eigen::Matrix4f T_candidate2 = Eigen::Matrix4f::Identity();
    std::vector<SeamMatchResult> seamResults;
    MatchVerdict verdict = MatchVerdict::Fail;
    QString verdictMessage;
    QString errorMessage;
};

Q_DECLARE_METATYPE(MatchResult)
