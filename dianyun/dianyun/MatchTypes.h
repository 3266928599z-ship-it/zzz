#pragma once

#include <Eigen/Core>
#include <QString>
#include <QMetaType>

struct MatchConfig {
    float leafSizeFactor = 0.01f;
    float edgePercentile = 0.25f;
    int sacRuns = 8;
    float sacInlierFrac = 0.15f;
    float ambiguityRatio = 1.10f;
    int icpMaxIterA = 50;
    int icpMaxIterB = 50;
};

enum class MatchVerdict { Pass, NeedsReview, Fail };

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
    bool wasAmbiguous = false;
    float ambiguityRatio = 0.0f;
    Eigen::Matrix4f T_candidate2 = Eigen::Matrix4f::Identity();
    MatchVerdict verdict = MatchVerdict::Fail;
    QString verdictMessage;
    QString errorMessage;
};

Q_DECLARE_METATYPE(MatchResult)
