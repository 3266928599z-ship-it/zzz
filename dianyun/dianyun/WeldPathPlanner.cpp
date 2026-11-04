#include "WeldPathPlanner.h"

#include "PoseMath.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

std::vector<Eigen::Vector3d> simplify(const std::vector<Eigen::Vector3f>& input,
    double minSpacing, double collinearToleranceDeg)
{
    std::vector<Eigen::Vector3d> spaced;
    spaced.reserve(input.size());
    for (const auto& point : input) {
        const Eigen::Vector3d p = point.cast<double>();
        if (spaced.empty() || (p - spaced.back()).norm() >= minSpacing) {
            spaced.push_back(p);
        }
    }
    // 末点必须保留：间距合并可能把真正的终点吞掉
    if (!input.empty() && spaced.size() > 1) {
        spaced.back() = input.back().cast<double>();
    }
    if (spaced.size() < 3) return spaced;

    const double cosTolerance = std::cos(collinearToleranceDeg * PoseMath::kDegToRad);
    std::vector<Eigen::Vector3d> out;
    out.reserve(spaced.size());
    out.push_back(spaced.front());
    for (std::size_t i = 1; i + 1 < spaced.size(); ++i) {
        const Eigen::Vector3d a = (spaced[i] - out.back()).normalized();
        const Eigen::Vector3d b = (spaced[i + 1] - spaced[i]).normalized();
        if (a.dot(b) < cosTolerance) out.push_back(spaced[i]);
    }
    out.push_back(spaced.back());
    return out;
}

} // namespace

bool planWeldPath(const std::vector<Eigen::Vector3f>& seamInBase,
    const WeldPathOptions& options, WeldPathPlan& plan, QString& errorMessage)
{
    plan = WeldPathPlan();
    errorMessage.clear();
    for (const auto& point : seamInBase) {
        if (!point.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝顶点包含无效坐标。");
            return false;
        }
    }
    if (!(options.approachMm > 0.0) || options.liftMm < 0.0
        || !std::isfinite(options.headingDeg)) {
        errorMessage = QString::fromUtf8("轨迹参数无效（接近高度须大于 0，抬高量不能为负）。");
        return false;
    }
    if (options.needReturn) {
        for (double value : options.returnPose) {
            if (!std::isfinite(value)) {
                errorMessage = QString::fromUtf8("返回位姿包含无效数值。");
                return false;
            }
        }
    }

    const std::vector<Eigen::Vector3d> path = simplify(
        seamInBase, std::max(options.minSpacingMm, 1e-6),
        options.collinearToleranceDeg);
    if (path.size() < 2) {
        errorMessage = QString::fromUtf8("焊缝有效长度过短，无法生成轨迹。");
        return false;
    }

    const auto makePose = [&options](const Eigen::Vector3d& p, double dz) {
        return std::array<double, 6>{ p.x(), p.y(), p.z() + options.liftMm + dz,
            180.0, 0.0, options.headingDeg };
    };

    plan.waypoints.push_back({ WeldWaypoint::Kind::Approach,
        makePose(path.front(), options.approachMm) });
    plan.waypoints.push_back({ WeldWaypoint::Kind::Descend, makePose(path.front(), 0.0) });
    plan.minSeamZ = std::numeric_limits<double>::infinity();
    plan.maxSeamZ = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
        plan.minSeamZ = std::min(plan.minSeamZ, path[i].z());
        plan.maxSeamZ = std::max(plan.maxSeamZ, path[i].z());
        if (i == 0) continue;
        plan.seamLengthMm += (path[i] - path[i - 1]).norm();
        plan.waypoints.push_back({ WeldWaypoint::Kind::Seam, makePose(path[i], 0.0) });
    }
    plan.waypoints.push_back({ WeldWaypoint::Kind::Retreat,
        makePose(path.back(), options.approachMm) });
    plan.seamEndIndex = plan.waypoints.size() - 2;
    plan.seamSegments = static_cast<int>(path.size()) - 1;

    // 返回拍摄起点：先竖直抬到两者中较高处，再水平平移并转到拍摄姿态，
    // 最后竖直降到位。直接斜穿可能在未抬够高度时撞到工件。
    if (options.needReturn) {
        constexpr double kHeightToleranceMm = 0.5;
        const auto& retreat = plan.waypoints.back().pose;
        const double safeZ = std::max(retreat[2], options.returnPose[2]);

        if (safeZ - retreat[2] > kHeightToleranceMm) {
            plan.waypoints.push_back({ WeldWaypoint::Kind::Return,
                { retreat[0], retreat[1], safeZ,
                  retreat[3], retreat[4], retreat[5] } });
        }
        plan.waypoints.push_back({ WeldWaypoint::Kind::Return,
            { options.returnPose[0], options.returnPose[1], safeZ,
              options.returnPose[3], options.returnPose[4],
              options.returnPose[5] } });
        if (safeZ - options.returnPose[2] > kHeightToleranceMm) {
            plan.waypoints.push_back({ WeldWaypoint::Kind::Return,
                options.returnPose });
        }
    }
    return true;
}
