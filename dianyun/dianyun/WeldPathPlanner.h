#pragma once

#include <Eigen/Core>
#include <QString>

#include <array>
#include <vector>

// 焊缝轨迹的一个 TCP 目标（基坐标，mm / 度，法奥固定轴 XYZ）
struct WeldWaypoint {
    enum class Kind { Approach, Descend, Seam, Retreat };
    Kind kind = Kind::Seam;
    std::array<double, 6> pose{};
};

struct WeldPathOptions {
    // 方案 A：全程调平，工具 Z 竖直向下，即 Rx 180 / Ry 0 / Rz 航向
    double headingDeg = 0.0;
    // 起点上方接近点、终点上方退出点的高度（沿基坐标 +Z）
    double approachMm = 30.0;
    // 整条轨迹额外抬高量，空走用；工具 1 自身已含 10 mm 安全余量
    double liftMm = 0.0;
    // 相邻顶点间距小于该值时合并
    double minSpacingMm = 0.5;
    // 三点夹角偏离直线不足该值时删去中间点，减少 MoveL 段数
    double collinearToleranceDeg = 0.5;
};

struct WeldPathPlan {
    std::vector<WeldWaypoint> waypoints;   // 接近 → 下降(起点) → 焊缝 → 退出
    double seamLengthMm = 0.0;
    double minSeamZ = 0.0;
    double maxSeamZ = 0.0;
    int seamSegments = 0;

    const std::array<double, 6>& seamStart() const { return waypoints[1].pose; }
    const std::array<double, 6>& seamEnd() const
    {
        return waypoints[waypoints.size() - 2].pose;
    }
};

// 把基坐标下的有序焊缝折线转成 TCP 目标序列。纯几何，不访问机械臂。
bool planWeldPath(const std::vector<Eigen::Vector3f>& seamInBase,
    const WeldPathOptions& options, WeldPathPlan& plan, QString& errorMessage);
