#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

// 法奥位姿约定：x/y/z 为 mm，rx/ry/rz 为度，绕**固定轴** X→Y→Z 依次旋转，
// 即 R = Rz·Ry·Rx。HiCalibrator 的「固定欧拉角 XYZ」与之相同。
namespace PoseMath {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

inline Eigen::Matrix3d rpyToMatrix(double rxDeg, double ryDeg, double rzDeg)
{
    return (Eigen::AngleAxisd(rzDeg * kDegToRad, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(ryDeg * kDegToRad, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rxDeg * kDegToRad, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

// rpyToMatrix 的逆。ry 取 [-90°, 90°]；万向锁（|ry| = 90°）时令 rz = 0。
inline void matrixToRpy(const Eigen::Matrix3d& r,
    double& rxDeg, double& ryDeg, double& rzDeg)
{
    const double sy = std::clamp(-r(2, 0), -1.0, 1.0);
    const double ry = std::asin(sy);
    double rx = 0.0;
    double rz = 0.0;
    if (std::cos(ry) > 1e-9) {
        rx = std::atan2(r(2, 1), r(2, 2));
        rz = std::atan2(r(1, 0), r(0, 0));
    }
    else {
        // rz = 0 时 R(0,1) = sy·sx，R(1,1) = cx
        rx = std::atan2(sy * r(0, 1), r(1, 1));
    }
    rxDeg = rx / kDegToRad;
    ryDeg = ry / kDegToRad;
    rzDeg = rz / kDegToRad;
}

inline Eigen::Matrix4d poseToMatrix(const double pose[6])
{
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3, 3>(0, 0) = rpyToMatrix(pose[3], pose[4], pose[5]);
    t.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    return t;
}

inline void matrixToPose(const Eigen::Matrix4d& t, double pose[6])
{
    pose[0] = t(0, 3);
    pose[1] = t(1, 3);
    pose[2] = t(2, 3);
    matrixToRpy(t.block<3, 3>(0, 0), pose[3], pose[4], pose[5]);
}

// 当前姿态下工具 X 轴的水平航向角（度）。调平姿态取 Rx 180 / Ry 0 / Rz 航向。
inline double headingDeg(const double pose[6])
{
    const Eigen::Vector3d toolX = rpyToMatrix(pose[3], pose[4], pose[5]).col(0);
    return std::atan2(toolX.y(), toolX.x()) / kDegToRad;
}

// 工具 Z 与竖直向下（基坐标 -Z）的夹角（度）
inline double tiltFromDownDeg(const double pose[6])
{
    const Eigen::Vector3d toolZ = rpyToMatrix(pose[3], pose[4], pose[5]).col(2);
    return std::acos(std::clamp(-toolZ.z(), -1.0, 1.0)) / kDegToRad;
}

} // namespace PoseMath
