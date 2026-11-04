#pragma once

#include <Eigen/Core>
#include <QString>

// 点云 → 机械臂基坐标的标定参数，存放在 exe 同目录的 calibration.json。
//
//   p_base = T_base_tcp(t) · T_tcp_sensor · M_axis · (x, 0, z)_SDK
//
// T_base_tcp(t)  扫描时刻 t 的 TCP 位姿（工具 1 激活时 GetActualTCPPose 的读数）
// T_tcp_sensor   HiCalibrator EIH-N 轮廓线法手眼结果（固定轴 XYZ，mm / 度）
// M_axis         SDK 输出轴到手眼标定传感器坐标系的轴置换（列 = SDK X/Y/Z 的方向）。
//                外参旋转接近单位阵，而物理上 SDK X 沿工具 Y、行进沿工具 X、SDK Z 朝相机，
//                故默认取 X↔Y 互换、Z 取反。X/Y 符号需用「点选点云 → 移到该点上方」核对，
//                左右镜像时改为 [[0,-1,0],[-1,0,0],[0,0,-1]]。
// timeOffsetUs   相机行时刻相对机器人遥测时刻的补偿，正值表示行实际更晚采集。
//                正反两向各扫一次，调到两次点云重合即可。
struct HandEyeConfig {
    double tcpToSensor[6] = { 43.528, 95.003, 290.166, -3.122, -0.027, 1.584 };
    Eigen::Matrix3d sensorAxis = (Eigen::Matrix3d() <<
        0, 1, 0,
        1, 0, 0,
        0, 0, -1).finished();
    double timeOffsetUs = 0.0;

    // T_tcp_sensor · M_axis，直接作用于 SDK 的 (x, 0, z)
    Eigen::Matrix4d tcpToSdk() const;

    static QString defaultPath();
    // 文件不存在时用默认值并写出一份模板，便于现场修改。
    // 返回 false 表示文件存在但内容无效，此时 config 保持默认值，message 给出原因。
    static bool load(const QString& path, HandEyeConfig& config, QString& message);
    bool save(const QString& path, QString& message) const;
};
