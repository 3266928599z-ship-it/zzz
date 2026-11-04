#pragma once

#include <QMetaType>
#include <QString>

// 机械臂遥测快照。
// 刻意不引用 fairino SDK 的任何类型：robot_types.h 会 typedef int errno_t 与
// uint8_t，和 CRT 冲突，只能在 RobotWorker.cpp 里出现。
struct RobotTelemetry {
    bool connected = false;
    int robotState = 0;        // 1-停止；2-运行；3-暂停；4-拖动
    int programState = 0;      // 1-停止；2-运行；3-暂停
    int robotMode = -1;        // 0-自动；1-手动
    int motionDone = 0;        // 1-到位
    int emergencyStop = 0;     // 1-急停按下
    int mainCode = 0;          // 主故障码
    int subCode = 0;           // 子故障码
    double tcpPose[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double jointPos[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    int rpcLatencyMs = -1;     // 单次状态查询耗时，-1 表示未知
    qint64 sampleUs = 0;       // 采样时刻（hostClockUs），扫描时用来对齐相机行
};

Q_DECLARE_METATYPE(RobotTelemetry)
