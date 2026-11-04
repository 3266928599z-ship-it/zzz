#pragma once

#include "RobotTypes.h"

#include <QObject>
#include <QMutex>
#include <QMetaType>
#include <QElapsedTimer>
#include <QString>
#include <atomic>
#include <memory>

class QTimer;

// 一条运动指令。
//
// 速度是**百分比**而不是 mm/s：当前以 velAccParamMode=0 调用 MoveL/MoveCart，只接受 [0~100]，
// 且没有把百分比换算成 mm/s 的公开接口。扫描所需的实际线速度由
// 「步距 × 行率」决定（本轮标定为 96 µm × 491 行/s ≈ 47 mm/s），
// 现场需先用一条空行程标出该线速度对应的百分比，再填到 velPercent。
struct RobotMoveTarget {
    double pose[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // x,y,z (mm) / rx,ry,rz (deg)
    // true 时 pose 是相对当前 TCP 的增量；false 时是绝对目标
    bool relative = false;
    // 增量所在坐标系。扫描必须用工具坐标系：传感器随手爪转，只有工具 Y
    // 才是「轮廓宽度方向的法向」，基坐标系下的 Y 与工件走向无关。
    bool relativeInTool = false;
    // true 走 MoveL（直线插补），false 走 MoveCart（关节空间解算的笛卡尔运动）
    bool linear = true;
    int toolId = 1;        // 工具坐标号，1 = 现场标定的 hik
    int userId = 0;        // 工件坐标号，0 = 基坐标
    float velPercent = 10.0f;
    float accPercent = 20.0f;
    float ovl = 100.0f;
    int timeoutMs = 0;     // <=0 用默认值
    QString description;   // 用于日志
};

Q_DECLARE_METATYPE(RobotMoveTarget)

// 机械臂链路生命周期分四层：
//   L1 SDK    FRRobot 对象，构造创建、析构销毁（仅在 RobotWorker.cpp 中可见）
//   L2 设备   connectRobot / disconnectRobot（RPC / CloseRPC）
//   L3 数据流 startMonitoring / stopMonitoring（状态轮询定时器）
//   L4 指令窗口 moveTo / stopMotion（非阻塞下发 + motion_done 轮询）
class RobotWorker : public QObject
{
    Q_OBJECT
public:
    enum class State { Disconnected, Connecting, Online, Busy, Fault };
    Q_ENUM(State)

    explicit RobotWorker(QObject* parent = nullptr);
    ~RobotWorker() override;

    State state() const { return m_state.load(); }
    bool isOnline() const { return state() == State::Online
        || state() == State::Busy; }
    QString lastError() const;
    QString controllerIp() const;
    RobotTelemetry currentTelemetry() const;

    // 控制器 IP，连接前设置；空表示用 fairino 默认地址（192.168.58.2）
    QString m_ip = QStringLiteral("192.168.58.2");
    // 状态轮询周期
    int m_pollIntervalMs = 100;

public slots:
    void connectRobot();
    void disconnectRobot();
    void startMonitoring();
    void stopMonitoring();
    void resetFault();
    void queryTelemetryOnce();

    // 下发一条运动指令。立即返回，进度由 moveFinished 通知。
    // 用非阻塞的 MoveL/MoveCart + 轮询 motion_done 实现，
    // 这样 worker 线程的事件循环不被占住，stopMotion 才能及时生效。
    void moveTo(RobotMoveTarget target);
    // 立即终止运动，用于扫描中途取消或异常
    void stopMotion();

signals:
    void linkStateChanged(RobotWorker::State state, QString detail);
    void telemetryUpdated(RobotTelemetry telemetry);
    void faultRaised(QString message);

    void moveStarted(QString description);
    // ok=false 时 error 给出可读原因
    void moveFinished(bool ok, QString error);

private slots:
    void poll();
    // 运动期间由独立定时器驱动，检查 motion_done
    void pollMotion();

private:
    struct Impl;

    void setState(State next, const QString& detail = QString());
    // 把一帧遥测折算成链路状态
    void applyTelemetry(const RobotTelemetry& telemetry);
    // 把目标解算成 SDK 需要的绝对位姿 + 关节角；失败时返回 false 并填 error
    bool resolveTarget(const RobotMoveTarget& target, double pose[6],
        double joints[6], QString* error);
    void finishMove(bool ok, const QString& error = QString());
    static QString robotStateName(int robotState);
    static QString errorText(int code);

    std::unique_ptr<Impl> m_impl;
    std::atomic<State> m_state{ State::Disconnected };
    QString m_lastStateDetail;       // 上一次上报的 detail，用于抑制重复信号
    QTimer* m_pollTimer = nullptr;
    bool m_monitoring = false;
    bool m_linkEstablished = false;  // RPC 链路是否仍建立，与状态机解耦
    bool m_moving = false;           // 指令通道占用中
    bool m_stopRequested = false;    // 运动期间收到停止请求
    bool m_moveIssued = false;       // 已成功下发指令
    bool m_moveStarted = false;      // 已观测到机器人进入运行态；短行程可能观察不到
    QElapsedTimer m_moveClock;       // 运动计时，用于超时
    int m_moveTimeoutMs = 0;
    RobotMoveTarget m_currentTarget;
    QTimer* m_moveTimer = nullptr;
    int m_failureStreak = 0;         // 连续轮询失败计数，用于抖动容错
    qint64 m_lastFrameTick = 0;      // 状态帧序号最近一次变化的时刻，用于检测状态流停更
    int m_lastFrameCnt = -1;         // 上一次看到的 20004 状态帧序号
    mutable QMutex m_telemetryMutex;
    RobotTelemetry m_lastTelemetry;
    QString m_lastError;
};
