#pragma once

#include "RobotTypes.h"

#include <QObject>
#include <QMutex>
#include <QString>
#include <atomic>
#include <memory>

class QTimer;

// 机械臂链路生命周期分四层：
//   L1 SDK    FRRobot 对象，构造创建、析构销毁（仅在 RobotWorker.cpp 中可见）
//   L2 设备   connectRobot / disconnectRobot（RPC / CloseRPC）
//   L3 数据流 startMonitoring / stopMonitoring（状态轮询定时器）
//   L4 指令窗口 当前阶段不下发任何运动指令，指令通道留到后续阶段
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

signals:
    void linkStateChanged(RobotWorker::State state, QString detail);
    void telemetryUpdated(RobotTelemetry telemetry);
    void faultRaised(QString message);

private slots:
    void poll();

private:
    struct Impl;

    void setState(State next, const QString& detail = QString());
    // 把一帧遥测折算成链路状态
    void applyTelemetry(const RobotTelemetry& telemetry);
    static QString robotStateName(int robotState);
    static QString errorText(int code);

    std::unique_ptr<Impl> m_impl;
    std::atomic<State> m_state{ State::Disconnected };
    QTimer* m_pollTimer = nullptr;
    bool m_monitoring = false;
    bool m_linkEstablished = false;  // RPC 链路是否仍建立，与状态机解耦
    int m_failureStreak = 0;         // 连续轮询失败计数，用于抖动容错
    mutable QMutex m_telemetryMutex;
    RobotTelemetry m_lastTelemetry;
    QString m_lastError;
};
