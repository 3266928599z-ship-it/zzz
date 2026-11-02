#include "RobotWorker.h"

#include <QElapsedTimer>
#include <QTimer>

#ifdef DIANYUN_ROBOT_SDK

// robot.h 内部会 include <winsock2.h>，必须先于 windows.h 引入，
// 否则 ws2def.h 的 sockaddr / fd_set / timeval 会重定义。
#include <winsock2.h>
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "robot.h"
#include "robot_error.h"

#endif // DIANYUN_ROBOT_SDK

struct RobotWorker::Impl
{
#ifdef DIANYUN_ROBOT_SDK
    FRRobot robot;
#endif
};

namespace {

#ifdef DIANYUN_ROBOT_SDK
RobotTelemetry toTelemetry(const ROBOT_STATE_PKG& pkg, int latencyMs)
{
    RobotTelemetry telemetry;
    telemetry.connected = true;
    telemetry.robotState = pkg.robot_state;
    telemetry.programState = pkg.program_state;
    telemetry.robotMode = pkg.robot_mode;
    telemetry.motionDone = pkg.motion_done;
    telemetry.emergencyStop = pkg.EmergencyStop;
    telemetry.mainCode = pkg.main_code;
    telemetry.subCode = pkg.sub_code;
    telemetry.rpcLatencyMs = latencyMs;
    for (int i = 0; i < 6; ++i) {
        telemetry.tcpPose[i] = pkg.tl_cur_pos[i];
        telemetry.jointPos[i] = pkg.jt_cur_pos[i];
    }
    return telemetry;
}
#endif

constexpr int kFailureTolerance = 5;

} // namespace

RobotWorker::RobotWorker(QObject* parent)
    : QObject(parent)
    , m_impl(std::make_unique<Impl>())
{
    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(m_pollIntervalMs);
    connect(m_pollTimer, &QTimer::timeout, this, &RobotWorker::poll);
}

RobotWorker::~RobotWorker()
{
    stopMonitoring();
#ifdef DIANYUN_ROBOT_SDK
    if (m_linkEstablished) {
        m_impl->robot.CloseRPC();
    }
#endif
}

QString RobotWorker::lastError() const
{
    return m_lastError;
}

QString RobotWorker::controllerIp() const
{
    return m_ip;
}

void RobotWorker::setState(State next, const QString& detail)
{
    if (m_state.load() == next && detail.isEmpty()) return;
    m_state.store(next);
    emit linkStateChanged(next, detail);
}

QString RobotWorker::robotStateName(int robotState)
{
    switch (robotState) {
    case 1: return QString::fromUtf8("停止");
    case 2: return QString::fromUtf8("运行");
    case 3: return QString::fromUtf8("暂停");
    case 4: return QString::fromUtf8("拖动");
    default: return QString::fromUtf8("未知");
    }
}

QString RobotWorker::errorText(int code)
{
#ifdef DIANYUN_ROBOT_SDK
    switch (code) {
    case ERR_SUCCESS:               return QString::fromUtf8("无故障");
    case ERR_OTHER:                 return QString::fromUtf8("未知错误");
    case ERR_SOCKET_COM_FAILED:     return QString::fromUtf8("网络通讯异常");
    case ERR_SOCKET_SEND_FAILED:    return QString::fromUtf8("网络发送失败");
    case ERR_SOCKET_RECV_FAILED:    return QString::fromUtf8("网络接收失败");
    case ERR_XMLRPC_COM_FAILED:     return QString::fromUtf8("XML-RPC 通讯失败");
    case ERR_XMLRPC_CMD_FAILED:     return QString::fromUtf8("指令执行失败");
    case ERR_PARAM_VALUE:           return QString::fromUtf8("参数超出范围");
    case ERR_SAFETY_STOP:           return QString::fromUtf8("安全停止已触发");
    case ERR_EXECUTION_FAILED:      return QString::fromUtf8("指令执行失败");
    case ERR_WAIT_TIMEOUT:          return QString::fromUtf8("等待超时");
    case ERR_NON_RESSETTABLE_FAULT:
        return QString::fromUtf8("不可复位故障，需断电重启控制箱");
    default:
        break;
    }
#endif
    if (code == 0) return QString::fromUtf8("无故障");
    return QString::fromUtf8("故障码 %1").arg(code);
}

void RobotWorker::connectRobot()
{
    const State current = state();
    if (current == State::Connecting || isOnline()) return;

    setState(State::Connecting, m_ip);
    m_lastError.clear();
    m_failureStreak = 0;

#ifdef DIANYUN_ROBOT_SDK
    errno_t result = m_impl->robot.RPC(m_ip.toLocal8Bit().constData());
    if (result != ERR_SUCCESS) {
        m_lastError = QString::fromUtf8(
            "连接机械臂失败：%1（%2），请检查网线、控制器上电状态与 IP 配置")
            .arg(errorText(result), m_ip);
        setState(State::Fault, m_lastError);
        return;
    }

    // 网络抖动时由 SDK 自行重连，掉线后不必手动复位
    m_impl->robot.SetReConnectParam(true, 30000, 50);
    m_linkEstablished = true;

    // 先取一帧确认链路真的通，再宣布 Online
    ROBOT_STATE_PKG pkg{};
    QElapsedTimer timer;
    timer.start();
    result = m_impl->robot.GetRobotRealTimeState(&pkg);
    const int latency = static_cast<int>(timer.elapsed());

    if (result != ERR_SUCCESS) {
        m_impl->robot.CloseRPC();
        m_linkEstablished = false;
        m_lastError = QString::fromUtf8("读取机器人状态失败：%1")
            .arg(errorText(result));
        setState(State::Fault, m_lastError);
        return;
    }

    RobotTelemetry telemetry = toTelemetry(pkg, latency);
    {
        QMutexLocker locker(&m_telemetryMutex);
        m_lastTelemetry = telemetry;
    }

    // 链路已通，先落到 Online，再由首帧遥测决定是否为 Busy / Fault
    setState(State::Online);
    emit telemetryUpdated(telemetry);
    applyTelemetry(telemetry);
    startMonitoring();
#else
    m_lastError = QString::fromUtf8(
        "当前构建未启用机械臂 SDK（仅 Release 配置支持）。");
    setState(State::Fault, m_lastError);
#endif
}

void RobotWorker::disconnectRobot()
{
    stopMonitoring();

#ifdef DIANYUN_ROBOT_SDK
    if (m_linkEstablished) {
        m_impl->robot.CloseRPC();
    }
#endif

    m_linkEstablished = false;
    m_failureStreak = 0;
    {
        QMutexLocker locker(&m_telemetryMutex);
        m_lastTelemetry = RobotTelemetry();
    }
    setState(State::Disconnected);
}

void RobotWorker::startMonitoring()
{
    // 只要 RPC 链路还在，就持续轮询，便于自动发现故障解除
    if (!m_linkEstablished || m_monitoring) return;
    m_monitoring = true;
    m_pollTimer->start(m_pollIntervalMs);
}

void RobotWorker::stopMonitoring()
{
    m_monitoring = false;
    if (m_pollTimer) m_pollTimer->stop();
}

void RobotWorker::resetFault()
{
#ifdef DIANYUN_ROBOT_SDK
    if (!isOnline() && state() != State::Fault) return;

    const errno_t result = m_impl->robot.ResetAllError();
    if (result != ERR_SUCCESS) {
        m_lastError = QString::fromUtf8("清除报警失败：%1")
            .arg(errorText(result));
        emit faultRaised(m_lastError);
        return;
    }
    m_lastError.clear();
    m_failureStreak = 0;

    // 清完立刻回读一次，把状态机拉回真实值
    queryTelemetryOnce();
#else
    m_lastError = QString::fromUtf8(
        "当前构建未启用机械臂 SDK，无法清除报警。");
    emit faultRaised(m_lastError);
#endif
}

void RobotWorker::queryTelemetryOnce()
{
    poll();
}

RobotTelemetry RobotWorker::currentTelemetry() const
{
    QMutexLocker locker(&m_telemetryMutex);
    return m_lastTelemetry;
}

void RobotWorker::applyTelemetry(const RobotTelemetry& telemetry)
{
    if (!telemetry.connected) return;

    const State before = state();
    if (telemetry.mainCode != 0 || telemetry.emergencyStop != 0) {
        const QString detail = telemetry.emergencyStop != 0
            ? QString::fromUtf8("急停已按下")
            : errorText(telemetry.mainCode);
        m_lastError = detail;
        setState(State::Fault, detail);
        if (before != State::Fault) emit faultRaised(detail);
    }
    else {
        // robot_state: 1-停止 2-运行 3-暂停 4-拖动；program_state: 2-运行
        const bool moving = telemetry.robotState != 1
            || telemetry.programState == 2;
        setState(moving ? State::Busy : State::Online,
            robotStateName(telemetry.robotState));
    }
}

void RobotWorker::poll()
{
#ifdef DIANYUN_ROBOT_SDK
    if (!m_linkEstablished) return;

    ROBOT_STATE_PKG pkg{};
    QElapsedTimer timer;
    timer.start();
    const errno_t result = m_impl->robot.GetRobotRealTimeState(&pkg);
    const int latency = static_cast<int>(timer.elapsed());

    if (result != ERR_SUCCESS) {
        // 单次读取失败可能只是网络抖动，连续失败才判定链路中断
        if (++m_failureStreak < kFailureTolerance) return;
        m_failureStreak = 0;

        m_lastError = QString::fromUtf8("机械臂状态查询失败：%1")
            .arg(errorText(result));
        stopMonitoring();
        setState(State::Fault, m_lastError);
        emit faultRaised(m_lastError);
        return;
    }
    m_failureStreak = 0;

    const RobotTelemetry telemetry = toTelemetry(pkg, latency);
    {
        QMutexLocker locker(&m_telemetryMutex);
        m_lastTelemetry = telemetry;
    }

    emit telemetryUpdated(telemetry);
    applyTelemetry(telemetry);
#endif
}
