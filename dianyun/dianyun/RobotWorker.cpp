#include "RobotWorker.h"
#include "ScanTypes.h"

#include <QDateTime>
#include <QElapsedTimer>
#include <QThread>
#include <QTimer>

#include <cmath>
#include <cstring>

#ifdef DIANYUN_ROBOT_SDK

// 使用法奥官方 fairino-cpp-sdk tag 3.9.3（SDK V2.3.3.0-robot v3.9.3），对应控制器
// web 版本 v3.9.3.1。只用 20003 XML-RPC 指令 + 20004 状态流 + 8080 命令，不依赖 CNDE。
#include "robot.h"
#include "robot_error.h"

#endif // DIANYUN_ROBOT_SDK

struct RobotWorker::Impl
{
#ifdef DIANYUN_ROBOT_SDK
    FRRobot robot;
    // SDK 内部线程从 20004 状态流持续刷新状态包，GetRobotRealTimeState 只是拷贝缓存，
    // 不做 IO，但会检查 socket 状态，可兼作链路探活。
    ROBOT_STATE_PKG state{};

    errno_t refreshState() { return robot.GetRobotRealTimeState(&state); }
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
    telemetry.sampleUs = hostClockUs();
    for (int i = 0; i < 6; ++i) {
        telemetry.tcpPose[i] = pkg.tl_cur_pos[i];
        telemetry.jointPos[i] = pkg.jt_cur_pos[i];
    }
    return telemetry;
}

constexpr int kFrameStaleLimitMs = 1000;

// RPC() 对不可达的 IP 也可能返回 ERR_SUCCESS，唯一可靠的握手信号是
// 20004 状态流真正开始推送（frame_head 为 5A5A 且帧序号在推进）。
constexpr uint16_t kStateFrameHead = 0x5A5A;
constexpr int kHandshakeTimeoutMs = 3000;

#endif // DIANYUN_ROBOT_SDK

constexpr int kFailureTolerance = 5;

// 运动指令的下发与到位检测周期。运动期间 poll() 仍保留（只刷新遥测），
// 到位判定交给独立的 pollMotion()，避免与 100 ms 遥测周期耦合。
constexpr int kMotionPollIntervalMs = 50;

// 现场还没有「百分比 → mm/s」的标定表，只能用行程/速度反推超时。
// 这里给一个宽松下限，真正的看护靠 motion_done 与 stopMotion。
constexpr int kDefaultMoveTimeoutMs = 60000;
// 下发指令后等待 robot_state 变成「运行」的宽限期：短行程的指令
// 可能在第一次检查前就已完成，不能因此判为失败。
constexpr int kMotionStartGraceMs = 500;

} // namespace

RobotWorker::RobotWorker(QObject* parent)
    : QObject(parent)
    , m_impl(std::make_unique<Impl>())
{
    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(m_pollIntervalMs);
    connect(m_pollTimer, &QTimer::timeout, this, &RobotWorker::poll);

    m_moveTimer = new QTimer(this);
    connect(m_moveTimer, &QTimer::timeout, this, &RobotWorker::pollMotion);
}

RobotWorker::~RobotWorker()
{
    stopMonitoring();
    if (m_moveTimer) m_moveTimer->stop();
#ifdef DIANYUN_ROBOT_SDK
    if (m_moving && m_linkEstablished) {
        m_impl->robot.StopMotion();
    }
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
    // 轮询每 100 ms 都会带同一份 detail（如“停止”）上报一次，
    // 若只比较状态会把稳态也当成变化，导致界面日志被刷屏。
    if (m_state.load() == next && m_lastStateDetail == detail) return;
    m_state.store(next);
    m_lastStateDetail = detail;
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
    // RPC 内部建立 20004 状态流与 8080 命令通道，并固定等待约 3 秒
    errno_t result = m_impl->robot.RPC(m_ip.toLocal8Bit().constData());
    if (result != ERR_SUCCESS) {
        m_lastError = QString::fromUtf8(
            "连接机械臂失败：%1（%2），请检查网线、控制器上电状态与 IP 配置")
            .arg(errorText(result), m_ip);
        setState(State::Fault, m_lastError);
        return;
    }

    // 每次调用都会顺带检查 socket 状态，链路断开时返回
    // ERR_SOCKET_COM_FAILED，因此轮询失败即可发现掉线。
    m_linkEstablished = true;

    // 等到 20004 状态流真正开始推送才算握手成功。
    // 这里每 50 ms 取一次帧序号，最多等 kHandshakeTimeoutMs。
    QElapsedTimer handshake;
    handshake.start();
    m_impl->refreshState();
    quint8 lastCnt = m_impl->state.frame_cnt;
    bool streamAlive = false;
    while (handshake.elapsed() < kHandshakeTimeoutMs) {
        result = m_impl->refreshState();
        if (result != ERR_SUCCESS) break;

        const quint8 cnt = m_impl->state.frame_cnt;
        if (m_impl->state.frame_head == kStateFrameHead && cnt != lastCnt) {
            streamAlive = true;
            break;
        }
        lastCnt = cnt;
        QThread::msleep(50);
    }

    if (!streamAlive) {
        m_impl->robot.CloseRPC();
        m_linkEstablished = false;
        m_lastError = result != ERR_SUCCESS
            ? QString::fromUtf8("连接机械臂失败：%1（%2），请检查网线、控制器上电状态与 IP 配置")
                .arg(errorText(result), m_ip)
            : QString::fromUtf8("连接机械臂失败：状态流无数据（%1），请确认 IP 是否正确").arg(m_ip);
        setState(State::Fault, m_lastError);
        return;
    }

    QElapsedTimer timer;
    timer.start();
    m_impl->refreshState();
    const int latency = static_cast<int>(timer.elapsed());

    m_lastFrameTick = QDateTime::currentMSecsSinceEpoch();
    m_lastFrameCnt = m_impl->state.frame_cnt;
    RobotTelemetry telemetry = toTelemetry(m_impl->state, latency);
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
    // 断开前先掐掉运动通道，否则定时器还会对着已关闭的 socket 发查询
    if (m_moving) {
#ifdef DIANYUN_ROBOT_SDK
        if (m_linkEstablished) m_impl->robot.StopMotion();
#endif
        finishMove(false, QString::fromUtf8("机械臂已断开，运动中止。"));
    }
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
        // 运动期间状态由指令通道主导，遥测可能滞后一两帧，
        // 此时不要把它改回 Online/Busy，避免界面闪烁
        if (m_moving) return;

        // robot_state: 1-停止 2-运行 3-暂停 4-拖动；program_state: 2-运行
        const bool moving = telemetry.robotState != 1
            || telemetry.programState == 2;
        // Online 不携带 detail，避免界面把“停止”拼进“已连接：…”的提示里
        setState(moving ? State::Busy : State::Online,
            moving ? robotStateName(telemetry.robotState) : QString());
    }
}

void RobotWorker::poll()
{
#ifdef DIANYUN_ROBOT_SDK
    if (!m_linkEstablished) return;

    // 读的是 SDK 状态线程缓存，本身不做 IO，但会检查 socket 状态：
    // 链路断开时返回 ERR_SOCKET_COM_FAILED，这是最廉价的探活手段。
    QElapsedTimer timer;
    timer.start();
    const errno_t result = m_impl->refreshState();
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

    // 状态流停更也要告警：20004 断流时 getter 仍能返回上一次的缓存值。
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (m_impl->state.frame_cnt != m_lastFrameCnt) {
        m_lastFrameCnt = m_impl->state.frame_cnt;
        m_lastFrameTick = now;
    }
    else if (now - m_lastFrameTick > kFrameStaleLimitMs) {
        m_lastError = QString::fromUtf8("机械臂状态流中断（20004 端口无数据）");
        stopMonitoring();
        setState(State::Fault, m_lastError);
        emit faultRaised(m_lastError);
        return;
    }

    const RobotTelemetry telemetry = toTelemetry(m_impl->state, latency);
    {
        QMutexLocker locker(&m_telemetryMutex);
        m_lastTelemetry = telemetry;
    }

    emit telemetryUpdated(telemetry);
    applyTelemetry(telemetry);
#endif
}

// ---------------------------------------------------------------------------
// L4 指令通道
//
// 设计要点：MoveL/MoveCart 一律用**非阻塞**参数（blendR/blendT = 0），
// 由 pollMotion 轮询 motion_done 判定到位。这样 worker 线程的事件循环
// 始终可用，stopMotion / cancel 才能在运动中生效；若用 blendR = -1
// （阻塞到位），一条 180 mm 的扫描会把事件循环占满十几秒。
// ---------------------------------------------------------------------------

namespace {

// 法奥的 rx/ry/rz 是绕**固定轴** X→Y→Z 依次旋转，故 R = Rz·Ry·Rx。
// 只在这里用一次，不值得引入完整矩阵库。
void rpyToMatrix(double rx, double ry, double rz, double m[3][3])
{
    const double d2r = 3.14159265358979323846 / 180.0;
    const double cx = std::cos(rx * d2r), sx = std::sin(rx * d2r);
    const double cy = std::cos(ry * d2r), sy = std::sin(ry * d2r);
    const double cz = std::cos(rz * d2r), sz = std::sin(rz * d2r);

    m[0][0] = cz * cy;
    m[0][1] = cz * sy * sx - sz * cx;
    m[0][2] = cz * sy * cx + sz * sx;
    m[1][0] = sz * cy;
    m[1][1] = sz * sy * sx + cz * cx;
    m[1][2] = sz * sy * cx - cz * sx;
    m[2][0] = -sy;
    m[2][1] = cy * sx;
    m[2][2] = cy * cx;
}

} // namespace

bool RobotWorker::resolveTarget(const RobotMoveTarget& target, double pose[6],
    double joints[6], QString* error)
{
#ifdef DIANYUN_ROBOT_SDK
    DescPose desc{};
    for (int i = 0; i < 6; ++i) {
        pose[i] = target.pose[i];
    }

    if (target.relative) {
        // 增量运动基于当前 TCP 自行求目标：SDK 的 offset_flag 偏移只作用于给定目标点，
        // 读出当前位姿再相加更直观。
        DescPose current{};
        const errno_t result = m_impl->robot.GetActualTCPPose(0, &current);
        if (result != ERR_SUCCESS) {
            *error = QString::fromUtf8("读取当前 TCP 位姿失败：%1")
                .arg(errorText(result));
            return false;
        }

        if (target.relativeInTool) {
            // 工具坐标系增量：平移量要先按当前姿态旋转到基坐标系，
            // 否则「沿工具 Y 走 180 mm」会变成世界 Y 方向的斜走。
            double rot[3][3] = {};
            rpyToMatrix(current.rpy.rx, current.rpy.ry, current.rpy.rz, rot);
            double rx = 0.0, ry = 0.0, rz = 0.0;
            for (int i = 0; i < 3; ++i) {
                rx += rot[0][i] * target.pose[i];
                ry += rot[1][i] * target.pose[i];
                rz += rot[2][i] * target.pose[i];
            }
            pose[0] = current.tran.x + rx;
            pose[1] = current.tran.y + ry;
            pose[2] = current.tran.z + rz;
        }
        else {
            pose[0] += current.tran.x;
            pose[1] += current.tran.y;
            pose[2] += current.tran.z;
        }

        pose[3] += current.rpy.rx;
        pose[4] += current.rpy.ry;
        pose[5] += current.rpy.rz;
    }

    desc.tran.x = pose[0];
    desc.tran.y = pose[1];
    desc.tran.z = pose[2];
    desc.rpy.rx = pose[3];
    desc.rpy.ry = pose[4];
    desc.rpy.rz = pose[5];

    // MoveL 要求同时给出关节角，用逆解并按当前关节配置取解，
    // 避免同一末端位姿解出另一组关节从而中途大幅摆臂。
    JointPos currentJoints{};
    const errno_t jointResult =
        m_impl->robot.GetActualJointPosDegree(0, &currentJoints);
    if (jointResult != ERR_SUCCESS) {
        *error = QString::fromUtf8("读取当前关节角失败：%1")
            .arg(errorText(jointResult));
        return false;
    }

    // SDK 内部不捕获 XML-RPC 解析异常，控制器返回非预期格式时会直接抛出并闪退，
    // 因此逆解调用必须自行兜住，并在参考解失败时退回按当前配置求解。
    JointPos solved{};
    errno_t ikResult = ERR_SUCCESS;
    QString ikNote;
    try {
        // type 0：desc 是基坐标系下的绝对位姿
        ikResult = m_impl->robot.GetInverseKinRef(0, &desc, &currentJoints, &solved);
    }
    catch (...) {
        ikResult = -1;
        ikNote = QString::fromUtf8("GetInverseKinRef 抛出异常");
    }
    if (ikResult != ERR_SUCCESS) {
        const QString firstFailure = ikNote.isEmpty()
            ? errorText(ikResult) : ikNote;
        try {
            ikResult = m_impl->robot.GetInverseKin(0, &desc, -1, &solved);
        }
        catch (...) {
            ikResult = -1;
        }
        if (ikResult != ERR_SUCCESS) {
            *error = QString::fromUtf8("目标位姿无逆解：%1；GetInverseKin 同样失败：%2")
                .arg(firstFailure, errorText(ikResult));
            return false;
        }
    }
    for (int i = 0; i < 6; ++i) {
        joints[i] = solved.jPos[i];
    }
    return true;
#else
    Q_UNUSED(target);
    Q_UNUSED(pose);
    Q_UNUSED(joints);
    *error = QString::fromUtf8("当前构建未启用机械臂 SDK（仅 Release 配置支持）。");
    return false;
#endif
}

void RobotWorker::moveTo(RobotMoveTarget target)
{
    if (m_moving) {
        emit moveFinished(false, QString::fromUtf8("上一条运动指令尚未结束。"));
        return;
    }
    if (!isOnline()) {
        emit moveFinished(false, QString::fromUtf8("机械臂未连接，无法下发运动指令。"));
        return;
    }

#ifdef DIANYUN_ROBOT_SDK
    double pose[6] = { 0, 0, 0, 0, 0, 0 };
    double joints[6] = { 0, 0, 0, 0, 0, 0 };
    QString error;
    if (!resolveTarget(target, pose, joints, &error)) {
        emit moveFinished(false, error);
        return;
    }

    DescPose desc{};
    desc.tran.x = pose[0];
    desc.tran.y = pose[1];
    desc.tran.z = pose[2];
    desc.rpy.rx = pose[3];
    desc.rpy.ry = pose[4];
    desc.rpy.rz = pose[5];

    JointPos jointPos{};
    for (int i = 0; i < 6; ++i) jointPos.jPos[i] = joints[i];

    ExaxisPos epos{};
    DescPose offset{};

    errno_t result = ERR_SUCCESS;
    try {
        if (target.linear) {
            // blendR = 0：不阻塞、不做平滑，交给 pollMotion 判定到位；blendMode 0 为内切过渡
            result = m_impl->robot.MoveL(&jointPos, &desc,
                target.toolId, target.userId,
                target.velPercent, target.accPercent, target.ovl,
                0.0f, 0, &epos, 0, 0, &offset);
        }
        else {
            result = m_impl->robot.MoveCart(&desc,
                target.toolId, target.userId,
                target.velPercent, target.accPercent, target.ovl,
                0.0f, -1);
        }
    }
    catch (...) {
        // 附带控制器侧状态，便于判断是手动模式、未使能还是固件/SDK 版本不匹配
        char sdkVersion[64] = {};
        m_impl->robot.GetSDKVersion(sdkVersion);
        m_impl->refreshState();
        const ROBOT_STATE_PKG& pkg = m_impl->state;
        emit moveFinished(false, QString::fromUtf8(
            "下发运动指令时 SDK 抛出异常（控制器响应格式异常）。"
            "robot_mode=%1（0自动/1手动） robot_state=%2 main_code=%3 sub_code=%4 "
            "tool=%5 user=%6 SDK=%7")
            .arg(pkg.robot_mode)
            .arg(pkg.robot_state)
            .arg(pkg.main_code)
            .arg(pkg.sub_code)
            .arg(target.toolId)
            .arg(target.userId)
            .arg(QString::fromLatin1(sdkVersion)));
        return;
    }

    if (result != ERR_SUCCESS) {
        emit moveFinished(false, QString::fromUtf8("下发运动指令失败：%1")
            .arg(errorText(result)));
        return;
    }

    m_currentTarget = target;
    m_moveTimeoutMs = target.timeoutMs > 0 ? target.timeoutMs : kDefaultMoveTimeoutMs;
    m_moveIssued = true;
    m_moveStarted = false;
    m_stopRequested = false;
    m_moving = true;
    m_moveClock.start();
    m_moveTimer->start(kMotionPollIntervalMs);

    emit moveStarted(target.description);
    setState(State::Busy, target.description);
#else
    emit moveFinished(false,
        QString::fromUtf8("当前构建未启用机械臂 SDK（仅 Release 配置支持）。"));
#endif
}

void RobotWorker::stopMotion()
{
    if (!m_moving) return;

    m_stopRequested = true;
#ifdef DIANYUN_ROBOT_SDK
    m_impl->robot.StopMotion();
#endif
    finishMove(false, QString::fromUtf8("运动已被用户终止。"));
}

void RobotWorker::pollMotion()
{
#ifdef DIANYUN_ROBOT_SDK
    if (!m_moving) {
        m_moveTimer->stop();
        return;
    }

    // 急停 / 故障优先：任何人都不该在故障态继续追运动到位
    if (state() == State::Fault) {
        finishMove(false, m_lastError.isEmpty()
            ? QString::fromUtf8("机械臂进入故障状态，运动中止。")
            : m_lastError);
        return;
    }

    if (m_moveClock.elapsed() > m_moveTimeoutMs) {
        m_impl->robot.StopMotion();
        finishMove(false, QString::fromUtf8("运动超时（%1 ms），已发送停止。")
            .arg(m_moveTimeoutMs));
        return;
    }

    // 状态流里的 robot_state == 2 表示确实在动，用来区分
    // 「已下达但还没启动」与「已经跑完」。
    m_impl->refreshState();
    if (m_impl->state.robot_state != 1) {
        m_moveStarted = true;
    }
    // 运动期间按运动轮询周期刷新遥测缓存：扫描拼接要靠带时间戳的
    // TCP 轨迹给每行定 Y，100 ms 的遥测周期太稀。
    {
        QMutexLocker locker(&m_telemetryMutex);
        m_lastTelemetry = toTelemetry(m_impl->state, m_lastTelemetry.rpcLatencyMs);
    }

    uint8_t done = 0;
    const errno_t result = m_impl->robot.GetRobotMotionDone(&done);
    if (result != ERR_SUCCESS) {
        // motion_done 查询失败连续出现才判失败，单次抖动忽略
        if (++m_failureStreak >= kFailureTolerance) {
            m_failureStreak = 0;
            m_impl->robot.StopMotion();
            finishMove(false, QString::fromUtf8("查询运动状态失败：%1")
                .arg(errorText(result)));
        }
        return;
    }
    m_failureStreak = 0;

    if (done != 1) return;

    // motion_done 在指令刚入队时也可能读到 1，必须等宽限期过了，
    // 或曾经观测到机器人真的动过，才认到位。
    if (!m_moveStarted && m_moveClock.elapsed() < kMotionStartGraceMs) return;

    finishMove(true);
#endif
}

void RobotWorker::finishMove(bool ok, const QString& error)
{
    if (!m_moving) return;

    m_moving = false;
    m_moveIssued = false;
    m_moveStarted = false;
    if (m_moveTimer) m_moveTimer->stop();

    const bool stopped = m_stopRequested;
    m_stopRequested = false;

    // 故障态不能被「运动结束」抹回 Online：机械臂此刻仍然处于故障，
    // 状态机必须保持 Fault 直到 resetFault()。
    if (state() != State::Fault) {
        setState(State::Online);
    }
    emit moveFinished(ok, stopped && error.isEmpty()
        ? QString::fromUtf8("运动已被用户终止。")
        : error);
}
