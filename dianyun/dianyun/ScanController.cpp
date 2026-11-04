#include "ScanController.h"

#include "HikCameraWorker.h"

#include <QTimer>

#include <algorithm>
#include <cmath>

namespace {
// 起流后等第一帧到达的时间。设备从 StartMeasure 到首帧通常 < 300 ms。
constexpr int kStreamStartGraceMs = 2000;
// 扫描结束后再收一点尾巴，避免正好卡在批次边界丢掉末行
constexpr int kStreamTailMs = 300;
// 定位运动的超时：位姿可能横跨上米,百分比速度未知,给足余量
constexpr int kDefaultPositionTimeoutMs = 60000;
// 机器人位移超过这个量才认为扫描真正开始。太小会被关节间隙抖动触发，
// 太大则会切掉行程开头；0.5 mm 相当于 5 行左右。
constexpr double kRecordingStartDisplacementMm = 0.5;
// 轨迹采样周期：比遥测刷新（运动中 50 ms）更快，保证每次刷新都被收进来
constexpr int kTrackSampleIntervalMs = 10;
}

ScanController::ScanController(HikCameraWorker* camera, RobotWorker* robot,
    QObject* parent)
    : QObject(parent)
    , m_camera(camera)
    , m_robot(robot)
{
    m_watchdog = new QTimer(this);
    m_watchdog->setSingleShot(true);
    connect(m_watchdog, &QTimer::timeout, this, &ScanController::onWatchdog);

    m_trackTimer = new QTimer(this);
    m_trackTimer->setInterval(kTrackSampleIntervalMs);
    m_trackTimer->setTimerType(Qt::PreciseTimer);
    connect(m_trackTimer, &QTimer::timeout, this, &ScanController::sampleTrack);

    if (m_camera) {
        connect(m_camera, &HikCameraWorker::scanFrameReady,
            this, &ScanController::onScanFrame);
        connect(m_camera, &HikCameraWorker::linkStateChanged,
            this, &ScanController::onCameraStateChanged);
        connect(m_camera, &HikCameraWorker::captureFinished,
            this, &ScanController::onCaptureFinished);
        connect(m_camera, &HikCameraWorker::captureFailed,
            this, [this](const QString& message) {
                if (isRunning()) fail(message);
            });
    }
    if (m_robot) {
        connect(m_robot, &RobotWorker::moveFinished,
            this, &ScanController::onMoveFinished);
    }
}

bool ScanController::isRunning() const
{
    switch (m_state) {
    case State::Positioning:
    case State::Starting:
    case State::Scanning:
    case State::Stopping:
        return true;
    default:
        return false;
    }
}

void ScanController::setState(State next)
{
    m_state = next;
}

int ScanController::scanTimeoutMs() const
{
    const double mmPerSec =
        static_cast<double>(m_request.stepUm)
        * static_cast<double>(m_request.lineRate) / 1000.0;
    // 速度未知时（参数没自洽）给一个不依赖速度的上限
    if (!(mmPerSec > 1.0)) return 120000;

    const double seconds = std::abs(m_request.scanLengthMm) / mmPerSec;
    return static_cast<int>(seconds * 1000.0) + 30000;
}

void ScanController::applyCameraParams()
{
    // 相机自身的看门狗在扫描模式下必须显式给出：frameCount=0 时它只有
    // 3 秒兜底，而一条 180 mm 行程要 4 秒左右，会被中途掐断。
    const int streamTimeout = scanTimeoutMs();

    // m_params 由 worker 线程在 applyParams() 里读取，
    // 必须切到 worker 线程写入，否则是跨线程数据竞争。
    HikCameraWorker* cam = m_camera;
    const ScanRequest request = m_request;
    QMetaObject::invokeMethod(cam, [cam, request, streamTimeout]() {
        cam->m_params.frameCount = 0;      // 持续采集
        cam->m_params.lineRate = request.lineRate;
        cam->m_params.stepDistanceUm = request.stepUm;
        cam->m_params.exposureTime = request.exposureUs;
        cam->m_params.gain = request.gainDb;
        cam->m_params.photosensitivity = request.photosensitivity;
        cam->m_params.keepStreaming = false;          // 扫描结束立即熄激光
        cam->m_params.captureTimeoutMs = streamTimeout;
    }, Qt::BlockingQueuedConnection);
}

void ScanController::start(ScanRequest request)
{
    if (isRunning()) {
        emit progress(QString::fromUtf8("上一次扫描仍在进行中。"));
        return;
    }
    if (m_returning) {
        emit progress(QString::fromUtf8("机械臂正在退回扫描起点，请稍候再扫。"));
        return;
    }
    if (!m_camera || !m_robot) {
        fail(QString::fromUtf8("相机或机械臂未初始化，无法开始扫描。"));
        return;
    }
    if (!m_camera->isConnected()) {
        fail(QString::fromUtf8("相机未连接，无法开始扫描。"));
        return;
    }
    if (!m_robot->isOnline()) {
        fail(QString::fromUtf8("机械臂未连接，无法开始扫描。"));
        return;
    }
    if (m_camera->isCapturing()) {
        fail(QString::fromUtf8(
            "相机正在采集（可能是其他任务的单帧拍摄），请先停止。"));
        return;
    }
    if (!(request.stepUm > 0.0f) || !(request.lineRate > 0.0f)) {
        fail(QString::fromUtf8("步进间距与轮廓行率必须大于 0。"));
        return;
    }
    if (std::abs(request.scanLengthMm) < 1.0) {
        fail(QString::fromUtf8("扫描行程过短（需大于 1 mm）。"));
        return;
    }

    // 三个速度量必须自洽，否则点云会被纵向拉伸。这里只告警不拦截：
    // 行率受设备上限（实测约 1120 行/s）约束，现场有时确实跑不满。
    const double expectedMmS =
        static_cast<double>(request.stepUm) * request.lineRate / 1000.0;
    if (expectedMmS > 0.0) {
        emit progress(QString::fromUtf8(
            "参数自洽提示：步距 %1 µm × 行率 %2 Hz = %3 mm/s，"
            "机器人速度需匹配该值。")
            .arg(request.stepUm, 0, 'f', 0)
            .arg(request.lineRate, 0, 'f', 0)
            .arg(expectedMmS, 0, 'f', 1));
    }

    m_request = request;
    m_lastError.clear();
    m_scanMoveStarted = false;
    m_recording = false;
    m_recordClock.invalidate();
    m_session.clear();
    m_session.reverseY = request.reverseY;
    m_session.begin(request.stepUm);
    std::copy_n(request.startPose, 6, m_moveOrigin);

    // 相机参数先下发：把扫描需要的参数统一交给 worker
    applyCameraParams();

    if (request.useStartPose) {
        setState(State::Positioning);
        m_liftPending = request.useLiftPose;
        if (m_liftPending) {
            emit progress(QString::fromUtf8("正在竖直抬到扫描安全高度…"));
            issuePositionMove(request.liftPose,
                QString::fromUtf8("扫描抬升"));
        }
        else {
            emit progress(QString::fromUtf8("正在调平/移动到扫描起始位姿…"));
            issuePositionMove(request.startPose,
                QString::fromUtf8("扫描定位"));
        }
        return;
    }

    // 已就位：直接起流
    setState(State::Starting);
    m_watchdog->start(kStreamStartGraceMs);
    QMetaObject::invokeMethod(m_camera, [cam = m_camera]() {
        cam->startCapture();
    }, Qt::QueuedConnection);
}

void ScanController::cancel()
{
    if (!isRunning()) return;

    if (m_robot) {
        QMetaObject::invokeMethod(m_robot, [robot = m_robot]() {
            robot->stopMotion();
        }, Qt::QueuedConnection);
    }
    if (m_camera) {
        QMetaObject::invokeMethod(m_camera, [cam = m_camera]() {
            cam->stopCapture();
        }, Qt::QueuedConnection);
    }
    fail(QString::fromUtf8("扫描已被用户终止。"));
}

void ScanController::onMoveFinished(bool ok, QString error)
{
    if (m_returning) {
        m_returning = false;
        emit progress(ok
            ? QString::fromUtf8("已退回扫描起点。")
            : QString::fromUtf8("退回扫描起点失败：%1").arg(error));
        return;
    }

    if (m_state == State::Positioning) {
        m_watchdog->stop();
        if (!ok) {
            fail(QString::fromUtf8("定位失败：%1").arg(error));
            return;
        }
        if (m_liftPending) {
            // 已在安全高度，接下来只转姿态，不再动 XY/Z
            m_liftPending = false;
            emit progress(QString::fromUtf8(
                "已到安全高度，正在调整扫描姿态…"));
            issuePositionMove(m_request.startPose,
                QString::fromUtf8("扫描姿态调整"));
            return;
        }
        setState(State::Starting);
        emit progress(QString::fromUtf8("调平完成，正在启动相机数据流…"));
        m_watchdog->start(kStreamStartGraceMs);
        QMetaObject::invokeMethod(m_camera, [cam = m_camera]() {
            cam->startCapture();
        }, Qt::QueuedConnection);
        return;
    }

    if (m_state == State::Scanning && m_scanMoveStarted) {
        m_scanMoveStarted = false;
        if (!ok) {
            // 运动中失败必须立刻停流，否则相机会一直亮着激光
            fail(QString::fromUtf8("扫描运动失败：%1").arg(error));
            return;
        }
        // 运动到位后再收 300 ms 尾巴：设备按批次打包输出，
        // 到位瞬间最后一批可能还没送到主机，立刻停流会切掉行程末端。
        setState(State::Stopping);
        emit progress(QString::fromUtf8("扫描运动完成，正在收尾…"));
        m_watchdog->start(kStreamTailMs);
        return;
    }
}

void ScanController::onScanFrame(ScanFrame::Ptr frame)
{
    if (m_state != State::Starting && m_state != State::Scanning) return;

    // 首帧到达只用来确认流已就绪：此刻机器人还没动，
    // 这一帧的轮廓是「原地重复扫描」，不能计入行进数据。
    if (m_state == State::Starting) {
        m_watchdog->stop();
        setState(State::Scanning);
        emit progress(QString::fromUtf8("开始扫描运动（%1 mm）…")
            .arg(m_request.scanLengthMm, 0, 'f', 1));
        issueScanMove();
        return;
    }

    // 等机器人真的开始位移后才开录，避免把启动加速段的原地轮廓
    // 写进点云（那会凭空多出几十行，把 Y 尺度整体压扁）。
    if (!m_recording) {
        if (travelledMm() < kRecordingStartDisplacementMm) return;
        m_recording = true;
        m_recordingStartMm = travelledMm();
        m_recordClock.start();
        emit progress(QString::fromUtf8("机器人已位移 %1 mm，开始记录轮廓。")
            .arg(m_recordingStartMm, 0, 'f', 2));
    }

    m_session.addFrame(frame);
}

void ScanController::issuePositionMove(const double pose[6],
    const QString& description)
{
    m_watchdog->start(kDefaultPositionTimeoutMs);

    RobotMoveTarget target;
    std::copy_n(pose, 6, target.pose);
    target.relative = false;
    target.linear = true;
    target.toolId = m_request.toolId;
    target.userId = m_request.userId;
    target.velPercent = m_request.approachVelPercent;
    target.accPercent = m_request.accPercent;
    target.timeoutMs = kDefaultPositionTimeoutMs;
    target.description = description;
    QMetaObject::invokeMethod(m_robot, [robot = m_robot, target]() {
        robot->moveTo(target);
    }, Qt::QueuedConnection);
}

void ScanController::issueScanMove()
{
    m_scanMoveStarted = true;
    m_watchdog->start(scanTimeoutMs());
    m_track.clear();
    sampleTrack();
    m_trackTimer->start();

    RobotMoveTarget target;
    // 沿指定的工具轴直线行进。姿态分量全 0 + MoveL 直线插补，
    // 保证整个行程中传感器姿态不变（即「调平保持」）。
    // 位移量本身不出现在点云坐标里：Y 由 ScanSession 按行序号补齐。
    int axis = 1;   // 默认工具 Y
    switch (m_request.scanAxis) {
    case ScanRequest::ScanAxis::ToolX: axis = 0; break;
    case ScanRequest::ScanAxis::ToolZ: axis = 2; break;
    case ScanRequest::ScanAxis::ToolY: axis = 1; break;
    }
    target.pose[0] = target.pose[1] = target.pose[2] = 0.0;
    target.pose[axis] = m_request.scanLengthMm;
    target.pose[3] = target.pose[4] = target.pose[5] = 0.0;
    target.relative = true;
    target.relativeInTool = true;
    target.linear = true;
    target.toolId = m_request.toolId;
    target.userId = m_request.userId;
    target.velPercent = m_request.scanVelPercent;
    target.accPercent = m_request.accPercent;
    target.timeoutMs = scanTimeoutMs();
    target.description = QString::fromUtf8("扫描行程");

    QMetaObject::invokeMethod(m_robot, [robot = m_robot, target]() {
        robot->moveTo(target);
    }, Qt::QueuedConnection);
}

void ScanController::issueReturnMove()
{
    RobotMoveTarget target;
    std::copy_n(m_moveOrigin, 6, target.pose);
    target.relative = false;
    target.linear = true;
    target.toolId = m_request.toolId;
    target.userId = m_request.userId;
    target.velPercent = m_request.approachVelPercent;
    target.accPercent = m_request.accPercent;
    target.timeoutMs = kDefaultPositionTimeoutMs;
    target.description = QString::fromUtf8("退回扫描起点");

    m_returning = true;
    emit progress(QString::fromUtf8("正在退回扫描起点…"));
    QMetaObject::invokeMethod(m_robot, [robot = m_robot, target]() {
        robot->moveTo(target);
    }, Qt::QueuedConnection);
}

double ScanController::travelledMm() const
{
    if (!m_robot) return 0.0;
    // 直接读 worker 的缓存值：100 ms 的 telemetryUpdated 周期对
    // 「是否已经开始位移」这个判据太粗糙，会让前几行被误吞。
    return travelledMm(m_robot->currentTelemetry());
}

double ScanController::travelledMm(const RobotTelemetry& telemetry) const
{
    if (!telemetry.connected) return 0.0;

    const double dx = telemetry.tcpPose[0] - m_moveOrigin[0];
    const double dy = telemetry.tcpPose[1] - m_moveOrigin[1];
    const double dz = telemetry.tcpPose[2] - m_moveOrigin[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void ScanController::sampleTrack()
{
    if (!m_robot) return;
    const RobotTelemetry telemetry = m_robot->currentTelemetry();
    if (!telemetry.connected || telemetry.sampleUs == 0) return;
    // 遥测缓存比本定时器更新得慢，按时间戳去重
    if (!m_track.empty() && telemetry.sampleUs <= m_track.back().us) return;
    ScanSession::TrackSample sample;
    sample.us = telemetry.sampleUs;
    sample.mm = travelledMm(telemetry);
    std::copy_n(telemetry.tcpPose, 6, sample.pose);
    m_track.push_back(sample);
}

void ScanController::onCameraStateChanged(
    HikCameraWorker::State state, QString detail)
{
    if (!isRunning()) return;

    // 只把「链路真的断了」当致命事件。采集正常收尾时相机会从 Capturing
    // 回到 Connected，那正是我们期待的路径，不能判失败。
    if (state == HikCameraWorker::State::Disconnected
        || state == HikCameraWorker::State::Fault) {
        fail(QString::fromUtf8("扫描过程中相机断开：%1").arg(detail));
    }
}

void ScanController::onCaptureFinished()
{
    if (m_state == State::Stopping) {
        assembleAndFinish();
    }
}

void ScanController::onWatchdog()
{
    switch (m_state) {
    case State::Positioning:
        fail(QString::fromUtf8("定位超时。"));
        break;
    case State::Starting:
        fail(QString::fromUtf8("相机起流超时，未收到任何轮廓数据。"));
        break;
    case State::Scanning:
        fail(QString::fromUtf8("扫描运动超时。"));
        break;
    case State::Stopping:
        // 收尾阶段等不到 captureFinished，也要把已有数据交出去
        assembleAndFinish();
        break;
    default:
        break;
    }
}

void ScanController::assembleAndFinish()
{
    m_watchdog->stop();

    // 兜底：无论走哪条路径都确保流已停
    if (m_camera && m_camera->isCapturing()) {
        const bool finished = QMetaObject::invokeMethod(m_camera,
            [cam = m_camera]() { cam->stopCapture(); },
            Qt::BlockingQueuedConnection);
        Q_UNUSED(finished);
    }

    // 扫描运动已到位、相机已停流，此时退回不会污染数据
    if (m_request.returnToStart && m_scanMoveStarted == false) {
        issueReturnMove();
    }

    if (m_session.rowCount() == 0) {
        fail(QString::fromUtf8("扫描未采集到任何轮廓数据。"));
        return;
    }

    // 复核帧号跳变是不是真丢批。
    //
    // 实测设备的 nFrameNum 并不保证逐批 +1；若把每次跳变都按平均行数补空行，
    // 实收数据只占 Y 向很小一段，点云会变成一条条稀疏的条纹。判据：实收行率
    // 已接近设定行率，说明没有丢数据，跳变只是帧号语义问题，补齐区间作废。
    const double recordSec = m_recordClock.isValid()
        ? m_recordClock.elapsed() / 1000.0 : 0.0;
    const double receivedRate = recordSec > 0.0
        ? static_cast<double>(m_session.receivedRows()) / recordSec : 0.0;
    const int gaps = m_session.gapCount();
    bool fillGaps = true;
    if (gaps > 0 && m_request.lineRate > 0.0f
        && receivedRate >= 0.8 * m_request.lineRate) {
        fillGaps = false;
    }
    emit progress(QString::fromUtf8(
        "采集统计：实收 %1 批 / %2 行，录制 %3 s，实收行率 %4 Hz（设定 %5 Hz）；"
        "帧号跳变 %6 处，最大跳变 %7，估算缺失 %8 行。")
        .arg(m_session.receivedFrames())
        .arg(m_session.receivedRows())
        .arg(recordSec, 0, 'f', 2)
        .arg(receivedRate, 0, 'f', 1)
        .arg(m_request.lineRate, 0, 'f', 0)
        .arg(gaps)
        .arg(m_session.maxFrameDelta())
        .arg(m_session.estimatedMissingRows()));

    // 首选轨迹模式：每行的 Y 取该行采集时刻机器人的实测行进距离。
    // 「行序号 × 等效步距」隐含匀速假设，加减速段会被拉伸或压缩，
    // 实物上的直边、圆弧在点云里因此变形。
    m_trackTimer->stop();
    sampleTrack();

    // 有手眼参数时直接输出基坐标点云，后续匹配与焊缝映射都落在基坐标里，
    // 映射结果可直接作为 TCP 目标下发。
    if (m_haveHandEye) {
        const ScanResult inBase = m_session.resultInBase(
            m_track, m_handEye.tcpToSdk(), m_handEye.timeOffsetUs);
        if (!inBase.empty()) {
            CloudRGB::Ptr cloud = ScanSession::cloudFrom(inBase);
            if (cloud && !cloud->empty()) {
                // 落地安装、传感器朝下：点云应整体位于 TCP 下方。
                // 反了说明 sensorAxis 轴置换不对，结果不能用于运动。
                double meanZ = 0.0;
                for (const auto& point : cloud->points) meanZ += point.z;
                meanZ /= static_cast<double>(cloud->size());
                const double tcpZ = m_track.front().pose[2];
                emit progress(QString::fromUtf8(
                    "扫描完成（基坐标）：轨迹采样 %1 点，实测行率 %2 Hz，"
                    "%3 行 × %4 列，有效点 %5；点云平均 Z %6 mm，扫描起点 TCP Z %7 mm。")
                    .arg(m_track.size())
                    .arg(m_session.measuredLineRate(), 0, 'f', 1)
                    .arg(inBase.rows)
                    .arg(inBase.cols)
                    .arg(cloud->size())
                    .arg(meanZ, 0, 'f', 2)
                    .arg(tcpZ, 0, 'f', 2));
                if (meanZ >= tcpZ) {
                    emit progress(QString::fromUtf8(
                        "警告：点云平均高度不低于 TCP，calibration.json 的 sensorAxis "
                        "很可能不对。请先用「点选点云 → 移到上方」核对后再执行焊接。"));
                }
                setState(State::Done);
                emit finished(std::move(cloud), CloudFrame::RobotBase);
                return;
            }
        }
        emit progress(QString::fromUtf8(
            "基坐标拼接失败（轨迹采样 %1 点），退回传感器坐标拼接，结果不可用于运动。")
            .arg(m_track.size()));
    }

    const ScanResult byTrack = m_session.resultByTrack(m_track);
    if (!byTrack.empty()) {
        const double span = std::abs(byTrack.yMm.back() - byTrack.yMm.front());
        const double lineRate = m_session.measuredLineRate();
        CloudRGB::Ptr cloud = ScanSession::cloudFrom(byTrack);
        setState(State::Done);
        emit progress(QString::fromUtf8(
            "扫描完成（轨迹定位）：轨迹采样 %1 点，实测行率 %2 Hz，"
            "%3 行 × %4 列，Y 向跨度 %5 mm，平均步距 %6 µm，有效点 %7。")
            .arg(m_track.size())
            .arg(lineRate, 0, 'f', 1)
            .arg(byTrack.rows)
            .arg(byTrack.cols)
            .arg(span, 0, 'f', 2)
            .arg(byTrack.rows > 1 ? span * 1000.0 / (byTrack.rows - 1) : 0.0, 0, 'f', 1)
            .arg(cloud ? cloud->size() : 0));
        emit finished(std::move(cloud), CloudFrame::Sensor);
        return;
    }
    emit progress(QString::fromUtf8(
        "机器人轨迹采样不足（%1 点），退回按行序号 × 步距拼接。")
        .arg(m_track.size()));

    const quint64 rows = m_session.rowCount(fillGaps);

    // 用实际位移复核 Y 尺度。
    //
    // 「步距 × 行率 = 速度」这个约束靠设备参数和机器人百分比速度各自实现，
    // 而百分比速度没有 mm/s 映射，现场第一次跑几乎必然不自洽。既然能读到
    // 真实位移，就直接反算等效步距：行数 × 等效步距 = 实际位移。
    // 只要偏差超过 5% 就采用实测值，并在日志里明确告知，让现场去标速度百分比。
    const double displacement = std::max(
        travelledMm() - m_recordingStartMm, 0.0);
    double effectiveStepUm = m_session.stepUm();
    if (m_recording && displacement > 1.0 && rows > 1) {
        const double measured = displacement * 1000.0 / static_cast<double>(rows - 1);
        const double nominal = static_cast<double>(m_session.stepUm());
        if (nominal > 0.0 && std::abs(measured - nominal) / nominal > 0.05) {
            effectiveStepUm = measured;
            emit progress(QString::fromUtf8(
                "Y 尺度校正：实测位移 %1 mm / %2 行 → 等效步距 %3 µm"
                "（名义 %4 µm，偏差 %5%）。原因是机器人速度百分比与实际"
                "线速度不匹配，建议据此标定 scanVelPercent。")
                .arg(displacement, 0, 'f', 2)
                .arg(rows - 1)
                .arg(measured, 0, 'f', 1)
                .arg(nominal, 0, 'f', 1)
                .arg((measured - nominal) / nominal * 100.0, 0, 'f', 1));
        }
    }

    if (gaps > 0) {
        emit progress(fillGaps
            ? QString::fromUtf8(
                "扫描完成，检测到 %1 处丢批，已按平均行数补齐空洞。").arg(gaps)
            : QString::fromUtf8(
                "帧号跳变 %1 处但实收行率正常，判定为帧号不连续而非丢批，"
                "未补空行。").arg(gaps));
    }

    CloudRGB::Ptr cloud = m_session.toCloud(effectiveStepUm, fillGaps);
    setState(State::Done);
    emit progress(QString::fromUtf8(
        "扫描完成：%1 行 × %2 列，Y 向跨度 %3 mm，有效点 %4。")
        .arg(rows)
        .arg(m_session.cols())
        .arg(m_session.travelMm(effectiveStepUm, fillGaps), 0, 'f', 2)
        .arg(cloud ? cloud->size() : 0));
    emit finished(std::move(cloud), CloudFrame::Sensor);
}

void ScanController::fail(const QString& message)
{
    m_lastError = message;
    m_watchdog->stop();
    m_trackTimer->stop();

    // 任何失败路径都必须把机器人和相机都拉回静止状态。
    // 判据用 worker 自身的 Busy 标志而非本控制器状态：定位阶段被取消时
    // 本控制器还停在 Positioning，但机器人已经在动了。
    if (m_robot && m_robot->state() == RobotWorker::State::Busy) {
        QMetaObject::invokeMethod(m_robot, [robot = m_robot]() {
            robot->stopMotion();
        }, Qt::QueuedConnection);
    }
    if (m_camera && m_camera->isCapturing()) {
        QMetaObject::invokeMethod(m_camera, [cam = m_camera]() {
            cam->stopCapture();
        }, Qt::QueuedConnection);
    }

    m_session.clear();
    setState(State::Failed);
    emit failed(message);
}
