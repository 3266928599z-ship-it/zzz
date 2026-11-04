#pragma once

#include "AppTypes.h"
#include "HandEyeConfig.h"
#include "HikCameraWorker.h"
#include "RobotWorker.h"
#include "ScanSession.h"

#include <QElapsedTimer>
#include <QObject>
#include <QString>

class RobotWorker;
class QTimer;

// 一次扫描的参数。
//
// 三个速度字段必须自洽：travelSpeedMmS = stepUm × lineRate / 1000。
// 本轮标定值 96 µm × 491 行/s ≈ 47 mm/s，这时点云 Y 向不会被拉伸。
struct ScanRequest {
    // 扫描行进方向所在的工具轴。
    //
    // 注意：这只决定**机器人怎么走**。传感器点云里的行永远是沿相机自身
    // Y 轴堆叠的，所以 ScanSession 仍按「全局行序号 × 步距」补 Y。
    // 该选择能成立的前提是「传感器 Y 轴 ≈ 所选的工具轴」，这由安装决定；
    // 本机标定 rx -3.1° / rz 1.6° 的斜装会让行进方向偏离约 3°。
    enum class ScanAxis { ToolX, ToolY, ToolZ };
    ScanAxis scanAxis = ScanAxis::ToolX;

    // ---- 起始位姿 ----
    // true 时先用 MoveL 走到 startPose（绝对，基坐标）再开始扫描；
    // false 表示机械臂已经在对的位置，直接从当前位置扫。
    bool useStartPose = false;
    double startPose[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // true 时定位拆成两段：先在当前 XY 竖直抬到 liftPose（保持当前姿态），
    // 再在安全高度上转成 startPose。单条 MoveL 从低处斜着插过去可能穿过工件。
    bool useLiftPose = false;
    double liftPose[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // ---- 扫描行程 ----
    // 沿 scanAxis 指定工具轴的行程，正数朝该轴正方向。
    // 扫描时姿态恒定，MoveL 保证插补过程中不旋转，即所谓「调平保持」。
    double scanLengthMm = 200.0;
    // 行进方向与传感器 Y 轴正方向相反时置 true，拼接时翻转 Y 符号
    bool reverseY = false;

    // ---- 相机几何 ----
    float stepUm = 96.0f;
    float lineRate = 600.0f;
    float exposureUs = 80.0f;
    float gainDb = 0.0f;
    int photosensitivity = 4;    // 4 = 自定义，否则曝光写不进去

    // ---- 机器人速度（百分比，MoveL 的 vel 参数）----
    float approachVelPercent = 20.0f;
    // 扫描结束后以 approachVelPercent 直线退回起点，保证每次扫的是同一段
    bool returnToStart = true;
    float scanVelPercent = 10.0f;
    float accPercent = 20.0f;
    int toolId = 1;              // 现场标定的 hik
    int userId = 0;
};

// 扫描编排：相机与机械臂各自跑在独立线程里，这里只做状态机与拼接。
//
//   Idle → Positioning（可选）→ Starting（起流）→ Scanning（MoveL）
//        → Stopping（停流）→ Done
//
// 相机先于运动起流、运动结束后才停流，保证行程两端都有轮廓数据；
// 代价是两端各有几十毫秒的「原地重复轮廓」，拼接时按行序号连续累加即可。
class ScanController : public QObject
{
    Q_OBJECT
public:
    enum class State {
        Idle, Positioning, Starting, Scanning, Stopping, Done, Failed
    };
    Q_ENUM(State)

    ScanController(HikCameraWorker* camera, RobotWorker* robot,
        QObject* parent = nullptr);

    State state() const { return m_state; }
    bool isRunning() const;
    QString lastError() const { return m_lastError; }
    const ScanSession& session() const { return m_session; }

    // 设置后，轨迹模式直接输出机械臂基坐标点云；未设置时输出传感器坐标点云
    void setHandEye(const HandEyeConfig& config) { m_handEye = config; m_haveHandEye = true; }
    void clearHandEye() { m_haveHandEye = false; }

public slots:
    void start(ScanRequest request);
    void cancel();

signals:
    void progress(QString stage);
    void finished(CloudRGB::Ptr cloud, CloudFrame frame);
    void failed(QString message);

private slots:
    void onScanFrame(ScanFrame::Ptr frame);
    void onMoveFinished(bool ok, QString error);
    void onCameraStateChanged(HikCameraWorker::State state, QString detail);
    void onCaptureFinished();
    void onWatchdog();

private:
    void setState(State next);
    void fail(const QString& message);
    void applyCameraParams();
    void assembleAndFinish();
    void issueScanMove();
    void issueReturnMove();
    void issuePositionMove(const double pose[6], const QString& description);
    // 与起始位姿的 TCP 位移（mm），用于判定机器人是否真的动起来
    double travelledMm() const;
    double travelledMm(const RobotTelemetry& telemetry) const;
    // 把最新一次带时间戳的遥测追加到轨迹，供拼接时按时刻插值 Y
    void sampleTrack();
    int scanTimeoutMs() const;

    HikCameraWorker* m_camera = nullptr;
    RobotWorker* m_robot = nullptr;
    ScanSession m_session;
    ScanRequest m_request;
    State m_state = State::Idle;
    QString m_lastError;
    QTimer* m_watchdog = nullptr;
    QTimer* m_trackTimer = nullptr;
    std::vector<ScanSession::TrackSample> m_track;
    HandEyeConfig m_handEye;
    bool m_haveHandEye = false;
    // 定位分几段走：先抬升（若给定 liftPose）再转姿态
    bool m_liftPending = false;

    // 起止窗口控制：机器人真正开始位移后才开录，避免把加速段的原地轮廓
    // 当成行进数据写进点云（那会让 Y 向整体被拉伸）。
    bool m_recording = false;
    double m_recordingStartMm = 0.0;
    QElapsedTimer m_recordClock;     // 录制时长，用于复核实收行率
    double m_moveOrigin[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    bool m_scanMoveStarted = false;
    bool m_returning = false;     // 回零运动在途，只用于汇报结果
};
