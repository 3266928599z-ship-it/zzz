#pragma once
#include "CameraCaptureState.h"
#include "ScanTypes.h"

#include <QObject>
#include <QMutex>
#include <QString>
#include <atomic>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Mv3dLpApi.h"
#include "Mv3dLpDefine.h"

class QTimer;

Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)

// 设备采集参数。
//
// 探针实测（MV-DP2240-01H / SDK 1.3.3.3）确认了三件事，直接决定了这里的字段：
//   1. ImageMode=4 输出的是 Profile/ABC16（3×int16），不是 float 点云，
//      并按 LSLStepDistance 把多行轮廓打包成一批；每批行数由设备自行决定。
//   2. AcquisitionFrameRate 控制的是**轮廓行率**，不是深度图帧率。
//   3. 回调里的 fXScale/fYScale/fZScale 恒为 0，真实换算系数只有
//      LSLProfileCoordXUnit/YUnit/ZUnit（µm/count）可用。
//
// 因此扫描时「速度 = 步距 × 行率」必须由三个字段共同保证自洽，
// 否则点云在 Y 向会被拉伸或压缩。
struct ScanParams {
    // ---- 曝光 ----
    float exposureTime = 0.0f;    // 曝光时间 (us)，0 表示保持设备当前值
    float gain = -1.0f;           // 增益 (dB)，<0 表示保持设备当前值
    int photosensitivity = -1;    // 感光灵敏度枚举，<0 保持；4=自定义（自定义下才能手写曝光）

    // ---- 扫描几何：三者必须自洽 ----
    float stepDistanceUm = 0.0f;  // 行间步距 (µm)，0 表示保持设备当前值
    float lineRate = 0.0f;        // 轮廓行率 (Hz)，0 表示保持设备当前值
    float travelSpeedMmS = 0.0f;  // 机器人扫描速度 (mm/s)，仅用于校验，不下发

    // ---- 采集窗口 ----
    int frameCount = 1;           // 期望累积的批次数；0 表示持续采到 stopCapture
    int captureTimeoutMs = 0;     // 采集看门狗超时，<=0 表示按帧数与行率自动推算
    bool keepStreaming = false;   // 采集结束后是否保持出流（扫描中途需要）
    bool laserEnabled = true;     // 扫描期间是否点亮激光
};


// 相机设备生命周期分四层：
//   L1 SDK    进程级，构造时初始化，析构时释放
//   L2 设备    connectCamera / disconnectCamera
//   L3 数据流  startCapture / stopCapture（是否停流由 ScanParams::keepStreaming 决定）
//   L4 采集窗口 m_captureState，每次采集独占，供 SDK 回调无锁判断
class HikCameraWorker : public QObject
{
    Q_OBJECT
public:
    enum class State { Disconnected, Connecting, Connected, Capturing, Fault };
    Q_ENUM(State)

    explicit HikCameraWorker(QObject* parent = nullptr);
    ~HikCameraWorker() override;

    State state() const { return m_state.load(); }
    bool isConnected() const { return state() == State::Connected; }
    bool isCapturing() const { return state() == State::Capturing; }
    QString lastError() const;
    QString deviceSerial() const;

    // 将参数暴露为公有，方便主界面动态修改；connect / startCapture 时生效
    ScanParams m_params;
    // 空表示取枚举到的第一台设备
    QString m_preferredSerial;

public slots:
    void connectCamera();
    void disconnectCamera();
    void startCapture();
    void stopCapture();

signals:
    void linkStateChanged(HikCameraWorker::State state, QString detail);
    void pointCloudReady(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    // 扫描路径：逐帧输出，由 ScanSession 按行序号拼接
    void scanFrameReady(ScanFrame::Ptr frame);
    void captureFinished();
    void captureFailed(QString errorMessage);

    // SDK 回调线程 → worker 线程，必须 Queued
    void internalCaptureDone(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void internalScanFrame(ScanFrame::Ptr frame);
    void internalCaptureError(QString errorMessage);

private slots:
    void onInternalCaptureDone(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void onInternalScanFrame(ScanFrame::Ptr frame);
    void onInternalCaptureError(QString errorMessage);
    void onCaptureTimeout();

private:
    void setState(State next, const QString& detail = QString());
    bool ensureSdkInitialized();
    bool openDevice();
    void closeDevice();
    bool applyParams();
    bool queryScanGeometry();
    void finishCapture();
    int captureTimeoutMs() const;
    void resetAccumulator();
    static QString statusText(MV3D_LP_STATUS status);
    static void __stdcall ImageCallBackEx(
        MV3D_LP_IMAGE_DATA* imageData, void* user);

    HANDLE m_hCamera = nullptr;
    std::atomic<State> m_state{ State::Disconnected };
    CameraCaptureState m_captureState;
    QTimer* m_watchdog = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_accumulatedCloud;
    int m_frameCount = 0;

    // 由设备回读的实际换算系数，回调解码时只读，故用原子量
    std::atomic<float> m_unitX{ 0.0f };   // µm/count
    std::atomic<float> m_unitZ{ 0.0f };
    std::atomic<float> m_stepUm{ 0.0f };
    bool m_scanMode = false;              // true: 逐帧输出；false: 单张点云

    bool m_deviceOpened = false;
    bool m_measuring = false;
    bool m_sdkInitialized = false;
    char m_serialNumber[16] = { 0 };
    mutable QMutex m_cloudMutex;
    mutable QMutex m_stateMutex;
    QString m_lastError;
};
