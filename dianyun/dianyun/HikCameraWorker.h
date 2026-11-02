#pragma once
#include "CameraCaptureState.h"

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

// 设备采集参数，坐标单位与偏移由 SDK 在每帧数据中给出，不再人工标定
struct ScanParams {
    float exposureTime = 0.0f;    // 曝光时间 (us)，0 表示保持设备当前值
    float frameRate = 0.0f;       // 采集帧率 (fps)，0 表示保持设备当前值
    int frameCount = 1;           // 单次采集期望累积的帧数（点云模式下一帧即一幅完整点云）
    int captureTimeoutMs = 0;     // 采集看门狗超时，<=0 表示按帧数与帧率自动推算
    bool keepStreaming = false;   // 采集结束后是否保持出流（焊接跟踪阶段才需要）
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
    void captureFinished();
    void captureFailed(QString errorMessage);

    // SDK 回调线程 → worker 线程，必须 Queued
    void internalCaptureDone(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void internalCaptureError(QString errorMessage);

private slots:
    void onInternalCaptureDone(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void onInternalCaptureError(QString errorMessage);
    void onCaptureTimeout();

private:
    void setState(State next, const QString& detail = QString());
    bool ensureSdkInitialized();
    bool openDevice();
    void closeDevice();
    bool applyParams();
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
    bool m_deviceOpened = false;
    bool m_measuring = false;
    bool m_sdkInitialized = false;
    char m_serialNumber[16] = { 0 };
    mutable QMutex m_cloudMutex;
    mutable QMutex m_stateMutex;
    QString m_lastError;
};
