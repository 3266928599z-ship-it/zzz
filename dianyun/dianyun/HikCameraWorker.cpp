#include "HikCameraWorker.h"

#include <QMutexLocker>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace {

// 设备图像模式：1 原始图 / 4 点云(实为 Profile ABC16) / 7 深度图
constexpr uint32_t kImageModePointCloud = 4;

// SDKEmptyPoint=0 时点云只保留有效点，不带空点占位
constexpr int kEmptyPointDisabled = 0;

// 未指定帧率时的看门狗兜底超时
constexpr int kDefaultCaptureTimeoutMs = 3000;

// 设备用 -32768 标记空点（三个分量同时为该值）
constexpr short kEmptyPointShort = -32768;

// 感光灵敏度「自定义」，只有该模式下才能手写曝光时间与增益
constexpr uint32_t kPhotosensitivityCustom = 4;

MV3D_LP_STATUS setEnumParam(HANDLE handle, const char* key, uint32_t value)
{
    MV3D_LP_PARAM param;
    memset(&param, 0, sizeof(param));
    param.enParamType = ParamType_Enum;
    param.ParamInfo.stEnumParam.nCurValue = value;
    return MV3D_LP_SetParam(handle, key, &param);
}

MV3D_LP_STATUS setIntParam(HANDLE handle, const char* key, int64_t value)
{
    MV3D_LP_PARAM param;
    memset(&param, 0, sizeof(param));
    param.enParamType = ParamType_Int;
    param.ParamInfo.stIntParam.nCurValue = value;
    return MV3D_LP_SetParam(handle, key, &param);
}

MV3D_LP_STATUS setFloatParam(HANDLE handle, const char* key, float value)
{
    MV3D_LP_PARAM param;
    memset(&param, 0, sizeof(param));
    param.enParamType = ParamType_Float;
    param.ParamInfo.stFloatParam.fCurValue = value;
    return MV3D_LP_SetParam(handle, key, &param);
}

MV3D_LP_STATUS getFloatParam(HANDLE handle, const char* key, float* out)
{
    MV3D_LP_PARAM param;
    memset(&param, 0, sizeof(param));
    const MV3D_LP_STATUS status = MV3D_LP_GetParam(handle, key, &param);
    if (status != MV3D_LP_OK) return status;
    if (param.enParamType != ParamType_Float) return MV3D_LP_E_PARAMETER;
    *out = param.ParamInfo.stFloatParam.fCurValue;
    return MV3D_LP_OK;
}

bool isValidPoint(float x, float y, float z)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return false;
    }
    // 无回波的空点通常以原点表示
    return x != 0.0f || y != 0.0f || z != 0.0f;
}

} // namespace

HikCameraWorker::HikCameraWorker(QObject* parent)
    : QObject(parent)
{
    resetAccumulator();

    // L1：进程级 SDK 初始化只做一次，失败不阻断构造，由连接时报错
    m_sdkInitialized = (MV3D_LP_Initialize() == MV3D_LP_OK);

    m_watchdog = new QTimer(this);
    m_watchdog->setSingleShot(true);
    connect(m_watchdog, &QTimer::timeout,
        this, &HikCameraWorker::onCaptureTimeout);

    // SDK 回调线程 → 本对象线程，必须排队，回调内不直接改状态
    connect(this, &HikCameraWorker::internalCaptureDone,
        this, &HikCameraWorker::onInternalCaptureDone,
        Qt::QueuedConnection);
    connect(this, &HikCameraWorker::internalScanFrame,
        this, &HikCameraWorker::onInternalScanFrame,
        Qt::QueuedConnection);
    connect(this, &HikCameraWorker::internalCaptureError,
        this, &HikCameraWorker::onInternalCaptureError,
        Qt::QueuedConnection);
}

HikCameraWorker::~HikCameraWorker()
{
    closeDevice();
    if (m_sdkInitialized) {
        // L1 只在析构时收，反复连接不会触发 Initialize/Finalize 循环
        MV3D_LP_Finalize();
        m_sdkInitialized = false;
    }
}

QString HikCameraWorker::lastError() const
{
    QMutexLocker locker(&m_cloudMutex);
    return m_lastError;
}

QString HikCameraWorker::deviceSerial() const
{
    return QString::fromLatin1(m_serialNumber);
}

QString HikCameraWorker::statusText(MV3D_LP_STATUS status)
{
    QString description;
    switch (static_cast<uint32_t>(status)) {
    case MV3D_LP_OK:                  description = QString::fromUtf8("成功"); break;
    case MV3D_LP_E_HANDLE:            description = QString::fromUtf8("设备句柄无效"); break;
    case MV3D_LP_E_SUPPORT:           description = QString::fromUtf8("设备不支持该功能"); break;
    case MV3D_LP_E_BUFOVER:           description = QString::fromUtf8("缓存已满"); break;
    case MV3D_LP_E_CALLORDER:         description = QString::fromUtf8("函数调用顺序错误"); break;
    case MV3D_LP_E_PARAMETER:         description = QString::fromUtf8("参数错误"); break;
    case MV3D_LP_E_RESOURCE:          description = QString::fromUtf8("资源申请失败"); break;
    case MV3D_LP_E_NODATA:            description = QString::fromUtf8("无数据"); break;
    case MV3D_LP_E_PRECONDITION:      description = QString::fromUtf8("前置条件不满足"); break;
    case MV3D_LP_E_VERSION:           description = QString::fromUtf8("SDK版本不匹配"); break;
    case MV3D_LP_E_NOENOUGH_BUF:      description = QString::fromUtf8("内存空间不足"); break;
    case MV3D_LP_E_ABNORMAL_IMAGE:    description = QString::fromUtf8("图像不完整，可能丢包"); break;
    case MV3D_LP_E_LOAD_LIBRARY:      description = QString::fromUtf8("动态库加载失败"); break;
    case MV3D_LP_E_ALGORITHM:         description = QString::fromUtf8("算法错误"); break;
    case MV3D_LP_E_DEVICE_OFFLINE:    description = QString::fromUtf8("设备离线"); break;
    case MV3D_LP_E_ACCESS_DENIED:     description = QString::fromUtf8("设备无访问权限"); break;
    case MV3D_LP_E_OUTOFRANGE:        description = QString::fromUtf8("数值超出范围"); break;
    default:                          description = QString::fromUtf8("未知错误"); break;
    }
    return QString::fromUtf8("%1（错误码：0x%2）")
        .arg(description)
        .arg(static_cast<uint32_t>(status), 8, 16, QLatin1Char('0'));
}

void HikCameraWorker::setState(State next, const QString& detail)
{
    {
        QMutexLocker locker(&m_stateMutex);
        if (m_state.load() == next && detail.isEmpty()) return;
        m_state.store(next);
    }
    emit linkStateChanged(next, detail);
}

bool HikCameraWorker::ensureSdkInitialized()
{
    if (m_sdkInitialized) return true;
    m_sdkInitialized = (MV3D_LP_Initialize() == MV3D_LP_OK);
    if (!m_sdkInitialized) {
        m_lastError = QString::fromUtf8("初始化3D相机SDK失败，请检查 Mv3dLp 运行库是否完整。");
    }
    return m_sdkInitialized;
}

void HikCameraWorker::resetAccumulator()
{
    QMutexLocker locker(&m_cloudMutex);
    m_accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    m_frameCount = 0;
}

bool HikCameraWorker::applyParams()
{
    if (!m_hCamera) {
        m_lastError = QString::fromUtf8("相机尚未打开，无法应用参数。");
        return false;
    }

    // ① 感光灵敏度必须最先确定：停在「高精度1」等预设时设备会强制自己的
    //    曝光/增益取值，后面写进去的 80 µs 会被静默忽略。
    if (m_params.photosensitivity >= 0) {
        setEnumParam(m_hCamera, "Photosensitivity",
            static_cast<uint32_t>(m_params.photosensitivity));
    }

    // ② 曝光与增益
    if (m_params.exposureTime > 0.0f) {
        setFloatParam(m_hCamera, "ExposureTime", m_params.exposureTime);
    }
    if (m_params.gain >= 0.0f) {
        setFloatParam(m_hCamera, "Gain", m_params.gain);
    }

    // ③ 激光
    {
        MV3D_LP_PARAM param;
        memset(&param, 0, sizeof(param));
        param.enParamType = ParamType_Bool;
        param.ParamInfo.bBoolParam = m_params.laserEnabled ? TRUE : FALSE;
        MV3D_LP_SetParam(m_hCamera, "LaserEnable", &param);
    }

    // ④ 步进间距与行率：这两项决定 Y 向密度，扫描前必须显式下发。
    //    速度(v) = 步距(δ) × 行率(f)，三者不自洽时点云会被纵向拉伸。
    if (m_params.stepDistanceUm > 0.0f) {
        const MV3D_LP_STATUS status = setFloatParam(
            m_hCamera, "LSLStepDistance", m_params.stepDistanceUm);
        if (status != MV3D_LP_OK) {
            m_lastError = QString::fromUtf8("设置步进间距失败：%1。")
                .arg(statusText(status));
            return false;
        }
    }
    if (m_params.lineRate > 0.0f) {
        // 行率控制必须先使能，否则写入的帧率不生效
        MV3D_LP_PARAM enable;
        memset(&enable, 0, sizeof(enable));
        enable.enParamType = ParamType_Bool;
        enable.ParamInfo.bBoolParam = TRUE;
        MV3D_LP_SetParam(m_hCamera, "AcquisitionFrameRateEnable", &enable);

        const MV3D_LP_STATUS status = setFloatParam(
            m_hCamera, "AcquisitionFrameRate", m_params.lineRate);
        if (status != MV3D_LP_OK) {
            m_lastError = QString::fromUtf8("设置轮廓行率失败：%1。")
                .arg(statusText(status));
            return false;
        }
    }

    // ⑤ 图像模式与空点策略
    MV3D_LP_STATUS status = setEnumParam(
        m_hCamera, MV3D_LP_ENUM_IMAGEMODE, kImageModePointCloud);
    if (status != MV3D_LP_OK) {
        m_lastError = QString::fromUtf8("设置相机点云图像模式失败：%1。")
            .arg(statusText(status));
        return false;
    }
    setIntParam(m_hCamera, MV3D_LP_INT_LSL_EMPTY_POINT, kEmptyPointDisabled);

    return queryScanGeometry();
}

// 回读设备实际的换算系数。回调内要对成千上万个 int16 做换算，
// 因此把系数缓存到原子量里，避免每帧都调 GetParam。
bool HikCameraWorker::queryScanGeometry()
{
    float unitX = 0.0f;
    float unitZ = 0.0f;
    float stepUm = 0.0f;

    if (getFloatParam(m_hCamera, "LSLProfileCoordXUnit", &unitX) != MV3D_LP_OK
        || unitX <= 0.0f) {
        m_lastError = QString::fromUtf8(
            "读取 X 轴换算系数失败，无法把轮廓编码值换算成毫米。");
        return false;
    }
    // Z 单位缺失时退化为与 X 相同，实测本机型三者一致（4 µm/count）
    if (getFloatParam(m_hCamera, "LSLProfileCoordZUnit", &unitZ) != MV3D_LP_OK
        || unitZ <= 0.0f) {
        unitZ = unitX;
    }
    if (getFloatParam(m_hCamera, "LSLStepDistance", &stepUm) != MV3D_LP_OK
        || stepUm <= 0.0f) {
        m_lastError = QString::fromUtf8(
            "读取步进间距失败，无法为轮廓补上 Y 坐标。");
        return false;
    }

    m_unitX.store(unitX);
    m_unitZ.store(unitZ);
    m_stepUm.store(stepUm);
    return true;
}

bool HikCameraWorker::openDevice()
{
    uint32_t deviceCount = 0;
    MV3D_LP_STATUS status = MV3D_LP_GetDeviceNumber(&deviceCount);
    if (status != MV3D_LP_OK || deviceCount == 0) {
        m_lastError = QString::fromUtf8("没有检测到可用的海康3D相机。");
        return false;
    }

    std::vector<MV3D_LP_DEVICE_INFO> devices(deviceCount);
    status = MV3D_LP_GetDeviceList(devices.data(), deviceCount, &deviceCount);
    if (status != MV3D_LP_OK || deviceCount == 0) {
        m_lastError = QString::fromUtf8("获取相机列表失败：%1。")
            .arg(statusText(status));
        return false;
    }

    std::size_t index = 0;
    if (!m_preferredSerial.isEmpty()) {
        const QByteArray wanted = m_preferredSerial.toLatin1();
        const auto end = devices.cbegin() + deviceCount;
        const auto found = std::find_if(devices.cbegin(), end,
            [&wanted](const MV3D_LP_DEVICE_INFO& info) {
                return wanted == info.chSerialNumber;
            });
        if (found == end) {
            m_lastError = QString::fromUtf8("未找到指定序列号的相机：%1。")
                .arg(m_preferredSerial);
            return false;
        }
        index = static_cast<std::size_t>(found - devices.cbegin());
    }

    status = MV3D_LP_OpenDeviceBySN(
        &m_hCamera, devices[index].chSerialNumber);
    if (status != MV3D_LP_OK || !m_hCamera) {
        m_hCamera = nullptr;
        m_lastError = QString::fromUtf8("打开相机失败：%1。")
            .arg(statusText(status));
        return false;
    }
    m_deviceOpened = true;

    memcpy(m_serialNumber, devices[index].chSerialNumber,
        sizeof(m_serialNumber) - 1);
    m_serialNumber[sizeof(m_serialNumber) - 1] = '\0';

    if (!applyParams()) {
        closeDevice();
        return false;
    }

    status = MV3D_LP_RegisterImageDataCallBack(
        m_hCamera, ImageCallBackEx, this);
    if (status != MV3D_LP_OK) {
        m_lastError = QString::fromUtf8("注册相机数据回调失败：%1。")
            .arg(statusText(status));
        closeDevice();
        return false;
    }

    return true;
}

void HikCameraWorker::closeDevice()
{
    m_captureState.finish();
    if (m_watchdog) m_watchdog->stop();

    // 空临界区等待掉在途回调退出，之后才能在无锁状态下调用 SDK
    { QMutexLocker barrier(&m_cloudMutex); }

    if (m_hCamera) {
        if (m_measuring) {
            MV3D_LP_StopMeasure(m_hCamera);
            m_measuring = false;
        }
        if (m_deviceOpened) {
            MV3D_LP_CloseDevice(&m_hCamera);
            m_deviceOpened = false;
        }
        m_hCamera = nullptr;
    }

    resetAccumulator();
}

void HikCameraWorker::connectCamera()
{
    const State current = state();
    if (current == State::Connecting ||
        current == State::Connected ||
        current == State::Capturing) {
        return;
    }

    setState(State::Connecting);
    m_lastError.clear();

    if (!ensureSdkInitialized()) {
        setState(State::Fault, m_lastError);
        return;
    }
    if (!openDevice()) {
        setState(State::Fault, m_lastError);
        return;
    }

    setState(State::Connected, deviceSerial());
}

void HikCameraWorker::disconnectCamera()
{
    const State current = state();
    if (current == State::Disconnected) return;

    const bool wasCapturing = (current == State::Capturing);
    closeDevice();
    setState(State::Disconnected);

    if (wasCapturing) {
        emit captureFinished();
    }
}

int HikCameraWorker::captureTimeoutMs() const
{
    if (m_params.captureTimeoutMs > 0) return m_params.captureTimeoutMs;
    // 扫描模式由调用方按扫描时长显式指定超时，这里只兜底
    if (m_params.frameCount <= 0) return kDefaultCaptureTimeoutMs;
    if (m_params.lineRate <= 0.0f) return kDefaultCaptureTimeoutMs;

    // 一批轮廓的行数由设备自行决定（实测 20~50 行），保守按每批 10 行估时
    const int frames = std::max(1, m_params.frameCount);
    const double expected =
        static_cast<double>(frames) * 10.0 / m_params.lineRate * 1000.0;
    return std::max(2000, static_cast<int>(std::ceil(expected)) + 2000);
}

void HikCameraWorker::startCapture()
{
    if (state() != State::Connected) {
        emit captureFailed(QString::fromUtf8("相机未连接，无法开始采集。"));
        return;
    }
    // frameCount == 0 表示持续采集，由 stopCapture() 结束
    if (m_params.frameCount < 0) {
        emit captureFailed(QString::fromUtf8("相机采集帧数无效。"));
        return;
    }

    // 已连接状态下允许调整曝光/帧率，每次开始采集前重新下发
    if (!applyParams()) {
        emit captureFailed(m_lastError);
        return;
    }

    resetAccumulator();
    // 扫描模式：一次采集会收到多帧，必须逐帧输出而不是压平后只交一帧
    m_scanMode = (m_params.frameCount != 1);

    if (!m_captureState.tryBegin()) {
        emit captureFailed(QString::fromUtf8("相机正在采集，请等待当前任务完成。"));
        return;
    }

    if (!m_measuring) {
        const MV3D_LP_STATUS status = MV3D_LP_StartMeasure(m_hCamera);
        if (status != MV3D_LP_OK) {
            m_captureState.finish();
            emit captureFailed(QString::fromUtf8("启动相机测量失败：%1。")
                .arg(statusText(status)));
            return;
        }
        m_measuring = true;
    }

    setState(State::Capturing);
    m_watchdog->start(captureTimeoutMs());
}

void HikCameraWorker::stopCapture()
{
    if (state() != State::Capturing) return;
    finishCapture();
    emit captureFinished();
}

void HikCameraWorker::finishCapture()
{
    // ① 先降 gate，回调立即短路
    m_captureState.finish();
    // ② 等掉在途回调，之后才能在无锁状态下调用 SDK
    { QMutexLocker barrier(&m_cloudMutex); }

    // ③ 默认采完即停流：激光常亮有安全风险，焊接跟踪阶段才置 keepStreaming
    if (!m_params.keepStreaming && m_measuring && m_hCamera) {
        MV3D_LP_StopMeasure(m_hCamera);
        m_measuring = false;
    }

    if (m_watchdog) m_watchdog->stop();
    resetAccumulator();
    setState(State::Connected);
}

void HikCameraWorker::onInternalCaptureDone(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (state() != State::Capturing) return;

    finishCapture();
    emit pointCloudReady(std::move(cloud));
    emit captureFinished();
}

void HikCameraWorker::onInternalScanFrame(ScanFrame::Ptr frame)
{
    if (state() != State::Capturing || !m_scanMode) return;
    emit scanFrameReady(std::move(frame));
}

void HikCameraWorker::onInternalCaptureError(QString errorMessage)
{
    if (state() != State::Capturing) return;

    finishCapture();
    emit captureFailed(std::move(errorMessage));
    emit captureFinished();
}

void HikCameraWorker::onCaptureTimeout()
{
    if (state() != State::Capturing) return;

    const QString message = QString::fromUtf8(
        "采集超时（%1 ms），未收到有效点云。").arg(captureTimeoutMs());
    finishCapture();
    emit captureFailed(message);
    emit captureFinished();
}

void __stdcall HikCameraWorker::ImageCallBackEx(
    MV3D_LP_IMAGE_DATA* imageData, void* user)
{
    auto* self = static_cast<HikCameraWorker*>(user);
    if (!self || !imageData || !self->m_captureState.isActive()) return;

    // 丢包帧标记为无效，跳过并等待下一帧，不中断本次采集
    if (imageData->bValid == FALSE) return;

    // 实测设备在 ImageMode=4 下返回的是 Profile/ABC16（3×int16），
    // 不是 ImageType_PointCloud。按 float32 解读会得到彻底的垃圾数据。
    if (imageData->enImageType != ImageType_Profile) {
        self->m_captureState.finish();
        emit self->internalCaptureError(QString::fromUtf8(
            "相机返回的图像格式为 0x%1，与预期的轮廓数据(ABC16)不符。")
            .arg(static_cast<uint32_t>(imageData->enImageType), 8, 16,
                QLatin1Char('0')));
        return;
    }

    if (!imageData->pData || imageData->nWidth == 0 || imageData->nHeight == 0) {
        return;   // 空帧不致命，等下一批
    }

    const std::size_t cols = imageData->nWidth;
    const std::size_t rows = imageData->nHeight;
    const std::size_t pointCount = cols * rows;
    const std::size_t bytesPerPoint = 3U * sizeof(short);
    if (pointCount * bytesPerPoint > static_cast<std::size_t>(imageData->nDataLen)) {
        self->m_captureState.finish();
        emit self->internalCaptureError(
            QString::fromUtf8("相机返回的轮廓数据长度不足。"));
        return;
    }

    const float unitX = self->m_unitX.load();
    const float unitZ = self->m_unitZ.load();
    if (!(unitX > 0.0f) || !(unitZ > 0.0f)) {
        self->m_captureState.finish();
        emit self->internalCaptureError(
            QString::fromUtf8("缺少坐标换算系数，无法解码轮廓数据。"));
        return;
    }

    // 扫描模式：逐帧交给调用方，由 ScanSession 按行序号拼接 Y 坐标
    if (self->m_scanMode) {
        auto frame = std::make_shared<ScanFrame>();
        frame->hostUs = hostClockUs();
        frame->frameNum = imageData->nFrameNum;
        frame->rows = static_cast<quint32>(rows);
        frame->cols = static_cast<quint32>(cols);
        frame->stepUm = self->m_stepUm.load();
        frame->xMm.resize(pointCount);
        frame->zMm.resize(pointCount);

        const short* src = reinterpret_cast<const short*>(imageData->pData);
        for (std::size_t i = 0; i < pointCount; ++i) {
            const short a = src[i * 3];
            const short c = src[i * 3 + 2];
            if (a == kEmptyPointShort || c == kEmptyPointShort) {
                frame->xMm[i] = std::numeric_limits<float>::quiet_NaN();
                frame->zMm[i] = std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            frame->xMm[i] = static_cast<float>(a) * unitX * 0.001f;   // µm → mm
            frame->zMm[i] = static_cast<float>(c) * unitZ * 0.001f;
        }

        emit self->internalScanFrame(std::move(frame));
        return;
    }

    // 单张模式：把这一批轮廓整理成一幅点云，Y 按行号 × 步距补
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr completedCloud;
    const float stepMm = self->m_stepUm.load() * 0.001f;
    {
        QMutexLocker locker(&self->m_cloudMutex);
        if (!self->m_captureState.isActive()) return;

        const short* src = reinterpret_cast<const short*>(imageData->pData);
        for (std::size_t r = 0; r < rows; ++r) {
            const float y = static_cast<float>(r) * stepMm;
            for (std::size_t c = 0; c < cols; ++c) {
                const std::size_t i = r * cols + c;
                const short a = src[i * 3];
                const short zc = src[i * 3 + 2];
                if (a == kEmptyPointShort || zc == kEmptyPointShort) continue;

                pcl::PointXYZRGB point;
                point.x = static_cast<float>(a) * unitX * 0.001f;
                point.y = y;
                point.z = static_cast<float>(zc) * unitZ * 0.001f;
                if (!isValidPoint(point.x, point.y, point.z)) continue;
                point.r = point.g = point.b = 255;
                self->m_accumulatedCloud->points.push_back(point);
            }
        }

        ++self->m_frameCount;
        if (self->m_frameCount >= std::max(1, self->m_params.frameCount)) {
            self->m_captureState.finish();
            completedCloud = std::move(self->m_accumulatedCloud);
            self->m_accumulatedCloud.reset(
                new pcl::PointCloud<pcl::PointXYZRGB>);
            self->m_frameCount = 0;
        }
    }

    if (!completedCloud) return;

    completedCloud->width = static_cast<std::uint32_t>(
        completedCloud->points.size());
    completedCloud->height = 1;
    completedCloud->is_dense = true;
    emit self->internalCaptureDone(std::move(completedCloud));
}
