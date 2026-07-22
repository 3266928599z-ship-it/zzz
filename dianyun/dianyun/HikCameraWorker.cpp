#include "HikCameraWorker.h"

#include <QMutexLocker>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

HikCameraWorker::HikCameraWorker(QObject* parent)
    : QObject(parent), m_hCamera(nullptr), m_frameCount(0)
{
    m_accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    m_params.runSpeed = 1.0f;
    m_params.xRes = 0.096f;
    m_params.yRes = 0.096f;
    m_params.targetLen = 50.0f;
    m_params.zScale = 100.0f;
}

HikCameraWorker::~HikCameraWorker()
{
    closeCamera();
}

bool HikCameraWorker::isCapturing() const
{
    return m_captureState.isActive();
}

QString HikCameraWorker::lastError() const
{
    return m_lastError;
}

void HikCameraWorker::releaseCameraHandle()
{
    m_captureState.finish();
    if (!m_hCamera) {
        m_deviceOpened = false;
        m_streamStarted = false;
        return;
    }

    if (m_streamStarted) {
        MV_CC_StopGrabbing(m_hCamera);
    }
    if (m_deviceOpened) {
        MV_CC_CloseDevice(m_hCamera);
    }
    MV_CC_DestroyHandle(m_hCamera);
    m_hCamera = nullptr;
    m_deviceOpened = false;
    m_streamStarted = false;
}

bool HikCameraWorker::openCamera()
{
    m_lastError.clear();
    if (m_captureState.isActive()) {
        m_lastError = QString::fromUtf8("相机正在采集，请等待当前任务完成。");
        return false;
    }
    if (m_params.xRes <= 0.0f || m_params.yRes <= 0.0f ||
        m_params.targetLen <= 0.0f || m_params.zScale == 0.0f) {
        m_lastError = QString::fromUtf8("相机扫描参数无效。");
        return false;
    }

    {
        QMutexLocker locker(&m_cloudMutex);
        m_accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        m_frameCount = 0;
    }

    if (!m_hCamera) {
        MV_CC_DEVICE_INFO_LIST deviceList;
        memset(&deviceList, 0, sizeof(deviceList));
        int result = MV_CC_EnumDevices(
            MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
        if (result != MV_OK || deviceList.nDeviceNum == 0) {
            m_lastError = QString::fromUtf8("没有检测到可用的海康3D相机。");
            return false;
        }

        result = MV_CC_CreateHandle(&m_hCamera, deviceList.pDeviceInfo[0]);
        if (result != MV_OK || !m_hCamera) {
            m_hCamera = nullptr;
            m_lastError = QString::fromUtf8("创建相机句柄失败，错误码：%1。").arg(result);
            return false;
        }

        result = MV_CC_OpenDevice(m_hCamera);
        if (result != MV_OK) {
            m_lastError = QString::fromUtf8("打开相机失败，错误码：%1。").arg(result);
            releaseCameraHandle();
            return false;
        }
        m_deviceOpened = true;

        result = MV_CC_RegisterImageCallBackEx(
            m_hCamera, ImageCallBackEx, this);
        if (result != MV_OK) {
            m_lastError = QString::fromUtf8("注册相机回调失败，错误码：%1。").arg(result);
            releaseCameraHandle();
            return false;
        }

        result = MV_CC_SetEnumValue(m_hCamera, "TriggerMode", 0);
        if (result != MV_OK) {
            m_lastError = QString::fromUtf8("设置相机触发模式失败，错误码：%1。").arg(result);
            releaseCameraHandle();
            return false;
        }

        if (!m_captureState.tryBegin()) {
            m_lastError = QString::fromUtf8("相机正在采集，请等待当前任务完成。");
            return false;
        }
        result = MV_CC_StartGrabbing(m_hCamera);
        if (result != MV_OK) {
            m_lastError = QString::fromUtf8("启动相机数据流失败，错误码：%1。").arg(result);
            releaseCameraHandle();
            return false;
        }
        m_streamStarted = true;
        return true;
    }

    // 数据流保持开启，新的单次采集只需重新接收回调数据。
    if (!m_captureState.tryBegin()) {
        m_lastError = QString::fromUtf8("相机正在采集，请等待当前任务完成。");
        return false;
    }
    return true;
}

void HikCameraWorker::closeCamera()
{
    releaseCameraHandle();
    QMutexLocker locker(&m_cloudMutex);
    if (m_accumulatedCloud) m_accumulatedCloud->clear();
    m_frameCount = 0;
}

void __stdcall HikCameraWorker::ImageCallBackEx(
    unsigned char* data, MV_FRAME_OUT_INFO_EX* frameInfo, void* user)
{
    auto* self = static_cast<HikCameraWorker*>(user);
    if (!self || !data || !frameInfo || !self->m_captureState.isActive()) return;

    constexpr unsigned int coordinatePixelType = 0x022000BB;
    if (frameInfo->enPixelType != coordinatePixelType) {
        self->m_captureState.finish();
        emit self->captureFailed(QString::fromUtf8("相机返回了不支持的坐标数据格式。"));
        emit self->captureFinished();
        return;
    }

    const std::size_t expectedBytes = static_cast<std::size_t>(frameInfo->nWidth) *
        static_cast<std::size_t>(frameInfo->nHeight) * 2U * sizeof(short);
    if (frameInfo->nFrameLen < expectedBytes) {
        self->m_captureState.finish();
        emit self->captureFailed(QString::fromUtf8("相机数据帧长度不足。"));
        emit self->captureFinished();
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr completedCloud;
    {
        QMutexLocker locker(&self->m_cloudMutex);
        if (!self->m_captureState.isActive()) return;

        const int targetFrames = std::max(1, static_cast<int>(
            self->m_params.targetLen / self->m_params.yRes));
        const int width = static_cast<int>(frameInfo->nWidth);
        const short* data16 = reinterpret_cast<const short*>(data);

        for (unsigned int row = 0;
            row < frameInfo->nHeight && self->m_frameCount < targetFrames;
            ++row) {
            const float currentY = self->m_frameCount * self->m_params.yRes;
            const short* lineX = data16 + static_cast<std::size_t>(row) * width * 2;
            const short* lineZ = lineX + width;

            for (int column = 0; column < width; ++column) {
                const short rawX = lineX[column];
                const short rawZ = lineZ[column];
                if (rawZ <= -30000 || rawZ >= 30000) continue;

                pcl::PointXYZRGB point;
                point.x = static_cast<float>(rawX - width / 2) * self->m_params.xRes;
                point.y = currentY;
                point.z = static_cast<float>(rawZ) / self->m_params.zScale;
                point.r = point.g = point.b = 255;
                self->m_accumulatedCloud->points.push_back(point);
            }
            ++self->m_frameCount;
        }

        if (self->m_frameCount >= targetFrames) {
            self->m_captureState.finish();
            completedCloud = std::move(self->m_accumulatedCloud);
            self->m_accumulatedCloud.reset(
                new pcl::PointCloud<pcl::PointXYZRGB>);
            self->m_frameCount = 0;
        }
    }

    if (completedCloud) {
        completedCloud->width = static_cast<std::uint32_t>(
            completedCloud->points.size());
        completedCloud->height = 1;
        completedCloud->is_dense = true;
        emit self->pointCloudReady(std::move(completedCloud));
        emit self->captureFinished();
    }
}
