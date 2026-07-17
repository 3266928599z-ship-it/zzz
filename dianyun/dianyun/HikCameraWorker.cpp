#include "HikCameraWorker.h"
#include <pcl/common/io.h>

HikCameraWorker::HikCameraWorker(QObject* parent)
    : QObject(parent), m_hCamera(nullptr), m_frameCount(0), m_isScanning(false)
{
    m_accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 初始化默认物理参数
    m_params.runSpeed = 1.0f;
    m_params.xRes = 0.096f;
    m_params.yRes = 0.096f;
    m_params.targetLen = 50.0f;
    m_params.zScale = 100.0f;
}

HikCameraWorker::~HikCameraWorker() {
    closeCamera();
}

bool HikCameraWorker::openCamera() {
    if (m_hCamera != nullptr) {
        m_accumulatedCloud->clear();
        m_frameCount = 0;
        m_isScanning = true;
        return true;
    }

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) return false;

    nRet = MV_CC_CreateHandle(&m_hCamera, stDeviceList.pDeviceInfo[0]);
    nRet = MV_CC_OpenDevice(m_hCamera);
    if (nRet != MV_OK) return false;

    MV_CC_RegisterImageCallBackEx(m_hCamera, ImageCallBackEx, this);
    MV_CC_SetEnumValue(m_hCamera, "TriggerMode", 0);
    nRet = MV_CC_StartGrabbing(m_hCamera);

    if (nRet == MV_OK) {
        m_isScanning = true;
        return true;
    }
    return false;
}

void HikCameraWorker::closeCamera() {
    if (m_hCamera) {
        m_isScanning = false;
        MV_CC_StopGrabbing(m_hCamera);
        MV_CC_CloseDevice(m_hCamera);
        MV_CC_DestroyHandle(m_hCamera);
        m_hCamera = nullptr;
    }
}

void __stdcall HikCameraWorker::ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    HikCameraWorker* pThis = static_cast<HikCameraWorker*>(pUser);
    if (!pThis || !pThis->m_isScanning) return;

    if (pFrameInfo->enPixelType == 0x022000BB)
    {
        // 自动计算本次扫描所需的总帧数
        int targetFrames = static_cast<int>(pThis->m_params.targetLen / pThis->m_params.yRes);
        int width = pFrameInfo->nWidth;
        short* pData16 = (short*)pData;

        for (int h = 0; h < pFrameInfo->nHeight; ++h) {
            float currentY = pThis->m_frameCount * pThis->m_params.yRes;
            short* pLineX = pData16 + h * (width * 2);
            short* pLineZ = pLineX + width;

            for (int w = 0; w < width; ++w) {
                short rawX = pLineX[w];
                short rawZ = pLineZ[w];

                if (rawZ > -30000 && rawZ < 30000) {
                    pcl::PointXYZRGB pt;
                    // 使用已初始化的物理参数进行空间重构
                    pt.x = static_cast<float>(rawX - width / 2) * pThis->m_params.xRes;
                    pt.y = currentY;
                    pt.z = static_cast<float>(rawZ) / pThis->m_params.zScale;

                    pt.r = 255; pt.g = 255; pt.b = 255;
                    pThis->m_accumulatedCloud->points.push_back(pt);
                }
            }
            pThis->m_frameCount++;
        }

        // 行程满足后触发停止
        if (pThis->m_frameCount >= targetFrames) {
            pThis->m_isScanning = false;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToUI(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*(pThis->m_accumulatedCloud), *cloudToUI);
            emit pThis->pointCloudReady(cloudToUI);

            pThis->m_accumulatedCloud->clear();
            pThis->m_frameCount = 0;
            emit pThis->captureFinished();
        }
    }
}