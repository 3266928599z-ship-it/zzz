#pragma once
#include <QObject>
#include <QDebug>
#include <atomic>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "MvCameraControl.h"

Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)

// 物理参数配置结构体
struct ScanParams {
    float runSpeed = 1.0f;        // 运行速度 (mm/s)
    float xRes = 0.096f;          // X方向单像素精度 (mm)
    float yRes = 0.096f;          // Y方向单像素精度 (mm)
    float targetLen = 10.0f;      // 扫描行程长度 (mm)
    float zScale = 100.0f;        // 底层缩放因子
};

class HikCameraWorker : public QObject
{
    Q_OBJECT
public:
    explicit HikCameraWorker(QObject* parent = nullptr);
    ~HikCameraWorker();

    bool openCamera();
    void closeCamera();

    // 将参数暴露为公有，方便主界面动态修改
    ScanParams m_params;

signals:
    void pointCloudReady(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void captureFinished();

private:
    void* m_hCamera;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_accumulatedCloud;
    int m_frameCount;
    std::atomic<bool> m_isScanning;

    static void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
};