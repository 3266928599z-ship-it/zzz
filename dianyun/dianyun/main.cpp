#include "dianyun.h"
#include <QtWidgets/QApplication>

// === 第一步：添加这三个必要的头文件 ===
#include <QMetaType>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);

    // === 第二步：在这里注册 PCL 和 Eigen 的自定义类型 ===
    // 必须放在 QApplication 创建之后，主窗口 dianyun 实例化之前！
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("PointCloudSource::Ptr");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");

    dianyun w;
    w.show();
    return a.exec();
}