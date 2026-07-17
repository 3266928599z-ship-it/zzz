#include "PointCloudLoader.h"

#include <QFileInfo>

#include <pcl/common/point_tests.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <algorithm>
#include <cstdint>

bool PointCloudLoader::load(
    const QString& filename,
    PointCloudFileData& output,
    QString& errorMessage) const
{
    errorMessage.clear();
    CloudRGB::Ptr loaded(new CloudRGB());
    const QString extension = QFileInfo(filename).suffix().toLower();
    const std::string localPath = filename.toLocal8Bit().toStdString();

    int result = -1;
    if (extension == QStringLiteral("pcd")) {
        result = pcl::io::loadPCDFile(localPath, *loaded);
    }
    else if (extension == QStringLiteral("ply")) {
        result = pcl::io::loadPLYFile(localPath, *loaded);
    }
    else {
        errorMessage = QString::fromUtf8("不支持的点云格式：") + extension;
        return false;
    }

    if (result < 0) {
        errorMessage = QString::fromUtf8("点云文件读取失败。");
        return false;
    }

    loaded->points.erase(
        std::remove_if(loaded->points.begin(), loaded->points.end(),
            [](const pcl::PointXYZRGB& point) {
                return !pcl::isFinite(point) ||
                    point.getVector3fMap().squaredNorm() < 1e-12f;
            }),
        loaded->points.end());
    loaded->width = static_cast<std::uint32_t>(loaded->points.size());
    loaded->height = 1;
    loaded->is_dense = true;

    if (loaded->empty()) {
        errorMessage = QString::fromUtf8("点云中没有有效三维点。");
        return false;
    }

    pcl::PointXYZRGB minimum, maximum;
    pcl::getMinMax3D(*loaded, minimum, maximum);
    output.cloud = std::move(loaded);
    output.diagonal = static_cast<double>(
        (maximum.getVector3fMap() - minimum.getVector3fMap()).norm());
    return true;
}
