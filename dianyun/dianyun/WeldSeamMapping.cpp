#include "WeldSeamService.h"

#include <cmath>
#include <utility>

namespace {

bool isRigidTransform(const Eigen::Matrix4f& transform)
{
    if (!transform.allFinite()) return false;

    const Eigen::Vector4f expectedBottomRow(0.0f, 0.0f, 0.0f, 1.0f);
    if (!transform.row(3).transpose().isApprox(expectedBottomRow, 1e-4f)) {
        return false;
    }

    const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    return (rotation.transpose() * rotation)
        .isApprox(Eigen::Matrix3f::Identity(), 1e-3f) &&
        std::abs(rotation.determinant() - 1.0f) <= 1e-3f;
}

Eigen::Vector3f transformPosition(
    const Eigen::Matrix4f& transform, const Eigen::Vector3f& position)
{
    return (transform * position.homogeneous()).head<3>();
}

}

bool WeldSeamService::mapSeam(
    const WeldSeamData& cadSeam,
    const Eigen::Matrix4f& cadToScan,
    WeldSeamData& output,
    QString& errorMessage) const
{
    errorMessage.clear();
    if (!cadSeam.points || cadSeam.points->empty() ||
        cadSeam.segments.empty()) {
        errorMessage = QString::fromUtf8("选中的焊缝没有可映射的几何数据。");
        return false;
    }
    if (!isRigidTransform(cadToScan)) {
        errorMessage = QString::fromUtf8(
            "模型匹配变换不是有效的刚性变换，无法映射焊缝。");
        return false;
    }

    WeldSeamData mapped;
    mapped.id = cadSeam.id;
    mapped.name = cadSeam.name;
    mapped.length = cadSeam.length;
    mapped.points->reserve(cadSeam.points->size());
    mapped.segments.reserve(cadSeam.segments.size());

    for (const auto& sourcePoint : cadSeam.points->points) {
        const Eigen::Vector3f sourcePosition = sourcePoint.getVector3fMap();
        if (!sourcePosition.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝采样点包含无效坐标。");
            return false;
        }

        const Eigen::Vector3f mappedPosition =
            transformPosition(cadToScan, sourcePosition);
        if (!mappedPosition.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝采样点映射后坐标无效。");
            return false;
        }

        pcl::PointXYZRGB mappedPoint = sourcePoint;
        mappedPoint.x = mappedPosition.x();
        mappedPoint.y = mappedPosition.y();
        mappedPoint.z = mappedPosition.z();
        mapped.points->push_back(mappedPoint);
    }

    for (const auto& sourceSegment : cadSeam.segments) {
        if (!sourceSegment.start.allFinite() ||
            !sourceSegment.end.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝线段包含无效坐标。");
            return false;
        }

        WeldSeamSegment mappedSegment;
        mappedSegment.start = transformPosition(cadToScan, sourceSegment.start);
        mappedSegment.end = transformPosition(cadToScan, sourceSegment.end);
        if (!mappedSegment.start.allFinite() ||
            !mappedSegment.end.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝线段映射后坐标无效。");
            return false;
        }
        mapped.segments.push_back(mappedSegment);
    }

    output = std::move(mapped);
    return true;
}
