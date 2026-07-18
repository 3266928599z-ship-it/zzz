#include "WeldSeamService.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <algorithm>
#include <cmath>

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

bool WeldSeamService::createSeam(
    const TopoDS_Face& faceA,
    const TopoDS_Face& faceB,
    int seamIndex,
    WeldSeamData& output,
    QString& errorMessage) const
{
    errorMessage.clear();
    BRepAlgoAPI_Section section(faceA, faceB, Standard_False);
    section.Build();
    if (!section.IsDone()) {
        errorMessage = QString::fromUtf8("面相交计算失败，未生成焊缝线。");
        return false;
    }

    const TopoDS_Shape seamShape = section.Shape();
    if (seamShape.IsNull()) {
        errorMessage = QString::fromUtf8("两个选定面之间没有检测到交线。");
        return false;
    }

    WeldSeamData seam;
    seam.id = std::to_string(seamIndex);
    seam.name = QString::fromUtf8("焊缝-") + QString::number(seamIndex);

    for (TopExp_Explorer edgeExplorer(seamShape, TopAbs_EDGE);
        edgeExplorer.More(); edgeExplorer.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
        BRepAdaptor_Curve curve(edge);
        const double first = curve.FirstParameter();
        const double last = curve.LastParameter();
        if (last <= first) continue;

        const double length = GCPnts_AbscissaPoint::Length(curve, first, last);
        if (!std::isfinite(length) || length <= 0.0) continue;
        seam.length += length;
        const int sampleCount = std::max(static_cast<int>(length / 1.5), 8);

        Eigen::Vector3f previous = Eigen::Vector3f::Zero();
        bool hasPrevious = false;
        for (int sample = 0; sample <= sampleCount; ++sample) {
            const double parameter = first + (last - first) *
                static_cast<double>(sample) / static_cast<double>(sampleCount);
            const gp_Pnt point = curve.Value(parameter);
            const Eigen::Vector3f current(
                static_cast<float>(point.X()),
                static_cast<float>(point.Y()),
                static_cast<float>(point.Z()));

            pcl::PointXYZRGB seamPoint;
            seamPoint.x = current.x();
            seamPoint.y = current.y();
            seamPoint.z = current.z();
            seamPoint.r = 0;
            seamPoint.g = 255;
            seamPoint.b = 0;
            seam.points->push_back(seamPoint);

            if (hasPrevious) {
                seam.segments.push_back({ previous, current });
            }
            previous = current;
            hasPrevious = true;
        }
    }

    if (seam.segments.empty() || seam.points->empty()) {
        errorMessage = QString::fromUtf8("检测到面关系，但没有生成可用焊缝点。");
        return false;
    }

    output = std::move(seam);
    return true;
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
