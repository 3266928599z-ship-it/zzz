#include "WeldSeamService.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <algorithm>
#include <cmath>

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
