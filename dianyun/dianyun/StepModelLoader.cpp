#include "StepModelLoader.h"

#include <STEPControl_Reader.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <pcl/conversions.h>
#include <pcl/common/io.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

bool StepModelLoader::load(
    const QString& filename, CadModelData& output, QString& errorMessage) const
{
    errorMessage.clear();
    STEPControl_Reader reader;
    const IFSelect_ReturnStatus status = reader.ReadFile(filename.toUtf8().constData());
    if (status != IFSelect_RetVoid && status != IFSelect_RetDone) {
        errorMessage = QString::fromUtf8("STEP文件读取失败。");
        return false;
    }

    reader.TransferRoots();
    CadModelData model;
    model.shape = reader.OneShape();
    if (model.shape.IsNull()) {
        errorMessage = QString::fromUtf8("STEP文件中没有有效实体。");
        return false;
    }

    Bnd_Box box;
    BRepBndLib::Add(model.shape, box);
    double xmin = 0.0, ymin = 0.0, zmin = 0.0;
    double xmax = 0.0, ymax = 0.0, zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    model.diagonal = std::sqrt(
        std::pow(xmax - xmin, 2) + std::pow(ymax - ymin, 2) +
        std::pow(zmax - zmin, 2));
    if (!std::isfinite(model.diagonal) || model.diagonal <= 0.0) {
        errorMessage = QString::fromUtf8("STEP模型尺寸无效。");
        return false;
    }

    const double linearDeflection = std::max(model.diagonal * 0.001, 0.01);
    const double sampleSpacing = std::max(model.diagonal * 0.005, 0.25);
    BRepMesh_IncrementalMesh meshBuilder(
        model.shape, linearDeflection, false, 0.5, true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(
        new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<pcl::Vertices> polygons;
    int vertexOffset = 0;
    int faceId = 0;

    for (TopExp_Explorer explorer(model.shape, TopAbs_FACE);
        explorer.More(); explorer.Next(), ++faceId) {
        const TopoDS_Face face = TopoDS::Face(explorer.Current());
        model.faces.push_back(face);

        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull()) continue;

        const gp_Trsf transform = location.Transformation();
        const int nodeCount = triangulation->NbNodes();
        const int triangleCount = triangulation->NbTriangles();

        for (int node = 1; node <= nodeCount; ++node) {
            const gp_Pnt point = triangulation->Node(node).Transformed(transform);
            vertices->push_back(pcl::PointXYZ(
                static_cast<float>(point.X()),
                static_cast<float>(point.Y()),
                static_cast<float>(point.Z())));
        }

        for (int triangle = 1; triangle <= triangleCount; ++triangle) {
            int n1 = 0, n2 = 0, n3 = 0;
            triangulation->Triangle(triangle).Get(n1, n2, n3);

            const gp_Pnt gp1 = triangulation->Node(n1).Transformed(transform);
            const gp_Pnt gp2 = triangulation->Node(n2).Transformed(transform);
            const gp_Pnt gp3 = triangulation->Node(n3).Transformed(transform);
            const Eigen::Vector3d p1(gp1.X(), gp1.Y(), gp1.Z());
            const Eigen::Vector3d p2(gp2.X(), gp2.Y(), gp2.Z());
            const Eigen::Vector3d p3(gp3.X(), gp3.Y(), gp3.Z());
            const double area = 0.5 * (p2 - p1).cross(p3 - p1).norm();
            const int sampleCount = std::clamp(
                static_cast<int>(std::ceil(area / (sampleSpacing * sampleSpacing))),
                1, 1000);

            for (int sample = 0; sample < sampleCount; ++sample) {
                double u = std::fmod((sample + 1) * 0.6180339887498949, 1.0);
                double v = std::fmod((sample + 1) * 0.4142135623730950, 1.0);
                if (u + v > 1.0) {
                    u = 1.0 - u;
                    v = 1.0 - v;
                }
                const Eigen::Vector3d sampled = p1 + u * (p2 - p1) + v * (p3 - p1);
                pcl::PointXYZRGB cadPoint;
                cadPoint.x = static_cast<float>(sampled.x());
                cadPoint.y = static_cast<float>(sampled.y());
                cadPoint.z = static_cast<float>(sampled.z());
                cadPoint.r = cadPoint.g = cadPoint.b = 200;
                model.sampleCloud->push_back(cadPoint);
            }

            pcl::Vertices polygon;
            polygon.vertices = {
                static_cast<pcl::index_t>(vertexOffset + n1 - 1),
                static_cast<pcl::index_t>(vertexOffset + n2 - 1),
                static_cast<pcl::index_t>(vertexOffset + n3 - 1)
            };
            polygons.push_back(polygon);
            model.cellToFaceMap.push_back(faceId);
        }
        vertexOffset += nodeCount;
    }

    pcl::toPCLPointCloud2(*vertices, model.mesh->cloud);
    model.mesh->polygons = std::move(polygons);
    if (model.sampleCloud->empty()) {
        pcl::copyPointCloud(*vertices, *model.sampleCloud);
    }
    if (!model.isValid()) {
        errorMessage = QString::fromUtf8("STEP模型三角化或表面采样失败。");
        return false;
    }

    output = std::move(model);
    return true;
}
