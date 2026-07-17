#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

struct CadModelData {
    TopoDS_Shape shape;
    std::vector<TopoDS_Face> faces;
    std::vector<int> cellToFaceMap;
    pcl::PolygonMesh::Ptr mesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());
    CloudRGB::Ptr sampleCloud = CloudRGB::Ptr(new CloudRGB());
    double diagonal = 0.0;

    bool isValid() const
    {
        return !shape.IsNull() && mesh && !mesh->polygons.empty() &&
            sampleCloud && !sampleCloud->empty();
    }
};
