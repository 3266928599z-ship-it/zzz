#pragma once

#include "WeldTypes.h"

#include <TopoDS_Face.hxx>
#include <QString>

class WeldSeamService
{
public:
    bool createSeam(
        const TopoDS_Face& faceA,
        const TopoDS_Face& faceB,
        int seamIndex,
        WeldSeamData& output,
        QString& errorMessage) const;

    bool mapSeam(
        const WeldSeamData& cadSeam,
        const Eigen::Matrix4f& cadToScan,
        WeldSeamData& output,
        QString& errorMessage) const;
};
