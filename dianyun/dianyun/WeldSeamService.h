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

    // 把离 anchor 最近的焊缝链端点设为起焊点，并重排点序/段序。
    // tolerance 为 anchor 到焊缝的最大允许距离，超出则拒绝修改。
    bool setStartAt(
        const WeldSeamData& input,
        const Eigen::Vector3f& anchor,
        double tolerance,
        WeldSeamData& output,
        QString& errorMessage) const;

    // 反转整条焊缝的焊接方向（首尾互换）。
    bool reverseDirection(
        const WeldSeamData& input,
        WeldSeamData& output,
        QString& errorMessage) const;
};
