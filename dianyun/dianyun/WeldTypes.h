#pragma once

#include "AppTypes.h"

#include <Eigen/Core>
#include <QString>
#include <string>
#include <vector>

struct WeldSeamSegment {
    Eigen::Vector3f start = Eigen::Vector3f::Zero();
    Eigen::Vector3f end = Eigen::Vector3f::Zero();
};

struct WeldSeamData {
    std::string id;
    QString name;
    CloudRGB::Ptr points = CloudRGB::Ptr(new CloudRGB());
    std::vector<WeldSeamSegment> segments;
    double length = 0.0;
    int faceA = -1;
    int faceB = -1;
    // 起焊方向是否已被显式设定。真实起点恒为 points[0]，
    // 该标志只用于界面提示，避免重复存储坐标。
    bool hasStart = false;
};
