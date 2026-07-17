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
};
