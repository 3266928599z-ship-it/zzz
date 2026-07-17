#pragma once

#include "AppTypes.h"

#include <QString>

struct PointCloudFileData {
    CloudRGB::Ptr cloud;
    double diagonal = 0.0;
};

class PointCloudLoader
{
public:
    bool load(
        const QString& filename,
        PointCloudFileData& output,
        QString& errorMessage) const;
};
