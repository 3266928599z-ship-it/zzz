#pragma once

#include "AppTypes.h"

#include <QString>

class StepModelLoader
{
public:
    bool load(const QString& filename, CadModelData& output, QString& errorMessage) const;
};
