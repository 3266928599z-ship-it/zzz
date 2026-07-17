#pragma once

#include "AppTypes.h"
#include "PointCloudLoader.h"

#include <QObject>
#include <QFutureWatcher>
#include <QString>

struct AsyncPointCloudLoadResult {
    PointCloudFileData data;
    QString errorMessage;
    bool success = false;
};

class PointCloudLoadController : public QObject
{
    Q_OBJECT

public:
    explicit PointCloudLoadController(QObject* parent = nullptr);
    ~PointCloudLoadController() override;

    bool isRunning() const;
    bool load(const QString& filename);

signals:
    void progress(QString message);
    void finished(CloudRGB::Ptr cloud, double diagonal);
    void failed(QString errorMessage);
    void busyChanged(bool busy);

private:
    QFutureWatcher<AsyncPointCloudLoadResult> m_watcher;
};
