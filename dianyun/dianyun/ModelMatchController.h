#pragma once

#include "AppTypes.h"
#include "MatchTypes.h"
#include "WeldTypes.h"

#include <QObject>
#include <QThread>

class ModelMatchWorker;

class ModelMatchController : public QObject
{
    Q_OBJECT

public:
    explicit ModelMatchController(QObject* parent = nullptr);
    ~ModelMatchController() override;

    bool isRunning() const;
    bool start(
        const CloudRGB::Ptr& cadCloud,
        const CloudRGB::Ptr& scanCloud,
        const std::vector<WeldSeamData>& seams,
        const MatchConfig& config = MatchConfig());

signals:
    void progress(QString stage);
    void processedScanReady(CloudRGB::Ptr cloud);
    void finished(MatchResult result);
    void busyChanged(bool busy);

private:
    QThread* m_thread = nullptr;
    ModelMatchWorker* m_worker = nullptr;
};
