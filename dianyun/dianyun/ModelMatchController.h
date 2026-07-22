#pragma once

#include "AppTypes.h"
#include "MatchTypes.h"

#include <QObject>
#include <QThread>
#include <QtTypes>

class ModelMatchWorker;

class ModelMatchController : public QObject
{
    Q_OBJECT

public:
    explicit ModelMatchController(QObject* parent = nullptr);
    ~ModelMatchController() override;

    bool isRunning() const;
    bool start(
        quint64 requestId,
        const CloudRGB::Ptr& cadCloud,
        const CloudRGB::Ptr& scanCloud,
        const MatchConfig& config = MatchConfig());

signals:
    void progress(quint64 requestId, QString stage);
    void processedScanReady(quint64 requestId, CloudRGB::Ptr cloud);
    void finished(quint64 requestId, MatchResult result);
    void busyChanged(bool busy);

private:
    QThread* m_thread = nullptr;
    ModelMatchWorker* m_worker = nullptr;
    quint64 m_requestId = 0;
};
