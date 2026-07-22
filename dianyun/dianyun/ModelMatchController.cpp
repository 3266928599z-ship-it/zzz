#include "ModelMatchController.h"

#include "ModelMatchWorker.h"

#include <utility>

ModelMatchController::ModelMatchController(QObject* parent)
    : QObject(parent),
      m_thread(new QThread(this)),
      m_worker(new ModelMatchWorker())
{
    m_worker->moveToThread(m_thread);
    connect(m_thread, &QThread::started, m_worker, &ModelMatchWorker::run);
    connect(m_worker, &ModelMatchWorker::progress,
        this, [this](QString stage) {
            emit progress(m_requestId, std::move(stage));
        });
    connect(m_worker, &ModelMatchWorker::processedScanReady,
        this, [this](CloudRGB::Ptr cloud) {
            emit processedScanReady(m_requestId, std::move(cloud));
        });
    connect(m_worker, &ModelMatchWorker::finished,
        this, [this](MatchResult result) {
            emit finished(m_requestId, std::move(result));
        });
    connect(m_worker, &ModelMatchWorker::finished,
        m_thread, &QThread::quit);
    connect(m_thread, &QThread::started, this, [this]() {
        emit busyChanged(true);
    });
    connect(m_thread, &QThread::finished, this, [this]() {
        emit busyChanged(false);
    });
}

ModelMatchController::~ModelMatchController()
{
    if (m_thread->isRunning()) {
        m_thread->quit();
        m_thread->wait();
    }
    delete m_worker;
}

bool ModelMatchController::isRunning() const
{
    return m_thread && m_thread->isRunning();
}

bool ModelMatchController::start(
    quint64 requestId,
    const CloudRGB::Ptr& cadCloud,
    const CloudRGB::Ptr& scanCloud,
    const MatchConfig& config)
{
    if (requestId == 0 || isRunning() || !cadCloud || cadCloud->empty() ||
        !scanCloud || scanCloud->empty()) {
        return false;
    }

    // 主窗口导入新数据时会替换智能指针，不会原地修改旧点云。
    // 因此匹配线程可以安全共享只读点云，避免在主线程复制数百万点。
    CloudRGB::Ptr cadCopy = cadCloud;
    CloudRGB::Ptr scanCopy = scanCloud;
    m_requestId = requestId;
    m_worker->setInput(cadCopy, scanCopy, config);
    m_thread->start();
    return true;
}
