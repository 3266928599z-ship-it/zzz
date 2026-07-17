#include "ModelMatchController.h"

#include "ModelMatchWorker.h"

ModelMatchController::ModelMatchController(QObject* parent)
    : QObject(parent),
      m_thread(new QThread(this)),
      m_worker(new ModelMatchWorker())
{
    m_worker->moveToThread(m_thread);
    connect(m_thread, &QThread::started, m_worker, &ModelMatchWorker::run);
    connect(m_worker, &ModelMatchWorker::progress,
        this, &ModelMatchController::progress);
    connect(m_worker, &ModelMatchWorker::processedScanReady,
        this, &ModelMatchController::processedScanReady);
    connect(m_worker, &ModelMatchWorker::finished,
        this, &ModelMatchController::finished);
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
    const CloudRGB::Ptr& cadCloud,
    const CloudRGB::Ptr& scanCloud,
    const std::vector<WeldSeamData>& seams,
    const MatchConfig& config)
{
    if (isRunning() || !cadCloud || cadCloud->empty() ||
        !scanCloud || scanCloud->empty()) {
        return false;
    }

    // 主窗口导入新数据时会替换智能指针，不会原地修改旧点云。
    // 因此匹配线程可以安全共享只读点云，避免在主线程复制数百万点。
    CloudRGB::Ptr cadCopy = cadCloud;
    CloudRGB::Ptr scanCopy = scanCloud;
    std::vector<CloudRGB::Ptr> seamPoints;
    std::vector<QString> seamNames;
    seamPoints.reserve(seams.size());
    seamNames.reserve(seams.size());
    for (const auto& seam : seams) {
        if (!seam.points) continue;
        seamPoints.push_back(CloudRGB::Ptr(new CloudRGB(*seam.points)));
        seamNames.push_back(seam.name);
    }

    m_worker->setInput(cadCopy, scanCopy, seamPoints, seamNames, config);
    m_thread->start();
    return true;
}
