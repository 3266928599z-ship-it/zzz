#include "PointCloudLoadController.h"

#include <QtConcurrent/QtConcurrentRun>

#include <exception>

PointCloudLoadController::PointCloudLoadController(QObject* parent)
    : QObject(parent)
{
    connect(&m_watcher, &QFutureWatcher<AsyncPointCloudLoadResult>::finished,
        this, [this]() {
            const AsyncPointCloudLoadResult result = m_watcher.result();
            if (result.success) {
                emit finished(result.data.cloud, result.data.diagonal);
            }
            else {
                emit failed(result.errorMessage);
            }
            emit busyChanged(false);
        });
}

PointCloudLoadController::~PointCloudLoadController()
{
    if (m_watcher.isRunning()) {
        m_watcher.waitForFinished();
    }
}

bool PointCloudLoadController::isRunning() const
{
    return m_watcher.isRunning();
}

bool PointCloudLoadController::load(const QString& filename)
{
    if (filename.isEmpty() || isRunning()) return false;

    emit busyChanged(true);
    emit progress(QString::fromUtf8("正在后台读取并清理完整点云..."));
    m_watcher.setFuture(QtConcurrent::run([filename]() {
        AsyncPointCloudLoadResult result;
        try {
            PointCloudLoader loader;
            result.success = loader.load(
                filename, result.data, result.errorMessage);
        }
        catch (const std::exception& exception) {
            result.errorMessage = QString::fromUtf8(
                "后台读取点云时发生异常：") +
                QString::fromUtf8(exception.what());
        }
        catch (...) {
            result.errorMessage = QString::fromUtf8(
                "后台读取点云时发生未知异常。");
        }
        return result;
    }));
    return true;
}
