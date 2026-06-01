#include "plydirectoryworker.h"

#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QtConcurrent/QtConcurrent>

#include <pcl/io/ply_io.h>

PlyDirectoryWorker::PlyDirectoryWorker(const QString& directoryPath, QObject* parent)
    : QObject(parent),
      directoryPath(directoryPath)
{
    verifyTimer.setInterval(200);
    verifyTimer.setSingleShot(false);
    retryDelayMs = kBaseRetryDelayMs;

    delayTimer.setInterval(5000);
    delayTimer.setSingleShot(true);

    connect(&watcher, &QFileSystemWatcher::directoryChanged,
            this, &PlyDirectoryWorker::onDirectoryChanged);

    connect(&verifyTimer, &QTimer::timeout,
            this, &PlyDirectoryWorker::verifyFileReady);

    connect(&delayTimer, &QTimer::timeout, this, [this]() {
        if (pendingFile.isEmpty() || loading) return;
        startVerify(pendingFile);
    });

    connect(&loadWatcher, &QFutureWatcher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::finished,
            this, &PlyDirectoryWorker::onLoadFinished);
}

void PlyDirectoryWorker::start()
{
    QDir dir(directoryPath);
    if (!dir.exists()) {
        emit statusMessage(QStringLiteral("Monitor directory not found: ") + directoryPath);
        return;
    }

    watcher.addPath(directoryPath);
    emit statusMessage(QStringLiteral("Start monitoring directory: ") + directoryPath);

    startupTime = QDateTime::currentDateTime();
}

void PlyDirectoryWorker::stop()
{
    watcher.removePath(directoryPath);
    verifyTimer.stop();
    delayTimer.stop();
    if (loadWatcher.isRunning()) {
        loadWatcher.cancel();
        loadWatcher.waitForFinished();
    }
    emit statusMessage(QStringLiteral("Directory monitoring stopped."));
}

void PlyDirectoryWorker::onDirectoryChanged(const QString& path)
{
    Q_UNUSED(path);

    const QString latestFile = selectLatestPlyFile(directoryPath);
    if (latestFile.isEmpty()) return;

    if (latestFile == loadingFile) return;

    QFileInfo latestInfo(latestFile);
    if (startupTime.isValid() && latestInfo.lastModified() <= startupTime) {
        return;
    }
    if (!lastLoadedFile.isEmpty()) {
        if (latestInfo.absoluteFilePath() == lastLoadedFile && !lastLoadedTime.isNull()) {
            if (latestInfo.lastModified() == lastLoadedTime && latestInfo.size() == lastLoadedSize) {
                return;
            }
        }
    }

    pendingFile = latestFile;
    retryFile.clear();
    retryCount = 0;
    retryDelayMs = kBaseRetryDelayMs;
    emit statusMessage(QStringLiteral("New point cloud file detected: ") + QFileInfo(latestFile).fileName());

    if (!loading) {
        delayTimer.start();
    }
}

void PlyDirectoryWorker::startVerify(const QString& filePath)
{
    if (filePath.isEmpty()) return;

    pendingFile = filePath;
    lastSize = -1;
    lastModified = QDateTime();
    stableCount = 0;

    if (!verifyTimer.isActive()) {
        verifyTimer.start();
    }
}

void PlyDirectoryWorker::verifyFileReady()
{
    if (pendingFile.isEmpty() || loading) return;

    QFileInfo info(pendingFile);
    info.refresh();
    qint64 currentSize = info.size();

    bool canOpen = false;
    QFile file(pendingFile);
    if (file.open(QIODevice::ReadOnly)) {
        canOpen = true;
        file.close();
    }

    if (!info.exists() || currentSize == 0 || !canOpen) {
        stableCount = 0;
        return;
    }

    const QDateTime currentModified = info.lastModified();

    if (currentSize == lastSize && currentModified == lastModified) {
        stableCount++;
    } else {
        stableCount = 0;
        lastSize = currentSize;
        lastModified = currentModified;
    }

    if (stableCount >= 5) {
        verifyTimer.stop();
        emit statusMessage(QStringLiteral("File is stable, start loading: ") + info.fileName());
        startAsyncLoad(pendingFile);
    }
}

void PlyDirectoryWorker::startAsyncLoad(const QString& filePath)
{
    if (filePath.isEmpty()) return;

    loading = true;
    loadingFile = filePath;
    pendingFile.clear();

    emit statusMessage(QStringLiteral("Loading in background: ") + QFileInfo(filePath).fileName());

    auto future = QtConcurrent::run([filePath]() -> pcl::PointCloud<pcl::PointXYZRGB>::Ptr {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 生成临时文件路径，保证纯粹英文且避免与原文件抢占锁
        QString tempPath = QDir::tempPath() + QStringLiteral("/dianyun_temp_load.ply");

        if (QFile::exists(tempPath)) {
            QFile::remove(tempPath);
        }

        // 尝试复制，如果复制失败，说明系统仍在独占该文件，安全回退到重试流程
        if (!QFile::copy(filePath, tempPath)) {
            return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
        }

        // 让 PCL 读取系统 Temp 目录下的无锁安全副本
        int result = pcl::io::loadPLYFile(tempPath.toUtf8().toStdString(), *cloud);

        // 无论成败，读取结束后立即销毁副本
        QFile::remove(tempPath);

        if (result < 0) {
            return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
        }
        return cloud;
    });

    loadWatcher.setFuture(future);
}

void PlyDirectoryWorker::onLoadFinished()
{
    loading = false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = loadWatcher.result();
    if (!cloud || cloud->empty()) {
        if (pendingFile.isEmpty() || pendingFile == loadingFile) {
            if (retryFile != loadingFile) {
                retryFile = loadingFile;
                retryCount = 0;
            }

            if (retryCount < kMaxRetries) {
                retryCount++;
                emit statusMessage(QStringLiteral("Load failed, retrying: ") + QFileInfo(loadingFile).fileName());
                pendingFile = loadingFile;
                loadingFile.clear();
                const int delayMs = retryDelayMs;
                retryDelayMs = qMin(retryDelayMs * 2, kMaxRetryDelayMs);
                QTimer::singleShot(delayMs, this, [this]() {
                    if (pendingFile.isEmpty() || loading) return;
                    startVerify(pendingFile);
                });
                return;
            }
        }

        const QString reason = QStringLiteral("PCL load failed or cloud is empty");
        emit loadFailed(reason, loadingFile);
    } else {
        lastLoadedFile = loadingFile;
        QFileInfo info(loadingFile);
        lastLoadedTime = info.lastModified();
        lastLoadedSize = info.size();
        retryCount = 0;
        retryFile.clear();
        retryDelayMs = kBaseRetryDelayMs;
        emit cloudLoaded(cloud, loadingFile);
    }

    loadingFile.clear();

    if (!pendingFile.isEmpty() && pendingFile != lastLoadedFile) {
        startVerify(pendingFile);
    }
}

QString PlyDirectoryWorker::selectLatestPlyFile(const QString& path) const
{
    QDir dir(path);
    QFileInfoList files = dir.entryInfoList(QStringList() << "*.ply", QDir::Files);

    if (files.isEmpty()) return QString();

    QFileInfo latest;
    for (const QFileInfo& info : files) {
        if (!latest.exists() || info.lastModified() > latest.lastModified()) {
            latest = info;
        }
    }

    return latest.exists() ? latest.absoluteFilePath() : QString();
}
