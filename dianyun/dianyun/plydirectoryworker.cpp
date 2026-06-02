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
    delayTimer.setInterval(20000);
    delayTimer.setSingleShot(true);

    connect(&watcher, &QFileSystemWatcher::directoryChanged,
            this, &PlyDirectoryWorker::onDirectoryChanged);

    connect(&delayTimer, &QTimer::timeout, this, [this]() {
        if (pendingFile.isEmpty() || loading) return;

        QFileInfo info(pendingFile);
        if (!info.exists()) {
            return;
        }

        constexpr qint64 kMinAgeMs = 20000;
        const qint64 ageMs = info.lastModified().msecsTo(QDateTime::currentDateTime());
        if (ageMs < kMinAgeMs) {
            delayTimer.start(static_cast<int>(kMinAgeMs - ageMs));
            return;
        }

        bool hasHeader = false;
        QFile file(pendingFile);
        if (file.open(QIODevice::ReadOnly)) {
            const QByteArray header = file.read(65536);
            hasHeader = header.contains("end_header");
            file.close();
        }

        if (!hasHeader) {
            delayTimer.start(2000);
            return;
        }

        startAsyncLoad(pendingFile);
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
    emit statusMessage(QStringLiteral("New point cloud file detected: ") + QFileInfo(latestFile).fileName());

    if (!loading) {
        delayTimer.start();
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
        if (pcl::io::loadPLYFile(filePath.toLocal8Bit().toStdString(), *cloud) < 0) {
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
        const QString reason = QStringLiteral("PCL load failed or cloud is empty");
        emit loadFailed(reason, loadingFile);
    } else {
        lastLoadedFile = loadingFile;
        QFileInfo info(loadingFile);
        lastLoadedTime = info.lastModified();
        lastLoadedSize = info.size();
        emit cloudLoaded(cloud, loadingFile);
    }

    loadingFile.clear();

    if (!pendingFile.isEmpty() && pendingFile != lastLoadedFile) {
        delayTimer.start();
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
