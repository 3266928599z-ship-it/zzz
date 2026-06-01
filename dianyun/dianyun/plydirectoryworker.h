#pragma once
#pragma once

#include <QtCore/qobjectdefs.h>
#include <QtCore/QObject>
#include <QtCore/QFileSystemWatcher>
#include <QtCore/QTimer>
#include <QtCore/QFutureWatcher>
#include <QtCore/QtGlobal>
#include <QtCore/QString>
#include <QtCore/QDateTime>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// 目录监控 + 文件校验 + 异步加载的独立工作类
class PlyDirectoryWorker : public QObject
{
    Q_OBJECT

public:
    explicit PlyDirectoryWorker(const QString& directoryPath, QObject* parent = nullptr);

    void start();
    void stop();

Q_SIGNALS:
    void statusMessage(const QString& message);
    void cloudLoaded(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const QString& filePath);
    void loadFailed(const QString& reason, const QString& filePath);

private Q_SLOTS:
    void onDirectoryChanged(const QString& path);
    void verifyFileReady();
    void onLoadFinished();

private:
    QString selectLatestPlyFile(const QString& path) const;
    void startVerify(const QString& filePath);
    void startAsyncLoad(const QString& filePath);
    QFileSystemWatcher watcher;
    QTimer verifyTimer;
    QTimer delayTimer;
    QFutureWatcher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> loadWatcher;

    QString directoryPath;
    QString pendingFile;
    QString loadingFile;
    QString lastLoadedFile;
    QString retryFile;

    QDateTime startupTime;

    qint64 lastSize = -1;
    QDateTime lastModified;
    int stableCount = 0;
    bool loading = false;
    int retryCount = 0;
    int retryDelayMs = 0;

    static constexpr int kMaxRetries = 5;
    static constexpr int kBaseRetryDelayMs = 500;
    static constexpr int kMaxRetryDelayMs = 8000;

    qint64 lastLoadedSize = -1;
    QDateTime lastLoadedTime;
};
