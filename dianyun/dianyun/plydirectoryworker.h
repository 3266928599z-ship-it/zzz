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


// Directory monitoring + delayed async loading
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
    void onLoadFinished();

private:
    QString selectLatestPlyFile(const QString& path) const;
    void startAsyncLoad(const QString& filePath);
    QFileSystemWatcher watcher;
    QTimer delayTimer;
    QFutureWatcher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> loadWatcher;

    QString directoryPath;
    QString pendingFile;
    QString loadingFile;
    QString lastLoadedFile;

    QDateTime startupTime;

    bool loading = false;
    qint64 lastLoadedSize = -1;
    QDateTime lastLoadedTime;
};
