#pragma once

#include "WeldPathPlanner.h"

#include <QObject>
#include <QString>

class RobotWorker;

// 逐点下发 MoveL 执行一条 TCP 轨迹（工具 1、基坐标）。
// 每段等 RobotWorker::moveFinished 到位后再发下一段；失败或取消即停机。
// 「移到点选位置上方」也走这里，只是轨迹只有一个点。
class WeldPathController : public QObject
{
    Q_OBJECT
public:
    struct Speeds {
        float travelPercent = 10.0f;   // 接近 / 下降 / 退出
        float seamPercent = 5.0f;      // 沿焊缝
        float accPercent = 20.0f;
    };

    WeldPathController(RobotWorker* robot, QObject* parent = nullptr);

    bool isRunning() const { return m_running; }

public slots:
    void start(std::vector<WeldWaypoint> waypoints, Speeds speeds,
        QString description);
    void cancel();

signals:
    void progress(QString message);
    void finished();
    void failed(QString message);

private slots:
    void onMoveFinished(bool ok, QString error);

private:
    void issueNext();
    void fail(const QString& message);

    RobotWorker* m_robot = nullptr;
    std::vector<WeldWaypoint> m_waypoints;
    Speeds m_speeds;
    QString m_description;
    std::size_t m_next = 0;
    bool m_running = false;
    bool m_moveInFlight = false;
};
