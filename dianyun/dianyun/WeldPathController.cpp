#include "WeldPathController.h"

#include "RobotWorker.h"

#include <algorithm>

namespace {
constexpr int kMoveTimeoutMs = 60000;

QString kindName(WeldWaypoint::Kind kind)
{
    switch (kind) {
    case WeldWaypoint::Kind::Approach: return QString::fromUtf8("接近");
    case WeldWaypoint::Kind::Descend:  return QString::fromUtf8("下降到起点");
    case WeldWaypoint::Kind::Seam:     return QString::fromUtf8("沿焊缝");
    case WeldWaypoint::Kind::Retreat:  return QString::fromUtf8("退出");
    }
    return QString();
}
}

WeldPathController::WeldPathController(RobotWorker* robot, QObject* parent)
    : QObject(parent)
    , m_robot(robot)
{
    if (m_robot) {
        connect(m_robot, &RobotWorker::moveFinished,
            this, &WeldPathController::onMoveFinished);
    }
}

void WeldPathController::start(std::vector<WeldWaypoint> waypoints,
    Speeds speeds, QString description)
{
    if (m_running) {
        emit progress(QString::fromUtf8("上一条轨迹仍在执行中。"));
        return;
    }
    if (!m_robot || !m_robot->isOnline()) {
        emit failed(QString::fromUtf8("机械臂未连接，无法执行轨迹。"));
        return;
    }
    if (m_robot->state() == RobotWorker::State::Busy) {
        emit failed(QString::fromUtf8("机械臂正在执行其它运动。"));
        return;
    }
    if (waypoints.empty()) {
        emit failed(QString::fromUtf8("轨迹为空。"));
        return;
    }

    m_waypoints = std::move(waypoints);
    m_speeds = speeds;
    m_description = description;
    m_next = 0;
    m_running = true;
    emit progress(QString::fromUtf8("%1：共 %2 个目标点，开始执行。")
        .arg(m_description).arg(m_waypoints.size()));
    issueNext();
}

void WeldPathController::cancel()
{
    if (!m_running) return;
    fail(QString::fromUtf8("%1已被用户终止。").arg(m_description));
}

void WeldPathController::issueNext()
{
    const WeldWaypoint& waypoint = m_waypoints[m_next];

    RobotMoveTarget target;
    std::copy(waypoint.pose.begin(), waypoint.pose.end(), target.pose);
    target.relative = false;
    target.linear = true;
    target.toolId = 1;
    target.userId = 0;
    target.velPercent = waypoint.kind == WeldWaypoint::Kind::Seam
        ? m_speeds.seamPercent : m_speeds.travelPercent;
    target.accPercent = m_speeds.accPercent;
    target.timeoutMs = kMoveTimeoutMs;
    target.description = QString::fromUtf8("%1 %2/%3（%4）")
        .arg(m_description).arg(m_next + 1).arg(m_waypoints.size())
        .arg(kindName(waypoint.kind));

    m_moveInFlight = true;
    QMetaObject::invokeMethod(m_robot, [robot = m_robot, target]() {
        robot->moveTo(target);
    }, Qt::QueuedConnection);
}

void WeldPathController::onMoveFinished(bool ok, QString error)
{
    // moveFinished 是广播信号，扫描编排器的运动也会触发，只认自己发出的那段
    if (!m_running || !m_moveInFlight) return;
    m_moveInFlight = false;

    if (!ok) {
        fail(QString::fromUtf8("%1第 %2 段失败：%3")
            .arg(m_description).arg(m_next + 1).arg(error));
        return;
    }

    ++m_next;
    if (m_next >= m_waypoints.size()) {
        m_running = false;
        emit progress(QString::fromUtf8("%1完成。").arg(m_description));
        emit finished();
        return;
    }
    issueNext();
}

void WeldPathController::fail(const QString& message)
{
    // 指令可能还排在 worker 队列里没开始执行，此时 Busy 尚未置位，也要补发停止
    const bool mayBeMoving = m_moveInFlight
        || (m_robot && m_robot->state() == RobotWorker::State::Busy);
    m_running = false;
    m_moveInFlight = false;
    if (m_robot && mayBeMoving) {
        QMetaObject::invokeMethod(m_robot, [robot = m_robot]() {
            robot->stopMotion();
        }, Qt::QueuedConnection);
    }
    emit failed(message);
}
