#pragma once

#include <QColor>
#include <QIcon>
#include <QObject>
#include <QString>
#include <QTimer>

class QRoundLed;
class QToolButton;

// 顶栏设备按钮与指示灯的统一状态，供 DeviceIndicator 与主界面共享
enum class DeviceLinkState {
    Offline,    // 未连接
    Connecting, // 正在建立链路
    Online,     // 已连接空闲
    Busy,       // 采集 / 运动 / 通信中
    Fault       // 通信异常或设备故障
};

// 一个按钮 + 一个指示灯 = 一个设备的完整状态表达。
// 负责图标着色（SVG 不受 QSS color 控制）、文字与 tooltip 切换、闪烁节奏。
class DeviceIndicator : public QObject
{
    Q_OBJECT
public:
    DeviceIndicator(QToolButton* button, QRoundLed* led,
        const QString& iconPath,
        const QString& offlineText,
        const QString& onlineText,
        const QString& busyText,
        QObject* parent = nullptr);

    void setState(DeviceLinkState state,
        const QString& detail = QString());
    DeviceLinkState state() const { return m_state; }

private slots:
    void tickBlink();

private:
    void apply();
    QIcon tint(const QColor& color) const;

    QToolButton* m_button = nullptr;
    QRoundLed* m_led = nullptr;
    QString m_iconPath;
    QString m_offlineText;
    QString m_onlineText;
    QString m_busyText;
    QString m_detail;

    QIcon m_grayIcon;
    QIcon m_greenIcon;
    QIcon m_dimGreenIcon;
    QIcon m_redIcon;

    QTimer m_blinkTimer;
    bool m_blinkOn = true;
    DeviceLinkState m_state = DeviceLinkState::Offline;
};
