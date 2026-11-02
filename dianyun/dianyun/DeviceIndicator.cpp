#include "DeviceIndicator.h"

#include "QRoundLed.h"

#include <QPainter>
#include <QPixmap>
#include <QStyle>
#include <QToolButton>

namespace {

constexpr int kIconSize = 22;
constexpr int kBlinkIntervalMs = 500;

const QColor kOnlineColor(0x2A, 0xC4, 0x4E);
const QColor kDimGreenColor(0x1B, 0x7A, 0x31);
const QColor kFaultColor(0xE5, 0x39, 0x35);
const QColor kOfflineColor(0xC8, 0xC8, 0xC8);

} // namespace

DeviceIndicator::DeviceIndicator(QToolButton* button, QRoundLed* led,
    const QString& iconPath,
    const QString& offlineText,
    const QString& onlineText,
    const QString& busyText,
    QObject* parent)
    : QObject(parent)
    , m_button(button)
    , m_led(led)
    , m_iconPath(iconPath)
    , m_offlineText(offlineText)
    , m_onlineText(onlineText)
    , m_busyText(busyText)
{
    m_grayIcon = tint(kOfflineColor);
    m_greenIcon = tint(kOnlineColor);
    m_dimGreenIcon = tint(kDimGreenColor);
    m_redIcon = tint(kFaultColor);

    // SVG 插件缺失时降级为纯文字，不留空白图片位
    if (m_button && m_grayIcon.isNull()) {
        m_button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        m_button->setMinimumWidth(88);
    }

    m_blinkTimer.setInterval(kBlinkIntervalMs);
    connect(&m_blinkTimer, &QTimer::timeout,
        this, &DeviceIndicator::tickBlink);

    apply();
}

QIcon DeviceIndicator::tint(const QColor& color) const
{
    const QIcon source(m_iconPath);
    if (source.isNull()) return QIcon();

    const QSize baseSize(kIconSize, kIconSize);
    // 优先取 DPR 对应的位图，避免高 DPI 下 SVG 先降采样再放大
    const qreal dpr = m_button ? m_button->devicePixelRatioF() : 1.0;
    const QSize targetSize(qRound(baseSize.width() * dpr),
        qRound(baseSize.height() * dpr));

    QPixmap pixmap = source.pixmap(targetSize);
    if (pixmap.isNull()) return QIcon();
    pixmap.setDevicePixelRatio(dpr);

    QPainter painter(&pixmap);
    painter.setCompositionMode(QPainter::CompositionMode_SourceIn);
    painter.fillRect(pixmap.rect(), color);
    painter.end();

    return QIcon(pixmap);
}

void DeviceIndicator::setState(DeviceLinkState state, const QString& detail)
{
    m_detail = detail;
    if (m_state == state) {
        apply();   // 状态未变但提示信息可能更新
        return;
    }
    m_state = state;
    apply();
}

void DeviceIndicator::apply()
{
    QString text = m_offlineText;
    QString linkState = QStringLiteral("offline");
    QString icon;
    bool useGreenIcon = false;
    bool useRedIcon = false;

    QColor ledColor = kOnlineColor;
    QRoundLed::State ledState = QRoundLed::State::Off;

    m_blinkTimer.stop();
    m_blinkOn = true;

    switch (m_state) {
    case DeviceLinkState::Offline:
        break;

    case DeviceLinkState::Connecting:
        // 连接中保持灰色按钮，避免"假绿"误导；LED 同步灰闪而非绿色
        linkState = QStringLiteral("connecting");
        ledColor = kOfflineColor;
        if (m_led) ledState = QRoundLed::State::Blink;
        m_blinkTimer.start();
        break;

    case DeviceLinkState::Online:
        text = m_onlineText;
        linkState = QStringLiteral("online");
        useGreenIcon = true;
        if (m_led) ledState = QRoundLed::State::Solid;
        break;

    case DeviceLinkState::Busy:
        text = m_busyText;
        linkState = QStringLiteral("busy");
        useGreenIcon = true;
        if (m_led) ledState = QRoundLed::State::Blink;
        m_blinkTimer.start();
        break;

    case DeviceLinkState::Fault:
        linkState = QStringLiteral("fault");
        useRedIcon = true;
        ledColor = kFaultColor;
        if (m_led) ledState = QRoundLed::State::Solid;
        break;
    }

    if (m_button) {
        m_button->setIcon(useGreenIcon ? m_greenIcon
            : useRedIcon ? m_redIcon
            : m_grayIcon);
        m_button->setText(text);
        m_button->setToolTip(m_detail.isEmpty()
            ? text
            : QStringLiteral("%1\n%2").arg(text, m_detail));

        m_button->setProperty("linkState", linkState);
        m_button->style()->unpolish(m_button);
        m_button->style()->polish(m_button);
    }

    if (m_led) {
        m_led->setColor(ledColor);
        m_led->setState(ledState, kBlinkIntervalMs);
    }
}

void DeviceIndicator::tickBlink()
{
    if (!m_button) return;
    if (m_state != DeviceLinkState::Busy) {
        m_blinkTimer.stop();
        return;
    }

    // 只闪图标，文字保持常亮，避免每 500ms 触发一次 QSS 重算
    m_blinkOn = !m_blinkOn;
    m_button->setIcon(m_blinkOn ? m_greenIcon : m_dimGreenIcon);
}
