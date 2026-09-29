#include "QRoundLed.h"

#include <QPainter>

QRoundLed::QRoundLed(QWidget* parent)
    : QWidget(parent)
{
    setToolTip(QString::fromUtf8("演示状态指示灯"));
    setMinimumSize(sizeHint());
    setMaximumSize(sizeHint());

    m_stopTimer.setSingleShot(true);
    connect(&m_blinkTimer, &QTimer::timeout, this, [this]() {
        m_isLit = !m_isLit;
        update();
    });
    connect(&m_stopTimer, &QTimer::timeout, this, [this]() {
        stopBlinking();
    });
}

void QRoundLed::startBlinking(int durationMs, int intervalMs)
{
    m_isBlinking = true;
    m_isLit = true;
    m_blinkTimer.start(qMax(100, intervalMs));
    m_stopTimer.start(qMax(0, durationMs));
    update();
}

void QRoundLed::stopBlinking()
{
    m_blinkTimer.stop();
    m_stopTimer.stop();
    m_isBlinking = false;
    m_isLit = false;
    update();
}

QSize QRoundLed::sizeHint() const
{
    return QSize(22, 22);
}

void QRoundLed::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    const QRect ledRect = rect().adjusted(2, 2, -2, -2);
    const QColor fillColor = m_isBlinking && m_isLit
        ? QColor(42, 196, 78)
        : QColor(92, 100, 108);

    painter.setPen(QPen(QColor(45, 52, 60), 1));
    painter.setBrush(fillColor);
    painter.drawEllipse(ledRect);

    if (m_isBlinking && m_isLit) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(214, 255, 221, 190));
        painter.drawEllipse(ledRect.adjusted(4, 3, -8, -9));
    }
}
