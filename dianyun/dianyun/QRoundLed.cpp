#include "QRoundLed.h"

#include <QPainter>

QRoundLed::QRoundLed(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(sizeHint());
    setMaximumSize(sizeHint());

    connect(&m_blinkTimer, &QTimer::timeout, this, [this]() {
        m_isLit = !m_isLit;
        update();
    });
}

void QRoundLed::setState(State state, int intervalMs)
{
    m_state = state;
    if (state == State::Blink) {
        m_isLit = true;
        m_blinkTimer.start(qMax(100, intervalMs));
    }
    else {
        m_blinkTimer.stop();
        m_isLit = (state == State::Solid);
    }
    update();
}

void QRoundLed::setColor(const QColor& onColor, const QColor& offColor)
{
    m_onColor = onColor;
    m_offColor = offColor;
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
    const QColor fillColor = m_isLit ? m_onColor : m_offColor;

    painter.setPen(QPen(QColor(45, 52, 60), 1));
    painter.setBrush(fillColor);
    painter.drawEllipse(ledRect);

    if (m_isLit) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 255, 255, 190));
        painter.drawEllipse(ledRect.adjusted(4, 3, -8, -9));
    }
}
