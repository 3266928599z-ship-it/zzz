#pragma once

#include <QColor>
#include <QTimer>
#include <QWidget>

class QRoundLed : public QWidget
{
public:
    enum class State { Off, Solid, Blink };

    explicit QRoundLed(QWidget* parent = nullptr);

    void setState(State state, int intervalMs = 500);
    void setColor(const QColor& onColor,
        const QColor& offColor = QColor(92, 100, 108));

    State state() const { return m_state; }

    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QTimer m_blinkTimer;
    State m_state = State::Off;
    bool m_isLit = false;
    QColor m_onColor{ 42, 196, 78 };
    QColor m_offColor{ 92, 100, 108 };
};
