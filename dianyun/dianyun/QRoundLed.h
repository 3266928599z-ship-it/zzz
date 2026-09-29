#pragma once

#include <QTimer>
#include <QWidget>

class QRoundLed : public QWidget
{
public:
    explicit QRoundLed(QWidget* parent = nullptr);

    void startBlinking(int durationMs, int intervalMs = 400);
    void stopBlinking();

    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QTimer m_blinkTimer;
    QTimer m_stopTimer;
    bool m_isBlinking = false;
    bool m_isLit = false;
};
