#pragma once

#include <atomic>

class CameraCaptureState
{
public:
    bool tryBegin()
    {
        bool expected = false;
        return m_active.compare_exchange_strong(expected, true);
    }

    void finish()
    {
        m_active.store(false);
    }

    bool isActive() const
    {
        return m_active.load();
    }

private:
    std::atomic<bool> m_active{ false };
};
