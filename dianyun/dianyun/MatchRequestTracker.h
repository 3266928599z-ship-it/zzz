#pragma once

#include <cstdint>

class MatchRequestTracker
{
public:
    void invalidateData()
    {
        ++m_dataRevision;
        m_activeRequestId = 0;
        m_activeDataRevision = 0;
    }

    std::uint64_t beginRequest()
    {
        m_activeRequestId = ++m_nextRequestId;
        m_activeDataRevision = m_dataRevision;
        return m_activeRequestId;
    }

    bool accepts(std::uint64_t requestId) const
    {
        return requestId != 0 && requestId == m_activeRequestId &&
            m_activeDataRevision == m_dataRevision;
    }

    void complete(std::uint64_t requestId)
    {
        if (!accepts(requestId)) return;
        m_activeRequestId = 0;
        m_activeDataRevision = 0;
    }

    bool hasActiveRequest() const
    {
        return accepts(m_activeRequestId);
    }

private:
    std::uint64_t m_dataRevision = 1;
    std::uint64_t m_nextRequestId = 0;
    std::uint64_t m_activeRequestId = 0;
    std::uint64_t m_activeDataRevision = 0;
};
