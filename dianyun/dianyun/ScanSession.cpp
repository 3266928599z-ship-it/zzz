#include "ScanSession.h"

#include "PoseMath.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
}

void ScanSession::begin(float stepUm)
{
    m_active = true;
    m_stepUm = stepUm;
    m_cols = 0;
    m_rows = 0;
    m_xMm.clear();
    m_zMm.clear();
    m_haveFrameNum = false;
    m_lastFrameNum = 0;
    m_frameNumUsable = true;
    m_frameNumUnit = FrameNumUnit::Unknown;
    m_lastFrameRows = 0;
    m_gapCount = 0;
    m_missingRows = 0;
    m_fillRanges.clear();
    m_maxFrameDelta = 0;
    m_stamps.clear();
    m_rowsKept = 0;
    m_framesKept = 0;
    m_rejectedFrames = 0;
}

quint32 ScanSession::averageRowsPerFrame() const
{
    if (m_framesKept == 0) return 0;
    return static_cast<quint32>(m_rowsKept / m_framesKept);
}

bool ScanSession::addFrame(const ScanFrame::Ptr& frame)
{
    if (!m_active || !frame || frame->empty()) return false;

    // 列数必须恒定：中途变化说明设备配置被改，强行拼接会错位
    if (m_cols == 0) {
        m_cols = frame->cols;
    }
    else if (frame->cols != m_cols) {
        ++m_rejectedFrames;
        return false;
    }
    if (!(m_stepUm > 0.0f) && frame->stepUm > 0.0f) {
        m_stepUm = frame->stepUm;
    }

    // 设备批次行数不固定，帧号一旦出现回退就停用丢帧判断，
    // 避免把正常的批次重组误判成丢帧。
    quint64 fillRows = 0;
    if (!m_haveFrameNum) {
        m_haveFrameNum = true;
        m_lastFrameNum = frame->frameNum;
    }
    else if (frame->frameNum <= m_lastFrameNum) {
        m_frameNumUsable = false;
    }
    else if (m_frameNumUsable) {
        const quint32 delta = frame->frameNum - m_lastFrameNum;
        m_maxFrameDelta = std::max(m_maxFrameDelta, delta);

        // 帧号可能逐批 +1，也可能按轮廓行计数（跳变 = 批行数）。
        // 首个能区分的跳变决定计数单位；前后两批行数相同时无法区分首行/末行编号，暂不判定。
        if (m_frameNumUnit == FrameNumUnit::Unknown) {
            const bool matchPrev = m_lastFrameRows > 1 && delta == m_lastFrameRows;
            const bool matchCur = frame->rows > 1 && delta == frame->rows;
            if (matchPrev && !matchCur) m_frameNumUnit = FrameNumUnit::RowFirst;
            else if (matchCur && !matchPrev) m_frameNumUnit = FrameNumUnit::RowLast;
            else if (!matchPrev && !matchCur) m_frameNumUnit = FrameNumUnit::Batch;
        }

        switch (m_frameNumUnit) {
        case FrameNumUnit::Batch:
            if (delta > 1) fillRows = static_cast<quint64>(delta - 1) * averageRowsPerFrame();
            break;
        case FrameNumUnit::RowFirst:
            if (delta > m_lastFrameRows) fillRows = delta - m_lastFrameRows;
            break;
        case FrameNumUnit::RowLast:
            if (delta > frame->rows) fillRows = delta - frame->rows;
            break;
        case FrameNumUnit::Unknown:
            break;
        }
        if (fillRows > 0) {
            ++m_gapCount;
            m_missingRows += fillRows;
            m_fillRanges.emplace_back(m_rows, fillRows);
        }
        m_lastFrameNum = frame->frameNum;
    }
    m_lastFrameRows = frame->rows;

    // 丢批处按估算行数补全空行：几何尺度因此保持正确，
    // 缺失区间内只有空洞，不会把后续点云整体前移。
    m_rows += fillRows;
    m_rows += frame->rows;
    m_xMm.resize(static_cast<std::size_t>(m_rows) * m_cols, kNaN);
    m_zMm.resize(static_cast<std::size_t>(m_rows) * m_cols, kNaN);

    const std::size_t base =
        static_cast<std::size_t>(m_rows - frame->rows) * m_cols;

    const std::size_t count = frame->pointCount();
    const std::size_t n = std::min(count, m_xMm.size() - base);
    std::copy_n(frame->xMm.begin(), n, m_xMm.begin() + base);
    std::copy_n(frame->zMm.begin(), n, m_zMm.begin() + base);

    m_rowsKept += frame->rows;
    ++m_framesKept;
    m_stamps.push_back({ m_rows - frame->rows, frame->rows, frame->hostUs });
    return true;
}

double ScanSession::travelMm(double stepUmOverride, bool fillGaps) const
{
    const double step = stepUmOverride > 0.0 ? stepUmOverride : m_stepUm;
    const quint64 rows = rowCount(fillGaps);
    if (rows == 0 || !(step > 0.0)) return 0.0;
    return static_cast<double>(rows - 1) * step * 0.001;
}

ScanResult ScanSession::result(double stepUmOverride, bool fillGaps) const
{
    ScanResult out;
    const quint64 outRows = rowCount(fillGaps);
    if (outRows == 0 || m_cols == 0) return out;

    const double step = stepUmOverride > 0.0 ? stepUmOverride : m_stepUm;

    out.cols = m_cols;
    out.rows = static_cast<quint32>(outRows);
    if (fillGaps || m_fillRanges.empty()) {
        out.xMm = m_xMm;
        out.zMm = m_zMm;
    }
    else {
        // 跳过补齐区间，把实收行首尾相接
        out.xMm.reserve(static_cast<std::size_t>(outRows) * m_cols);
        out.zMm.reserve(static_cast<std::size_t>(outRows) * m_cols);
        quint64 from = 0;
        auto appendRows = [&](quint64 begin, quint64 end) {
            const std::size_t a = static_cast<std::size_t>(begin) * m_cols;
            const std::size_t b = static_cast<std::size_t>(end) * m_cols;
            out.xMm.insert(out.xMm.end(), m_xMm.begin() + a, m_xMm.begin() + b);
            out.zMm.insert(out.zMm.end(), m_zMm.begin() + a, m_zMm.begin() + b);
        };
        for (const auto& [start, length] : m_fillRanges) {
            appendRows(from, start);
            from = start + length;
        }
        appendRows(from, m_rows);
    }
    out.yMm.resize(out.xMm.size());

    const double stepMm = step * 0.001;
    const double sign = reverseY ? -1.0 : 1.0;
    for (quint64 r = 0; r < outRows; ++r) {
        const float y = static_cast<float>(
            yOffsetMm + sign * static_cast<double>(r) * stepMm);
        const std::size_t base = static_cast<std::size_t>(r) * m_cols;
        std::fill_n(out.yMm.begin() + base, m_cols, y);
    }
    return out;
}

CloudRGB::Ptr ScanSession::toCloud(double stepUmOverride, bool fillGaps) const
{
    return cloudFrom(result(stepUmOverride, fillGaps));
}

double ScanSession::measuredLineRate() const
{
    if (m_stamps.size() < 2) return 0.0;
    const qint64 spanUs = m_stamps.back().hostUs - m_stamps.front().hostUs;
    if (spanUs <= 0) return 0.0;
    // 首批到达时其行已采完，计时区间内产生的是首批之后的行
    const quint64 rows = m_rowsKept - m_stamps.front().rows;
    return static_cast<double>(rows) * 1.0e6 / static_cast<double>(spanUs);
}

template <typename Visitor>
bool ScanSession::forEachRowTime(Visitor&& visit) const
{
    const double rate = measuredLineRate();
    if (!(rate > 0.0)) return false;
    const double periodUs = 1.0e6 / rate;
    for (const FrameStamp& stamp : m_stamps) {
        for (quint32 r = 0; r < stamp.rows; ++r) {
            // 批内末行 ≈ 到达时刻，往前每行早一个行周期
            const double us = static_cast<double>(stamp.hostUs)
                - static_cast<double>(stamp.rows - 1 - r) * periodUs;
            visit(stamp.firstRow + r, us);
        }
    }
    return true;
}

namespace {

// 在轨迹上定位时刻 us：返回左侧样本下标与插值系数 t∈[0,1]，超出两端时夹紧
std::pair<std::size_t, double> locate(
    const std::vector<ScanSession::TrackSample>& track, double us)
{
    if (us <= static_cast<double>(track.front().us)) return { 0, 0.0 };
    if (us >= static_cast<double>(track.back().us)) return { track.size() - 2, 1.0 };
    const auto it = std::upper_bound(track.begin(), track.end(), us,
        [](double value, const ScanSession::TrackSample& s) {
            return value < static_cast<double>(s.us);
        });
    const std::size_t index = static_cast<std::size_t>(it - track.begin()) - 1;
    const double span = static_cast<double>(it->us - track[index].us);
    const double t = span > 0.0
        ? (us - static_cast<double>(track[index].us)) / span : 0.0;
    return { index, t };
}

} // namespace

ScanResult ScanSession::resultByTrack(const std::vector<TrackSample>& track) const
{
    ScanResult out;
    if (track.size() < 2 || m_cols == 0) return out;

    const std::size_t total = static_cast<std::size_t>(m_rowsKept) * m_cols;
    out.xMm.reserve(total);
    out.yMm.reserve(total);
    out.zMm.reserve(total);

    const double sign = reverseY ? -1.0 : 1.0;
    bool haveOrigin = false;
    double origin = 0.0;
    const bool ok = forEachRowTime([&](quint64 row, double us) {
        const auto [index, t] = locate(track, us);
        const double pos = track[index].mm + t * (track[index + 1].mm - track[index].mm);
        if (!haveOrigin) {
            origin = pos;
            haveOrigin = true;
        }
        const float y = static_cast<float>(yOffsetMm + sign * (pos - origin));

        const std::size_t base = static_cast<std::size_t>(row) * m_cols;
        out.xMm.insert(out.xMm.end(), m_xMm.begin() + base, m_xMm.begin() + base + m_cols);
        out.zMm.insert(out.zMm.end(), m_zMm.begin() + base, m_zMm.begin() + base + m_cols);
        out.yMm.insert(out.yMm.end(), m_cols, y);
    });
    if (!ok) return ScanResult();
    out.cols = m_cols;
    out.rows = static_cast<quint32>(m_rowsKept);
    return out;
}

ScanResult ScanSession::resultInBase(const std::vector<TrackSample>& track,
    const Eigen::Matrix4d& tcpToSdk, double timeOffsetUs) const
{
    ScanResult out;
    if (track.size() < 2 || m_cols == 0 || !tcpToSdk.allFinite()) return out;

    // 每个轨迹样本预先算好 T_base_sdk；平移线性插值、姿态四元数球面插值。
    // 扫描是姿态恒定的 MoveL，姿态插值只是兜底。
    std::vector<Eigen::Quaterniond> rotations;
    std::vector<Eigen::Vector3d> translations;
    rotations.reserve(track.size());
    translations.reserve(track.size());
    for (const TrackSample& sample : track) {
        const Eigen::Matrix4d baseToSdk = PoseMath::poseToMatrix(sample.pose) * tcpToSdk;
        rotations.emplace_back(Eigen::Matrix3d(baseToSdk.block<3, 3>(0, 0)));
        translations.emplace_back(baseToSdk.block<3, 1>(0, 3));
    }

    const std::size_t total = static_cast<std::size_t>(m_rowsKept) * m_cols;
    out.xMm.reserve(total);
    out.yMm.reserve(total);
    out.zMm.reserve(total);

    const bool ok = forEachRowTime([&](quint64 row, double us) {
        const auto [index, t] = locate(track, us + timeOffsetUs);
        const Eigen::Matrix3d rotation =
            rotations[index].slerp(t, rotations[index + 1]).toRotationMatrix();
        const Eigen::Vector3d translation =
            translations[index] + t * (translations[index + 1] - translations[index]);

        const std::size_t base = static_cast<std::size_t>(row) * m_cols;
        for (quint32 c = 0; c < m_cols; ++c) {
            const float x = m_xMm[base + c];
            const float z = m_zMm[base + c];
            if (!std::isfinite(x) || !std::isfinite(z)) {
                out.xMm.push_back(kNaN);
                out.yMm.push_back(kNaN);
                out.zMm.push_back(kNaN);
                continue;
            }
            const Eigen::Vector3d p = rotation
                * Eigen::Vector3d(x, 0.0, z) + translation;
            out.xMm.push_back(static_cast<float>(p.x()));
            out.yMm.push_back(static_cast<float>(p.y()));
            out.zMm.push_back(static_cast<float>(p.z()));
        }
    });
    if (!ok) return ScanResult();
    out.cols = m_cols;
    out.rows = static_cast<quint32>(m_rowsKept);
    return out;
}

CloudRGB::Ptr ScanSession::cloudFrom(const ScanResult& r)
{
    CloudRGB::Ptr cloud(new CloudRGB);
    if (r.empty()) return cloud;

    cloud->points.reserve(r.size());
    for (std::size_t i = 0; i < r.size(); ++i) {
        const float x = r.xMm[i];
        const float z = r.zMm[i];
        if (!std::isfinite(x) || !std::isfinite(z)) continue;

        pcl::PointXYZRGB point;
        point.x = x;
        point.y = r.yMm[i];
        point.z = z;
        point.r = point.g = point.b = 255;
        cloud->points.push_back(point);
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

void ScanSession::clear()
{
    m_active = false;
    m_cols = 0;
    m_rows = 0;
    m_xMm.clear();
    m_zMm.clear();
    m_haveFrameNum = false;
    m_lastFrameNum = 0;
    m_frameNumUsable = true;
    m_frameNumUnit = FrameNumUnit::Unknown;
    m_lastFrameRows = 0;
    m_gapCount = 0;
    m_missingRows = 0;
    m_fillRanges.clear();
    m_maxFrameDelta = 0;
    m_stamps.clear();
    m_rowsKept = 0;
    m_framesKept = 0;
    m_rejectedFrames = 0;
}
