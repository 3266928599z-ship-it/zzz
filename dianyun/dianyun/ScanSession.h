#pragma once

#include "AppTypes.h"
#include "ScanTypes.h"

#include <Eigen/Core>
#include <QString>
#include <cstdint>
#include <utility>
#include <vector>

// 一次扫描运动的拼接器。
//
// 设备的 ImageMode=4 批次行数不固定（实测 20~50 行浮动），帧内也没有
// 任何行坐标（B 通道恒 0、时间戳全 0），因此 Y 只能由上位机按
//
//     Y = 全局行序号 × 步距
//
// 补齐。行序号在批次之间连续累加，这正是「一次扫描必须匀速、且速度 =
// 步距 × 行率」的原因：速度一旦和参数不一致，Y 就会被均匀拉伸或压缩。
//
// 调用约定：begin() → 若干次 addFrame() → result()/toCloud()。
// 全部在 GUI 线程执行（帧从 worker 的 QueuedConnection 送来），无需加锁。
class ScanSession
{
public:
    void begin(float stepUm);
    // 返回 false 表示该帧被拒收（列数不一致等），计入 rejectedFrames()
    bool addFrame(const ScanFrame::Ptr& frame);

    bool isOpen() const { return m_active; }
    // fillGaps = false 时不计丢批补齐的空行，只算实收行
    quint64 rowCount(bool fillGaps = true) const
    {
        return fillGaps ? m_rows : m_rows - m_missingRows;
    }
    quint32 cols() const { return m_cols; }
    float stepUm() const { return m_stepUm; }
    // 首尾行之间的 Y 跨度（mm）。stepUmOverride <= 0 时用名义步距。
    double travelMm(double stepUmOverride = -1.0, bool fillGaps = true) const;

    // 帧号不连续（设备丢批）的次数。丢批时按已收帧的平均行数估算缺失行数，
    // 保持几何尺度正确，但缺失区间内没有数据点。
    // 帧号跳变未必是真丢批（帧号可能不是逐批 +1），所以补齐区间单独记录，
    // 收尾时由调用方结合实收行率决定是否采用。
    int gapCount() const { return m_gapCount; }
    quint64 estimatedMissingRows() const { return m_missingRows; }
    quint64 receivedRows() const { return m_rowsKept; }
    quint32 receivedFrames() const { return m_framesKept; }
    quint32 maxFrameDelta() const { return m_maxFrameDelta; }
    int rejectedFrames() const { return m_rejectedFrames; }

    // 收尾：返回带 Y 坐标的完整深度图。行优先，空点保持 NaN。
    ScanResult result(double stepUmOverride = -1.0, bool fillGaps = true) const;
    // 直接转成可显示的 PCL 点云（跳过 NaN）
    CloudRGB::Ptr toCloud(double stepUmOverride = -1.0, bool fillGaps = true) const;

    // 机器人实测轨迹的一个采样：时刻（hostClockUs）、已行进距离（mm）
    // 与该时刻的 TCP 位姿（基坐标，mm / 度，法奥固定轴 XYZ）
    struct TrackSample {
        qint64 us = 0;
        double mm = 0.0;
        double pose[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    };
    // 轨迹模式：每行的采集时刻由「本批到达时刻 − 批内行序 × 行周期」推出，
    // 再在轨迹上插值得到该时刻的行进距离作为 Y。不依赖「步距 × 行率 = 速度」，
    // 加减速段和行率偏离设定值都不会再拉伸点云；丢批只留空洞、不移位。
    // track 须按时间递增；样本不足两点或缺少时间戳时返回空结果。
    ScanResult resultByTrack(const std::vector<TrackSample>& track) const;
    // 基坐标模式：每行按采集时刻在轨迹上插值出 TCP 位姿，
    // p_base = T_base_tcp(t) · tcpToSdk · (x, 0, z)。输出的 x/y/z 均为基坐标（mm），
    // 不受 reverseY / yOffsetMm 影响。timeOffsetUs 加到每行时刻上。
    ScanResult resultInBase(const std::vector<TrackSample>& track,
        const Eigen::Matrix4d& tcpToSdk, double timeOffsetUs = 0.0) const;
    // 由批次到达时间反推的实际行率（Hz），不足两批时返回 0
    double measuredLineRate() const;

    static CloudRGB::Ptr cloudFrom(const ScanResult& result);

    // 扫描方向与传感器的 Y 轴相反时置 true；
    // 通常由「机器人沿工具 Z 方向后退扫描」还是「前进扫描」决定。
    bool reverseY = false;
    // 拼接起点偏移（mm），便于把多段扫描接在一起
    double yOffsetMm = 0.0;

    void clear();

private:
    quint32 averageRowsPerFrame() const;
    // 按批次到达时刻反推每个实收行的采集时刻，回调参数为（行号，时刻 µs）。
    // 行率未知时返回 false。
    template <typename Visitor>
    bool forEachRowTime(Visitor&& visit) const;

    bool m_active = false;
    float m_stepUm = 0.0f;
    quint32 m_cols = 0;
    quint64 m_rows = 0;

    // 行优先存储，长度恒为 m_rows * m_cols
    std::vector<float> m_xMm;
    std::vector<float> m_zMm;

    bool m_haveFrameNum = false;
    quint32 m_lastFrameNum = 0;
    // 帧号看起来是单调递增的才用它判丢帧，否则只做行数累加
    bool m_frameNumUsable = true;
    // 帧号计数单位：逐批 +1，或按轮廓行计数（编号为本批首行 / 末行）
    enum class FrameNumUnit { Unknown, Batch, RowFirst, RowLast };
    FrameNumUnit m_frameNumUnit = FrameNumUnit::Unknown;
    quint32 m_lastFrameRows = 0;

    int m_gapCount = 0;
    quint64 m_missingRows = 0;
    // 补齐空行区间：{起始行（含补齐的全局行号），行数}，按起始行递增
    std::vector<std::pair<quint64, quint64>> m_fillRanges;
    quint32 m_maxFrameDelta = 0;

    // 每批的位置与到达时刻，轨迹模式用
    struct FrameStamp {
        quint64 firstRow;   // 本批首行在 m_xMm 中的行号（含补齐行）
        quint32 rows;
        qint64 hostUs;
    };
    std::vector<FrameStamp> m_stamps;
    quint64 m_rowsKept = 0;      // 实际收到的行数，用于估算平均值
    quint32 m_framesKept = 0;
    int m_rejectedFrames = 0;
};
