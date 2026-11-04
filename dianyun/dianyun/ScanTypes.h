#pragma once

#include <QMetaType>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

// 进程内统一的单调时钟（µs）。相机回调与机器人遥测都用它打时间戳，
// 才能把每一行轮廓对到机器人轨迹上的同一时刻。
inline qint64 hostClockUs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 一帧轮廓数据（对应设备一次打包输出，通常 20~50 行）。
//
// 设备在 ImageMode=4 下实际返回 Profile/ABC16：
//   每个点 3 个 int16，A = X 编码值，B 恒为 0，C = Z 编码值，
//   空点三个分量同时为 -32768。
// A/C 乘以 LSLProfileCoordXUnit / ZUnit（µm/count）得到 µm。
// B 不携带行坐标，Y 必须由调用方按「行序号 × 步距」补齐。
//
// 本结构由相机回调线程填充，跨线程只传递 shared_ptr，不做深拷贝。
struct ScanFrame {
    using Ptr = std::shared_ptr<ScanFrame>;

    quint32 frameNum = 0;     // 设备帧号，用于检测丢帧
    quint32 rows = 0;         // 本帧轮廓行数
    quint32 cols = 0;         // 每行点数（通常等于 Width）
    float stepUm = 0.0f;      // 行间步距（µm），等于 LSLStepDistance
    qint64 hostUs = 0;        // 回调收到本批的时刻（hostClockUs），≈ 末行采集时刻

    // 与「行优先、列其次」顺序一一对应，长度 = rows * cols。
    // 空点用 NaN 表示，调用方用 std::isfinite 判定。
    std::vector<float> xMm;   // 传感器坐标系下的横向坐标（mm）
    std::vector<float> zMm;   // 传感器坐标系下的深度坐标（mm）

    bool empty() const { return rows == 0 || cols == 0 || xMm.empty(); }
    std::size_t pointCount() const
    {
        return static_cast<std::size_t>(rows) * cols;
    }
};

Q_DECLARE_METATYPE(ScanFrame::Ptr)

// 扫描运动的产物：把多帧轮廓按行序号首尾相接后得到的一整幅深度图，
// 已补上 Y 坐标（传感器坐标系，mm）。
struct ScanResult {
    std::vector<float> xMm;
    std::vector<float> yMm;
    std::vector<float> zMm;
    quint32 cols = 0;
    quint32 rows = 0;

    std::size_t size() const { return xMm.size(); }
    bool empty() const { return xMm.empty(); }
};

// 扫描点云所在坐标系。只有 RobotBase 的点云及其匹配/焊缝结果才能下发给机械臂；
// 文件导入、单帧拍摄与轨迹不足时的回退拼接都属于 Sensor。
enum class CloudFrame { Sensor, RobotBase };

Q_DECLARE_METATYPE(CloudFrame)
