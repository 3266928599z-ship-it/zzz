#include "../dianyun/WeldSeamService.h"
#include "../dianyun/MatchRequestTracker.h"
#include "../dianyun/CameraCaptureState.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>

#include "../dianyun/ScanSession.h"
#include "../dianyun/PoseMath.h"
#include "../dianyun/WeldPathPlanner.h"

namespace {

bool near(float actual, float expected, float tolerance = 1e-4f)
{
    return std::abs(actual - expected) <= tolerance;
}

int fail(const char* message);

// 构造一帧：rows 行、cols 列，X/Z 全部给同一个已知值，便于校验 Y 补齐
ScanFrame::Ptr makeScanFrame(quint32 frameNum, quint32 rows, quint32 cols,
    float xMm = 1.0f, float zMm = -200.0f)
{
    auto frame = std::make_shared<ScanFrame>();
    frame->frameNum = frameNum;
    frame->rows = rows;
    frame->cols = cols;
    frame->stepUm = 96.0f;
    frame->xMm.assign(static_cast<std::size_t>(rows) * cols, xMm);
    frame->zMm.assign(static_cast<std::size_t>(rows) * cols, zMm);
    return frame;
}

int checkScanSession()
{
    // 1. 两帧连续拼接：Y 必须按全局行序号 × 步距递增
    ScanSession session;
    session.begin(96.0f);
    if (!session.addFrame(makeScanFrame(1, 10, 4))) {
        return fail("first scan frame was rejected");
    }
    if (!session.addFrame(makeScanFrame(2, 5, 4))) {
        return fail("second scan frame was rejected");
    }
    if (session.rowCount() != 15 || session.cols() != 4) {
        return fail("scan session row/col count is wrong");
    }
    if (session.gapCount() != 0) {
        return fail("consecutive frames were reported as a gap");
    }

    const ScanResult stitched = session.result();
    if (stitched.rows != 15 || stitched.cols != 4) {
        return fail("scan result dimensions are wrong");
    }
    // 第 0 行 Y=0，第 14 行 Y=14×0.096=1.344 mm
    if (!near(stitched.yMm.front(), 0.0f) ||
        !near(stitched.yMm[14 * 4], 1.344f, 1e-4f)) {
        return fail("scan result Y coordinates are not row-indexed");
    }
    if (!near(session.travelMm(), 1.344)) {
        return fail("scan session travel distance is wrong");
    }
    // 用实测步距覆盖时，尺度必须跟着变
    if (!near(session.travelMm(192.0), 2.688)) {
        return fail("scan session ignored the measured step override");
    }

    // 2. 丢批：帧号 2 → 5，中间缺帧 2/3/4 共 3 批，按平均 10 行补 30 行空洞
    ScanSession gapped;
    gapped.begin(96.0f);
    gapped.addFrame(makeScanFrame(1, 10, 4));
    gapped.addFrame(makeScanFrame(5, 10, 4));
    if (gapped.gapCount() != 1) {
        return fail("frame number skip was not detected as a gap");
    }
    if (gapped.rowCount() != 50) {
        return fail("gap fill did not preserve the row budget");
    }
    // 补出来的空洞必须是 NaN，不能被当成有效点
    const ScanResult gapResult = gapped.result();
    if (std::isfinite(gapResult.xMm[10 * 4])) {
        return fail("gap rows were not left empty");
    }
    if (!near(gapResult.xMm[49 * 4], 1.0f)) {
        return fail("data after a gap was shifted or lost");
    }
    // 不补齐时实收行首尾相接：20 行，第 10 行即第二帧首行
    if (gapped.rowCount(false) != 20 || gapped.maxFrameDelta() != 4) {
        return fail("compact row count or max frame delta is wrong");
    }
    const ScanResult compact = gapped.result(-1.0, false);
    if (compact.rows != 20 || compact.xMm.size() != 20 * 4 ||
        !near(compact.xMm[10 * 4], 1.0f) ||
        !near(compact.yMm[19 * 4], 19 * 0.096f, 1e-4f)) {
        return fail("compact result did not drop the filled rows");
    }
    // Y 跨度必须包含空洞，否则几何会被压扁
    if (!near(gapped.travelMm(), 49 * 0.096)) {
        return fail("gap rows were not counted in the travel distance");
    }
    const CloudRGB::Ptr cloud = gapped.toCloud();
    if (cloud->size() != 80) {
        return fail("gap rows leaked into the point cloud");
    }

    // 2b. 帧号按轮廓行计数（跳变 = 上一批行数）时不是丢批；
    //     只有跳变超过上一批行数的部分才补空行
    ScanSession perRow;
    perRow.begin(96.0f);
    perRow.addFrame(makeScanFrame(100, 30, 4));
    perRow.addFrame(makeScanFrame(130, 20, 4));
    perRow.addFrame(makeScanFrame(150, 35, 4));
    if (perRow.gapCount() != 0 || perRow.rowCount() != 85) {
        return fail("row-counted frame numbers were treated as dropped batches");
    }
    perRow.addFrame(makeScanFrame(195, 10, 4));   // 150 + 35 + 缺 10 行
    if (perRow.gapCount() != 1 || perRow.estimatedMissingRows() != 10) {
        return fail("row-counted frame gap was not sized in rows");
    }

    // 3. 列数变化必须整帧拒收，否则点云会错位
    ScanSession mismatched;
    mismatched.begin(96.0f);
    mismatched.addFrame(makeScanFrame(1, 3, 4));
    if (mismatched.addFrame(makeScanFrame(2, 3, 8))) {
        return fail("frame with a different column count was accepted");
    }
    if (mismatched.rejectedFrames() != 1 || mismatched.rowCount() != 3) {
        return fail("rejected frame polluted the scan session");
    }

    // 4. reverseY 只翻转符号，不改变跨度
    ScanSession reversed;
    reversed.begin(96.0f);
    reversed.reverseY = true;
    reversed.addFrame(makeScanFrame(1, 5, 2));
    const ScanResult rev = reversed.result();
    if (!near(rev.yMm.front(), 0.0f) || !near(rev.yMm[4 * 2], -0.384f, 1e-4f)) {
        return fail("reverseY did not flip the scan direction");
    }

    // 5. 轨迹定位：两批各 10 行，到达时刻 10 ms / 20 ms → 行率 1000 Hz；
    //    轨迹 1 mm/ms 匀速，则第 0 行在 1 ms、末行在 20 ms，Y 跨度 19 mm。
    //    帧号跳变不影响：轨迹模式只用实收行。
    ScanSession tracked;
    tracked.begin(96.0f);
    auto f1 = makeScanFrame(1, 10, 2);
    f1->hostUs = 10000;
    auto f2 = makeScanFrame(5, 10, 2);
    f2->hostUs = 20000;
    tracked.addFrame(f1);
    tracked.addFrame(f2);
    if (!near(tracked.measuredLineRate(), 1000.0)) {
        return fail("measured line rate from frame stamps is wrong");
    }
    const std::vector<ScanSession::TrackSample> track = {
        { 0, 0.0 }, { 15000, 15.0 }, { 30000, 30.0 } };
    const ScanResult tr = tracked.resultByTrack(track);
    if (tr.rows != 20 || tr.yMm.size() != 40 ||
        !near(tr.yMm.front(), 0.0f) ||
        !near(tr.yMm[10 * 2], 10.0f, 1e-3f) ||
        !near(tr.yMm.back(), 19.0f, 1e-3f)) {
        return fail("track-based Y assignment is wrong");
    }

    // 6. 基坐标输出：TCP 竖直向下（Rx 180）沿基 X 匀速 1 mm/ms，传感器在
    //    TCP 下方 100 mm。SDK 点 (1, 0, -200) → 工具系 (1, 0, -100) →
    //    基坐标 (1 + mm, 0, 500 + 100)。
    const std::vector<ScanSession::TrackSample> poseTrack = {
        { 0, 0.0, { 0.0, 0.0, 500.0, 180.0, 0.0, 0.0 } },
        { 15000, 15.0, { 15.0, 0.0, 500.0, 180.0, 0.0, 0.0 } },
        { 30000, 30.0, { 30.0, 0.0, 500.0, 180.0, 0.0, 0.0 } } };
    Eigen::Matrix4d tcpToSdk = Eigen::Matrix4d::Identity();
    tcpToSdk(2, 3) = 100.0;
    const ScanResult inBase = tracked.resultInBase(poseTrack, tcpToSdk);
    if (inBase.rows != 20 || inBase.xMm.size() != 40 ||
        !near(inBase.xMm.front(), 2.0f, 1e-3f) ||
        !near(inBase.xMm.back(), 21.0f, 1e-3f) ||
        !near(inBase.yMm.back(), 0.0f, 1e-3f) ||
        !near(inBase.zMm.front(), 600.0f, 1e-3f)) {
        return fail("base-frame scan assembly is wrong");
    }
    // 时间偏置 +1 ms 等价于整体沿行进方向前移 1 mm
    const ScanResult shifted = tracked.resultInBase(poseTrack, tcpToSdk, 1000.0);
    if (shifted.empty() || !near(shifted.xMm.front(), 3.0f, 1e-3f)) {
        return fail("time offset was not applied to the base-frame scan");
    }
    if (!tracked.resultInBase({ poseTrack.front() }, tcpToSdk).empty()) {
        return fail("base-frame scan accepted a single-sample track");
    }

    return 0;
}

int checkPoseMath()
{
    const double pose[6] = { 100.0, -50.0, 300.0, 170.0, -20.0, 35.0 };
    double back[6] = {};
    PoseMath::matrixToPose(PoseMath::poseToMatrix(pose), back);
    for (int i = 0; i < 6; ++i) {
        if (std::abs(back[i] - pose[i]) > 1e-9) {
            return fail("pose matrix round trip is wrong");
        }
    }
    // 法奥固定轴 XYZ：R = Rz·Ry·Rx。Rx 180 / Rz 90 时工具 X 指向基 +Y、工具 Z 朝下
    const double down[6] = { 0.0, 0.0, 0.0, 180.0, 0.0, 90.0 };
    const Eigen::Matrix4d m = PoseMath::poseToMatrix(down);
    if ((m.block<3, 1>(0, 0) - Eigen::Vector3d::UnitY()).norm() > 1e-9 ||
        (m.block<3, 1>(0, 2) + Eigen::Vector3d::UnitZ()).norm() > 1e-9) {
        return fail("rpy convention is not fixed-axis XYZ");
    }
    if (std::abs(PoseMath::headingDeg(down) - 90.0) > 1e-9 ||
        std::abs(PoseMath::tiltFromDownDeg(down)) > 1e-4) {
        return fail("heading or tilt of a leveled pose is wrong");
    }
    return 0;
}

int checkWeldPathPlanner()
{
    // 共线中间点与过近点应被删去，只剩折角
    const std::vector<Eigen::Vector3f> seam = {
        { 0.0f, 0.0f, 10.0f }, { 5.0f, 0.0f, 10.0f }, { 5.2f, 0.0f, 10.0f },
        { 10.0f, 0.0f, 10.0f }, { 10.0f, 10.0f, 12.0f } };
    WeldPathOptions options;
    options.headingDeg = 45.0;
    options.approachMm = 30.0;
    options.liftMm = 20.0;
    WeldPathPlan plan;
    QString error;
    if (!planWeldPath(seam, options, plan, error)) {
        return fail("valid seam path was rejected by the planner");
    }
    // 接近、下降(起点)、2 段焊缝、退出
    if (plan.waypoints.size() != 5 || plan.seamSegments != 2) {
        return fail("planner did not simplify collinear points");
    }
    using Kind = WeldWaypoint::Kind;
    const Kind expected[5] = {
        Kind::Approach, Kind::Descend, Kind::Seam, Kind::Seam, Kind::Retreat };
    for (int i = 0; i < 5; ++i) {
        if (plan.waypoints[i].kind != expected[i]) {
            return fail("planner waypoint order is wrong");
        }
    }
    const auto& approach = plan.waypoints.front().pose;
    const auto& retreat = plan.waypoints.back().pose;
    if (std::abs(approach[2] - 60.0) > 1e-6 || std::abs(retreat[2] - 62.0) > 1e-6 ||
        std::abs(plan.seamStart()[2] - 30.0) > 1e-6 ||
        std::abs(plan.seamEnd()[1] - 10.0) > 1e-6) {
        return fail("planner lift or approach height is wrong");
    }
    if (approach[3] != 180.0 || approach[4] != 0.0 || approach[5] != 45.0) {
        return fail("planner did not keep the leveled orientation");
    }
    if (std::abs(plan.minSeamZ - 10.0) > 1e-6 || std::abs(plan.maxSeamZ - 12.0) > 1e-6) {
        return fail("planner seam height range is wrong");
    }

    if (planWeldPath({ { 1.0f, 1.0f, 1.0f }, { 1.1f, 1.0f, 1.0f } },
            options, plan, error)) {
        return fail("planner accepted a seam shorter than the spacing");
    }
    return 0;
}

} // namespace

namespace {

bool nearVector(const Eigen::Vector3f& actual, const Eigen::Vector3f& expected)
{
    return near(actual.x(), expected.x()) &&
        near(actual.y(), expected.y()) &&
        near(actual.z(), expected.z());
}

int fail(const char* message)
{
    std::cerr << message << std::endl;
    return 1;
}

WeldSeamData makeLineSeam(
    const std::vector<Eigen::Vector3f>& path)
{
    WeldSeamData seam;
    seam.id = "line";
    seam.name = QStringLiteral("line");
    for (size_t index = 0; index < path.size(); ++index) {
        pcl::PointXYZRGB point;
        point.x = path[index].x();
        point.y = path[index].y();
        point.z = path[index].z();
        seam.points->push_back(point);
        if (index > 0) {
            seam.segments.push_back({ path[index - 1], path[index] });
            seam.length += static_cast<double>(
                (path[index] - path[index - 1]).norm());
        }
    }
    return seam;
}

bool sameSegment(
    const WeldSeamSegment& segment,
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end)
{
    return nearVector(segment.start, start) &&
        nearVector(segment.end, end);
}

}

int main()
{
    if (const int scanResult = checkScanSession()) return scanResult;
    if (const int poseResult = checkPoseMath()) return poseResult;
    if (const int planResult = checkWeldPathPlanner()) return planResult;

    {
        // 有序路径必须按段序（即焊接方向）输出
        WeldSeamService pathService;
        const WeldSeamData line = makeLineSeam({
            { 0.0f, 0.0f, 0.0f }, { 10.0f, 0.0f, 0.0f }, { 10.0f, 10.0f, 0.0f } });
        std::vector<Eigen::Vector3f> ordered;
        bool closed = true;
        QString pathError;
        if (!pathService.orderedPath(line, ordered, closed, pathError) ||
            closed || ordered.size() != 3 ||
            !nearVector(ordered.front(), Eigen::Vector3f(0.0f, 0.0f, 0.0f)) ||
            !nearVector(ordered.back(), Eigen::Vector3f(10.0f, 10.0f, 0.0f))) {
            return fail("ordered seam path lost the welding direction");
        }
        WeldSeamData split = line;
        split.segments.push_back({ Eigen::Vector3f(50.0f, 50.0f, 0.0f),
            Eigen::Vector3f(60.0f, 50.0f, 0.0f) });
        if (pathService.orderedPath(split, ordered, closed, pathError)) {
            return fail("disconnected seam was accepted as one path");
        }
    }

    CameraCaptureState captureState;
    if (!captureState.tryBegin() || captureState.tryBegin()) {
        return fail("camera capture state allowed overlapping captures");
    }
    captureState.finish();
    if (captureState.isActive() || !captureState.tryBegin()) {
        return fail("camera capture state did not reset after completion");
    }
    captureState.finish();

    MatchRequestTracker requests;
    const std::uint64_t firstRequest = requests.beginRequest();
    if (firstRequest == 0 || !requests.accepts(firstRequest)) {
        return fail("current match request was not accepted");
    }
    requests.complete(firstRequest);
    if (requests.accepts(firstRequest)) {
        return fail("completed match request was still accepted");
    }

    const std::uint64_t staleRequest = requests.beginRequest();
    requests.invalidateData();
    if (requests.accepts(staleRequest)) {
        return fail("request created for old input data was accepted");
    }
    const std::uint64_t currentRequest = requests.beginRequest();
    if (currentRequest <= staleRequest || !requests.accepts(currentRequest)) {
        return fail("request ids were not monotonic after data replacement");
    }

    WeldSeamData source;
    source.id = "7";
    source.name = QStringLiteral("seam-7");
    source.length = 12.5;

    pcl::PointXYZRGB point;
    point.x = 1.0f;
    point.y = 0.0f;
    point.z = 2.0f;
    source.points->push_back(point);
    source.segments.push_back({
        Eigen::Vector3f(1.0f, 0.0f, 2.0f),
        Eigen::Vector3f(0.0f, 2.0f, 2.0f)
    });

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(1.57079632679f, Eigen::Vector3f::UnitZ())
        .toRotationMatrix();
    transform.block<3, 1>(0, 3) = Eigen::Vector3f(10.0f, 20.0f, 30.0f);

    WeldSeamData mapped;
    QString errorMessage;
    WeldSeamService service;
    if (!service.mapSeam(source, transform, mapped, errorMessage)) {
        return fail("valid rigid mapping was rejected");
    }
    if (mapped.id != source.id || mapped.name != source.name ||
        std::abs(mapped.length - source.length) > 1e-9) {
        return fail("source seam metadata was not preserved");
    }
    if (mapped.points->size() != 1 || mapped.segments.size() != 1) {
        return fail("mapped geometry counts are incorrect");
    }
    if (!nearVector(mapped.points->front().getVector3fMap(),
        Eigen::Vector3f(10.0f, 21.0f, 32.0f))) {
        return fail("mapped point coordinates are incorrect");
    }
    if (!nearVector(mapped.segments.front().start,
        Eigen::Vector3f(10.0f, 21.0f, 32.0f)) ||
        !nearVector(mapped.segments.front().end,
            Eigen::Vector3f(8.0f, 20.0f, 32.0f))) {
        return fail("mapped segment coordinates are incorrect");
    }

    Eigen::Matrix4f invalid = transform;
    invalid(3, 0) = 1.0f;
    if (service.mapSeam(source, invalid, mapped, errorMessage)) {
        return fail("non-homogeneous transform was accepted");
    }

    invalid = transform;
    invalid.block<3, 1>(0, 0) *= -1.0f;
    if (service.mapSeam(source, invalid, mapped, errorMessage)) {
        return fail("reflection transform was accepted");
    }

    invalid = transform;
    invalid.block<3, 3>(0, 0) *= 2.0f;
    if (service.mapSeam(source, invalid, mapped, errorMessage)) {
        return fail("scaled transform was accepted");
    }

    WeldSeamData empty;
    if (service.mapSeam(empty, transform, mapped, errorMessage)) {
        return fail("empty seam was accepted");
    }

    WeldSeamData nonFinite = source;
    nonFinite.points = CloudRGB::Ptr(new CloudRGB(*source.points));
    nonFinite.points->front().x = std::numeric_limits<float>::quiet_NaN();
    if (service.mapSeam(nonFinite, transform, mapped, errorMessage)) {
        return fail("non-finite seam point was accepted");
    }

    // ---- 起点设定 ----
    const WeldSeamData lineSeam = makeLineSeam({
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(10.0f, 0.0f, 0.0f),
        Eigen::Vector3f(20.0f, 0.0f, 0.0f),
        Eigen::Vector3f(30.0f, 0.0f, 0.0f)
    });

    WeldSeamData withStart;
    if (!service.setStartAt(
        lineSeam, Eigen::Vector3f(29.0f, 0.5f, 0.0f), 5.0,
        withStart, errorMessage)) {
        return fail("valid start pickup was rejected");
    }
    if (!withStart.hasStart || withStart.segments.size() != 3) {
        return fail("start flag or segment count is incorrect");
    }
    if (!nearVector(withStart.points->front().getVector3fMap(),
        Eigen::Vector3f(30.0f, 0.0f, 0.0f))) {
        return fail("start point was not moved to the far end");
    }
    if (!sameSegment(withStart.segments.front(),
        Eigen::Vector3f(30.0f, 0.0f, 0.0f),
        Eigen::Vector3f(20.0f, 0.0f, 0.0f))) {
        return fail("first segment does not follow the new start");
    }
    if (!nearVector(withStart.points->back().getVector3fMap(),
        Eigen::Vector3f(0.0f, 0.0f, 0.0f))) {
        return fail("start reversal did not reach the original start");
    }

    WeldSeamData idempotent;
    if (!service.setStartAt(
        withStart, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 5.0,
        idempotent, errorMessage)) {
        return fail("repeated start pickup was rejected");
    }
    if (!nearVector(idempotent.points->front().getVector3fMap(),
        Eigen::Vector3f(0.0f, 0.0f, 0.0f))) {
        return fail("repeated start pickup did not keep the same end");
    }

    WeldSeamData rejected = lineSeam;
    if (service.setStartAt(
        lineSeam, Eigen::Vector3f(0.0f, 100.0f, 0.0f), 2.0,
        rejected, errorMessage)) {
        return fail("pickup outside tolerance was accepted");
    }
    if (rejected.hasStart || rejected.segments.size() != 3 ||
        !nearVector(rejected.points->front().getVector3fMap(),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f))) {
        return fail("rejected pickup modified the seam data");
    }

    // 两条分离链：命中的短链应排到最前，起点取该链近端。
    WeldSeamData twoChains;
    twoChains.id = "two";
    twoChains.name = QStringLiteral("two");
    twoChains.segments.push_back({
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(10.0f, 0.0f, 0.0f) });
    twoChains.segments.push_back({
        Eigen::Vector3f(10.0f, 0.0f, 0.0f),
        Eigen::Vector3f(20.0f, 0.0f, 0.0f) });
    twoChains.segments.push_back({
        Eigen::Vector3f(0.0f, 50.0f, 0.0f),
        Eigen::Vector3f(0.0f, 60.0f, 0.0f) });
    WeldSeamData twoChainsStart;
    if (!service.setStartAt(
        twoChains, Eigen::Vector3f(0.5f, 50.5f, 0.0f), 5.0,
        twoChainsStart, errorMessage)) {
        return fail("multi-chain start pickup was rejected");
    }
    if (!nearVector(twoChainsStart.segments.front().start,
        Eigen::Vector3f(0.0f, 50.0f, 0.0f))) {
        return fail("hit chain was not promoted to the front");
    }
    if (!nearVector(twoChainsStart.segments.back().end,
        Eigen::Vector3f(20.0f, 0.0f, 0.0f))) {
        return fail("remaining chain order was not preserved");
    }

    // 闭合回路：起点按点击位置旋转，首尾仍闭合。
    WeldSeamData loop = makeLineSeam({
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(10.0f, 0.0f, 0.0f),
        Eigen::Vector3f(10.0f, 10.0f, 0.0f),
        Eigen::Vector3f(0.0f, 10.0f, 0.0f),
        Eigen::Vector3f(0.0f, 0.0f, 0.0f)
    });
    WeldSeamData loopStart;
    if (!service.setStartAt(
        loop, Eigen::Vector3f(10.0f, 10.5f, 0.0f), 5.0,
        loopStart, errorMessage)) {
        return fail("closed loop start pickup was rejected");
    }
    if (!nearVector(loopStart.points->front().getVector3fMap(),
        Eigen::Vector3f(10.0f, 10.0f, 0.0f))) {
        return fail("closed loop start was not rotated to the pick");
    }
    if (!nearVector(loopStart.segments.front().start,
        loopStart.segments.back().end)) {
        return fail("closed loop was broken by start rotation");
    }
    if (loopStart.segments.size() != loop.segments.size()) {
        return fail("closed loop segment count changed");
    }

    // ---- 反转方向 ----
    WeldSeamData reversed;
    if (!service.reverseDirection(lineSeam, reversed, errorMessage)) {
        return fail("valid reverse was rejected");
    }
    if (!reversed.hasStart || !sameSegment(reversed.segments.front(),
        Eigen::Vector3f(30.0f, 0.0f, 0.0f),
        Eigen::Vector3f(20.0f, 0.0f, 0.0f))) {
        return fail("reverse did not flip the welding direction");
    }
    WeldSeamData restored;
    if (!service.reverseDirection(reversed, restored, errorMessage)) {
        return fail("second reverse was rejected");
    }
    if (restored.segments.size() != lineSeam.segments.size() ||
        !sameSegment(restored.segments.front(),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(10.0f, 0.0f, 0.0f))) {
        return fail("double reverse did not restore the original order");
    }

    WeldSeamData mappedStart;
    if (!service.mapSeam(withStart, transform, mappedStart, errorMessage)) {
        return fail("mapping a seam with a start point was rejected");
    }
    if (!mappedStart.hasStart ||
        !nearVector(mappedStart.points->front().getVector3fMap(),
            Eigen::Vector3f(10.0f, 50.0f, 30.0f))) {
        return fail("mapping did not preserve the start point");
    }

    std::cout << "WeldSeamMappingTests passed" << std::endl;
    return 0;
}
