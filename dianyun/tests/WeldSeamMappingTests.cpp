#include "../dianyun/WeldSeamService.h"
#include "../dianyun/MatchRequestTracker.h"
#include "../dianyun/CameraCaptureState.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>

namespace {

bool near(float actual, float expected, float tolerance = 1e-4f)
{
    return std::abs(actual - expected) <= tolerance;
}

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
