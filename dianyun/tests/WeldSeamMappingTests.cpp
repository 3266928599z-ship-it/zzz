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

    std::cout << "WeldSeamMappingTests passed" << std::endl;
    return 0;
}
