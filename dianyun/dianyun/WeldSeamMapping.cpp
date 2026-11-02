#include "WeldSeamService.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace {

bool isRigidTransform(const Eigen::Matrix4f& transform)
{
    if (!transform.allFinite()) return false;

    const Eigen::Vector4f expectedBottomRow(0.0f, 0.0f, 0.0f, 1.0f);
    if (!transform.row(3).transpose().isApprox(expectedBottomRow, 1e-4f)) {
        return false;
    }

    const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    return (rotation.transpose() * rotation)
        .isApprox(Eigen::Matrix3f::Identity(), 1e-3f) &&
        std::abs(rotation.determinant() - 1.0f) <= 1e-3f;
}

Eigen::Vector3f transformPosition(
    const Eigen::Matrix4f& transform, const Eigen::Vector3f& position)
{
    return (transform * position.homogeneous()).head<3>();
}

// 一条首尾相接的焊缝采样链，按焊接顺序排列。
struct SeamChain {
    std::vector<Eigen::Vector3f> path;
    double length = 0.0;
    bool closed = false;
};

const float kConnectionTolerance = 1e-6f;

float connectionTolerance(const std::vector<WeldSeamSegment>& segments)
{
    Eigen::Vector3f minimum = Eigen::Vector3f::Constant(
        std::numeric_limits<float>::infinity());
    Eigen::Vector3f maximum = Eigen::Vector3f::Constant(
        -std::numeric_limits<float>::infinity());
    for (const auto& segment : segments) {
        minimum = minimum.cwiseMin(segment.start).cwiseMin(segment.end);
        maximum = maximum.cwiseMax(segment.start).cwiseMax(segment.end);
    }
    if (!minimum.allFinite() || !maximum.allFinite()) {
        return kConnectionTolerance;
    }
    return std::max(static_cast<float>((maximum - minimum).norm()) * 1e-5f,
        kConnectionTolerance);
}

double pathLength(const std::vector<Eigen::Vector3f>& path)
{
    double length = 0.0;
    for (size_t index = 1; index < path.size(); ++index) {
        length += static_cast<double>((path[index] - path[index - 1]).norm());
    }
    return length;
}

std::vector<Eigen::Vector3f> reversePath(const std::vector<Eigen::Vector3f>& path)
{
    return std::vector<Eigen::Vector3f>(path.rbegin(), path.rend());
}

// 把线段集合按端点连通性拼成若干条有序链。与渲染侧的拼接思路一致，
// 但这里要把结果落成确定的数据顺序，供映射和后续轨迹生成使用。
std::vector<SeamChain> buildChains(
    const std::vector<WeldSeamSegment>& segments, float tolerance)
{
    const float toleranceSquared = tolerance * tolerance;
    std::vector<bool> used(segments.size(), false);
    std::vector<SeamChain> chains;

    const auto samePoint = [toleranceSquared](
        const Eigen::Vector3f& left, const Eigen::Vector3f& right) {
        return (left - right).squaredNorm() <= toleranceSquared;
    };

    for (size_t seed = 0; seed < segments.size(); ++seed) {
        if (used[seed]) continue;
        used[seed] = true;

        SeamChain chain;
        if (!segments[seed].start.allFinite() ||
            !segments[seed].end.allFinite()) {
            continue;
        }
        chain.path = { segments[seed].start, segments[seed].end };

        bool extended = true;
        while (extended) {
            extended = false;
            for (size_t index = 0; index < segments.size(); ++index) {
                if (used[index]) continue;
                const auto& segment = segments[index];
                if (!segment.start.allFinite() || !segment.end.allFinite()) {
                    continue;
                }
                if (samePoint(chain.path.back(), segment.start)) {
                    chain.path.push_back(segment.end);
                }
                else if (samePoint(chain.path.back(), segment.end)) {
                    chain.path.push_back(segment.start);
                }
                else if (samePoint(chain.path.front(), segment.end)) {
                    chain.path.insert(chain.path.begin(), segment.start);
                }
                else if (samePoint(chain.path.front(), segment.start)) {
                    chain.path.insert(chain.path.begin(), segment.end);
                }
                else {
                    continue;
                }
                used[index] = true;
                extended = true;
                break;
            }
        }

        // 拼接过程中可能因为容差把端点插入成相邻重复点，压一下更稳。
        std::vector<Eigen::Vector3f> deduped;
        deduped.reserve(chain.path.size());
        for (const auto& point : chain.path) {
            if (deduped.empty() || !samePoint(deduped.back(), point)) {
                deduped.push_back(point);
            }
        }

        if (deduped.size() < 2) continue;
        chain.closed = deduped.size() > 2 &&
            samePoint(deduped.front(), deduped.back());
        chain.length = pathLength(deduped);
        chain.path = std::move(deduped);
        chains.push_back(std::move(chain));
    }

    // 长链优先，保证主焊缝始终排在第一条，拼接顺序不再受线段输入顺序影响。
    std::stable_sort(chains.begin(), chains.end(),
        [](const SeamChain& left, const SeamChain& right) {
            return left.length > right.length;
        });
    return chains;
}

CloudRGB::Ptr buildCloudFromSegments(
    const std::vector<WeldSeamSegment>& segments)
{
    CloudRGB::Ptr cloud(new CloudRGB());
    cloud->reserve(segments.size() + 1);
    const auto pushPoint = [&cloud](const Eigen::Vector3f& position) {
        pcl::PointXYZRGB sample;
        sample.x = position.x();
        sample.y = position.y();
        sample.z = position.z();
        sample.r = 0;
        sample.g = 255;
        sample.b = 0;
        cloud->push_back(sample);
    };
    for (const auto& segment : segments) {
        pushPoint(segment.start);
    }
    if (!segments.empty()) pushPoint(segments.back().end);
    return cloud;
}

std::vector<WeldSeamSegment> segmentsFromPath(
    const std::vector<Eigen::Vector3f>& path)
{
    std::vector<WeldSeamSegment> segments;
    if (path.size() < 2) return segments;
    segments.reserve(path.size() - 1);
    for (size_t index = 1; index < path.size(); ++index) {
        segments.push_back({ path[index - 1], path[index] });
    }
    return segments;
}

// 用重排后的链集合回写焊缝：段序按链顺序展开，点云与段序保持一致，
// 因此 points[0] 就是起焊点。
bool applyChains(
    const std::vector<SeamChain>& chains, WeldSeamData& output)
{
    std::vector<WeldSeamSegment> segments;
    for (const auto& chain : chains) {
        if (chain.path.size() < 2) continue;
        const auto chainSegments = segmentsFromPath(chain.path);
        segments.insert(segments.end(),
            chainSegments.begin(), chainSegments.end());
    }
    if (segments.empty()) return false;

    output.points = buildCloudFromSegments(segments);
    output.segments = std::move(segments);
    return true;
}


}

bool WeldSeamService::mapSeam(
    const WeldSeamData& cadSeam,
    const Eigen::Matrix4f& cadToScan,
    WeldSeamData& output,
    QString& errorMessage) const
{
    errorMessage.clear();
    if (!cadSeam.points || cadSeam.points->empty() ||
        cadSeam.segments.empty()) {
        errorMessage = QString::fromUtf8("选中的焊缝没有可映射的几何数据。");
        return false;
    }
    if (!isRigidTransform(cadToScan)) {
        errorMessage = QString::fromUtf8(
            "模型匹配变换不是有效的刚性变换，无法映射焊缝。");
        return false;
    }

    WeldSeamData mapped;
    mapped.id = cadSeam.id;
    mapped.name = cadSeam.name;
    mapped.length = cadSeam.length;
    mapped.hasStart = cadSeam.hasStart;
    mapped.points->reserve(cadSeam.points->size());
    mapped.segments.reserve(cadSeam.segments.size());

    for (const auto& sourcePoint : cadSeam.points->points) {
        const Eigen::Vector3f sourcePosition = sourcePoint.getVector3fMap();
        if (!sourcePosition.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝采样点包含无效坐标。");
            return false;
        }

        const Eigen::Vector3f mappedPosition =
            transformPosition(cadToScan, sourcePosition);
        if (!mappedPosition.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝采样点映射后坐标无效。");
            return false;
        }

        pcl::PointXYZRGB mappedPoint = sourcePoint;
        mappedPoint.x = mappedPosition.x();
        mappedPoint.y = mappedPosition.y();
        mappedPoint.z = mappedPosition.z();
        mapped.points->push_back(mappedPoint);
    }

    for (const auto& sourceSegment : cadSeam.segments) {
        if (!sourceSegment.start.allFinite() ||
            !sourceSegment.end.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝线段包含无效坐标。");
            return false;
        }

        WeldSeamSegment mappedSegment;
        mappedSegment.start = transformPosition(cadToScan, sourceSegment.start);
        mappedSegment.end = transformPosition(cadToScan, sourceSegment.end);
        if (!mappedSegment.start.allFinite() ||
            !mappedSegment.end.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝线段映射后坐标无效。");
            return false;
        }
        mapped.segments.push_back(mappedSegment);
    }

    output = std::move(mapped);
    return true;
}

bool WeldSeamService::setStartAt(
    const WeldSeamData& input,
    const Eigen::Vector3f& anchor,
    double tolerance,
    WeldSeamData& output,
    QString& errorMessage) const
{
    errorMessage.clear();
    if (!anchor.allFinite()) {
        errorMessage = QString::fromUtf8("拾取到的起点坐标无效。");
        return false;
    }
    if (input.segments.empty()) {
        errorMessage = QString::fromUtf8("该焊缝没有可用的几何线段。");
        return false;
    }
    for (const auto& segment : input.segments) {
        if (!segment.start.allFinite() || !segment.end.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝线段包含无效坐标。");
            return false;
        }
    }

    std::vector<SeamChain> chains =
        buildChains(input.segments, connectionTolerance(input.segments));
    if (chains.empty()) {
        errorMessage = QString::fromUtf8("无法从焊缝线段恢复连通结构。");
        return false;
    }

    double bestDistanceSquared = std::numeric_limits<double>::infinity();
    size_t bestChain = 0;
    size_t bestSample = 0;
    for (size_t chainIndex = 0; chainIndex < chains.size(); ++chainIndex) {
        const auto& path = chains[chainIndex].path;
        for (size_t sample = 0; sample < path.size(); ++sample) {
            const double distanceSquared =
                static_cast<double>((path[sample] - anchor).squaredNorm());
            if (distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestChain = chainIndex;
                bestSample = sample;
            }
        }
    }

    if (tolerance > 0.0 &&
        std::sqrt(bestDistanceSquared) > tolerance) {
        errorMessage = QString::fromUtf8(
            "单击位置距焊缝 %1，超出允许容差 %2，请重新选择。")
            .arg(std::sqrt(bestDistanceSquared), 0, 'f', 2)
            .arg(tolerance, 0, 'f', 2);
        return false;
    }

    SeamChain& chain = chains[bestChain];
    if (chain.closed) {
        // 闭合焊缝没有端点，按点击位置循环旋转即可。
        chain.path.pop_back();   // 先去掉重复的闭合末点
        const size_t count = chain.path.size();
        const size_t offset = bestSample < count ? bestSample : 0;
        std::rotate(chain.path.begin(),
            chain.path.begin() + static_cast<std::ptrdiff_t>(offset),
            chain.path.end());
        chain.path.push_back(chain.path.front());
    }
    else {
        // 开链只允许端点起焊：选离点击位置更近的一端。
        const double frontDistance = static_cast<double>(
            (chain.path.front() - anchor).squaredNorm());
        const double backDistance = static_cast<double>(
            (chain.path.back() - anchor).squaredNorm());
        if (backDistance < frontDistance) {
            chain.path = reversePath(chain.path);
        }
    }

    // 命中链排到最前，其余链保持原有相对顺序。
    if (bestChain != 0) {
        std::swap(chains[0], chains[bestChain]);
    }

    WeldSeamData result = input;
    if (!applyChains(chains, result)) {
        errorMessage = QString::fromUtf8("重排焊缝几何后没有可用数据。");
        return false;
    }
    result.hasStart = true;
    output = std::move(result);
    return true;
}

bool WeldSeamService::reverseDirection(
    const WeldSeamData& input,
    WeldSeamData& output,
    QString& errorMessage) const
{
    errorMessage.clear();
    if (input.segments.empty()) {
        errorMessage = QString::fromUtf8("该焊缝没有可用的几何线段。");
        return false;
    }
    for (const auto& segment : input.segments) {
        if (!segment.start.allFinite() || !segment.end.allFinite()) {
            errorMessage = QString::fromUtf8("焊缝线段包含无效坐标。");
            return false;
        }
    }

    const float tolerance = connectionTolerance(input.segments);
    std::vector<SeamChain> chains = buildChains(input.segments, tolerance);
    if (chains.empty()) {
        errorMessage = QString::fromUtf8("无法从焊缝线段恢复连通结构。");
        return false;
    }
    for (auto& chain : chains) {
        chain.path = reversePath(chain.path);
    }

    WeldSeamData result = input;
    if (!applyChains(chains, result)) {
        errorMessage = QString::fromUtf8("反转焊缝方向后没有可用数据。");
        return false;
    }
    result.hasStart = true;
    output = std::move(result);
    return true;
}
