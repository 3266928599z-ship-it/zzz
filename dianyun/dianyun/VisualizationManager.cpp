#include "VisualizationManager.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkTubeFilter.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

void VisualizationManager::setViewers(
    const ViewerPtr& cadViewer, const ViewerPtr& scanViewer)
{
    m_cadViewer = cadViewer;
    m_scanViewer = scanViewer;
}

double VisualizationManager::coordinateScale(double diagonal)
{
    return std::max(diagonal * 0.03, 0.1);
}

std::string VisualizationManager::seamActorId(const std::string& seamId)
{
    return "seam_tube_" + seamId;
}

std::string VisualizationManager::mappedSeamActorId(const std::string& seamId)
{
    return "mapped_seam_tube_" + seamId;
}

void VisualizationManager::setSeamActorAppearance(
    const ViewerPtr& viewer,
    const std::string& actorId,
    double red,
    double green,
    double blue,
    bool selected)
{
    if (!viewer) return;
    const auto actor = viewer->getShapeActorMap()->find(actorId);
    if (actor == viewer->getShapeActorMap()->end()) return;

    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        red, green, blue, actorId);
    if (auto* tubeActor = vtkActor::SafeDownCast(actor->second)) {
        tubeActor->GetProperty()->SetAmbient(selected ? 0.9 : 0.65);
        tubeActor->GetProperty()->SetDiffuse(selected ? 0.1 : 0.35);
    }
}

void VisualizationManager::showCadModel(const CadModelData& model)
{
    if (!m_cadViewer || !model.mesh) return;
    m_cadDiagonal =
        std::isfinite(model.diagonal) && model.diagonal > 0.0
        ? model.diagonal
        : 0.0;
    m_cadViewer->removeAllPointClouds();
    m_cadViewer->removeAllShapes();
    m_cadViewer->addPolygonMesh(*model.mesh, "mesh_model");
    m_cadViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.8, "mesh_model");
    m_cadViewer->addCoordinateSystem(coordinateScale(model.diagonal));
    m_cadViewer->resetCamera();
}

void VisualizationManager::resetProgressiveScanState()
{
    m_progressiveScanCloud.reset();
    m_nextScanPoint = 0;
    m_renderedScanPoints = 0;
    m_totalScanPoints = 0;
    m_scanChunkIndex = 0;
    m_scanDiagonal = 0.0;
}

void VisualizationManager::showScanCloud(
    const CloudRGB::Ptr& cloud, double diagonal)
{
    if (!m_scanViewer || !cloud || cloud->empty()) return;
    resetProgressiveScanState();
    m_scanViewer->removeAllPointClouds();
    m_scanViewer->removeAllShapes();
    m_mappedSeamActorIds.clear();
    m_progressiveScanCloud = cloud;
    m_totalScanPoints = cloud->size();
    m_scanDiagonal = diagonal;
    if (!std::isfinite(m_scanDiagonal) || m_scanDiagonal <= 0.0) {
        pcl::PointXYZRGB minimum, maximum;
        pcl::getMinMax3D(*cloud, minimum, maximum);
        m_scanDiagonal = static_cast<double>(
            (maximum.getVector3fMap() - minimum.getVector3fMap()).norm());
    }
}

void VisualizationManager::appendNextScanChunk()
{
    if (!m_scanViewer || !m_progressiveScanCloud ||
        m_nextScanPoint >= m_progressiveScanCloud->size()) {
        return;
    }

    const size_t end = std::min(
        m_nextScanPoint + m_scanChunkSize,
        m_progressiveScanCloud->size());
    CloudRGB::Ptr chunk(new CloudRGB());
    chunk->reserve(end - m_nextScanPoint);
    for (size_t index = m_nextScanPoint; index < end; ++index) {
        chunk->push_back(m_progressiveScanCloud->points[index]);
    }

    const std::string actorId =
        "point_cloud_chunk_" + std::to_string(m_scanChunkIndex++);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB>
        zColor(chunk, "z");
    m_scanViewer->addPointCloud<pcl::PointXYZRGB>(chunk, zColor, actorId);
    m_scanViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, actorId);

    m_nextScanPoint = end;
    m_renderedScanPoints = end;
    if (m_scanChunkIndex == 1) {
        m_scanViewer->addCoordinateSystem(coordinateScale(m_scanDiagonal));
        m_scanViewer->resetCamera();
    }
    if (m_nextScanPoint >= m_progressiveScanCloud->size()) {
        m_progressiveScanCloud.reset();
    }
}

void VisualizationManager::showProcessedScan(const CloudRGB::Ptr& cloud)
{
    if (!m_scanViewer || !cloud || cloud->empty()) return;
    resetProgressiveScanState();
    m_scanViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
        color(cloud, 80, 220, 255);
    m_scanViewer->addPointCloud<pcl::PointXYZRGB>(cloud, color, "match_input");
    m_scanViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "match_input");
    m_scanViewer->spinOnce(1, true);
}

void VisualizationManager::showAlignedCad(
    const CloudRGB::Ptr& cadCloud, const Eigen::Matrix4f& cadToScan)
{
    if (!m_scanViewer || !cadCloud || cadCloud->empty()) return;
    CloudRGB::Ptr transformed(new CloudRGB());
    pcl::transformPointCloud(*cadCloud, *transformed, cadToScan);
    m_scanViewer->removePointCloud("cad_aligned");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
        color(transformed, 150, 150, 150);
    m_scanViewer->addPointCloud<pcl::PointXYZRGB>(
        transformed, color, "cad_aligned");
    m_scanViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cad_aligned");
    m_scanViewer->spinOnce(50, true);
}

void VisualizationManager::highlightFace(
    const CadModelData& model, int faceId, const std::string& actorId)
{
    if (!m_cadViewer || !model.mesh) return;
    pcl::PolygonMesh::Ptr faceMesh(new pcl::PolygonMesh());
    faceMesh->cloud = model.mesh->cloud;
    for (size_t index = 0; index < model.cellToFaceMap.size(); ++index) {
        if (model.cellToFaceMap[index] == faceId &&
            index < model.mesh->polygons.size()) {
            faceMesh->polygons.push_back(model.mesh->polygons[index]);
        }
    }
    if (faceMesh->polygons.empty()) return;

    m_cadViewer->removePolygonMesh(actorId);
    m_cadViewer->addPolygonMesh(*faceMesh, actorId);
    const bool firstFace = actorId == "highlight_A";
    m_cadViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        firstFace ? 1.0 : 0.0, 0.0, firstFace ? 0.0 : 1.0, actorId);
    m_cadViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, actorId);

    const auto actor = m_cadViewer->getShapeActorMap()->find(actorId);
    if (actor != m_cadViewer->getShapeActorMap()->end()) {
        actor->second->SetPickable(0);
    }
    m_cadViewer->spinOnce(50, true);
}

void VisualizationManager::clearFaceHighlights()
{
    if (!m_cadViewer) return;
    m_cadViewer->removePolygonMesh("highlight_A");
    m_cadViewer->removePolygonMesh("highlight_B");
    m_cadViewer->spinOnce(20, true);
}

bool VisualizationManager::showSeamTube(
    const ViewerPtr& viewer,
    const WeldSeamData& seam,
    const std::string& actorId,
    double scaleHint,
    double red,
    double green,
    double blue)
{
    if (!viewer || seam.segments.empty()) return false;

    auto points = vtkSmartPointer<vtkPoints>::New();
    auto lines = vtkSmartPointer<vtkCellArray>::New();
    Eigen::Vector3f minimum = Eigen::Vector3f::Constant(
        std::numeric_limits<float>::infinity());
    Eigen::Vector3f maximum = Eigen::Vector3f::Constant(
        -std::numeric_limits<float>::infinity());
    std::vector<WeldSeamSegment> validSegments;
    validSegments.reserve(seam.segments.size());

    for (const auto& segment : seam.segments) {
        if (!segment.start.allFinite() || !segment.end.allFinite() ||
            (segment.end - segment.start).squaredNorm() <= 1e-12f) {
            continue;
        }
        validSegments.push_back(segment);
        minimum = minimum.cwiseMin(segment.start).cwiseMin(segment.end);
        maximum = maximum.cwiseMax(segment.start).cwiseMax(segment.end);
    }
    if (validSegments.empty()) return false;

    double scale = scaleHint;
    if (!std::isfinite(scale) || scale <= 0.0) {
        scale = static_cast<double>((maximum - minimum).norm());
    }
    if (!std::isfinite(scale) || scale <= 0.0) {
        scale = std::max(seam.length, 1.0);
    }
    const float connectionTolerance = static_cast<float>(
        std::max(scale * 1e-5, 1e-6));
    const float connectionToleranceSquared =
        connectionTolerance * connectionTolerance;

    const auto appendPath = [&](const std::vector<Eigen::Vector3f>& path) {
        if (path.size() < 2) return;

        auto polyline = vtkSmartPointer<vtkPolyLine>::New();
        polyline->GetPointIds()->SetNumberOfIds(
            static_cast<vtkIdType>(path.size()));
        for (size_t index = 0; index < path.size(); ++index) {
            const auto& point = path[index];
            const vtkIdType pointId = points->InsertNextPoint(
                point.x(), point.y(), point.z());
            polyline->GetPointIds()->SetId(
                static_cast<vtkIdType>(index), pointId);
        }
        lines->InsertNextCell(polyline);
    };

    std::vector<bool> used(validSegments.size(), false);
    for (size_t seed = 0; seed < validSegments.size(); ++seed) {
        if (used[seed]) continue;
        used[seed] = true;
        std::vector<Eigen::Vector3f> path = {
            validSegments[seed].start,
            validSegments[seed].end
        };

        bool extended = true;
        while (extended) {
            extended = false;
            for (size_t index = 0; index < validSegments.size(); ++index) {
                if (used[index]) continue;
                const auto& segment = validSegments[index];
                if ((path.back() - segment.start).squaredNorm() <=
                    connectionToleranceSquared) {
                    path.push_back(segment.end);
                }
                else if ((path.back() - segment.end).squaredNorm() <=
                    connectionToleranceSquared) {
                    path.push_back(segment.start);
                }
                else if ((path.front() - segment.end).squaredNorm() <=
                    connectionToleranceSquared) {
                    path.insert(path.begin(), segment.start);
                }
                else if ((path.front() - segment.start).squaredNorm() <=
                    connectionToleranceSquared) {
                    path.insert(path.begin(), segment.end);
                }
                else {
                    continue;
                }

                used[index] = true;
                extended = true;
                break;
            }
        }
        appendPath(path);
    }

    if (lines->GetNumberOfCells() == 0) return false;

    auto centerline = vtkSmartPointer<vtkPolyData>::New();
    centerline->SetPoints(points);
    centerline->SetLines(lines);

    auto tube = vtkSmartPointer<vtkTubeFilter>::New();
    tube->SetInputData(centerline);
    tube->SetRadius(std::max(scale * 0.0025, 1e-6));
    tube->SetNumberOfSides(12);
    tube->CappingOn();
    tube->Update();
    if (!tube->GetOutput() || tube->GetOutput()->GetNumberOfPoints() == 0) {
        return false;
    }

    viewer->removeShape(actorId);
    if (!viewer->addModelFromPolyData(tube->GetOutput(), actorId)) return false;
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        red, green, blue, actorId);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, actorId);

    const auto actor = viewer->getShapeActorMap()->find(actorId);
    if (actor != viewer->getShapeActorMap()->end()) {
        actor->second->SetPickable(0);
        if (auto* tubeActor = vtkActor::SafeDownCast(actor->second)) {
            tubeActor->GetProperty()->SetAmbient(0.65);
            tubeActor->GetProperty()->SetDiffuse(0.35);
        }
    }
    viewer->spinOnce(50, true);
    return true;
}

void VisualizationManager::showSeam(const WeldSeamData& seam)
{
    const bool selected = seam.id == m_selectedSeamId;
    const std::string actorId = seamActorId(seam.id);
    if (showSeamTube(
        m_cadViewer, seam, actorId, m_cadDiagonal,
        selected ? 1.0 : 0.1,
        selected ? 0.15 : 1.0,
        selected ? 0.75 : 0.1)) {
        setSeamActorAppearance(
            m_cadViewer, actorId,
            selected ? 1.0 : 0.1,
            selected ? 0.15 : 1.0,
            selected ? 0.75 : 0.1,
            selected);
    }
}

void VisualizationManager::removeSeam(const WeldSeamData& seam)
{
    if (!m_cadViewer) return;
    m_cadViewer->removeShape(seamActorId(seam.id));
    m_cadViewer->spinOnce(20, true);
}

void VisualizationManager::showMappedSeam(const WeldSeamData& seam)
{
    const bool selected = seam.id == m_selectedSeamId;
    const std::string actorId = mappedSeamActorId(seam.id);
    if (showSeamTube(
        m_scanViewer, seam, actorId, m_cadDiagonal,
        1.0,
        selected ? 0.15 : 0.72,
        selected ? 0.75 : 0.05)) {
        m_mappedSeamActorIds.insert(actorId);
        setSeamActorAppearance(
            m_scanViewer, actorId,
            1.0,
            selected ? 0.15 : 0.72,
            selected ? 0.75 : 0.05,
            selected);
    }
}

void VisualizationManager::removeMappedSeam(const std::string& seamId)
{
    const std::string actorId = mappedSeamActorId(seamId);
    m_mappedSeamActorIds.erase(actorId);
    if (!m_scanViewer) return;
    m_scanViewer->removeShape(actorId);
    m_scanViewer->spinOnce(20, true);
}

void VisualizationManager::clearMappedSeams()
{
    if (m_scanViewer) {
        for (const auto& actorId : m_mappedSeamActorIds) {
            m_scanViewer->removeShape(actorId);
        }
        if (!m_mappedSeamActorIds.empty()) {
            m_scanViewer->spinOnce(20, true);
        }
    }
    m_mappedSeamActorIds.clear();
}

void VisualizationManager::selectSeam(const std::string& seamId)
{
    if (seamId == m_selectedSeamId) return;

    if (!m_selectedSeamId.empty()) {
        setSeamActorAppearance(
            m_cadViewer, seamActorId(m_selectedSeamId),
            0.1, 1.0, 0.1, false);
        setSeamActorAppearance(
            m_scanViewer, mappedSeamActorId(m_selectedSeamId),
            1.0, 0.72, 0.05, false);
    }

    m_selectedSeamId = seamId;
    if (!m_selectedSeamId.empty()) {
        setSeamActorAppearance(
            m_cadViewer, seamActorId(m_selectedSeamId),
            1.0, 0.15, 0.75, true);
        setSeamActorAppearance(
            m_scanViewer, mappedSeamActorId(m_selectedSeamId),
            1.0, 0.15, 0.75, true);
    }

    if (m_cadViewer && m_cadViewer->getRenderWindow()) {
        m_cadViewer->getRenderWindow()->Render();
    }
    if (m_scanViewer && m_scanViewer->getRenderWindow()) {
        m_scanViewer->getRenderWindow()->Render();
    }
}

void VisualizationManager::clearSeamSelection()
{
    selectSeam(std::string());
}

void VisualizationManager::update()
{
    appendNextScanChunk();
    if (m_cadViewer) m_cadViewer->spinOnce(10, true);
    if (m_scanViewer) m_scanViewer->spinOnce(10, true);
}

bool VisualizationManager::isScanRendering() const
{
    return m_renderedScanPoints < m_totalScanPoints;
}

size_t VisualizationManager::renderedScanPointCount() const
{
    return m_renderedScanPoints;
}

size_t VisualizationManager::totalScanPointCount() const
{
    return m_totalScanPoints;
}
