#include "VisualizationManager.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <algorithm>
#include <cmath>

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

void VisualizationManager::showCadModel(const CadModelData& model)
{
    if (!m_cadViewer || !model.mesh) return;
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

void VisualizationManager::showSeam(const WeldSeamData& seam)
{
    if (!m_cadViewer) return;
    for (size_t index = 0; index < seam.segments.size(); ++index) {
        const auto& segment = seam.segments[index];
        const pcl::PointXYZ start(
            segment.start.x(), segment.start.y(), segment.start.z());
        const pcl::PointXYZ end(
            segment.end.x(), segment.end.y(), segment.end.z());
        const std::string lineId =
            "seam_line_" + seam.id + "_" + std::to_string(index);
        m_cadViewer->addLine(start, end, 0.1, 1.0, 0.1, lineId);
        m_cadViewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6.0, lineId);
    }
    m_cadViewer->spinOnce(50, true);
}

void VisualizationManager::removeSeam(const WeldSeamData& seam)
{
    if (!m_cadViewer) return;
    for (size_t index = 0; index < seam.segments.size(); ++index) {
        m_cadViewer->removeShape(
            "seam_line_" + seam.id + "_" + std::to_string(index));
    }
    m_cadViewer->spinOnce(20, true);
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
