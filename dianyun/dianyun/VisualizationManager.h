#pragma once

#include "AppTypes.h"
#include "WeldTypes.h"

#include <Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>

class VisualizationManager
{
public:
    using ViewerPtr = pcl::visualization::PCLVisualizer::Ptr;

    void setViewers(const ViewerPtr& cadViewer, const ViewerPtr& scanViewer);
    void showCadModel(const CadModelData& model);
    void showScanCloud(const CloudRGB::Ptr& cloud, double diagonal = 0.0);
    void showProcessedScan(const CloudRGB::Ptr& cloud);
    void showAlignedCad(const CloudRGB::Ptr& cadCloud, const Eigen::Matrix4f& cadToScan);
    void highlightFace(const CadModelData& model, int faceId, const std::string& actorId);
    void clearFaceHighlights();
    void showSeam(const WeldSeamData& seam);
    void removeSeam(const WeldSeamData& seam);
    void update();
    bool isScanRendering() const;
    size_t renderedScanPointCount() const;
    size_t totalScanPointCount() const;

private:
    static double coordinateScale(double diagonal);
    void appendNextScanChunk();
    void resetProgressiveScanState();

    ViewerPtr m_cadViewer;
    ViewerPtr m_scanViewer;
    CloudRGB::Ptr m_progressiveScanCloud;
    size_t m_nextScanPoint = 0;
    size_t m_renderedScanPoints = 0;
    size_t m_totalScanPoints = 0;
    size_t m_scanChunkIndex = 0;
    size_t m_scanChunkSize = 100000;
    double m_scanDiagonal = 0.0;
};
