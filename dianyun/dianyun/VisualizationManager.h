#pragma once

#include "AppTypes.h"
#include "WeldTypes.h"

#include <Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>

#include <unordered_set>

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
    void showMappedSeam(const WeldSeamData& seam);
    void removeMappedSeam(const std::string& seamId);
    void clearMappedSeams();
    void selectSeam(const std::string& seamId);
    void clearSeamSelection();
    void update();
    bool isScanRendering() const;
    size_t renderedScanPointCount() const;
    size_t totalScanPointCount() const;

private:
    static double coordinateScale(double diagonal);
    static std::string seamActorId(const std::string& seamId);
    static std::string mappedSeamActorId(const std::string& seamId);
    static void setSeamActorAppearance(
        const ViewerPtr& viewer,
        const std::string& actorId,
        double red,
        double green,
        double blue,
        bool selected);
    static bool showSeamTube(
        const ViewerPtr& viewer,
        const WeldSeamData& seam,
        const std::string& actorId,
        double scaleHint,
        double red,
        double green,
        double blue);
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
    double m_cadDiagonal = 0.0;
    double m_scanDiagonal = 0.0;
    std::unordered_set<std::string> m_mappedSeamActorIds;
    std::string m_selectedSeamId;
};
