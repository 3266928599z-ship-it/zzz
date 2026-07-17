#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "AppTypes.h"
#include "HikCameraWorker.h"
#include "MatchTypes.h"
#include "ModelMatchController.h"
#include "PointCloudLoadController.h"
#include "StepModelLoader.h"
#include "VisualizationManager.h"
#include "WeldSeamService.h"
#include "WeldTypes.h"

#include <QtWidgets/QMainWindow>
#include <QListWidget>
#include <QResizeEvent>
#include <QTimer>

#include <pcl/visualization/pcl_visualizer.h>

#include <vector>

namespace Ui { class dianyunClass; }

class dianyun : public QMainWindow
{
    Q_OBJECT

public:
    explicit dianyun(QWidget* parent = nullptr);
    ~dianyun() override;

protected:
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void on_btnOpen_clicked();
    void on_btnOpen_2_clicked();
    void on_btnOpenCamera_clicked();
    void onCameraPointCloudReady(CloudRGB::Ptr newCloud);
    void onPointCloudLoadFinished(CloudRGB::Ptr cloud, double diagonal);
    void onPointCloudLoadFailed(QString errorMessage);
    void onPointCloudLoadBusyChanged(bool busy);

    void on_btnOpen_3_clicked();
    void onMatchProgress(QString stage);
    void onProcessedScanReady(CloudRGB::Ptr processedCloud);
    void onMatchFinished(MatchResult result);

    void on_btnNewSeam_clicked();
    void on_btnCancel_clicked();
    void on_btnDeleteSeam_clicked();
    void on_listWeldSeams_itemClicked(QListWidgetItem* item);
    void updatePCLWindow();

private:
    enum State { STATE_IDLE, STATE_PICK_FACE_A, STATE_PICK_FACE_B };

    Ui::dianyunClass* ui = nullptr;
    HikCameraWorker* m_cameraWorker = nullptr;
    ModelMatchController* m_matchController = nullptr;
    PointCloudLoadController* m_pointCloudLoadController = nullptr;

    StepModelLoader m_stepModelLoader;
    WeldSeamService m_weldSeamService;
    VisualizationManager m_visualization;

    CadModelData m_cadModel;
    CloudRGB::Ptr m_scanCloud = CloudRGB::Ptr(new CloudRGB());
    MatchResult m_lastMatchResult;
    std::vector<WeldSeamData> m_seams;

    pcl::visualization::PCLVisualizer::Ptr m_cadViewer;
    pcl::visualization::PCLVisualizer::Ptr m_scanViewer;
    QTimer* m_renderTimer = nullptr;

    State m_state = STATE_IDLE;
    int m_seamCounter = 0;
    int m_selectedFaceA = -1;
    int m_selectedFaceB = -1;
    bool m_reportProgressiveRender = false;
    int m_lastRenderPercent = -1;

    void initializeViewers();
    void updateStatusUI();
    void appendLog(const QString& message);
    void resetSeamSelection();
    void clearSeams();
    void createSelectedSeam();

    void mouseEventOccurred(
        const pcl::visualization::MouseEvent& event, void* args);
};
