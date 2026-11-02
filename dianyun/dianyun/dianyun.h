#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "AppTypes.h"
#include "DeviceIndicator.h"
#include "HikCameraWorker.h"
#include "MatchTypes.h"
#include "MatchRequestTracker.h"
#include "ModelMatchController.h"
#include "PointCloudLoadController.h"
#include "RobotTypes.h"
#include "RobotWorker.h"
#include "StepModelLoader.h"
#include "VisualizationManager.h"
#include "WeldSeamService.h"
#include "WeldTypes.h"

#include <QtWidgets/QMainWindow>
#include <QAction>
#include <QLabel>
#include <QListWidget>
#include <QResizeEvent>
#include <QSet>
#include <QShowEvent>
#include <QThread>
#include <QTimer>
#include <QTreeWidget>

#include <pcl/visualization/pcl_visualizer.h>

#include <unordered_map>
#include <vector>

namespace Ui { class dianyunClass; }

class dianyun : public QMainWindow
{
    Q_OBJECT

public:
    explicit dianyun(QWidget* parent = nullptr);
    ~dianyun() override;

    enum class LogLevel { Normal, Success, Warning, Error, Info };
    enum class AlarmLevel { Info, Warning, Error };

protected:
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;

private slots:
    void on_btnOpen_clicked();
    void on_btnOpen_2_clicked();
    void on_btnOpenCamera_clicked();
    void on_btnRobotWeld_clicked();
    void onCameraPointCloudReady(CloudRGB::Ptr newCloud);
    void onCameraLinkStateChanged(HikCameraWorker::State state,
        const QString& detail);
    void onCameraCaptureFailed(const QString& errorMessage);
    void onPointCloudLoadFinished(CloudRGB::Ptr cloud, double diagonal);
    void onPointCloudLoadFailed(QString errorMessage);
    void onPointCloudLoadBusyChanged(bool busy);

    void on_btnOpen_3_clicked();
    void onMatchProgress(quint64 requestId, QString stage);
    void onProcessedScanReady(quint64 requestId, CloudRGB::Ptr processedCloud);
    void onMatchFinished(quint64 requestId, MatchResult result);

    void on_btnNewSeam_clicked();
    void on_btnMapSeam_clicked();
    void on_btnSeamStart_clicked();
    void on_btnSeamReverse_clicked();
    void on_btnCancel_clicked();
    void on_btnDeleteSeam_clicked();
    void on_btnAckAlarm_clicked();
    void on_btnFitView_clicked();
    void on_listWeldSeams_currentItemChanged(
        QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void updatePCLWindow();

    void onCameraConnectAction();
    void onCameraDisconnectAction();
    void onCameraDeviceInfo();
    void onCommSelfCheck();

    void onRobotLinkStateChanged(RobotWorker::State state,
        const QString& detail);
    void onRobotTelemetryUpdated(RobotTelemetry telemetry);
    void onRobotFault(const QString& message);
    void onRobotConnectAction();
    void onRobotDisconnectAction();
    void onRobotStatusInfo();
    void onRobotResetFault();

private:
    enum State { STATE_IDLE, STATE_PICK_FACE_A, STATE_PICK_FACE_B,
                 STATE_PICK_SEAM_START };

    Ui::dianyunClass* ui = nullptr;
    QThread* m_cameraThread = nullptr;
    HikCameraWorker* m_cameraWorker = nullptr;
    QThread* m_robotThread = nullptr;
    RobotWorker* m_robotWorker = nullptr;
    ModelMatchController* m_matchController = nullptr;
    PointCloudLoadController* m_pointCloudLoadController = nullptr;
    DeviceIndicator* m_cameraIndicator = nullptr;
    DeviceIndicator* m_robotIndicator = nullptr;
    QAction* m_actCameraConnect = nullptr;
    QAction* m_actCameraDisconnect = nullptr;
    QAction* m_actCameraInfo = nullptr;
    QAction* m_actRobotConnect = nullptr;
    QAction* m_actRobotDisconnect = nullptr;
    QAction* m_actRobotStatus = nullptr;
    QAction* m_actRobotResetFault = nullptr;

    StepModelLoader m_stepModelLoader;
    WeldSeamService m_weldSeamService;
    VisualizationManager m_visualization;

    CadModelData m_cadModel;
    CloudRGB::Ptr m_scanCloud = CloudRGB::Ptr(new CloudRGB());
    MatchResult m_lastMatchResult;
    MatchRequestTracker m_matchRequests;
    std::vector<WeldSeamData> m_seams;
    std::unordered_map<std::string, WeldSeamData> m_mappedSeams;

    pcl::visualization::PCLVisualizer::Ptr m_cadViewer;
    pcl::visualization::PCLVisualizer::Ptr m_scanViewer;
    QTimer* m_renderTimer = nullptr;

    State m_state = STATE_IDLE;
    DeviceLinkState m_cameraState = DeviceLinkState::Offline;
    DeviceLinkState m_robotState = DeviceLinkState::Offline;
    RobotTelemetry m_robotTelemetry;
    int m_seamCounter = 0;
    int m_selectedFaceA = -1;
    int m_selectedFaceB = -1;
    bool m_reportProgressiveRender = false;
    int m_lastRenderPercent = -1;
    bool m_initialSplitApplied = false;

    QLabel* m_statusStage = nullptr;
    QLabel* m_statusCad = nullptr;
    QLabel* m_statusCloud = nullptr;
    QLabel* m_statusMatch = nullptr;
    QLabel* m_statusSeam = nullptr;

    int m_alarmCount = 0;
    AlarmLevel m_worstAlarmLevel = AlarmLevel::Info;
    QString m_lastAlarmText;
    QSet<QString> m_seenAlarms;

    void initializeViewers();
    void fitToAvailableScreen();
    void setupThemeAssets();
    void setupDeviceIndicators();
    void setupCameraWorker();
    void setupRobotWorker();
    void setupSplitters();
    void applyInitialSplitSizes();
    void setupMenuAndShortcuts();
    void initStatusBar();

    void updateStatusUI();
    void setStatusText(const QString& text,
        AlarmLevel tone = AlarmLevel::Info);

    void showAlarm(AlarmLevel level, const QString& message);
    void clearAlarms();
    void refreshAlarmBanner();

    void showMatchCard(const MatchResult& result);
    void resetMatchCard();
    void refreshDataSummary();

    void appendLog(const QString& message);
    void appendLog(const QString& message, LogLevel level);

    void syncSeamRowMapping(const std::string& seamId);
    void refreshSeamRow(const WeldSeamData& seam);
    void refreshMappedSeam(const std::string& seamId);
    bool rebuildMappedSeam(
        const std::string& seamId, QString& errorMessage);
    WeldSeamData* findSeam(const std::string& seamId);
    void resetSeamSelection();
    void clearSeams();
    void clearMappedSeamResults();
    void createSelectedSeam();
    void applySeamStart(const Eigen::Vector3f& anchor);
    void replaceSeamGeometry(
        const std::string& seamId, const WeldSeamData& updated);
    const WeldSeamData* selectedSeam() const;
    void invalidateMatchResult();
    void updateActionStates();

    void mouseEventOccurred(
        const pcl::visualization::MouseEvent& event, void* args);
};
