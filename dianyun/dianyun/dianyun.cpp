#include "dianyun.h"
#include "ui_dianyun.h"

#include "PoseMath.h"
#include "WeldPathPlanner.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QHeaderView>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMetaType>
#include <QScrollBar>
#include <QStyle>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTime>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkCellPicker.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace {

QString deviceStateName(DeviceLinkState state)
{
    switch (state) {
    case DeviceLinkState::Offline:    return QString::fromUtf8("未连接");
    case DeviceLinkState::Connecting: return QString::fromUtf8("连接中");
    case DeviceLinkState::Online:     return QString::fromUtf8("已连接");
    case DeviceLinkState::Busy:       return QString::fromUtf8("运行中");
    case DeviceLinkState::Fault:      return QString::fromUtf8("故障");
    }
    return QString();
}

} // namespace

dianyun::dianyun(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::dianyunClass)
{
    ui->setupUi(this);

    if (ui->widget) {
        ui->widget->setAttribute(Qt::WA_OpaquePaintEvent);
        ui->widget->setAttribute(Qt::WA_PaintOnScreen);
        ui->widget->setAttribute(Qt::WA_NoSystemBackground);
    }
    if (ui->widget_2) {
        ui->widget_2->setAttribute(Qt::WA_OpaquePaintEvent);
        ui->widget_2->setAttribute(Qt::WA_PaintOnScreen);
        ui->widget_2->setAttribute(Qt::WA_NoSystemBackground);
    }

    qRegisterMetaType<CloudRGB::Ptr>("CloudRGB::Ptr");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");
    qRegisterMetaType<MatchResult>("MatchResult");
    // 相机状态在 worker 线程产生，跨线程回传需要注册
    qRegisterMetaType<HikCameraWorker::State>("HikCameraWorker::State");
    qRegisterMetaType<RobotWorker::State>("RobotWorker::State");
    qRegisterMetaType<RobotTelemetry>("RobotTelemetry");
    // 轮廓帧从相机回调线程跨到 GUI 线程拼接
    qRegisterMetaType<ScanFrame::Ptr>("ScanFrame::Ptr");
    qRegisterMetaType<CloudFrame>("CloudFrame");

    setupThemeAssets();
    setupDeviceIndicators();
    setupSplitters();
    setupMenuAndShortcuts();
    initStatusBar();
    fitToAvailableScreen();

    if (ui->warningBanner) ui->warningBanner->setVisible(false);
    if (ui->lblSeamEmpty) ui->lblSeamEmpty->setVisible(true);

    initializeViewers();
    m_visualization.setViewers(m_cadViewer, m_scanViewer);

    m_renderTimer = new QTimer(this);
    connect(m_renderTimer, &QTimer::timeout,
        this, &dianyun::updatePCLWindow);
    m_renderTimer->start(30);

    m_matchController = new ModelMatchController(this);
    connect(m_matchController, &ModelMatchController::progress,
        this, &dianyun::onMatchProgress);
    connect(m_matchController, &ModelMatchController::processedScanReady,
        this, &dianyun::onProcessedScanReady);
    connect(m_matchController, &ModelMatchController::finished,
        this, &dianyun::onMatchFinished);
    connect(m_matchController, &ModelMatchController::busyChanged,
        this, [this](bool) { updateActionStates(); });

    m_pointCloudLoadController = new PointCloudLoadController(this);
    connect(m_pointCloudLoadController, &PointCloudLoadController::progress,
        this, [this](const QString& message) { appendLog(message); });
    connect(m_pointCloudLoadController, &PointCloudLoadController::finished,
        this, &dianyun::onPointCloudLoadFinished);
    connect(m_pointCloudLoadController, &PointCloudLoadController::failed,
        this, &dianyun::onPointCloudLoadFailed);
    connect(m_pointCloudLoadController, &PointCloudLoadController::busyChanged,
        this, &dianyun::onPointCloudLoadBusyChanged);

    setupCameraWorker();
    setupRobotWorker();
    setupScanController();
    setupPathController();

    updateStatusUI();
    resetMatchCard();
    updateActionStates();
    refreshDataSummary();
    appendLog(QString::fromUtf8("系统已启动，操作日志就绪。"));
}


dianyun::~dianyun()
{
    // 先让相机线程收尾，再释放 UI 与其它控制器
    if (m_cameraThread) {
        if (m_cameraWorker && m_cameraThread->isRunning()) {
            HikCameraWorker* worker = m_cameraWorker;
            QMetaObject::invokeMethod(worker,
                [worker]() { worker->disconnectCamera(); },
                Qt::BlockingQueuedConnection);
        }
        m_cameraThread->quit();
        if (!m_cameraThread->wait(5000)) {
            appendLog(QString::fromUtf8("相机线程未能在 5 秒内退出。"),
                LogLevel::Warning);
        }
        m_cameraWorker = nullptr;   // 随线程 finished 信号 deleteLater 释放
        m_cameraThread = nullptr;   // QThread 以 this 为 parent，随后统一析构
    }

    if (m_robotThread) {
        if (m_robotWorker && m_robotThread->isRunning()) {
            RobotWorker* worker = m_robotWorker;
            QMetaObject::invokeMethod(worker,
                [worker]() { worker->disconnectRobot(); },
                Qt::BlockingQueuedConnection);
        }
        m_robotThread->quit();
        if (!m_robotThread->wait(5000)) {
            appendLog(QString::fromUtf8("机械臂线程未能在 5 秒内退出。"),
                LogLevel::Warning);
        }
        m_robotWorker = nullptr;
        m_robotThread = nullptr;
    }

    delete m_pointCloudLoadController;
    m_pointCloudLoadController = nullptr;
    delete m_matchController;
    m_matchController = nullptr;
    delete ui;
}

void dianyun::initializeViewers()
{
    if (ui->widget) {
        auto* vtkWidget = new QVTKOpenGLNativeWidget(ui->widget);
        auto* layout = new QVBoxLayout(ui->widget);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(vtkWidget);

        auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        auto renderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow->AddRenderer(renderer);
        vtkWidget->setRenderWindow(renderWindow);

        m_cadViewer.reset(new pcl::visualization::PCLVisualizer(
            renderer, renderWindow, "Viewer3D", false));
        m_cadViewer->setBackgroundColor(0.09, 0.09, 0.09);
        m_cadViewer->setShowFPS(false);
        m_cadViewer->setupInteractor(
            vtkWidget->interactor(), vtkWidget->renderWindow());
        m_cadViewer->registerMouseCallback(
            &dianyun::mouseEventOccurred, *this);
    }

    if (ui->widget_2) {
        auto* vtkWidget = new QVTKOpenGLNativeWidget(ui->widget_2);
        auto* layout = new QVBoxLayout(ui->widget_2);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(vtkWidget);

        auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        auto renderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow->AddRenderer(renderer);
        vtkWidget->setRenderWindow(renderWindow);

        m_scanViewer.reset(new pcl::visualization::PCLVisualizer(
            renderer, renderWindow, "ViewerCloud", false));
        m_scanViewer->setBackgroundColor(0.09, 0.09, 0.09);
        m_scanViewer->setShowFPS(false);
        m_scanViewer->setupInteractor(
            vtkWidget->interactor(), vtkWidget->renderWindow());
        // PCL 默认 Shift+左键拾取点
        m_scanViewer->registerPointPickingCallback(
            &dianyun::scanPointPicked, *this);
    }
}

void dianyun::fitToAvailableScreen()
{
    // 设计尺寸 1600x950，屏幕不足时按可用区域收敛，避免右栏和状态栏被挤出屏幕
    const QRect available = screen() ? screen()->availableGeometry()
                                     : QRect(0, 0, 1280, 760);
    const int width = qMin(1600, available.width() - 40);
    const int height = qMin(950, available.height() - 40);
    resize(qMax(width, 1100), qMax(height, 700));

    const QRect frame = frameGeometry();
    move(available.center().x() - frame.width() / 2,
        available.center().y() - frame.height() / 2);
}

void dianyun::setupThemeAssets()
{
    // 相机与机械臂按钮的图标由 DeviceIndicator 按状态着色，此处不再设置
    struct IconSpec { QToolButton* button; const char* path; };
    const IconSpec icons[] = {
        { ui->btnOpen,       ":/dianyun/icons/import-cad.svg"   },
        { ui->btnOpen_2,     ":/dianyun/icons/import-cloud.svg" },
        { ui->btnOpen_3,     ":/dianyun/icons/match.svg"        },
        { ui->btnFitView,    ":/dianyun/icons/fit-view.svg"     },
    };

    for (const auto& spec : icons) {
        if (!spec.button) continue;
        const QIcon icon(QString::fromLatin1(spec.path));
        // SVG 为矢量图形，availableSizes() 可能为空，不能据此判断加载失败
        const bool loaded = !icon.isNull() &&
            !icon.pixmap(QSize(22, 22)).isNull();
        if (!loaded) {
            // SVG 插件缺失时降级为纯文字，不留空白图片位
            spec.button->setToolButtonStyle(Qt::ToolButtonTextOnly);
            spec.button->setMinimumWidth(88);
        }
        else {
            spec.button->setIcon(icon);
        }
    }
}

void dianyun::setupDeviceIndicators()
{
    m_cameraIndicator = new DeviceIndicator(ui->btnOpenCamera, ui->ledCapture,
        QStringLiteral(":/dianyun/icons/camera.svg"),
        QString::fromUtf8("连接相机"),
        QString::fromUtf8("开始拍摄"),
        QString::fromUtf8("停止拍摄"),
        this);

    m_robotIndicator = new DeviceIndicator(ui->btnRobotWeld, ui->ledRobotWeld,
        QStringLiteral(":/dianyun/icons/robot-arm.svg"),
        QString::fromUtf8("连接机械臂"),
        QString::fromUtf8("机械臂就绪"),
        QString::fromUtf8("机械臂运行"),
        this);
}

void dianyun::setupCameraWorker()
{
    m_cameraThread = new QThread(this);
    m_cameraWorker = new HikCameraWorker;   // 无 parent，随线程 finished 释放
    m_cameraWorker->moveToThread(m_cameraThread);

    connect(m_cameraThread, &QThread::finished,
        m_cameraWorker, &QObject::deleteLater);
    connect(m_cameraWorker, &HikCameraWorker::linkStateChanged,
        this, &dianyun::onCameraLinkStateChanged);
    connect(m_cameraWorker, &HikCameraWorker::pointCloudReady,
        this, &dianyun::onCameraPointCloudReady);
    connect(m_cameraWorker, &HikCameraWorker::captureFailed,
        this, &dianyun::onCameraCaptureFailed);
    connect(m_cameraWorker, &HikCameraWorker::captureFinished,
        this, [this]() { updateActionStates(); });

    m_cameraThread->start();
}

void dianyun::setupRobotWorker()
{
    m_robotThread = new QThread(this);
    m_robotWorker = new RobotWorker;   // 无 parent，随线程 finished 释放
    m_robotWorker->moveToThread(m_robotThread);

    connect(m_robotThread, &QThread::finished,
        m_robotWorker, &QObject::deleteLater);
    connect(m_robotWorker, &RobotWorker::linkStateChanged,
        this, &dianyun::onRobotLinkStateChanged);
    connect(m_robotWorker, &RobotWorker::telemetryUpdated,
        this, &dianyun::onRobotTelemetryUpdated);
    connect(m_robotWorker, &RobotWorker::faultRaised,
        this, &dianyun::onRobotFault);

    m_robotThread->start();
}

void dianyun::setupScanController()
{
    // 编排器活在 GUI 线程：它只收帧、算拼接、发指令，
    // 真正的 SDK 调用都在两个 worker 线程里，故不需要独立线程。
    m_scanController = new ScanController(m_cameraWorker, m_robotWorker, this);
    connect(m_scanController, &ScanController::progress,
        this, &dianyun::onScanProgress);
    connect(m_scanController, &ScanController::finished,
        this, &dianyun::onScanFinished);
    connect(m_scanController, &ScanController::failed,
        this, &dianyun::onScanFailed);
    loadHandEyeConfig();
}

void dianyun::setupPathController()
{
    m_pathController = new WeldPathController(m_robotWorker, this);
    connect(m_pathController, &WeldPathController::progress,
        this, &dianyun::onPathProgress);
    connect(m_pathController, &WeldPathController::finished,
        this, &dianyun::onPathFinished);
    connect(m_pathController, &WeldPathController::failed,
        this, &dianyun::onPathFailed);
}

void dianyun::loadHandEyeConfig()
{
    QString message;
    m_handEyeValid = HandEyeConfig::load(
        HandEyeConfig::defaultPath(), m_handEye, message);
    if (m_handEyeValid) {
        if (m_scanController) m_scanController->setHandEye(m_handEye);
        const double* h = m_handEye.tcpToSensor;
        appendLog(QString::fromUtf8(
            "%1\n手眼 T_tcp_sensor：%2, %3, %4 mm / %5, %6, %7°；"
            "sensorAxis 为单位阵：%8；时间补偿 %9 µs。")
            .arg(message)
            .arg(h[0], 0, 'f', 3).arg(h[1], 0, 'f', 3).arg(h[2], 0, 'f', 3)
            .arg(h[3], 0, 'f', 3).arg(h[4], 0, 'f', 3).arg(h[5], 0, 'f', 3)
            .arg(m_handEye.sensorAxis.isIdentity(1e-9)
                ? QString::fromUtf8("是") : QString::fromUtf8("否"))
            .arg(m_handEye.timeOffsetUs, 0, 'f', 0),
            LogLevel::Info);
    }
    else {
        // 标定文件损坏时宁可不出基坐标点云，也不能用默认值悄悄驱动机械臂
        if (m_scanController) m_scanController->clearHandEye();
        appendLog(message + QString::fromUtf8(
            "\n扫描将只输出传感器坐标点云，焊缝轨迹执行已禁用。"),
            LogLevel::Error);
        showAlarm(AlarmLevel::Error, message);
    }
}

void dianyun::setupSplitters()
{
    // 分栏比例依赖窗口真实尺寸，构造期宽度尚未确定，统一放到 showEvent 中应用
    ui->workspaceSplitter->setStretchFactor(0, 0);
    ui->workspaceSplitter->setStretchFactor(1, 1);
    ui->workspaceSplitter->setStretchFactor(2, 0);

    ui->viewportSplitter->setStretchFactor(0, 1);
    ui->viewportSplitter->setStretchFactor(1, 1);

    ui->mainSplitter->setStretchFactor(0, 1);
    ui->mainSplitter->setStretchFactor(1, 0);

    QHeaderView* header = ui->listWeldSeams->header();
    header->setSectionResizeMode(0, QHeaderView::Stretch);
    header->setSectionResizeMode(1, QHeaderView::Fixed);
    header->setSectionResizeMode(2, QHeaderView::Fixed);
    header->setSectionResizeMode(3, QHeaderView::Fixed);
    header->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    ui->listWeldSeams->setColumnWidth(1, 68);
    ui->listWeldSeams->setColumnWidth(2, 62);
    ui->listWeldSeams->setColumnWidth(3, 52);
}

void dianyun::applyInitialSplitSizes()
{
    // 左焊缝栏固定、右操作栏固定，中部视口吃掉剩余宽度
    const int wsWidth = ui->workspaceSplitter->width();
    if (wsWidth > 0) {
        const int sideWidth = qBound(280, 300, qMax(280, wsWidth / 4));
        const int viewportWidth = qMax(360, wsWidth - sideWidth * 2);
        ui->workspaceSplitter->setSizes(
            { sideWidth, viewportWidth, sideWidth });
    }

    const int vpWidth = ui->viewportSplitter->width();
    if (vpWidth > 0) {
        ui->viewportSplitter->setSizes({ vpWidth / 2, vpWidth - vpWidth / 2 });
    }

    const int msHeight = ui->mainSplitter->height();
    if (msHeight > 0) {
        const int logHeight = qBound(150, 190, qMax(150, msHeight / 4));
        ui->mainSplitter->setSizes(
            { qMax(240, msHeight - logHeight), logHeight });
    }
}

void dianyun::setupMenuAndShortcuts()
{
    QMenu* fileMenu = ui->menuBar->addMenu(QString::fromUtf8("文件"));
    fileMenu->addAction(QString::fromUtf8("导入三维模型"),
        QKeySequence::Open, this, &dianyun::on_btnOpen_clicked);
    fileMenu->addAction(QString::fromUtf8("导入点云模型"),
        QKeySequence(QStringLiteral("Ctrl+Shift+O")),
        this, &dianyun::on_btnOpen_2_clicked);
    fileMenu->addAction(QString::fromUtf8("模型匹配"),
        QKeySequence(QStringLiteral("Ctrl+M")),
        this, &dianyun::on_btnOpen_3_clicked);
    fileMenu->addSeparator();
    fileMenu->addAction(QString::fromUtf8("退出"),
        QKeySequence::Quit, this, &QWidget::close);

    QMenu* viewMenu = ui->menuBar->addMenu(QString::fromUtf8("视图"));
    viewMenu->addAction(QString::fromUtf8("视图复位"),
        QKeySequence(Qt::Key_F5), this, &dianyun::on_btnFitView_clicked);

    QMenu* commMenu = ui->menuBar->addMenu(QString::fromUtf8("通信"));
    m_actCameraConnect = commMenu->addAction(
        QString::fromUtf8("连接相机"),
        QKeySequence(QStringLiteral("Ctrl+Shift+C")),
        this, &dianyun::onCameraConnectAction);
    m_actCameraDisconnect = commMenu->addAction(
        QString::fromUtf8("断开相机"),
        QKeySequence(QStringLiteral("Ctrl+Shift+X")),
        this, &dianyun::onCameraDisconnectAction);
    m_actCameraInfo = commMenu->addAction(
        QString::fromUtf8("相机设备信息"),
        this, &dianyun::onCameraDeviceInfo);
    commMenu->addSeparator();

    QMenu* robotMenu = commMenu->addMenu(QString::fromUtf8("机械臂"));
    m_actRobotConnect = robotMenu->addAction(
        QString::fromUtf8("连接机械臂"),
        this, &dianyun::onRobotConnectAction);
    m_actRobotDisconnect = robotMenu->addAction(
        QString::fromUtf8("断开机械臂"),
        this, &dianyun::onRobotDisconnectAction);
    m_actRobotStatus = robotMenu->addAction(
        QString::fromUtf8("机器人状态"),
        this, &dianyun::onRobotStatusInfo);
    m_actRobotResetFault = robotMenu->addAction(
        QString::fromUtf8("清除报警"),
        this, &dianyun::onRobotResetFault);
    robotMenu->addSeparator();
    m_actScanStart = robotMenu->addAction(
        QString::fromUtf8("开始扫描采集"),
        QKeySequence(QStringLiteral("Ctrl+Shift+S")),
        this, &dianyun::onScanRequestAction);
    m_actScanCancel = robotMenu->addAction(
        QString::fromUtf8("取消扫描"),
        this, [this]() {
            if (m_scanController) m_scanController->cancel();
        });
    robotMenu->addSeparator();
    m_actMoveAbovePick = robotMenu->addAction(
        QString::fromUtf8("移到点选位置上方 30 mm（标定核对）"),
        this, &dianyun::onMoveAbovePickedPoint);
    m_actWeldDryRun = robotMenu->addAction(
        QString::fromUtf8("焊缝空走（抬高 20 mm）"),
        this, &dianyun::onWeldDryRun);
    m_actWeldExecute = robotMenu->addAction(
        QString::fromUtf8("沿焊缝执行"),
        this, &dianyun::onWeldExecute);
    m_actPathCancel = robotMenu->addAction(
        QString::fromUtf8("停止轨迹运动"),
        QKeySequence(Qt::Key_Escape),
        this, [this]() {
            if (m_pathController) m_pathController->cancel();
        });
    robotMenu->addSeparator();
    m_actReloadCalibration = robotMenu->addAction(
        QString::fromUtf8("重新加载手眼标定"),
        this, &dianyun::onReloadCalibration);

    commMenu->addSeparator();
    commMenu->addAction(QString::fromUtf8("通信自检"),
        QKeySequence(QStringLiteral("Ctrl+Shift+D")),
        this, &dianyun::onCommSelfCheck);

    QMenu* helpMenu = ui->menuBar->addMenu(QString::fromUtf8("帮助"));
    helpMenu->addAction(QString::fromUtf8("操作说明"), this, [this]() {
        QMessageBox::information(this, QString::fromUtf8("操作说明"),
            QString::fromUtf8(
                "1. 导入三维模型（STEP/STP）\n"
                "2. 导入点云模型（PLY/PCD）\n"
                "3. 执行模型匹配，等待结论\n"
                "4. 点击 + 新建焊缝，按住 Ctrl 在三维视图中单击两个面\n"
                "5. 选中焊缝后点击焊缝映射"));
    });
    helpMenu->addAction(QString::fromUtf8("关于"), this, [this]() {
        QMessageBox::about(this, QString::fromUtf8("关于 dianyun"),
            QString::fromUtf8(
                "dianyun 焊接视觉系统\nQt 6 / PCL / OpenCascade / VTK"));
    });
}

void dianyun::initStatusBar()
{
    auto makeLabel = [](const QString& role) {
        auto* label = new QLabel;
        label->setProperty("role", role);
        return label;
    };

    m_statusStage = makeLabel(QStringLiteral("statusItem"));
    m_statusCad = makeLabel(QStringLiteral("statusValue"));
    m_statusCloud = makeLabel(QStringLiteral("statusValue"));
    m_statusMatch = makeLabel(QStringLiteral("statusValue"));
    m_statusSeam = makeLabel(QStringLiteral("statusValue"));

    ui->statusBar->addWidget(m_statusStage);

    auto* right = new QWidget(ui->statusBar);
    auto* row = new QHBoxLayout(right);
    row->setContentsMargins(0, 0, 8, 0);
    row->setSpacing(16);
    row->addWidget(m_statusCad);
    row->addWidget(m_statusCloud);
    row->addWidget(m_statusMatch);
    row->addWidget(m_statusSeam);
    ui->statusBar->addPermanentWidget(right);
}

void dianyun::setStatusText(const QString& text, AlarmLevel tone)
{
    if (ui->lblStatus) {
        ui->lblStatus->setText(text);
    }
    if (m_statusStage) {
        m_statusStage->setText(QString::fromUtf8("● ") + text);
    }
    if (ui->lblStatusDot) {
        const char* color =
            tone == AlarmLevel::Error ? "#E53935" :
            tone == AlarmLevel::Warning ? "#F57C00" : "#2AC44E";
        ui->lblStatusDot->setStyleSheet(
            QStringLiteral("color: %1; font-size: 10pt;")
                .arg(QString::fromLatin1(color)));
    }
}

void dianyun::showAlarm(AlarmLevel level, const QString& message)
{
    if (!ui->warningBanner) return;

    if (!m_seenAlarms.contains(message)) {
        m_seenAlarms.insert(message);
        ++m_alarmCount;

        auto* item = new QListWidgetItem(
            QTime::currentTime().toString("[HH:mm:ss] ") + message);
        if (level == AlarmLevel::Error) {
            item->setForeground(QColor(0xE5, 0x39, 0x35));
        }
        else if (level == AlarmLevel::Warning) {
            item->setForeground(QColor(0xF5, 0x7C, 0x00));
        }
        ui->listAlarms->insertItem(0, item);
        while (ui->listAlarms->count() > 200) {
            delete ui->listAlarms->takeItem(ui->listAlarms->count() - 1);
        }
        ui->logTabs->setTabText(1,
            QString::fromUtf8("报警 (%1)").arg(m_alarmCount));
    }

    m_lastAlarmText = message;
    if (level == AlarmLevel::Error) {
        m_worstAlarmLevel = AlarmLevel::Error;
    }
    else if (level == AlarmLevel::Warning &&
        m_worstAlarmLevel != AlarmLevel::Error) {
        m_worstAlarmLevel = AlarmLevel::Warning;
    }
    refreshAlarmBanner();
}

void dianyun::refreshAlarmBanner()
{
    if (!ui->warningBanner) return;

    const bool hasAlarm = m_alarmCount > 0;
    ui->warningBanner->setVisible(hasAlarm);
    if (!hasAlarm) return;

    const bool isError = m_worstAlarmLevel == AlarmLevel::Error;
    ui->warningBanner->setProperty("level",
        isError ? QStringLiteral("error") : QStringLiteral("warning"));
    ui->warningBanner->style()->unpolish(ui->warningBanner);
    ui->warningBanner->style()->polish(ui->warningBanner);

    ui->lblWarningText->setText(m_lastAlarmText);
    ui->lblAlarmCount->setText(
        m_alarmCount > 1 ? QStringLiteral("(%1)").arg(m_alarmCount) : QString());
}

void dianyun::clearAlarms()
{
    m_alarmCount = 0;
    m_worstAlarmLevel = AlarmLevel::Info;
    m_lastAlarmText.clear();
    m_seenAlarms.clear();
    if (ui->listAlarms) ui->listAlarms->clear();
    if (ui->logTabs) ui->logTabs->setTabText(1, QString::fromUtf8("报警"));
    refreshAlarmBanner();
}

void dianyun::on_btnAckAlarm_clicked()
{
    clearAlarms();
    appendLog(QString::fromUtf8("告警已解除。"));
}

void dianyun::on_btnFitView_clicked()
{
    if (m_cadViewer) {
        m_cadViewer->resetCamera();
    }
    if (m_scanViewer) {
        m_scanViewer->resetCamera();
    }
    appendLog(QString::fromUtf8("视图已复位。"));
}

void dianyun::showMatchCard(const MatchResult& result)
{
    QString verdictText;
    QString tone;
    if (!result.success) {
        verdictText = QString::fromUtf8("匹配失败");
        tone = QStringLiteral("fail");
    }
    else if (result.verdict == MatchVerdict::Pass) {
        verdictText = QString::fromUtf8("匹配通过");
        tone = QStringLiteral("pass");
    }
    else {
        verdictText = QString::fromUtf8("质量不足");
        tone = QStringLiteral("fail");
    }

    ui->lblVerdict->setText(verdictText);
    ui->lblVerdict->setProperty("tone", tone);
    ui->lblVerdict->style()->unpolish(ui->lblVerdict);
    ui->lblVerdict->style()->polish(ui->lblVerdict);

    ui->lblVerdictDot->setText(QString::fromUtf8("●"));
    ui->lblVerdictDot->setStyleSheet(
        QStringLiteral("color: %1; font-size: 9pt;")
            .arg(tone == QStringLiteral("pass") ? QStringLiteral("#2AC44E")
                                                : QStringLiteral("#E53935")));

    ui->lblOverlap->setText(result.success
        ? QString::fromUtf8("覆盖 %1%")
              .arg(result.overlapRatio * 100.0f, 0, 'f', 1)
        : QStringLiteral("—"));
    ui->lblRmse->setText(result.success
        ? QString::number(result.rmse, 'f', 3) : QStringLiteral("—"));
    ui->lblP95->setText(result.success
        ? QString::number(result.p95Residual, 'f', 3) : QStringLiteral("—"));
    ui->lblNormalConsistency->setText(result.success
        ? QString::number(result.normalConsistency * 100.0, 'f', 1) +
              QStringLiteral("%")
        : QStringLiteral("—"));

    if (m_statusMatch) {
        m_statusMatch->setText(QString::fromUtf8("匹配 ") + verdictText);
    }
}

void dianyun::resetMatchCard()
{
    if (!ui->lblVerdict) return;

    ui->lblVerdict->setText(QString::fromUtf8("—"));
    ui->lblVerdict->setProperty("tone", QStringLiteral("none"));
    ui->lblVerdict->style()->unpolish(ui->lblVerdict);
    ui->lblVerdict->style()->polish(ui->lblVerdict);
    ui->lblVerdictDot->setText(QString());
    ui->lblOverlap->setText(QString::fromUtf8("—"));
    ui->lblRmse->setText(QString::fromUtf8("—"));
    ui->lblP95->setText(QString::fromUtf8("—"));
    ui->lblNormalConsistency->setText(QString::fromUtf8("—"));
    if (m_statusMatch) m_statusMatch->setText(QString::fromUtf8("匹配 —"));
}

void dianyun::refreshDataSummary()
{
    const int cadPoints = m_cadModel.sampleCloud
        ? static_cast<int>(m_cadModel.sampleCloud->size()) : 0;
    const int cloudPoints = m_scanCloud
        ? static_cast<int>(m_scanCloud->size()) : 0;

    if (ui->lblCadPoints) {
        ui->lblCadPoints->setText(QString::number(cadPoints));
    }
    if (ui->lblCloudPoints) {
        ui->lblCloudPoints->setText(QString::number(cloudPoints));
    }
    if (ui->lblSeamCount) {
        ui->lblSeamCount->setText(
            QString::number(static_cast<int>(m_seams.size())));
    }
    if (ui->lblMappedCount) {
        ui->lblMappedCount->setText(
            QString::number(static_cast<int>(m_mappedSeams.size())));
    }
    if (m_statusCad) {
        m_statusCad->setText(
            QString::fromUtf8("CAD %1 点").arg(cadPoints));
    }
    if (m_statusCloud) {
        m_statusCloud->setText(
            QString::fromUtf8("点云 %1 点").arg(cloudPoints));
    }
    if (m_statusSeam) {
        m_statusSeam->setText(QString::fromUtf8("焊缝 %1 / 已映射 %2")
            .arg(m_seams.size()).arg(m_mappedSeams.size()));
    }
    if (ui->lblSeamEmpty) {
        ui->lblSeamEmpty->setVisible(m_seams.empty());
    }
}

void dianyun::syncSeamRowMapping(const std::string& seamId)
{
    if (!ui->listWeldSeams) return;
    for (int i = 0; i < ui->listWeldSeams->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = ui->listWeldSeams->topLevelItem(i);
        if (item->data(0, Qt::UserRole).toString().toStdString() == seamId) {
            item->setText(3, QString::fromUtf8("●"));
            item->setForeground(3, QBrush(QColor(0x2A, 0xC4, 0x4E)));
            break;
        }
    }
}

void dianyun::refreshSeamRow(const WeldSeamData& seam)
{
    if (!ui->listWeldSeams) return;
    for (int i = 0; i < ui->listWeldSeams->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = ui->listWeldSeams->topLevelItem(i);
        if (item->data(0, Qt::UserRole).toString().toStdString() != seam.id) {
            continue;
        }

        item->setText(0, (seam.hasStart
            ? QString::fromUtf8("▶ ") : QString()) + seam.name);
        item->setText(1, QString::number(seam.length, 'f', 1));
        item->setText(2, QStringLiteral("A%1·B%2")
            .arg(seam.faceA + 1).arg(seam.faceB + 1));
        const bool mapped = m_mappedSeams.count(seam.id) > 0;
        item->setText(3, mapped
            ? QString::fromUtf8("●") : QString::fromUtf8("○"));
        item->setForeground(3, QBrush(mapped
            ? QColor(0x2A, 0xC4, 0x4E) : QColor(0x4A, 0x4A, 0x50)));

        if (seam.hasStart && seam.points && !seam.points->empty()) {
            const Eigen::Vector3f start =
                seam.points->front().getVector3fMap();
            item->setToolTip(0, QString::fromUtf8(
                "起点 (%1, %2, %3)").arg(start.x(), 0, 'f', 2)
                .arg(start.y(), 0, 'f', 2).arg(start.z(), 0, 'f', 2));
        }
        else {
            item->setToolTip(0, QString::fromUtf8("未设定起焊方向"));
        }
        break;
    }
}

WeldSeamData* dianyun::findSeam(const std::string& seamId)
{
    const auto seam = std::find_if(
        m_seams.begin(), m_seams.end(),
        [&seamId](const WeldSeamData& candidate) {
            return candidate.id == seamId;
        });
    return seam == m_seams.end() ? nullptr : &(*seam);
}

void dianyun::refreshMappedSeam(const std::string& seamId)
{
    // 只有此前已映射过的焊缝才重建，避免改动起点时顺手生成新映射。
    if (m_mappedSeams.count(seamId) == 0) return;

    QString errorMessage;
    if (!rebuildMappedSeam(seamId, errorMessage) &&
        !errorMessage.isEmpty()) {
        appendLog(QString::fromUtf8("焊缝映射更新失败：") + errorMessage,
            LogLevel::Warning);
    }
}

bool dianyun::rebuildMappedSeam(
    const std::string& seamId, QString& errorMessage)
{
    errorMessage.clear();
    const WeldSeamData* seam = findSeam(seamId);
    if (!seam) {
        errorMessage = QString::fromUtf8("焊缝已不存在。");
        return false;
    }

    // 只有当前匹配结果可用时才重建映射，否则静默清除旧结果等待重新匹配。
    if (!m_lastMatchResult.success ||
        m_lastMatchResult.verdict != MatchVerdict::Pass) {
        m_visualization.removeMappedSeam(seamId);
        m_visualization.removeSeamStart(seamId, true);
        m_mappedSeams.erase(seamId);
        errorMessage.clear();
        return false;
    }

    WeldSeamData mappedSeam;
    if (!m_weldSeamService.mapSeam(
        *seam, m_lastMatchResult.T_final, mappedSeam, errorMessage)) {
        m_visualization.removeMappedSeam(seamId);
        m_visualization.removeSeamStart(seamId, true);
        m_mappedSeams.erase(seamId);
        return false;
    }

    m_mappedSeams.insert_or_assign(seamId, mappedSeam);
    m_visualization.showMappedSeam(mappedSeam);
    // 映射结果与 CAD 侧共用起点，但坐标已变换到点云坐标系。
    if (mappedSeam.hasStart) {
        m_visualization.showSeamStart(mappedSeam, true);
    }
    else {
        m_visualization.removeSeamStart(seamId, true);
    }
    syncSeamRowMapping(seamId);
    return true;
}

void dianyun::appendLog(const QString& message)
{
    appendLog(message, LogLevel::Normal);
}

void dianyun::appendLog(const QString& message, LogLevel level)
{
    if (!ui->textLog) return;

    QColor color(0xE4, 0xE4, 0xE7);
    switch (level) {
        case LogLevel::Success: color = QColor(0x2A, 0xC4, 0x4E); break;
        case LogLevel::Warning: color = QColor(0xF5, 0x7C, 0x00); break;
        case LogLevel::Error:   color = QColor(0xE5, 0x39, 0x35); break;
        case LogLevel::Info:    color = QColor(0x29, 0xB6, 0xF6); break;
        default: break;
    }

    QTextCharFormat format;
    format.setForeground(color);

    QTextCursor cursor(ui->textLog->document());
    cursor.movePosition(QTextCursor::End);
    if (!ui->textLog->document()->isEmpty()) {
        cursor.insertBlock();
    }
    cursor.setCharFormat(format);
    cursor.insertText(
        QTime::currentTime().toString("[HH:mm:ss] ") + message);
    QScrollBar* bar = ui->textLog->verticalScrollBar();
    if (bar) bar->setValue(bar->maximum());
}

void dianyun::updateStatusUI()
{
    QString status;
    AlarmLevel tone = AlarmLevel::Info;
    if (m_state == STATE_IDLE) {
        status = QString::fromUtf8("就绪");
    }
    else if (m_state == STATE_PICK_FACE_A) {
        status = QString::fromUtf8("请选择面 A（按住 Ctrl 单击）");
        tone = AlarmLevel::Warning;
    }
    else if (m_state == STATE_PICK_FACE_B) {
        status = QString::fromUtf8("请选择面 B（按住 Ctrl 单击）");
        tone = AlarmLevel::Warning;
    }
    else {
        status = QString::fromUtf8("请按住 Ctrl 单击焊缝起点端");
        tone = AlarmLevel::Warning;
    }
    setStatusText(status, tone);
}


void dianyun::resetSeamSelection()
{
    m_state = STATE_IDLE;
    m_selectedFaceA = -1;
    m_selectedFaceB = -1;
    m_visualization.clearFaceHighlights();
    updateStatusUI();
}

void dianyun::clearSeams()
{
    m_visualization.clearSeamSelection();
    clearMappedSeamResults();
    for (const auto& seam : m_seams) {
        m_visualization.removeSeam(seam);
    }
    m_seams.clear();
    m_seamCounter = 0;
    if (ui->listWeldSeams) ui->listWeldSeams->clear();
    resetSeamSelection();
    updateActionStates();
    refreshDataSummary();
}

void dianyun::clearMappedSeamResults()
{
    for (const auto& entry : m_mappedSeams) {
        m_visualization.removeSeamStart(entry.first, true);
    }
    m_visualization.clearMappedSeams();
    m_mappedSeams.clear();
    updateActionStates();
}

const WeldSeamData* dianyun::selectedSeam() const
{
    if (!ui->listWeldSeams || !ui->listWeldSeams->currentItem()) {
        return nullptr;
    }

    const std::string seamId = ui->listWeldSeams->currentItem()
        ->data(0, Qt::UserRole).toString().toStdString();
    const auto seam = std::find_if(
        m_seams.cbegin(), m_seams.cend(),
        [&seamId](const WeldSeamData& candidate) {
            return candidate.id == seamId;
        });
    return seam == m_seams.cend() ? nullptr : &(*seam);
}

void dianyun::invalidateMatchResult()
{
    m_matchRequests.invalidateData();
    m_lastMatchResult = MatchResult();
    clearMappedSeamResults();
}

void dianyun::updateActionStates()
{
    if (!ui) return;
    const bool matchRunning = m_matchController &&
        m_matchController->isRunning();
    const bool cloudLoading = m_pointCloudLoadController &&
        m_pointCloudLoadController->isRunning();
    const bool cameraCapturing = m_cameraWorker &&
        m_cameraWorker->isCapturing();
    const bool scanning = m_scanController && m_scanController->isRunning();
    // 采集期间禁止其它数据操作，但相机按钮自身必须保持可点击（用于停止）
    const bool dataActionsEnabled =
        !matchRunning && !cloudLoading && !cameraCapturing;
    // 扫描期间相机按钮要禁用：此时点它会停掉扫描的持续采集
    const bool cameraButtonEnabled =
        !matchRunning && !cloudLoading && !scanning;

    if (ui->btnOpen) ui->btnOpen->setEnabled(dataActionsEnabled);
    if (ui->btnOpen_2) ui->btnOpen_2->setEnabled(dataActionsEnabled);
    if (ui->btnOpenCamera) {
        ui->btnOpenCamera->setEnabled(cameraButtonEnabled);
    }
    if (ui->btnOpen_3) {
        ui->btnOpen_3->setEnabled(dataActionsEnabled &&
            !m_reportProgressiveRender && m_cadModel.isValid() &&
            m_scanCloud && !m_scanCloud->empty());
    }
    if (ui->btnMapSeam) {
        ui->btnMapSeam->setEnabled(dataActionsEnabled &&
            selectedSeam() && m_lastMatchResult.success &&
            m_lastMatchResult.verdict == MatchVerdict::Pass);
    }
    const bool seamSelected = dataActionsEnabled && selectedSeam() != nullptr;
    if (ui->btnSeamStart) {
        ui->btnSeamStart->setEnabled(seamSelected);
    }
    if (ui->btnSeamReverse) {
        ui->btnSeamReverse->setEnabled(seamSelected);
    }

    // 故障态也允许断开，用于把链路复位回未连接
    const bool cameraOffline = m_cameraState == DeviceLinkState::Offline;
    if (m_actCameraConnect) {
        m_actCameraConnect->setEnabled(cameraOffline ||
            m_cameraState == DeviceLinkState::Fault);
    }
    if (m_actCameraDisconnect) {
        m_actCameraDisconnect->setEnabled(!cameraOffline);
    }
    if (m_actCameraInfo) {
        m_actCameraInfo->setEnabled(m_cameraWorker != nullptr);
    }

    const bool robotOffline = m_robotState == DeviceLinkState::Offline;
    if (m_actRobotConnect) {
        m_actRobotConnect->setEnabled(robotOffline ||
            m_robotState == DeviceLinkState::Fault);
    }
    if (m_actRobotDisconnect) {
        m_actRobotDisconnect->setEnabled(!robotOffline);
    }
    if (m_actRobotStatus) {
        m_actRobotStatus->setEnabled(m_robotWorker != nullptr);
    }
    if (m_actRobotResetFault) {
        m_actRobotResetFault->setEnabled(
            !robotOffline && m_robotState != DeviceLinkState::Connecting);
    }

    // 扫描要求相机与机械臂都在线且都没有在跑别的任务
    const bool scanPossible = m_cameraState == DeviceLinkState::Online
        && m_robotState == DeviceLinkState::Online
        && !matchRunning && !cloudLoading;
    const bool pathRunning = m_pathController && m_pathController->isRunning();
    if (m_actScanStart) {
        m_actScanStart->setEnabled(scanPossible && !scanning && !pathRunning);
    }
    if (m_actScanCancel) m_actScanCancel->setEnabled(scanning);

    // 轨迹运动要求：点云在基坐标、标定有效、机械臂空闲
    const bool motionPossible = m_robotState == DeviceLinkState::Online
        && m_handEyeValid && m_scanCloudFrame == CloudFrame::RobotBase
        && !scanning && !pathRunning && !matchRunning;
    const bool seamReady = motionPossible && selectedSeam() != nullptr
        && m_lastMatchResult.success
        && m_lastMatchResult.verdict == MatchVerdict::Pass;
    if (m_actMoveAbovePick) {
        m_actMoveAbovePick->setEnabled(motionPossible && m_havePickedPoint);
    }
    if (m_actWeldDryRun) m_actWeldDryRun->setEnabled(seamReady);
    if (m_actWeldExecute) m_actWeldExecute->setEnabled(seamReady);
    if (m_actPathCancel) m_actPathCancel->setEnabled(pathRunning);
    if (m_actReloadCalibration) {
        m_actReloadCalibration->setEnabled(!scanning && !pathRunning);
    }
}

void dianyun::mouseEventOccurred(
    const pcl::visualization::MouseEvent& event, void*)
{
    if (event.getType() !=
        pcl::visualization::MouseEvent::MouseButtonPress ||
        event.getButton() != pcl::visualization::MouseEvent::LeftButton ||
        !(event.getKeyboardModifiers() & 2) ||
        !m_cadViewer || m_cadModel.cellToFaceMap.empty()) {
        return;
    }

    vtkRenderWindowInteractor* interactor =
        m_cadViewer->getRenderWindow()->GetInteractor();
    if (!interactor) return;

    auto picker = vtkSmartPointer<vtkCellPicker>::New();
    picker->SetTolerance(0.001);
    vtkRenderer* renderer = m_cadViewer->getRenderWindow()
        ->GetRenderers()->GetFirstRenderer();
    if (!renderer || picker->Pick(
        event.getX(), event.getY(), 0, renderer) == 0) {
        if (m_state == STATE_PICK_SEAM_START) {
            appendLog(QString::fromUtf8(
                "未命中任何模型表面，请按住 Ctrl 单击焊缝所在位置。"),
                LogLevel::Warning);
        }
        return;
    }

    const vtkIdType cellId = picker->GetCellId();
    if (m_state == STATE_PICK_SEAM_START) {
        // 焊缝管道不可拾取，单击命中的是 CAD 网格，直接取其世界坐标。
        double position[3];
        picker->GetPickPosition(position);
        applySeamStart(Eigen::Vector3f(
            static_cast<float>(position[0]),
            static_cast<float>(position[1]),
            static_cast<float>(position[2])));
        return;
    }

    if (cellId < 0 || static_cast<size_t>(cellId) >=
        m_cadModel.cellToFaceMap.size()) {
        return;
    }

    const int faceId = m_cadModel.cellToFaceMap[static_cast<size_t>(cellId)];
    if (m_state == STATE_PICK_FACE_A) {
        m_selectedFaceA = faceId;
        m_visualization.highlightFace(
            m_cadModel, faceId, "highlight_A");
        m_state = STATE_PICK_FACE_B;
        updateStatusUI();
        return;
    }

    if (m_state == STATE_PICK_FACE_B) {
        if (faceId == m_selectedFaceA) {
            QMessageBox::warning(
                this, QString::fromUtf8("提示"),
                QString::fromUtf8("请选择两个不同的CAD面。"));
            return;
        }
        m_selectedFaceB = faceId;
        m_visualization.highlightFace(
            m_cadModel, faceId, "highlight_B");
        createSelectedSeam();
    }
}

void dianyun::createSelectedSeam()
{
    if (m_selectedFaceA < 0 || m_selectedFaceB < 0 ||
        static_cast<size_t>(m_selectedFaceA) >= m_cadModel.faces.size() ||
        static_cast<size_t>(m_selectedFaceB) >= m_cadModel.faces.size()) {
        appendLog(QString::fromUtf8("选面索引无效，无法生成焊缝。"));
        resetSeamSelection();
        return;
    }

    WeldSeamData seam;
    QString errorMessage;
    const int nextIndex = m_seamCounter + 1;
    const bool success = m_weldSeamService.createSeam(
        m_cadModel.faces[m_selectedFaceA],
        m_cadModel.faces[m_selectedFaceB],
        nextIndex, seam, errorMessage);
    if (!success) {
        appendLog(errorMessage, LogLevel::Warning);
        showAlarm(AlarmLevel::Warning, errorMessage);
        resetSeamSelection();
        return;
    }

    seam.faceA = m_selectedFaceA;
    seam.faceB = m_selectedFaceB;
    m_seamCounter = nextIndex;
    m_visualization.showSeam(seam);
    m_seams.push_back(seam);
    if (ui->listWeldSeams) {
        auto* item = new QTreeWidgetItem;
        item->setText(0, seam.name);
        item->setText(1, QString::number(seam.length, 'f', 1));
        item->setText(2, QStringLiteral("A%1·B%2")
            .arg(seam.faceA + 1).arg(seam.faceB + 1));
        item->setText(3, m_mappedSeams.count(seam.id)
            ? QString::fromUtf8("●") : QString::fromUtf8("○"));
        item->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
        item->setTextAlignment(3, Qt::AlignCenter);
        item->setData(0, Qt::UserRole, QString::fromStdString(seam.id));
        item->setForeground(3, QBrush(QColor(0x4A, 0x4A, 0x50)));
        item->setToolTip(0, QString::fromUtf8("未设定起焊方向"));
        ui->listWeldSeams->addTopLevelItem(item);
    }
    appendLog(QString::fromUtf8("已创建%1：长度 %2，%3 个管道段。")
        .arg(seam.name)
        .arg(seam.length, 0, 'f', 3)
        .arg(seam.segments.size()), LogLevel::Success);
    refreshDataSummary();
    resetSeamSelection();
    updateActionStates();
}

void dianyun::on_btnOpen_clicked()
{
    const QString filename = QFileDialog::getOpenFileName(
        this, QString::fromUtf8("导入三维模型"), QString(),
        QString::fromUtf8("STEP Files (*.step *.stp);;All Files (*)"));
    if (filename.isEmpty()) return;

    appendLog(QString::fromUtf8("正在导入三维模型：") +
        QFileInfo(filename).fileName());
    CadModelData loadedModel;
    QString errorMessage;
    if (!m_stepModelLoader.load(filename, loadedModel, errorMessage)) {
        appendLog(QString::fromUtf8("三维模型导入失败：") + errorMessage,
            LogLevel::Error);
        showAlarm(AlarmLevel::Error,
            QString::fromUtf8("三维模型导入失败：") + errorMessage);
        return;
    }

    clearSeams();
    m_cadModel = std::move(loadedModel);
    invalidateMatchResult();
    m_visualization.showCadModel(m_cadModel);
    appendLog(QString::fromUtf8("三维模型导入成功，CAD采样点数：") +
        QString::number(m_cadModel.sampleCloud->size()), LogLevel::Success);
    refreshDataSummary();
}

void dianyun::on_btnOpen_2_clicked()
{
    const QString filename = QFileDialog::getOpenFileName(
        this, QString::fromUtf8("导入点云模型"), QString(),
        QString::fromUtf8("Point Cloud Files (*.ply *.pcd);;All Files (*)"));
    if (filename.isEmpty()) return;

    if (m_pointCloudLoadController->isRunning()) {
        appendLog(QString::fromUtf8("点云文件正在加载，请稍候。"));
        return;
    }

    appendLog(QString::fromUtf8("准备导入完整点云：") +
        QFileInfo(filename).fileName());
    if (!m_pointCloudLoadController->load(filename)) {
        appendLog(QString::fromUtf8("无法启动点云后台加载任务。"));
    }
}

void dianyun::onPointCloudLoadFinished(CloudRGB::Ptr loadedCloud, double diagonal)
{
    if (!loadedCloud || loadedCloud->empty()) {
        onPointCloudLoadFailed(QString::fromUtf8("点云中没有有效三维点。"));
        return;
    }

    m_scanCloud = std::move(loadedCloud);
    // 导入文件不知道坐标系，一律按传感器坐标处理，不允许驱动机械臂
    m_scanCloudFrame = CloudFrame::Sensor;
    m_havePickedPoint = false;
    invalidateMatchResult();
    m_visualization.showScanCloud(m_scanCloud, diagonal);
    m_reportProgressiveRender = true;
    m_lastRenderPercent = -1;
    updateActionStates();
    appendLog(QString::fromUtf8(
        "完整点云后台加载成功，共 %1 点，正在分批显示全部点云...")
        .arg(m_scanCloud->size()), LogLevel::Success);
    setStatusText(QString::fromUtf8("正在显示完整点云"));
    refreshDataSummary();
}

void dianyun::onPointCloudLoadFailed(QString errorMessage)
{
    appendLog(QString::fromUtf8("点云模型导入失败：") + errorMessage,
        LogLevel::Error);
    setStatusText(QString::fromUtf8("点云导入失败"), AlarmLevel::Error);
    showAlarm(AlarmLevel::Error,
        QString::fromUtf8("点云模型导入失败：") + errorMessage);
}

void dianyun::onPointCloudLoadBusyChanged(bool busy)
{
    if (busy) {
        setStatusText(QString::fromUtf8("正在后台加载点云"));
    }
    updateActionStates();
}

void dianyun::on_btnNewSeam_clicked()
{
    if (!m_cadModel.isValid()) {
        appendLog(QString::fromUtf8("请先导入三维模型。"));
        return;
    }
    m_state = STATE_PICK_FACE_A;
    m_selectedFaceA = -1;
    m_selectedFaceB = -1;
    m_visualization.clearFaceHighlights();
    updateStatusUI();
    appendLog(QString::fromUtf8(
        "开始选择焊缝面：请按住 Ctrl 单击三维模型曲面。"));
}

void dianyun::on_btnCancel_clicked()
{
    resetSeamSelection();
}

void dianyun::on_btnSeamStart_clicked()
{
    const WeldSeamData* seam = selectedSeam();
    if (!seam) {
        appendLog(QString::fromUtf8("请先在焊缝列表中选择一条焊缝。"));
        updateActionStates();
        return;
    }
    if (!m_cadModel.isValid()) {
        appendLog(QString::fromUtf8("请先导入三维模型。"));
        return;
    }

    m_state = STATE_PICK_SEAM_START;
    m_selectedFaceA = -1;
    m_selectedFaceB = -1;
    m_visualization.clearFaceHighlights();
    updateStatusUI();
    appendLog(QString::fromUtf8(
        "设定起点：请按住 Ctrl 单击焊缝的起点端点（%）")
        .arg(seam->name));
}

void dianyun::on_btnSeamReverse_clicked()
{
    const WeldSeamData* selected = selectedSeam();
    if (!selected) {
        appendLog(QString::fromUtf8("请先在焊缝列表中选择一条焊缝。"));
        updateActionStates();
        return;
    }

    WeldSeamData reversed;
    QString errorMessage;
    if (!m_weldSeamService.reverseDirection(
        *selected, reversed, errorMessage)) {
        appendLog(QString::fromUtf8("反转焊接方向失败：") + errorMessage,
            LogLevel::Warning);
        showAlarm(AlarmLevel::Warning,
            QString::fromUtf8("反转焊接方向失败：") + errorMessage);
        return;
    }

    const std::string seamId = selected->id;
    const QString name = selected->name;
    replaceSeamGeometry(seamId, reversed);
    refreshMappedSeam(seamId);
    appendLog(QString::fromUtf8("%1 焊接方向已反转，新起点 %2。")
        .arg(name)
        .arg(reversed.points && !reversed.points->empty()
            ? QString::fromUtf8("(%1, %2, %3)")
                .arg(reversed.points->front().x, 0, 'f', 2)
                .arg(reversed.points->front().y, 0, 'f', 2)
                .arg(reversed.points->front().z, 0, 'f', 2)
            : QString::fromUtf8("无效")),
        LogLevel::Success);
}

void dianyun::applySeamStart(const Eigen::Vector3f& anchor)
{
    if (m_state != STATE_PICK_SEAM_START) return;

    const WeldSeamData* selected = selectedSeam();
    if (!selected) {
        appendLog(QString::fromUtf8("焊缝选择已失效，请重新选择。"),
            LogLevel::Warning);
        resetSeamSelection();
        return;
    }

    const double tolerance = std::max(0.01 * m_cadModel.diagonal, 1.0);
    WeldSeamData updated;
    QString errorMessage;
    if (!m_weldSeamService.setStartAt(
        *selected, anchor, tolerance, updated, errorMessage)) {
        appendLog(errorMessage, LogLevel::Warning);
        showAlarm(AlarmLevel::Warning, errorMessage);
        // 保留拾取状态，方便直接重试点击。
        return;
    }

    const std::string seamId = selected->id;
    const QString name = selected->name;
    double distance = 0.0;
    if (updated.points && !updated.points->empty()) {
        distance = static_cast<double>(
            (updated.points->front().getVector3fMap() - anchor).norm());
    }

    replaceSeamGeometry(seamId, updated);
    refreshMappedSeam(seamId);
    appendLog(QString::fromUtf8("%1 起点已设定，距点击点 %2。")
        .arg(name).arg(distance, 0, 'f', 2), LogLevel::Success);
    resetSeamSelection();
}

void dianyun::replaceSeamGeometry(
    const std::string& seamId, const WeldSeamData& updated)
{
    WeldSeamData* seam = findSeam(seamId);
    if (!seam) return;

    const WeldSeamData previous = *seam;
    m_visualization.removeSeam(previous);
    m_visualization.removeMappedSeam(seamId);
    m_mappedSeams.erase(seamId);

    *seam = updated;
    m_visualization.showSeam(*seam);
    m_visualization.removeSeamStart(seam->id, true);
    if (seam->hasStart) {
        m_visualization.showSeamStart(*seam, false);
    }
    else {
        m_visualization.removeSeamStart(seam->id, false);
    }
    refreshSeamRow(*seam);
    refreshDataSummary();
    updateActionStates();
}

void dianyun::on_btnMapSeam_clicked()
{
    const WeldSeamData* seam = selectedSeam();
    if (!seam) {
        appendLog(QString::fromUtf8("请先在焊缝列表中选择一条焊缝。"));
        updateActionStates();
        return;
    }
    if (!m_lastMatchResult.success ||
        m_lastMatchResult.verdict != MatchVerdict::Pass) {
        appendLog(QString::fromUtf8(
            "最新模型匹配尚未通过，不能进行焊缝映射。"));
        updateActionStates();
        return;
    }

    const std::string seamId = seam->id;
    QString mapError;
    if (!rebuildMappedSeam(seamId, mapError)) {
        const QString message = mapError.isEmpty()
            ? QString::fromUtf8("焊缝映射失败：几何数据无效。")
            : QString::fromUtf8("焊缝映射失败：") + mapError;
        appendLog(message, LogLevel::Error);
        showAlarm(AlarmLevel::Error, message);
        return;
    }
    const WeldSeamData& mappedSeam = m_mappedSeams.at(seamId);
    appendLog(QString::fromUtf8(
        "%1映射完成：%2 个采样点，%3 个管道段。")
        .arg(mappedSeam.name)
        .arg(mappedSeam.points ? mappedSeam.points->size() : 0)
        .arg(static_cast<int>(mappedSeam.segments.size())),
        LogLevel::Success);
    refreshDataSummary();
}

void dianyun::on_btnDeleteSeam_clicked()
{
    if (!ui->listWeldSeams) return;
    const WeldSeamData* selected = selectedSeam();
    if (!selected) return;

    QTreeWidgetItem* current = ui->listWeldSeams->currentItem();
    if (!current) return;

    const int row = ui->listWeldSeams->indexOfTopLevelItem(current);
    const std::string seamId = selected->id;
    const QString name = selected->name;
    m_visualization.removeSeam(*selected);
    m_visualization.removeMappedSeam(seamId);
    m_mappedSeams.erase(seamId);
    const auto seam = std::find_if(
        m_seams.begin(), m_seams.end(),
        [&seamId](const WeldSeamData& candidate) {
            return candidate.id == seamId;
        });
    if (seam != m_seams.end()) m_seams.erase(seam);
    delete ui->listWeldSeams->takeTopLevelItem(row);
    appendLog(QString::fromUtf8("已删除焊缝：") + name);
    refreshDataSummary();
    updateActionStates();
}

void dianyun::on_listWeldSeams_currentItemChanged(
    QTreeWidgetItem* current, QTreeWidgetItem*)
{
    if (current) {
        m_visualization.selectSeam(
            current->data(0, Qt::UserRole).toString().toStdString());
    }
    else {
        m_visualization.clearSeamSelection();
    }
    updateActionStates();
}

void dianyun::updatePCLWindow()
{
    m_visualization.update();
    if (!m_reportProgressiveRender) return;

    const size_t total = m_visualization.totalScanPointCount();
    const size_t rendered = m_visualization.renderedScanPointCount();
    const int percent = total == 0 ? 100 : static_cast<int>(
        std::min<size_t>(100, rendered * 100 / total));
    if (percent != m_lastRenderPercent) {
        m_lastRenderPercent = percent;
        setStatusText(
            QString::fromUtf8("正在显示完整点云 %1%").arg(percent));
    }
    if (!m_visualization.isScanRendering()) {
        m_reportProgressiveRender = false;
        appendLog(QString::fromUtf8("完整点云显示完成，共 %1 点。")
            .arg(total), LogLevel::Success);
        setStatusText(QString::fromUtf8("点云就绪"));
        updateActionStates();
    }
}

void dianyun::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
}

void dianyun::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);
    if (m_initialSplitApplied) return;
    m_initialSplitApplied = true;

    applyInitialSplitSizes();

    // 布局在事件循环下一轮才稳定，再校正一次分栏比例
    QTimer::singleShot(0, this, [this]() { applyInitialSplitSizes(); });
}

void dianyun::on_btnOpenCamera_clicked()
{
    if (!m_cameraWorker) return;

    // m_cameraState 是 UI 侧缓存，与 worker 的真实状态在每次切换后同步
    switch (m_cameraState) {
    case DeviceLinkState::Offline:
    case DeviceLinkState::Fault:
        m_cameraState = DeviceLinkState::Connecting;
        m_cameraIndicator->setState(m_cameraState,
            QString::fromUtf8("正在枚举并打开设备"));
        {
            HikCameraWorker* worker = m_cameraWorker;
            QMetaObject::invokeMethod(worker,
                [worker]() { worker->connectCamera(); },
                Qt::QueuedConnection);
        }
        appendLog(QString::fromUtf8("正在连接相机…"), LogLevel::Info);
        break;

    case DeviceLinkState::Online:
        // 扫描占用相机时不允许插入单帧拍摄：那会把扫描流掐断
        if (m_scanController && m_scanController->isRunning()) {
            appendLog(QString::fromUtf8("扫描进行中，无法单帧拍摄。"),
                LogLevel::Warning);
            break;
        }
        // 机械臂在线时，「开始拍摄」就是整幅扫描：相机出流 + 机械臂沿工具 X 行进。
        // 只有机械臂没连时才退回单帧拍摄（只得到一条轮廓，不会有位移）。
        if (m_robotState == DeviceLinkState::Online) {
            onScanRequestAction();
            break;
        }
        appendLog(QString::fromUtf8(
            "机械臂未连接，仅单帧拍摄（只有一条轮廓，不会移动）。"),
            LogLevel::Warning);
        {
            HikCameraWorker* worker = m_cameraWorker;
            QMetaObject::invokeMethod(worker, [worker]() {
                worker->m_params.frameCount = 1;
                worker->m_params.captureTimeoutMs = 0;
                worker->startCapture();
            }, Qt::QueuedConnection);
        }
        break;

    case DeviceLinkState::Busy:
        {
            HikCameraWorker* worker = m_cameraWorker;
            QMetaObject::invokeMethod(worker,
                [worker]() { worker->stopCapture(); },
                Qt::QueuedConnection);
        }
        break;

    case DeviceLinkState::Connecting:
        // 枚举过程中重复点击直接忽略，避免并发 OpenDevice
        break;
    }

    updateActionStates();
}

void dianyun::on_btnRobotWeld_clicked()
{
    if (!m_robotWorker) return;

    // m_robotState 是 UI 侧缓存，与 worker 的真实状态在每次切换后同步
    switch (m_robotState) {
    case DeviceLinkState::Offline:
    case DeviceLinkState::Fault:
        m_robotState = DeviceLinkState::Connecting;
        m_robotIndicator->setState(m_robotState,
            QString::fromUtf8("正在连接机器人控制器"));
        {
            RobotWorker* worker = m_robotWorker;
            QMetaObject::invokeMethod(worker,
                [worker]() { worker->connectRobot(); },
                Qt::QueuedConnection);
        }
        appendLog(QString::fromUtf8("正在连接机械臂…"), LogLevel::Info);
        break;

    case DeviceLinkState::Online:
        // 机械臂已就绪时，这个按钮的语义是「开始扫描采集」
        onScanRequestAction();
        break;

    case DeviceLinkState::Busy:
        // 机械臂正在运动：若正是本次扫描或轨迹，则终止；否则只提示
        if (m_scanController && m_scanController->isRunning()) {
            m_scanController->cancel();
        }
        else if (m_pathController && m_pathController->isRunning()) {
            m_pathController->cancel();
        }
        else {
            appendLog(QString::fromUtf8("机械臂正在运行，请等待其到位。"),
                LogLevel::Warning);
        }
        break;

    case DeviceLinkState::Connecting:
        break;
    }

    updateActionStates();
}

void dianyun::onCameraLinkStateChanged(
    HikCameraWorker::State state, const QString& detail)
{
    switch (state) {
    case HikCameraWorker::State::Disconnected:
        m_cameraState = DeviceLinkState::Offline;
        m_cameraIndicator->setState(m_cameraState,
            QString::fromUtf8("未检测到设备"));
        appendLog(QString::fromUtf8("相机已断开。"), LogLevel::Info);
        setStatusText(QString::fromUtf8("就绪"));
        break;

    case HikCameraWorker::State::Connecting:
        m_cameraState = DeviceLinkState::Connecting;
        m_cameraIndicator->setState(m_cameraState,
            QString::fromUtf8("正在枚举并打开设备"));
        break;

    case HikCameraWorker::State::Connected:
        m_cameraState = DeviceLinkState::Online;
        m_cameraIndicator->setState(m_cameraState,
            QString::fromUtf8("已连接：%1（只读，未标定）").arg(detail));
        appendLog(QString::fromUtf8("相机连接成功，SN：%1").arg(detail),
            LogLevel::Success);
        setStatusText(QString::fromUtf8("相机已连接"));
        break;

    case HikCameraWorker::State::Capturing:
        m_cameraState = DeviceLinkState::Busy;
        m_cameraIndicator->setState(m_cameraState,
            QString::fromUtf8("采集中，可点击停止"));
        setStatusText(QString::fromUtf8("采集中"), AlarmLevel::Warning);
        break;

    case HikCameraWorker::State::Fault:
        m_cameraState = DeviceLinkState::Fault;
        m_cameraIndicator->setState(m_cameraState, detail);
        appendLog(QString::fromUtf8("相机故障：%1").arg(detail),
            LogLevel::Error);
        showAlarm(AlarmLevel::Error,
            QString::fromUtf8("相机故障：") + detail);
        setStatusText(QString::fromUtf8("相机故障"), AlarmLevel::Error);
        break;
    }

    updateActionStates();
}

void dianyun::onCameraCaptureFailed(const QString& errorMessage)
{
    appendLog(QString::fromUtf8("采集失败：%1").arg(errorMessage),
        LogLevel::Error);
    showAlarm(AlarmLevel::Error,
        QString::fromUtf8("采集失败：") + errorMessage);
    setStatusText(QString::fromUtf8("采集失败"), AlarmLevel::Error);
    updateActionStates();
}

void dianyun::onCameraConnectAction()
{
    if (m_cameraState == DeviceLinkState::Connecting ||
        m_cameraState == DeviceLinkState::Online ||
        m_cameraState == DeviceLinkState::Busy) {
        appendLog(QString::fromUtf8("相机已连接或正在连接。"));
        return;
    }
    if (!m_cameraWorker) return;

    m_cameraState = DeviceLinkState::Connecting;
    m_cameraIndicator->setState(m_cameraState,
        QString::fromUtf8("正在枚举并打开设备"));
    QMetaObject::invokeMethod(m_cameraWorker,
        &HikCameraWorker::connectCamera, Qt::QueuedConnection);
    appendLog(QString::fromUtf8("正在连接相机…"), LogLevel::Info);
    updateActionStates();
}

void dianyun::onCameraDisconnectAction()
{
    if (!m_cameraWorker || m_cameraState == DeviceLinkState::Offline) {
        appendLog(QString::fromUtf8("相机当前未连接。"));
        return;
    }
    QMetaObject::invokeMethod(m_cameraWorker,
        &HikCameraWorker::disconnectCamera, Qt::QueuedConnection);
    updateActionStates();
}

void dianyun::onCameraDeviceInfo()
{
    if (!m_cameraWorker || !m_cameraThread ||
        !m_cameraThread->isRunning()) {
        appendLog(QString::fromUtf8("相机通信线程未运行。"),
            LogLevel::Warning);
        return;
    }

    // 读序列号需要在 worker 线程内完成，避免与连接流程竞争
    QString serial;
    QString lastError;
    const bool ok = QMetaObject::invokeMethod(m_cameraWorker,
        [this, &serial, &lastError]() {
            serial = m_cameraWorker->deviceSerial();
            lastError = m_cameraWorker->lastError();
        }, Qt::BlockingQueuedConnection);
    if (!ok) {
        appendLog(QString::fromUtf8("读取相机设备信息失败。"),
            LogLevel::Error);
        return;
    }

    const QString info = QString::fromUtf8(
        "状态：%1\n序列号：%2\n最近错误：%3")
        .arg(deviceStateName(m_cameraState),
            serial.isEmpty() ? QString::fromUtf8("（未连接）") : serial,
            lastError.isEmpty() ? QString::fromUtf8("无") : lastError);
    appendLog(QString::fromUtf8("相机设备信息：状态 %1，SN %2")
        .arg(deviceStateName(m_cameraState),
            serial.isEmpty() ? QStringLiteral("-") : serial),
        LogLevel::Info);
    QMessageBox::information(this, QString::fromUtf8("相机设备信息"), info);
}

void dianyun::onCommSelfCheck()
{
    const bool threadRunning = m_cameraThread && m_cameraThread->isRunning();
    const bool workerAlive = m_cameraWorker != nullptr;
    const bool robotThreadRunning = m_robotThread &&
        m_robotThread->isRunning();
    const bool robotAlive = m_robotWorker != nullptr;

    QString report = QString::fromUtf8(
        "相机线程：%1\n相机工作对象：%2\n相机链路：%3\n\n"
        "机械臂线程：%4\n机械臂工作对象：%5\n机械臂链路：%6")
        .arg(threadRunning ? QString::fromUtf8("运行中")
                           : QString::fromUtf8("已停止"),
             workerAlive ? QString::fromUtf8("正常")
                         : QString::fromUtf8("未创建"),
             deviceStateName(m_cameraState),
             robotThreadRunning ? QString::fromUtf8("运行中")
                                : QString::fromUtf8("已停止"),
             robotAlive ? QString::fromUtf8("正常")
                        : QString::fromUtf8("未创建"),
             deviceStateName(m_robotState));

    const bool allGood = threadRunning && workerAlive &&
        robotThreadRunning && robotAlive;
    appendLog(QString::fromUtf8("通信自检：相机 %1 / 机械臂 %2")
        .arg(deviceStateName(m_cameraState),
             deviceStateName(m_robotState)),
        allGood ? LogLevel::Success : LogLevel::Warning);
    QMessageBox::information(this, QString::fromUtf8("通信自检"), report);
}

void dianyun::onRobotLinkStateChanged(
    RobotWorker::State state, const QString& detail)
{
    switch (state) {
    case RobotWorker::State::Disconnected:
        m_robotState = DeviceLinkState::Offline;
        m_robotIndicator->setState(m_robotState,
            QString::fromUtf8("未连接控制器"));
        appendLog(QString::fromUtf8("机械臂已断开。"), LogLevel::Info);
        break;

    case RobotWorker::State::Connecting:
        m_robotState = DeviceLinkState::Connecting;
        m_robotIndicator->setState(m_robotState,
            QString::fromUtf8("正在连接机器人控制器"));
        break;

    case RobotWorker::State::Online:
        m_robotState = DeviceLinkState::Online;
        m_robotIndicator->setState(m_robotState,
            QString::fromUtf8("已连接：%1（只读，未标定）")
                .arg(detail.isEmpty()
                    ? m_robotWorker->controllerIp() : detail));
        appendLog(QString::fromUtf8("机械臂连接成功。"), LogLevel::Success);
        setStatusText(QString::fromUtf8("机械臂已就绪"));
        break;

    case RobotWorker::State::Busy:
        m_robotState = DeviceLinkState::Busy;
        m_robotIndicator->setState(m_robotState,
            QString::fromUtf8("机器人运动中（%1），本阶段只读监控")
                .arg(detail));
        setStatusText(QString::fromUtf8("机械臂运行中"), AlarmLevel::Warning);
        break;

    case RobotWorker::State::Fault:
        m_robotState = DeviceLinkState::Fault;
        m_robotIndicator->setState(m_robotState, detail);
        appendLog(QString::fromUtf8("机械臂故障：%1").arg(detail),
            LogLevel::Error);
        setStatusText(QString::fromUtf8("机械臂故障"), AlarmLevel::Error);
        break;
    }

    updateActionStates();
}

void dianyun::onRobotTelemetryUpdated(RobotTelemetry telemetry)
{
    m_robotTelemetry = telemetry;
    if (telemetry.connected) {
        std::copy_n(telemetry.tcpPose, 6, m_lastTcpPose);
        m_haveTcpPose = true;
    }
    updateActionStates();
}

void dianyun::onRobotFault(const QString& message)
{
    showAlarm(AlarmLevel::Error,
        QString::fromUtf8("机械臂：") + message);
}

void dianyun::onRobotConnectAction()
{
    if (!m_robotWorker) return;
    if (m_robotState == DeviceLinkState::Connecting ||
        m_robotState == DeviceLinkState::Online ||
        m_robotState == DeviceLinkState::Busy) {
        appendLog(QString::fromUtf8("机械臂已连接或正在连接。"));
        return;
    }

    m_robotState = DeviceLinkState::Connecting;
    m_robotIndicator->setState(m_robotState,
        QString::fromUtf8("正在连接机器人控制器"));
    RobotWorker* worker = m_robotWorker;
    QMetaObject::invokeMethod(worker,
        [worker]() { worker->connectRobot(); }, Qt::QueuedConnection);
    appendLog(QString::fromUtf8("正在连接机械臂…"), LogLevel::Info);
    updateActionStates();
}

void dianyun::onRobotDisconnectAction()
{
    if (!m_robotWorker || m_robotState == DeviceLinkState::Offline) {
        appendLog(QString::fromUtf8("机械臂当前未连接。"));
        return;
    }
    RobotWorker* worker = m_robotWorker;
    QMetaObject::invokeMethod(worker,
        [worker]() { worker->disconnectRobot(); }, Qt::QueuedConnection);
    updateActionStates();
}

void dianyun::onRobotStatusInfo()
{
    if (!m_robotWorker || !m_robotThread ||
        !m_robotThread->isRunning()) {
        appendLog(QString::fromUtf8("机械臂通信线程未运行。"),
            LogLevel::Warning);
        return;
    }

    // 遥测缓存由 worker 持有，读一次快照避免与轮询竞争
    RobotTelemetry telemetry;
    const bool ok = QMetaObject::invokeMethod(m_robotWorker,
        [this, &telemetry]() {
            telemetry = m_robotWorker->currentTelemetry();
        }, Qt::BlockingQueuedConnection);
    if (!ok) {
        appendLog(QString::fromUtf8("读取机器人状态失败。"), LogLevel::Error);
        return;
    }

    if (!telemetry.connected) {
        appendLog(QString::fromUtf8("机器人未连接，无实时状态。"),
            LogLevel::Warning);
        QMessageBox::information(this, QString::fromUtf8("机器人状态"),
            QString::fromUtf8("机器人未连接。"));
        return;
    }

    static const char* const kStateName[] = {
        "未知", "停止", "运行", "暂停", "拖动" };
    const int stateIndex = (telemetry.robotState >= 0 &&
        telemetry.robotState <= 4) ? telemetry.robotState : 0;
    const int modeIndex = telemetry.robotMode == 0 ? 0
        : (telemetry.robotMode == 1 ? 1 : -1);

    const QString info = QString::fromUtf8(
        "控制器：%1\n"
        "运动状态：%2\n"
        "程序状态：%3\n"
        "手自动：%4\n"
        "到位信号：%5\n"
        "急停：%6\n"
        "故障码：%7 / %8\n"
        "RPC 耗时：%9 ms\n\n"
        "TCP 位姿：X %10  Y %11  Z %12\n"
        "          RX %13  RY %14  RZ %15")
        .arg(m_robotWorker->controllerIp(),
            QString::fromUtf8(kStateName[stateIndex]),
            telemetry.programState == 2
                ? QString::fromUtf8("运行")
                : QString::fromUtf8("停止/暂停"),
            modeIndex == 0 ? QString::fromUtf8("自动")
                : (modeIndex == 1 ? QString::fromUtf8("手动")
                                  : QString::fromUtf8("未知")),
            telemetry.motionDone ? QString::fromUtf8("已到位")
                                 : QString::fromUtf8("未到位"),
            telemetry.emergencyStop ? QString::fromUtf8("已按下")
                                    : QString::fromUtf8("未按下"),
            QString::number(telemetry.mainCode),
            QString::number(telemetry.subCode),
            QString::number(telemetry.rpcLatencyMs),
            QString::number(telemetry.tcpPose[0], 'f', 2),
            QString::number(telemetry.tcpPose[1], 'f', 2),
            QString::number(telemetry.tcpPose[2], 'f', 2),
            QString::number(telemetry.tcpPose[3], 'f', 2),
            QString::number(telemetry.tcpPose[4], 'f', 2),
            QString::number(telemetry.tcpPose[5], 'f', 2));

    appendLog(QString::fromUtf8("机器人状态：%1，RPC %2 ms")
        .arg(QString::fromUtf8(kStateName[stateIndex]),
             QString::number(telemetry.rpcLatencyMs)),
        LogLevel::Info);
    QMessageBox::information(this, QString::fromUtf8("机器人状态"), info);
}

void dianyun::onRobotResetFault()
{
    if (!m_robotWorker || m_robotState == DeviceLinkState::Offline) {
        appendLog(QString::fromUtf8("机械臂未连接，无法清除报警。"),
            LogLevel::Warning);
        return;
    }

    const auto answer = QMessageBox::question(this,
        QString::fromUtf8("清除报警"),
        QString::fromUtf8(
            "确认清除机器人控制器上的所有报警？\n"
            "该操作不涉及任何运动指令，但会复位控制器故障状态。"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes) return;

    RobotWorker* worker = m_robotWorker;
    QMetaObject::invokeMethod(worker,
        [worker]() { worker->resetFault(); }, Qt::QueuedConnection);
    appendLog(QString::fromUtf8("已请求清除机器人报警。"), LogLevel::Info);
}

void dianyun::onCameraPointCloudReady(CloudRGB::Ptr newCloud)
{
    if (!newCloud || newCloud->empty()) return;
    m_scanCloud = std::move(newCloud);
    m_scanCloudFrame = CloudFrame::Sensor;
    m_havePickedPoint = false;
    invalidateMatchResult();
    m_visualization.showScanCloud(m_scanCloud);
    m_reportProgressiveRender = true;
    m_lastRenderPercent = -1;
    updateActionStates();
    appendLog(QString::fromUtf8("实时拼接完成，正在分批显示，节点数：") +
        QString::number(m_scanCloud->size()), LogLevel::Success);
    refreshDataSummary();
}

void dianyun::onScanRequestAction()
{
    if (!m_scanController) return;

    if (m_cameraState != DeviceLinkState::Online) {
        appendLog(QString::fromUtf8("请先连接相机。"), LogLevel::Warning);
        return;
    }
    if (m_robotState != DeviceLinkState::Online) {
        appendLog(QString::fromUtf8("请先连接机械臂。"), LogLevel::Warning);
        return;
    }
    if (m_reportProgressiveRender) {
        appendLog(QString::fromUtf8("点云仍在分批显示，请稍候。"));
        return;
    }

    // 扫描姿态固定：工具 Z 竖直向下，工具 Y 平行基坐标 +X（Rz = 90°），
    // 高度取基坐标 Z 的绝对值 —— 低处相机视野收不全工件。
    // 先竖直抬到安全高度再转姿态，避免低位斜插的直线穿过工件。
    constexpr double kScanYawDeg = 90.0;
    constexpr double kScanTcpZmm = 150.0;

    double heading = 0.0;   // 仅用于 levelHeading 的倾角检查，扫描时被固定值取代
    double tiltDeg = 0.0;
    if (!levelHeading(heading, tiltDeg)) return;

    const double liftZ = std::max(m_lastTcpPose[2], kScanTcpZmm);
    const bool needLift = std::fabs(m_lastTcpPose[2] - liftZ) > 0.5;

    double liftPose[6] = { m_lastTcpPose[0], m_lastTcpPose[1], liftZ,
        m_lastTcpPose[3], m_lastTcpPose[4], m_lastTcpPose[5] };
    double levelPose[6] = { m_lastTcpPose[0], m_lastTcpPose[1], kScanTcpZmm,
        180.0, 0.0, kScanYawDeg };

    const auto answer = QMessageBox::question(this,
        QString::fromUtf8("开始扫描采集"),
        QString::fromUtf8(
            "① %1；\n"
            "② 调平到基坐标 Z = %2 mm：工具 Z 竖直向下、工具 Y 平行基坐标 +X"
            "（Rx 180 / Ry 0 / Rz %3°）；\n"
            "③ 沿工具 X 轴（基坐标 +Y）直线移动 200 mm，行程约 7 秒；\n"
            "④ 结束后自动退回扫描起点。\n"
            "请确认上方与激光范围内无人无障碍。\n\n"
            "当前位姿：X %4  Y %5  Z %6 (mm)  Rx %7  Ry %8  Rz %9 (deg)\n"
            "扫描位姿：X %4  Y %5  Z %2 (mm)  Rx 180.00  Ry 0.00  Rz %3 (deg)\n"
            "当前工具 Z 偏离竖直向下 %10°。")
            .arg(needLift
                    ? QString::fromUtf8("竖直抬到基坐标 Z = %1 mm")
                        .arg(liftZ, 0, 'f', 2)
                    : QString::fromUtf8("已在安全高度，无需抬升"))
            .arg(kScanTcpZmm, 0, 'f', 2)
            .arg(kScanYawDeg, 0, 'f', 2)
            .arg(m_lastTcpPose[0], 0, 'f', 2)
            .arg(m_lastTcpPose[1], 0, 'f', 2)
            .arg(m_lastTcpPose[2], 0, 'f', 2)
            .arg(m_lastTcpPose[3], 0, 'f', 2)
            .arg(m_lastTcpPose[4], 0, 'f', 2)
            .arg(m_lastTcpPose[5], 0, 'f', 2)
            .arg(tiltDeg, 0, 'f', 1),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes) return;

    ScanRequest request;
    std::copy_n(levelPose, 6, request.startPose);
    std::copy_n(liftPose, 6, request.liftPose);
    request.useStartPose = true;    // 先走到扫描位姿，再起流扫描
    request.useLiftPose = needLift; // 已在安全高度以上则跳过竖直抬升
    request.scanAxis = ScanRequest::ScanAxis::ToolX;
    request.scanLengthMm = 200.0;
    request.stepUm = 96.0f;
    request.lineRate = 600.0f;
    request.exposureUs = 80.0f;
    request.gainDb = 0.0f;
    request.photosensitivity = 4;

    appendLog(QString::fromUtf8(
        "扫描开始：%1。扫描位姿：%2, %3, %4 mm / 180.00, 0.00, %5°。")
        .arg(needLift
                ? QString::fromUtf8("先抬到 Z = ") + QString::number(liftZ, 'f', 2)
                    + QString::fromUtf8(" mm 再调平")
                : QString::fromUtf8("已在安全高度，直接调平"))
        .arg(levelPose[0], 0, 'f', 2)
        .arg(levelPose[1], 0, 'f', 2)
        .arg(levelPose[2], 0, 'f', 2)
        .arg(levelPose[5], 0, 'f', 2),
        LogLevel::Info);
    setStatusText(QString::fromUtf8("扫描中"), AlarmLevel::Warning);
    m_scanController->start(request);
    updateActionStates();
}

void dianyun::onScanProgress(QString stage)
{
    appendLog(stage, LogLevel::Info);
    setStatusText(stage);
    updateActionStates();
}

void dianyun::onScanFinished(CloudRGB::Ptr cloud, CloudFrame frame)
{
    if (!cloud || cloud->empty()) {
        appendLog(QString::fromUtf8("扫描结束但没有得到有效点。"),
            LogLevel::Warning);
        updateActionStates();
        return;
    }

    m_scanCloud = cloud;
    m_scanCloudFrame = frame;
    m_havePickedPoint = false;
    invalidateMatchResult();
    m_visualization.showScanCloud(m_scanCloud);
    m_reportProgressiveRender = true;
    m_lastRenderPercent = -1;
    setStatusText(QString::fromUtf8("扫描完成"));
    appendLog(QString::fromUtf8("扫描完成，点云节点数：%1，坐标系：%2。")
        .arg(m_scanCloud->size())
        .arg(frame == CloudFrame::RobotBase
            ? QString::fromUtf8("机械臂基坐标（可 Shift+单击点云核对标定）")
            : QString::fromUtf8("传感器坐标（不可用于运动）")),
        LogLevel::Success);
    refreshDataSummary();
    updateActionStates();
}

void dianyun::onScanFailed(QString message)
{
    appendLog(QString::fromUtf8("扫描失败：%1").arg(message),
        LogLevel::Error);
    showAlarm(AlarmLevel::Error, QString::fromUtf8("扫描失败：") + message);
    setStatusText(QString::fromUtf8("扫描失败"), AlarmLevel::Error);
    updateActionStates();
}

bool dianyun::levelHeading(double& headingDeg, double& tiltDeg)
{
    if (!m_haveTcpPose) {
        appendLog(QString::fromUtf8(
            "尚未收到机械臂位姿，请确认已连接并等待状态刷新。"),
            LogLevel::Warning);
        return false;
    }
    // 调平转角超过该值多半是示教姿态与预期不符，自动转过去有撞件风险
    constexpr double kMaxLevelTiltDeg = 30.0;
    headingDeg = PoseMath::headingDeg(m_lastTcpPose);
    tiltDeg = PoseMath::tiltFromDownDeg(m_lastTcpPose);
    if (tiltDeg > kMaxLevelTiltDeg) {
        appendLog(QString::fromUtf8(
            "当前工具 Z 偏离竖直向下 %1°，超过自动调平上限 %2°，"
            "请先用示教器把末端大致摆成朝下。")
            .arg(tiltDeg, 0, 'f', 1).arg(kMaxLevelTiltDeg, 0, 'f', 0),
            LogLevel::Warning);
        return false;
    }
    return true;
}

bool dianyun::robotMotionAllowed()
{
    QString reason;
    if (m_robotState != DeviceLinkState::Online) {
        reason = QString::fromUtf8("机械臂未就绪（需已连接且空闲）。");
    }
    else if (!m_handEyeValid) {
        reason = QString::fromUtf8("手眼标定文件无效，请修正 calibration.json 后重新加载。");
    }
    else if (m_scanCloudFrame != CloudFrame::RobotBase) {
        reason = QString::fromUtf8(
            "当前点云不在机械臂基坐标下（文件导入或回退拼接），不能用于运动，请重新扫描。");
    }
    else if (m_scanController && m_scanController->isRunning()) {
        reason = QString::fromUtf8("扫描进行中。");
    }
    else if (m_pathController && m_pathController->isRunning()) {
        reason = QString::fromUtf8("已有轨迹在执行。");
    }
    if (reason.isEmpty()) return true;
    appendLog(reason, LogLevel::Warning);
    return false;
}

void dianyun::scanPointPicked(
    const pcl::visualization::PointPickingEvent& event, void*)
{
    if (event.getPointIndex() < 0) return;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    event.getPoint(x, y, z);
    m_pickedPoint = Eigen::Vector3f(x, y, z);
    m_havePickedPoint = true;

    if (m_scanCloudFrame == CloudFrame::RobotBase) {
        QString tcpText;
        if (m_haveTcpPose) {
            tcpText = QString::fromUtf8("；当前 TCP %1, %2, %3 mm，点相对 TCP 高差 %4 mm")
                .arg(m_lastTcpPose[0], 0, 'f', 2)
                .arg(m_lastTcpPose[1], 0, 'f', 2)
                .arg(m_lastTcpPose[2], 0, 'f', 2)
                .arg(z - m_lastTcpPose[2], 0, 'f', 2);
        }
        appendLog(QString::fromUtf8("点选（基坐标）：X %1  Y %2  Z %3 mm%4。")
            .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2).arg(tcpText),
            LogLevel::Info);
    }
    else {
        appendLog(QString::fromUtf8("点选（传感器坐标）：X %1  Y %2  Z %3 mm，不可用于运动。")
            .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2));
    }
    updateActionStates();
}

void dianyun::onMoveAbovePickedPoint()
{
    if (!m_havePickedPoint || !robotMotionAllowed()) return;
    double heading = 0.0;
    double tiltDeg = 0.0;
    if (!levelHeading(heading, tiltDeg)) return;

    constexpr double kAboveMm = 30.0;
    WeldWaypoint waypoint;
    waypoint.kind = WeldWaypoint::Kind::Approach;
    waypoint.pose = { m_pickedPoint.x(), m_pickedPoint.y(),
        m_pickedPoint.z() + kAboveMm, 180.0, 0.0, heading };

    const double distance = std::sqrt(
        std::pow(waypoint.pose[0] - m_lastTcpPose[0], 2)
        + std::pow(waypoint.pose[1] - m_lastTcpPose[1], 2)
        + std::pow(waypoint.pose[2] - m_lastTcpPose[2], 2));
    const auto answer = QMessageBox::question(this,
        QString::fromUtf8("移到点选位置上方"),
        QString::fromUtf8(
            "TCP（工具 1）将以 10% 速度直线移到：\n"
            "X %1  Y %2  Z %3 mm / Rx 180  Ry 0  Rz %4°\n"
            "直线距离约 %5 mm。\n\n"
            "TCP 在点选位置正上方 %6 mm，真实尖端再高 10 mm。\n"
            "到位后用尺子量尖端到该点的偏差，即为标定链总误差；\n"
            "若明显偏离或方向相反，请修改 calibration.json 的 sensorAxis。\n"
            "请确认路径上无人无障碍，手放在急停上。")
            .arg(waypoint.pose[0], 0, 'f', 2)
            .arg(waypoint.pose[1], 0, 'f', 2)
            .arg(waypoint.pose[2], 0, 'f', 2)
            .arg(heading, 0, 'f', 2)
            .arg(distance, 0, 'f', 1)
            .arg(kAboveMm, 0, 'f', 0),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes) return;

    WeldPathController::Speeds speeds;
    speeds.travelPercent = 10.0f;
    m_pathController->start({ waypoint }, speeds,
        QString::fromUtf8("移到点选位置上方"));
    updateActionStates();
}

void dianyun::onWeldDryRun()
{
    runSelectedSeam(true);
}

void dianyun::onWeldExecute()
{
    runSelectedSeam(false);
}

void dianyun::runSelectedSeam(bool dryRun)
{
    if (!robotMotionAllowed()) return;
    const WeldSeamData* seam = selectedSeam();
    if (!seam) {
        appendLog(QString::fromUtf8("请先在焊缝列表中选中一条焊缝。"), LogLevel::Warning);
        return;
    }
    if (!m_lastMatchResult.success
        || m_lastMatchResult.verdict != MatchVerdict::Pass) {
        appendLog(QString::fromUtf8("模型匹配未通过，不能执行焊缝轨迹。"),
            LogLevel::Warning);
        return;
    }
    if (!seam->hasStart) {
        appendLog(QString::fromUtf8(
            "该焊缝尚未设定起焊点，焊接方向不确定。请先点「焊缝起点」。"),
            LogLevel::Warning);
        return;
    }
    const auto mapped = m_mappedSeams.find(seam->id);
    if (mapped == m_mappedSeams.end()) {
        appendLog(QString::fromUtf8("请先对该焊缝执行焊缝映射。"), LogLevel::Warning);
        return;
    }

    std::vector<Eigen::Vector3f> path;
    bool closed = false;
    QString errorMessage;
    if (!m_weldSeamService.orderedPath(mapped->second, path, closed, errorMessage)) {
        appendLog(errorMessage, LogLevel::Warning);
        return;
    }

    double heading = 0.0;
    double tiltDeg = 0.0;
    if (!levelHeading(heading, tiltDeg)) return;

    // 空走在 TCP 自带 10 mm 余量之上再抬 20 mm，尖端距焊缝约 30 mm
    constexpr double kDryRunLiftMm = 20.0;
    WeldPathOptions options;
    options.headingDeg = heading;
    options.approachMm = 30.0;
    options.liftMm = dryRun ? kDryRunLiftMm : 0.0;
    WeldPathPlan plan;
    if (!planWeldPath(path, options, plan, errorMessage)) {
        appendLog(errorMessage, LogLevel::Warning);
        return;
    }

    // 焊缝明显高于当前 TCP 说明点云或标定方向反了，硬拦截
    if (plan.maxSeamZ + options.liftMm > m_lastTcpPose[2] + 100.0) {
        appendLog(QString::fromUtf8(
            "焊缝最高点 Z %1 mm 比当前 TCP 高出 100 mm 以上，疑似标定方向错误，已拒绝执行。")
            .arg(plan.maxSeamZ, 0, 'f', 2), LogLevel::Error);
        return;
    }

    WeldPathController::Speeds speeds;
    speeds.travelPercent = 10.0f;
    speeds.seamPercent = 5.0f;
    const auto& start = plan.seamStart();
    const auto& end = plan.seamEnd();
    const QString title = dryRun
        ? QString::fromUtf8("焊缝空走") : QString::fromUtf8("沿焊缝执行");
    const auto answer = QMessageBox::question(this, title,
        QString::fromUtf8(
            "焊缝：%1（%2，%3 段，长 %4 mm）\n"
            "起点 TCP：X %5  Y %6  Z %7 mm\n"
            "终点 TCP：X %8  Y %9  Z %10 mm\n"
            "焊缝高度范围 Z %11 ~ %12 mm；姿态 Rx 180 / Ry 0 / Rz %13°（全程调平）\n"
            "接近/退出高度 %14 mm（%15% 速度），焊缝段 %16% 速度。\n"
            "%17\n\n"
            "请确认路径上无人无障碍，手放在急停上。按 Esc 可停止。")
            .arg(seam->name)
            .arg(closed ? QString::fromUtf8("闭合") : QString::fromUtf8("开放"))
            .arg(plan.seamSegments)
            .arg(plan.seamLengthMm, 0, 'f', 1)
            .arg(start[0], 0, 'f', 2).arg(start[1], 0, 'f', 2).arg(start[2], 0, 'f', 2)
            .arg(end[0], 0, 'f', 2).arg(end[1], 0, 'f', 2).arg(end[2], 0, 'f', 2)
            .arg(plan.minSeamZ, 0, 'f', 2).arg(plan.maxSeamZ, 0, 'f', 2)
            .arg(heading, 0, 'f', 2)
            .arg(options.approachMm, 0, 'f', 0)
            .arg(speeds.travelPercent, 0, 'f', 0)
            .arg(speeds.seamPercent, 0, 'f', 0)
            .arg(dryRun
                ? QString::fromUtf8("空走：整条轨迹抬高 %1 mm，真实尖端距焊缝约 %2 mm。")
                    .arg(kDryRunLiftMm, 0, 'f', 0).arg(kDryRunLiftMm + 10.0, 0, 'f', 0)
                : QString::fromUtf8("贴合：TCP 落在焊缝上，真实尖端距焊缝 10 mm。")),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes) return;

    appendLog(QString::fromUtf8("%1开始：%2 个目标点。")
        .arg(title).arg(plan.waypoints.size()), LogLevel::Info);
    m_pathController->start(plan.waypoints, speeds, title);
    updateActionStates();
}

void dianyun::onPathProgress(QString message)
{
    appendLog(message, LogLevel::Info);
    setStatusText(message);
    updateActionStates();
}

void dianyun::onPathFinished()
{
    setStatusText(QString::fromUtf8("轨迹完成"));
    updateActionStates();
}

void dianyun::onPathFailed(QString message)
{
    appendLog(message, LogLevel::Error);
    showAlarm(AlarmLevel::Error, message);
    setStatusText(QString::fromUtf8("轨迹中止"), AlarmLevel::Error);
    updateActionStates();
}

void dianyun::onReloadCalibration()
{
    loadHandEyeConfig();
    // 已有的基坐标点云是用旧参数算的，必须重扫
    if (m_scanCloudFrame == CloudFrame::RobotBase) {
        m_scanCloudFrame = CloudFrame::Sensor;
        appendLog(QString::fromUtf8(
            "标定已重新加载，当前点云按旧参数生成，已禁用运动，请重新扫描。"),
            LogLevel::Warning);
    }
    updateActionStates();
}

void dianyun::on_btnOpen_3_clicked()
{
    if (m_reportProgressiveRender) {
        appendLog(QString::fromUtf8("完整点云仍在分批显示，请稍候。"));
        return;
    }
    if (!m_cadModel.isValid()) {
        appendLog(QString::fromUtf8("请先导入三维模型。"));
        return;
    }
    if (!m_scanCloud || m_scanCloud->empty()) {
        appendLog(QString::fromUtf8("请先导入点云模型。"));
        return;
    }
    if (m_matchController->isRunning()) {
        appendLog(QString::fromUtf8("匹配正在进行中，请稍候。"));
        return;
    }

    m_lastMatchResult = MatchResult();
    clearMappedSeamResults();
    resetMatchCard();
    clearAlarms();
    const quint64 requestId = m_matchRequests.beginRequest();
    if (!m_matchController->start(
        requestId, m_cadModel.sampleCloud, m_scanCloud,
        MatchConfig())) {
        m_matchRequests.complete(requestId);
        appendLog(QString::fromUtf8("无法启动模型匹配。"), LogLevel::Error);
        updateActionStates();
        return;
    }

    updateActionStates();
    appendLog(QString::fromUtf8(
        "模型匹配开始：全场景自动分割与CAD引导裁剪。"), LogLevel::Info);
    setStatusText(QString::fromUtf8("匹配中"));
}

void dianyun::onMatchProgress(quint64 requestId, QString stage)
{
    if (!m_matchRequests.accepts(requestId)) return;
    setStatusText(stage);
}

void dianyun::onProcessedScanReady(
    quint64 requestId, CloudRGB::Ptr processedCloud)
{
    if (!m_matchRequests.accepts(requestId)) return;
    if (!processedCloud || processedCloud->empty()) return;
    m_visualization.showProcessedScan(processedCloud);
}

void dianyun::onMatchFinished(quint64 requestId, MatchResult result)
{
    if (!m_matchRequests.accepts(requestId)) {
        appendLog(QString::fromUtf8("已忽略过期的模型匹配结果。"));
        return;
    }
    m_matchRequests.complete(requestId);
    m_lastMatchResult = result;
    updateActionStates();

    if (!result.success) {
        setStatusText(QString::fromUtf8("匹配失败"), AlarmLevel::Error);
        appendLog(QString::fromUtf8("匹配失败：") + result.errorMessage,
            LogLevel::Error);
        showMatchCard(result);
        showAlarm(AlarmLevel::Error,
            QString::fromUtf8("模型匹配失败：") + result.errorMessage);
        return;
    }

    const bool passed = result.verdict == MatchVerdict::Pass;
    setStatusText(passed ? QString::fromUtf8("匹配通过")
                         : QString::fromUtf8("匹配质量不足"),
        passed ? AlarmLevel::Info : AlarmLevel::Error);

    appendLog(QString::fromUtf8("%1：覆盖率 %2%，RMSE %3，P95 %4。")
        .arg(passed ? QString::fromUtf8("模型匹配通过")
                    : QString::fromUtf8("模型匹配质量不足"))
        .arg(result.overlapRatio * 100.0f, 0, 'f', 1)
        .arg(result.rmse, 0, 'f', 3)
        .arg(result.p95Residual, 0, 'f', 3),
        passed ? LogLevel::Success : LogLevel::Error);

    showMatchCard(result);

    if (!passed) {
        showAlarm(AlarmLevel::Error, QString::fromUtf8(
            "匹配质量不足：覆盖率 %1%，低于通过门槛，焊缝映射已禁用")
            .arg(result.overlapRatio * 100.0f, 0, 'f', 1));
        appendLog(QString::fromUtf8(
            "诊断：CAD支持 %1%，特征覆盖 %2%，空间覆盖 %3%，法向一致性 %4%。")
            .arg(result.reverseOverlapRatio * 100.0f, 0, 'f', 1)
            .arg(result.featureOverlapRatio * 100.0f, 0, 'f', 1)
            .arg(result.spatialCoverageRatio * 100.0f, 0, 'f', 1)
            .arg(result.normalConsistency * 100.0f, 0, 'f', 1));
        if (!result.coarseCandidateReport.isEmpty()) {
            appendLog(result.coarseCandidateReport);
        }
    }
    else {
        if (result.wasAmbiguous) {
            appendLog(QString::fromUtf8(
                "提示：存在分数接近的其他候选（分数比 %1），已采用得分最高的姿态。")
                .arg(result.ambiguityRatio, 0, 'f', 3), LogLevel::Warning);
        }
        if (!result.convergedStageB) {
            appendLog(QString::fromUtf8(
                "提示：点到面精配准未收敛或已回退，采用点到点ICP的最佳姿态。"),
                LogLevel::Warning);
        }
    }

    if (!passed) {
        appendLog(QString::fromUtf8(
            "结果未达到质量门槛，不显示对齐后的CAD模型。"),
            LogLevel::Error);
        return;
    }

    m_visualization.showAlignedCad(
        m_cadModel.sampleCloud, result.T_final);
}
