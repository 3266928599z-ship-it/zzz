#include "dianyun.h"
#include "ui_dianyun.h"

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

    setupThemeAssets();
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

    updateStatusUI();
    resetMatchCard();
    updateActionStates();
    refreshDataSummary();
    appendLog(QString::fromUtf8("系统已启动，操作日志就绪。"));
}


dianyun::~dianyun()
{
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
    struct IconSpec { QToolButton* button; const char* path; };
    const IconSpec icons[] = {
        { ui->btnOpen,       ":/dianyun/icons/import-cad.svg"   },
        { ui->btnOpen_2,     ":/dianyun/icons/import-cloud.svg" },
        { ui->btnOpen_3,     ":/dianyun/icons/match.svg"        },
        { ui->btnOpenCamera, ":/dianyun/icons/camera.svg"       },
        { ui->btnRobotWeld,  ":/dianyun/icons/robot-arm.svg"    },
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
    else if (result.verdict == MatchVerdict::NeedsReview) {
        verdictText = QString::fromUtf8("需要人工确认");
        tone = QStringLiteral("review");
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
            .arg(tone == QStringLiteral("pass") ? QStringLiteral("#2AC44E") :
                 tone == QStringLiteral("review") ? QStringLiteral("#F57C00") :
                                                    QStringLiteral("#E53935")));

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
    else {
        status = QString::fromUtf8("请选择面 B（按住 Ctrl 单击）");
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
    const bool dataActionsEnabled =
        !matchRunning && !cloudLoading && !cameraCapturing;

    if (ui->btnOpen) ui->btnOpen->setEnabled(dataActionsEnabled);
    if (ui->btnOpen_2) ui->btnOpen_2->setEnabled(dataActionsEnabled);
    if (ui->btnOpenCamera) {
        ui->btnOpenCamera->setEnabled(dataActionsEnabled);
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
        return;
    }

    const vtkIdType cellId = picker->GetCellId();
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
        ui->listWeldSeams->addTopLevelItem(item);
    }
    appendLog(QString::fromUtf8("已创建%1：长度 %2，%3 个管道段。")
        .arg(seam.name)
        .arg(seam.length, 0, 'f', 3)
        .arg(seam.segments.size()), LogLevel::Success);
    refreshDataSummary();
    resetSeamSelection();
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

    WeldSeamData mappedSeam;
    QString errorMessage;
    if (!m_weldSeamService.mapSeam(
        *seam, m_lastMatchResult.T_final, mappedSeam, errorMessage)) {
        appendLog(QString::fromUtf8("焊缝映射失败：") + errorMessage,
            LogLevel::Error);
        showAlarm(AlarmLevel::Error,
            QString::fromUtf8("焊缝映射失败：") + errorMessage);
        return;
    }

    const std::string seamId = seam->id;
    m_mappedSeams.insert_or_assign(seamId, mappedSeam);
    m_visualization.showMappedSeam(mappedSeam);
    appendLog(QString::fromUtf8(
        "%1映射完成：%2 个采样点，%3 个管道段。")
        .arg(mappedSeam.name)
        .arg(mappedSeam.points ? mappedSeam.points->size() : 0)
        .arg(mappedSeam.segments.size()), LogLevel::Success);
    syncSeamRowMapping(seamId);
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
    if (ui->ledCapture) {
        ui->ledCapture->startBlinking(20 * 1000);
    }
    appendLog(QString::fromUtf8(
        "演示模式：相机采集尚未接入，仅触发状态指示灯。"),
        LogLevel::Warning);
}

void dianyun::on_btnRobotWeld_clicked()
{
    if (ui->ledRobotWeld) {
        ui->ledRobotWeld->startBlinking(40 * 1000);
    }
    appendLog(QString::fromUtf8(
        "演示模式：机械臂焊接尚未接入，仅触发状态指示灯。"),
        LogLevel::Warning);
}

void dianyun::onCameraPointCloudReady(CloudRGB::Ptr newCloud)
{
    if (!newCloud || newCloud->empty()) return;
    m_scanCloud = std::move(newCloud);
    invalidateMatchResult();
    m_visualization.showScanCloud(m_scanCloud);
    m_reportProgressiveRender = true;
    m_lastRenderPercent = -1;
    updateActionStates();
    appendLog(QString::fromUtf8("实时拼接完成，正在分批显示，节点数：") +
        QString::number(m_scanCloud->size()), LogLevel::Success);
    refreshDataSummary();
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

    const QString statusText = result.verdict == MatchVerdict::Pass
        ? QString::fromUtf8("匹配通过")
        : (result.verdict == MatchVerdict::NeedsReview
            ? QString::fromUtf8("需要人工确认")
            : QString::fromUtf8("匹配质量不足"));
    setStatusText(statusText,
        result.verdict == MatchVerdict::Pass
            ? AlarmLevel::Info
            : (result.verdict == MatchVerdict::NeedsReview
                ? AlarmLevel::Warning
                : AlarmLevel::Error));

    appendLog(QString::fromUtf8("%1：覆盖率 %2%，RMSE %3，P95 %4。")
        .arg(result.verdict == MatchVerdict::Pass
            ? QString::fromUtf8("模型匹配通过")
            : (result.verdict == MatchVerdict::NeedsReview
                ? QString::fromUtf8("模型匹配需要人工确认")
                : QString::fromUtf8("模型匹配质量不足")))
        .arg(result.overlapRatio * 100.0f, 0, 'f', 1)
        .arg(result.rmse, 0, 'f', 3)
        .arg(result.p95Residual, 0, 'f', 3),
        result.verdict == MatchVerdict::Pass
            ? LogLevel::Success
            : (result.verdict == MatchVerdict::NeedsReview
                ? LogLevel::Warning
                : LogLevel::Error));

    showMatchCard(result);

    if (result.verdict == MatchVerdict::Fail) {
        showAlarm(AlarmLevel::Error, QString::fromUtf8(
            "匹配质量不足：覆盖率 %1%，低于通过门槛，焊缝映射已禁用")
            .arg(result.overlapRatio * 100.0f, 0, 'f', 1));
    }
    else if (result.verdict == MatchVerdict::NeedsReview) {
        showAlarm(AlarmLevel::Warning, QString::fromUtf8(
            "匹配需要人工确认：覆盖率 %1%，RMSE %2，请复核对齐结果")
            .arg(result.overlapRatio * 100.0f, 0, 'f', 1)
            .arg(result.rmse, 0, 'f', 3));
    }

    if (result.verdict != MatchVerdict::Pass) {
        appendLog(QString::fromUtf8(
            "诊断：CAD支持 %1%，特征覆盖 %2%，空间覆盖 %3%，法向一致性 %4%。")
            .arg(result.reverseOverlapRatio * 100.0f, 0, 'f', 1)
            .arg(result.featureOverlapRatio * 100.0f, 0, 'f', 1)
            .arg(result.spatialCoverageRatio * 100.0f, 0, 'f', 1)
            .arg(result.normalConsistency * 100.0f, 0, 'f', 1));
        if (!result.coarseCandidateReport.isEmpty()) {
            appendLog(result.coarseCandidateReport);
        }
        if (result.wasAmbiguous) {
            appendLog(QString::fromUtf8(
                "粗配准存在多解歧义，已选择适应度最优解。"));
        }
    }

    if (result.verdict == MatchVerdict::Fail) {
        appendLog(QString::fromUtf8(
            "结果未达到质量门槛，不显示对齐后的CAD模型。"),
            LogLevel::Error);
        return;
    }

    m_visualization.showAlignedCad(
        m_cadModel.sampleCloud, result.T_final);
}
