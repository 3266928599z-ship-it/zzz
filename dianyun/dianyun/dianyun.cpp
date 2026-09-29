#include "dianyun.h"
#include "ui_dianyun.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QMetaType>
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
    updateActionStates();
    if (ui->groupBoxLog) {
        ui->groupBoxLog->setTitle(QString::fromUtf8("操作日志"));
    }
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
        m_cadViewer->setBackgroundColor(0.2, 0.2, 0.2);
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
        m_scanViewer->setBackgroundColor(0.1, 0.1, 0.1);
        m_scanViewer->setupInteractor(
            vtkWidget->interactor(), vtkWidget->renderWindow());
    }
}

void dianyun::appendLog(const QString& message)
{
    if (ui->textLog) {
        ui->textLog->append(
            QTime::currentTime().toString("[HH:mm:ss] ") + message);
    }
}

void dianyun::updateStatusUI()
{
    QString status = QString::fromUtf8("当前状态：");
    if (m_state == STATE_IDLE) {
        status += QString::fromUtf8("就绪");
    }
    else if (m_state == STATE_PICK_FACE_A) {
        status += QString::fromUtf8("正在选择面A...");
    }
    else {
        status += QString::fromUtf8("正在选择面B...");
    }

    if (ui->lblStatus) {
        ui->lblStatus->setText(
            QString("<style>p { color: black; font-weight: bold; }</style>"
                "<p align=\"center\">%1</p>").arg(status));
    }
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
        ->data(Qt::UserRole).toString().toStdString();
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
        appendLog(errorMessage);
        resetSeamSelection();
        return;
    }

    m_seamCounter = nextIndex;
    m_visualization.showSeam(seam);
    m_seams.push_back(seam);
    if (ui->listWeldSeams) {
        auto* item = new QListWidgetItem(seam.name);
        item->setData(Qt::UserRole, QString::fromStdString(seam.id));
        ui->listWeldSeams->addItem(item);
    }
    appendLog(QString::fromUtf8("已创建%1：长度 %2，%3 个管道段。")
        .arg(seam.name)
        .arg(seam.length, 0, 'f', 3)
        .arg(seam.segments.size()));
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
        appendLog(QString::fromUtf8("三维模型导入失败：") + errorMessage);
        return;
    }

    clearSeams();
    m_cadModel = std::move(loadedModel);
    invalidateMatchResult();
    m_visualization.showCadModel(m_cadModel);
    appendLog(QString::fromUtf8("三维模型导入成功，CAD采样点数：") +
        QString::number(m_cadModel.sampleCloud->size()));
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
        .arg(m_scanCloud->size()));
    if (ui->lblStatus) {
        ui->lblStatus->setText(QString::fromUtf8("当前状态：正在显示完整点云"));
    }
}

void dianyun::onPointCloudLoadFailed(QString errorMessage)
{
    appendLog(QString::fromUtf8("点云模型导入失败：") + errorMessage);
    if (ui->lblStatus) {
        ui->lblStatus->setText(QString::fromUtf8("当前状态：点云导入失败"));
    }
}

void dianyun::onPointCloudLoadBusyChanged(bool busy)
{
    if (busy && ui->lblStatus) {
        ui->lblStatus->setText(QString::fromUtf8("当前状态：正在后台加载点云"));
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
        appendLog(QString::fromUtf8("焊缝映射失败：") + errorMessage);
        return;
    }

    const std::string seamId = seam->id;
    m_mappedSeams.insert_or_assign(seamId, mappedSeam);
    m_visualization.showMappedSeam(mappedSeam);
    appendLog(QString::fromUtf8(
        "%1映射完成：%2 个采样点，%3 个管道段。")
        .arg(mappedSeam.name)
        .arg(mappedSeam.points ? mappedSeam.points->size() : 0)
        .arg(mappedSeam.segments.size()));
}

void dianyun::on_btnDeleteSeam_clicked()
{
    if (!ui->listWeldSeams) return;
    const WeldSeamData* selected = selectedSeam();
    if (!selected) return;

    const int row = ui->listWeldSeams->currentRow();
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
    delete ui->listWeldSeams->takeItem(row);
    appendLog(QString::fromUtf8("已删除焊缝：") + name);
    updateActionStates();
}

void dianyun::on_listWeldSeams_currentItemChanged(
    QListWidgetItem* current, QListWidgetItem*)
{
    if (current) {
        m_visualization.selectSeam(
            current->data(Qt::UserRole).toString().toStdString());
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
    if (percent != m_lastRenderPercent && ui->lblStatus) {
        m_lastRenderPercent = percent;
        ui->lblStatus->setText(QString::fromUtf8("当前状态：正在显示完整点云 %1%")
            .arg(percent));
    }
    if (!m_visualization.isScanRendering()) {
        m_reportProgressiveRender = false;
        appendLog(QString::fromUtf8("完整点云显示完成，共 %1 点。")
            .arg(total));
        if (ui->lblStatus) {
            ui->lblStatus->setText(QString::fromUtf8("当前状态：点云就绪"));
        }
        updateActionStates();
    }
}

void dianyun::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
}

void dianyun::on_btnOpenCamera_clicked()
{
    if (ui->ledCapture) {
        ui->ledCapture->startBlinking(20 * 1000);
    }
}

void dianyun::on_btnRobotWeld_clicked()
{
    if (ui->ledRobotWeld) {
        ui->ledRobotWeld->startBlinking(40 * 1000);
    }
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
        QString::number(m_scanCloud->size()));
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
    const quint64 requestId = m_matchRequests.beginRequest();
    if (!m_matchController->start(
        requestId, m_cadModel.sampleCloud, m_scanCloud,
        MatchConfig())) {
        m_matchRequests.complete(requestId);
        appendLog(QString::fromUtf8("无法启动模型匹配。"));
        updateActionStates();
        return;
    }

    updateActionStates();
    appendLog(QString::fromUtf8(
        "模型匹配开始：全场景自动分割与CAD引导裁剪。"));
    if (ui->lblStatus) {
        ui->lblStatus->setText(QString::fromUtf8("当前状态：匹配中"));
    }
}

void dianyun::onMatchProgress(quint64 requestId, QString stage)
{
    if (!m_matchRequests.accepts(requestId)) return;
    if (ui->lblStatus) {
        ui->lblStatus->setText(QString::fromUtf8("当前状态：") + stage);
    }
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
        if (ui->lblStatus) {
            ui->lblStatus->setText(QString::fromUtf8("当前状态：匹配失败"));
        }
        appendLog(QString::fromUtf8("匹配失败：") + result.errorMessage);
        return;
    }

    const QString status = result.verdict == MatchVerdict::Pass
        ? QString::fromUtf8("当前状态：匹配通过")
        : (result.verdict == MatchVerdict::NeedsReview
            ? QString::fromUtf8("当前状态：需要人工确认")
            : QString::fromUtf8("当前状态：匹配质量不足"));
    if (ui->lblStatus) ui->lblStatus->setText(status);

    appendLog(QString::fromUtf8("%1：覆盖率 %2%，RMSE %3，P95 %4。")
        .arg(result.verdict == MatchVerdict::Pass
            ? QString::fromUtf8("模型匹配通过")
            : (result.verdict == MatchVerdict::NeedsReview
                ? QString::fromUtf8("模型匹配需要人工确认")
                : QString::fromUtf8("模型匹配质量不足")))
        .arg(result.overlapRatio * 100.0f, 0, 'f', 1)
        .arg(result.rmse, 0, 'f', 3)
        .arg(result.p95Residual, 0, 'f', 3));

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
            "结果未达到质量门槛，不显示对齐后的CAD模型。"));
        return;
    }

    m_visualization.showAlignedCad(
        m_cadModel.sampleCloud, result.T_final);
}
