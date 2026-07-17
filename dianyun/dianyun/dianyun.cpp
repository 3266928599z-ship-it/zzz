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
    for (const auto& seam : m_seams) {
        m_visualization.removeSeam(seam);
    }
    m_seams.clear();
    m_seamCounter = 0;
    if (ui->listWeldSeams) ui->listWeldSeams->clear();
    resetSeamSelection();
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
    appendLog(QString::fromUtf8("选中网格单元 %1，对应CAD面 %2。")
        .arg(cellId).arg(faceId));

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

    appendLog(QString::fromUtf8("正在计算CAD面 %1 与面 %2 的交线...")
        .arg(m_selectedFaceA).arg(m_selectedFaceB));
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
    if (ui->listWeldSeams) ui->listWeldSeams->addItem(seam.name);
    appendLog(QString::fromUtf8("已生成分析项：") + seam.name);
    appendLog(QString::fromUtf8("焊缝长度 %1，线段数 %2。")
        .arg(seam.length, 0, 'f', 3).arg(seam.segments.size()));
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
    m_lastMatchResult = MatchResult();
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
    m_lastMatchResult = MatchResult();
    m_visualization.showScanCloud(m_scanCloud, diagonal);
    m_reportProgressiveRender = true;
    m_lastRenderPercent = -1;
    if (ui->btnOpen_3) ui->btnOpen_3->setEnabled(false);
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
    if (ui->btnOpen_2) ui->btnOpen_2->setEnabled(!busy);
    if (ui->btnOpen_3) {
        ui->btnOpen_3->setEnabled(!busy &&
            !m_reportProgressiveRender && m_matchController &&
            !m_matchController->isRunning());
    }
    if (busy && ui->lblStatus) {
        ui->lblStatus->setText(QString::fromUtf8("当前状态：正在后台加载点云"));
    }
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
    appendLog(QString::fromUtf8("已取消当前操作。"));
}

void dianyun::on_btnDeleteSeam_clicked()
{
    if (!ui->listWeldSeams) return;
    const int row = ui->listWeldSeams->currentRow();
    if (row < 0 || static_cast<size_t>(row) >= m_seams.size()) return;

    const QString name = m_seams[row].name;
    m_visualization.removeSeam(m_seams[row]);
    m_seams.erase(m_seams.begin() + row);
    delete ui->listWeldSeams->takeItem(row);
    appendLog(QString::fromUtf8("删除了记录：") + name);
}

void dianyun::on_listWeldSeams_itemClicked(QListWidgetItem* item)
{
    if (item) appendLog(QString::fromUtf8("选中记录：") + item->text());
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
        if (ui->btnOpen_3 && m_matchController &&
            !m_matchController->isRunning() && m_pointCloudLoadController &&
            !m_pointCloudLoadController->isRunning()) {
            ui->btnOpen_3->setEnabled(true);
        }
    }
}

void dianyun::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
}

void dianyun::on_btnOpenCamera_clicked()
{
    if (!m_cameraWorker) {
        m_cameraWorker = new HikCameraWorker(this);
        connect(m_cameraWorker, &HikCameraWorker::pointCloudReady,
            this, &dianyun::onCameraPointCloudReady);
    }

    appendLog(QString::fromUtf8("正在尝试独占连接3D相机..."));
    if (m_cameraWorker->openCamera()) {
        appendLog(QString::fromUtf8("相机连接成功，正在采集并拼装三维点云。"));
    }
    else {
        appendLog(QString::fromUtf8(
            "相机打开失败，请检查连接并关闭占用相机的海康软件。"));
    }
}

void dianyun::onCameraPointCloudReady(CloudRGB::Ptr newCloud)
{
    if (!newCloud || newCloud->empty()) return;
    m_scanCloud = std::move(newCloud);
    m_visualization.showScanCloud(m_scanCloud);
    m_reportProgressiveRender = true;
    m_lastRenderPercent = -1;
    if (ui->btnOpen_3) ui->btnOpen_3->setEnabled(false);
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

    if (!m_matchController->start(
        m_cadModel.sampleCloud, m_scanCloud, m_seams, MatchConfig())) {
        appendLog(QString::fromUtf8("无法启动模型匹配。"));
        return;
    }

    appendLog(QString::fromUtf8("开始模型匹配流水线..."));
    appendLog(QString::fromUtf8("匹配输入：全场景自动分割与CAD引导裁剪。"));
    if (ui->btnOpen_3) ui->btnOpen_3->setEnabled(false);
    if (ui->lblStatus) {
        ui->lblStatus->setText(QString::fromUtf8("当前状态：匹配中"));
    }
}

void dianyun::onMatchProgress(QString stage)
{
    if (ui->lblStatus) {
        ui->lblStatus->setText(QString::fromUtf8("当前状态：") + stage);
    }
    appendLog(QString::fromUtf8("进度：") + stage);
}

void dianyun::onProcessedScanReady(CloudRGB::Ptr processedCloud)
{
    if (!processedCloud || processedCloud->empty()) return;
    m_visualization.showProcessedScan(processedCloud);
    appendLog(QString::fromUtf8("已显示实际参与匹配的工件候选点云：") +
        QString::number(processedCloud->size()) + QString::fromUtf8(" 点。"));
}

void dianyun::onMatchFinished(MatchResult result)
{
    m_lastMatchResult = result;
    if (ui->btnOpen_3) {
        ui->btnOpen_3->setEnabled(!m_reportProgressiveRender &&
            m_pointCloudLoadController &&
            !m_pointCloudLoadController->isRunning());
    }

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

    appendLog(QString::fromUtf8("匹配计算完成：") + result.verdictMessage);
    appendLog(QString::fromUtf8("局部覆盖率：%1%，RMSE：%2，P95残差：%3")
        .arg(result.overlapRatio * 100.0f, 0, 'f', 1)
        .arg(result.rmse, 0, 'f', 3)
        .arg(result.p95Residual, 0, 'f', 3));
    appendLog(QString::fromUtf8(
        "CAD反向支持：%1%，关键特征覆盖：%2%，空间结构覆盖：%3%，法向一致性：%4%")
        .arg(result.reverseOverlapRatio * 100.0f, 0, 'f', 1)
        .arg(result.featureOverlapRatio * 100.0f, 0, 'f', 1)
        .arg(result.spatialCoverageRatio * 100.0f, 0, 'f', 1)
        .arg(result.normalConsistency * 100.0f, 0, 'f', 1));
    if (!result.coarseCandidateReport.isEmpty()) {
        appendLog(result.coarseCandidateReport);
    }
    if (result.wasAmbiguous) {
        appendLog(QString::fromUtf8(
            "粗配准存在多解歧义，已自动选取适应度最优解。"));
    }

    if (result.verdict == MatchVerdict::Fail) {
        appendLog(QString::fromUtf8(
            "结果未达到质量门槛，不显示对齐后的CAD模型。"));
        return;
    }

    m_visualization.showAlignedCad(
        m_cadModel.sampleCloud, result.T_final);
    appendLog(QString::fromUtf8("已将对齐后的CAD模型渲染至点云视图。"));
}
