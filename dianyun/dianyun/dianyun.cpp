#include "dianyun.h"
#include "ui_dianyun.h"

#define NOMINMAX
#include <windows.h>

// Qt
#include <QFileDialog>
#include <QDir>
#include <QDebug>
#include <QApplication> 
#include <QMessageBox>
#include <QTime>
#include <QFileInfo>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <QVBoxLayout>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h> 
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>

// VTK
#include <vtkRenderWindow.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <Windows.h>
#include <vtkCellPicker.h> 
#include <vtkIntersectionPolyDataFilter.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>

// OCCT
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Triangle.hxx>
#include <gp_Pnt.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>


// OCCT Algo
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>

// Math
#include <Eigen/Dense>
#include <random>

// =========================================================================
// 核心逻辑 1：STEP 转 Mesh (只提取顶点，不填充)
// =========================================================================
bool dianyun::LoadStepFileToPCL(const std::string& filename)
{
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filename.c_str());
    if (status != IFSelect_RetVoid && status != IFSelect_RetDone) return false;

    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) return false;

    m_model_faces.clear();
    m_cell_to_face_map.clear();
    cloud->clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_polygons;
    int mesh_vertex_offset = 0;

    // 1. 离散化
    // 使用相对高精度，保证曲面光滑，但不需要极端致密
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    double diagonal = std::sqrt(std::pow(xmax - xmin, 2) + std::pow(ymax - ymin, 2) + std::pow(zmax - zmin, 2));

    // 动态精度：尺寸的 0.1% (足够光滑且不用太多内存)
    double linear_deflection = diagonal * 0.001;
    if (linear_deflection < 0.01) linear_deflection = 0.01;

    BRepMesh_IncrementalMesh imesh(shape, linear_deflection, false, 0.5, true);

    TopExp_Explorer explorer(shape, TopAbs_FACE);
    int face_id = 0;

    for (; explorer.More(); explorer.Next())
    {
        TopoDS_Face face = TopoDS::Face(explorer.Current());
        m_model_faces.push_back(face);

        TopLoc_Location loc;
        Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

        if (triangulation.IsNull()) { face_id++; continue; }

        const int numNodes = triangulation->NbNodes();
        const int numTriangles = triangulation->NbTriangles();
        gp_Trsf trsf = loc.Transformation();

        // 提取顶点
        for (int i = 1; i <= numNodes; ++i) {
            gp_Pnt p = triangulation->Node(i).Transformed(trsf);

            // 存入 Mesh 顶点
            mesh_vertices->push_back(pcl::PointXYZ(p.X(), p.Y(), p.Z()));

            // 存入 Cloud 顶点 (用于包围盒计算和备用)
            pcl::PointXYZRGB pt;
            pt.x = p.X(); pt.y = p.Y(); pt.z = p.Z();
            cloud->push_back(pt);
        }

        // 提取三角形 (构建 Mesh)
        for (int i = 1; i <= numTriangles; ++i) {
            Poly_Triangle tri = triangulation->Triangle(i);
            int n1, n2, n3;
            tri.Get(n1, n2, n3);

            pcl::Vertices v;
            v.vertices.push_back(mesh_vertex_offset + n1 - 1);
            v.vertices.push_back(mesh_vertex_offset + n2 - 1);
            v.vertices.push_back(mesh_vertex_offset + n3 - 1);
            mesh_polygons.push_back(v);

            // 记录映射
            m_cell_to_face_map.push_back(face_id);
        }

        mesh_vertex_offset += numNodes;
        face_id++;
    }

    pcl::toPCLPointCloud2(*mesh_vertices, mesh->cloud);
    mesh->polygons = mesh_polygons;

    return !cloud->empty();
}

// =========================================================================
// 构造函数
// =========================================================================
dianyun::dianyun(QWidget* parent) : QMainWindow(parent), ui(new Ui::dianyunClass)
{
    ui->setupUi(this);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    mesh.reset(new pcl::PolygonMesh);

    // 双视图：匹配 UI 文件中的 widget(3D) 和 widget_2(点云)
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

    updateStatusUI();
    initialWindow();
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &dianyun::updatePCLWindow);
    timer->start(30);

    // 初始化日志和状态
    ui->groupBoxLog->setTitle(QString::fromUtf8("操作日志")); // 将Logs改为操作日志
    appendLog(QString::fromUtf8("系统已启动，操作日志就绪。"));
}

dianyun::~dianyun() { delete ui; }

// =========================================================================
// 日志操作
// =========================================================================
void dianyun::appendLog(const QString& msg)
{
    // 追加日志内容到 UI（textLog 控件）
    if (ui->textLog) {
        QString timeStr = QTime::currentTime().toString("[HH:mm:ss] ");
        ui->textLog->append(timeStr + msg);
    }
}

// =========================================================================
// 初始化视窗（分别为3D模型和点云模型创建Viewer）
// =========================================================================
void dianyun::initialWindow()
{
    // 渲染器 3D
    if(ui->widget) {
        auto vtkWidget3D = new QVTKOpenGLNativeWidget(ui->widget);
        auto layout3D = new QVBoxLayout(ui->widget);
        layout3D->setContentsMargins(0, 0, 0, 0);
        layout3D->addWidget(vtkWidget3D);

        auto renderWindow3D = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        auto renderer3D = vtkSmartPointer<vtkRenderer>::New();
        renderWindow3D->AddRenderer(renderer3D);
        vtkWidget3D->setRenderWindow(renderWindow3D);

        viewer3D.reset(new pcl::visualization::PCLVisualizer(renderer3D, renderWindow3D, "Viewer3D", false));
        viewer3D->setBackgroundColor(0.2, 0.2, 0.2);
        viewer3D->setupInteractor(vtkWidget3D->interactor(), vtkWidget3D->renderWindow());
        viewer3D->registerMouseCallback(&dianyun::mouseEventOccurred, *this);
    }

    // 渲染器 Cloud
    if(ui->widget_2) {
        auto vtkWidgetCloud = new QVTKOpenGLNativeWidget(ui->widget_2);
        auto layoutCloud = new QVBoxLayout(ui->widget_2);
        layoutCloud->setContentsMargins(0, 0, 0, 0);
        layoutCloud->addWidget(vtkWidgetCloud);

        auto renderWindowCloud = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        auto rendererCloud = vtkSmartPointer<vtkRenderer>::New();
        renderWindowCloud->AddRenderer(rendererCloud);
        vtkWidgetCloud->setRenderWindow(renderWindowCloud);

        viewerCloud.reset(new pcl::visualization::PCLVisualizer(rendererCloud, renderWindowCloud, "ViewerCloud", false));
        viewerCloud->setBackgroundColor(0.1, 0.1, 0.1);
        viewerCloud->setupInteractor(vtkWidgetCloud->interactor(), vtkWidgetCloud->renderWindow());
    }
}

// =========================================================================
// 核心逻辑 2：鼠标 (Ctrl + 左键选面)
// =========================================================================
void dianyun::mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* args)
{
    // 1. 只响应左键按下
    if (event.getType() != pcl::visualization::MouseEvent::MouseButtonPress ||
        event.getButton() != pcl::visualization::MouseEvent::LeftButton)
    {
        return;
    }

    // 2. 【核心修改】必须按住 Ctrl 键才触发选面
    // 如果没有按 Ctrl，直接返回，交给 PCL 处理默认的旋转操作
    if (!(event.getKeyboardModifiers() & 2)) {
        return;
    }

    if (m_cell_to_face_map.empty()) return;

    // 3. VTK Cell Picker
    vtkRenderWindowInteractor* interactor = viewer3D->getRenderWindow()->GetInteractor();
    if (!interactor) return;
    int x = event.getX();
    int y = event.getY();

    vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
    picker->SetTolerance(0.001);
    vtkRenderer* renderer = viewer3D->getRenderWindow()->GetRenderers()->GetFirstRenderer();
    int pickResult = picker->Pick(x, y, 0, renderer);

    if (pickResult == 0) return;

    vtkIdType cellId = picker->GetCellId();
    if (cellId != -1 && cellId < m_cell_to_face_map.size()) {

        int clicked_face_id = m_cell_to_face_map[cellId];
        appendLog(QString::fromUtf8("Picked Mesh Cell:") + QString::number(cellId) + QString(" -> Face") + QString::number(clicked_face_id));

        // --- 状态机 ---
        if (current_state == STATE_PICK_FACE_A) {
            m_selected_face_id_A = clicked_face_id;
            highlightFaceMesh(m_selected_face_id_A, "highlight_A");

            current_state = STATE_PICK_FACE_B;
            QMetaObject::invokeMethod(this, [this]() { updateStatusUI(); });
        }
        else if (current_state == STATE_PICK_FACE_B) {
            m_selected_face_id_B = clicked_face_id;

            if (m_selected_face_id_B == m_selected_face_id_A) {
                QMessageBox::warning(this, "Warning", "Same face selected!");
                return;
            }

            highlightFaceMesh(m_selected_face_id_B, "highlight_B");
            CalculateSeam_OCCT();

            current_state = STATE_IDLE;
            QMetaObject::invokeMethod(this, [this]() { updateStatusUI(); });
        }
    }
}

// =========================================================================
// 核心逻辑 3：Mesh 实心高亮 (且不可拾取)
// =========================================================================
void dianyun::highlightFaceMesh(int face_id, std::string id)
{
    pcl::PolygonMesh::Ptr face_mesh(new pcl::PolygonMesh);
    face_mesh->cloud = mesh->cloud;

    for (size_t i = 0; i < m_cell_to_face_map.size(); ++i) {
        if (m_cell_to_face_map[i] == face_id) {
            face_mesh->polygons.push_back(mesh->polygons[i]);
        }
    }

    // 移除旧的
    viewer3D->removePolygonMesh(id);
    // 添加新的
    viewer3D->addPolygonMesh(*face_mesh, id);

    // 设置颜色
    if (id == "highlight_A")
        viewer3D->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, id);
    else
        viewer3D->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, id);

    viewer3D->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id);

    // 【核心修复】将高亮层设为“不可拾取”
    // 这样下次点击时，射线会直接穿透红色层，击中底下的灰色原始模型
    // 避免 vtkCellPicker 错误地返回高亮层的 ID
    pcl::visualization::ShapeActorMap::iterator it = viewer3D->getShapeActorMap()->find(id);
    if (it != viewer3D->getShapeActorMap()->end()) {
        it->second->SetPickable(0); // 0 = False
    }

    viewer3D->spinOnce(100, true);
}

// 顶部确保包含以下头文件
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopExp.hxx>

// =========================================================================
// 核心逻辑 4：VTK 网格求交 
// =========================================================================
void dianyun::CalculateSeam_OCCT()
{
    if (m_selected_face_id_A < 0 || m_selected_face_id_B < 0) return;
    if (m_selected_face_id_A >= static_cast<int>(m_model_faces.size()) ||
        m_selected_face_id_B >= static_cast<int>(m_model_faces.size())) {
        appendLog(QString::fromUtf8("选面索引无效，无法生成焊缝。"));
        return;
    }

    appendLog(QString::fromUtf8("触发了特征面相交算法。"));
    appendLog(QString::fromUtf8("选定面 A ID: ") + QString::number(m_selected_face_id_A) +
              QString::fromUtf8("，面 B ID: ") + QString::number(m_selected_face_id_B));

    TopoDS_Face faceA = m_model_faces[m_selected_face_id_A];
    TopoDS_Face faceB = m_model_faces[m_selected_face_id_B];

    BRepAlgoAPI_Section section(faceA, faceB, Standard_False);
    section.Build();
    if (!section.IsDone()) {
        appendLog(QString::fromUtf8("面相交计算失败，未生成焊缝线。"));
        return;
    }

    TopoDS_Shape seamShape = section.Shape();
    if (seamShape.IsNull()) {
        appendLog(QString::fromUtf8("未检测到交线，无法生成焊缝线。"));
        return;
    }

    seam_counter++;
    QString seamName = QString::fromUtf8("焊缝-") + QString::number(seam_counter);
    if(ui->listWeldSeams) {
        ui->listWeldSeams->addItem(seamName);
    }

    int segmentCount = 0;
    int edgeIndex = 0;
    for (TopExp_Explorer edgeExp(seamShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next(), ++edgeIndex)
    {
        TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
        BRepAdaptor_Curve curve(edge);

        double first = curve.FirstParameter();
        double last = curve.LastParameter();
        if (last <= first) continue;

        double length = GCPnts_AbscissaPoint::Length(curve, first, last);
        int sampleCount = static_cast<int>(length / 1.5);
        if (sampleCount < 8) sampleCount = 8;

        gp_Pnt prevPoint;
        bool hasPrev = false;
        for (int i = 0; i <= sampleCount; ++i)
        {
            double t = first + (last - first) * static_cast<double>(i) / static_cast<double>(sampleCount);
            gp_Pnt p = curve.Value(t);

            if (hasPrev) {
                pcl::PointXYZ p1(prevPoint.X(), prevPoint.Y(), prevPoint.Z());
                pcl::PointXYZ p2(p.X(), p.Y(), p.Z());

                std::string lineId = "seam_line_" + std::to_string(seam_counter) + "_" +
                    std::to_string(edgeIndex) + "_" + std::to_string(i);

                viewer3D->addLine(p1, p2, 0.1, 1.0, 0.1, lineId);
                viewer3D->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6.0, lineId);
                segmentCount++;
            }

            prevPoint = p;
            hasPrev = true;
        }
    }

    if (segmentCount == 0) {
        appendLog(QString::fromUtf8("检测到面关系，但未生成可绘制的焊缝线段。"));
        return;
    }

    // 焊缝生成后，恢复被选面的原始显示状态
    viewer3D->removePolygonMesh("highlight_A");
    viewer3D->removePolygonMesh("highlight_B");
    m_selected_face_id_A = -1;
    m_selected_face_id_B = -1;

    viewer3D->spinOnce(50, true);
    appendLog(QString::fromUtf8("已生成分析项：") + seamName);
    appendLog(QString::fromUtf8("已生成明亮绿色焊缝线，线段数：") + QString::number(segmentCount));
}

// =========================================================================
// 导入三维模型 (STEP)
// =========================================================================
void dianyun::on_btnOpen_clicked()
{
    QString qFilename = QFileDialog::getOpenFileName(this, QString::fromUtf8("导入三维模型"), "", "STEP Files (*.step *.stp);;All Files (*)");
    if (qFilename.isEmpty()) return;

    appendLog(QString::fromUtf8("正在导入三维模型: ") + QFileInfo(qFilename).fileName());
    
    // OCCT 内部主要支持 UTF-8 编码路径
    if (LoadStepFileToPCL(qFilename.toUtf8().toStdString())) {
        viewer3D->removeAllPointClouds();
        viewer3D->removeAllShapes();
        viewer3D->addPolygonMesh(*mesh, "mesh_model");
        viewer3D->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.8, "mesh_model");
        
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*cloud, minPt, maxPt);
        
        // 左侧网格窗口同样将坐标轴缩减至全局尺寸包围盒对角线的 3% (0.03)
        double scale = (maxPt.getVector3fMap() - minPt.getVector3fMap()).norm() * 0.03;
        if (scale < 0.1) scale = 0.1;
        viewer3D->addCoordinateSystem(scale);

        viewer3D->resetCamera();
        appendLog(QString::fromUtf8("三维模型导入成功！"));
    } else {
        appendLog(QString::fromUtf8("三维模型导入失败。"));
    }
}

// =========================================================================
// 导入点云模型 (PLY/PCD)
// =========================================================================
void dianyun::on_btnOpen_2_clicked()
{
    QString qFilename = QFileDialog::getOpenFileName(this, QString::fromUtf8("导入点云模型"), "", "Point Cloud Files (*.ply *.pcd);;All Files (*)");
    if (qFilename.isEmpty()) return;

    appendLog(QString::fromUtf8("正在导入点云模型: ") + QFileInfo(qFilename).fileName());

    cloud->clear();
    std::string ext = QFileInfo(qFilename).suffix().toLower().toStdString();
    
    // PCL IO 处理本地路径时依赖 Local8Bit (GBK) 编码
    if (ext == "pcd") {
        pcl::io::loadPCDFile(qFilename.toLocal8Bit().toStdString(), *cloud);
    } else if (ext == "ply") {
        pcl::io::loadPLYFile(qFilename.toLocal8Bit().toStdString(), *cloud);
    }

    if (!cloud->empty()) {
        viewerCloud->removeAllPointClouds();
        viewerCloud->removeAllShapes();
        
        // 使用 Z 轴高速伪彩色渲染代替原生 RGB，解决模型偏黑无色彩的问题
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> z_color(cloud, "z");
        viewerCloud->addPointCloud<pcl::PointXYZRGB>(cloud, z_color, "point_cloud");
        
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*cloud, minPt, maxPt);
        
        // 将坐标轴缩放比例从 15% (0.15) 大幅减小至 3% (0.03)，避免遮挡模型
        double scale = (maxPt.getVector3fMap() - minPt.getVector3fMap()).norm() * 0.03;
        if (scale < 0.1) scale = 0.1;
        viewerCloud->addCoordinateSystem(scale);

        viewerCloud->resetCamera();
        appendLog(QString::fromUtf8("点云模型导入成功！节点数：") + QString::number(cloud->points.size()));
    } else {
        appendLog(QString::fromUtf8("点云模型导入失败。"));
    }
}

// =========================================================================
// 其他 UI 交互功能修复 (全部应用 viewer3D 等双窗口命名体系，并使用中文)
// =========================================================================
void dianyun::updateStatusUI() {
    QString statusText = QString::fromUtf8("当前状态：");
    if (current_state == STATE_IDLE) statusText += QString::fromUtf8("就绪");
    else if (current_state == STATE_PICK_FACE_A) statusText += QString::fromUtf8("正在选择面A...");
    else if (current_state == STATE_PICK_FACE_B) statusText += QString::fromUtf8("正在选择面B...");

    if (ui->lblStatus) {
        ui->lblStatus->setText(QString("<style>p { color: black; font-weight: bold; }</style><p align=\"center\">%1</p>").arg(statusText));
    }
}

void dianyun::on_btnNewSeam_clicked() {
    current_state = STATE_PICK_FACE_A;
    updateStatusUI();
    appendLog(QString::fromUtf8("开始交互选取：请按住 Ctrl 单击三维模型曲面。"));
}

void dianyun::on_btnCancel_clicked() {
    current_state = STATE_IDLE;
    if(viewer3D){
        viewer3D->removePolygonMesh("highlight_A");
        viewer3D->removePolygonMesh("highlight_B");
        viewer3D->spinOnce(100, true);
    }
    updateStatusUI();
    appendLog(QString::fromUtf8("已取消当前操作。"));
}

void dianyun::on_btnDeleteSeam_clicked() {
    if(ui->listWeldSeams){
        int row = ui->listWeldSeams->currentRow();
        if (row < 0) return;
        QListWidgetItem* item = ui->listWeldSeams->takeItem(row);
        appendLog(QString::fromUtf8("删除了记录：") + item->text());
        delete item;
    }
}

void dianyun::on_listWeldSeams_itemClicked(QListWidgetItem* item) {
    if(!item) return;
    appendLog(QString::fromUtf8("选中记录：") + item->text());
}

void dianyun::updatePCLWindow() {
    if(viewer3D) viewer3D->spinOnce(10, true);
    if(viewerCloud) viewerCloud->spinOnce(10, true);
}

void dianyun::adjustWindowSize() {
    // 强制同步大小
}

void dianyun::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    adjustWindowSize();
}