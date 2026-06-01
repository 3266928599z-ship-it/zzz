#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <QtWidgets/QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <QResizeEvent>
#include <QListWidget>
#include <QLabel>
#include <QMessageBox>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

// OCCT
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <vector>

Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)

class PlyDirectoryWorker;

namespace Ui { class dianyunClass; }

struct WeldSeamData {
    std::string id;
    QString name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
    double length;
};

class dianyun : public QMainWindow
{
    Q_OBJECT

public:
    dianyun(QWidget* parent = nullptr);
    ~dianyun();

protected:
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void on_btnOpen_clicked();     // 对应 ui 中 btnOpen  (导入三维模型)
    void on_btnOpen_2_clicked();   // 对应 ui 中 btnOpen_2(导入点云模型)
    void on_btnNewSeam_clicked();
    void on_btnCancel_clicked();
    void on_btnDeleteSeam_clicked();
    void on_listWeldSeams_itemClicked(QListWidgetItem* item);
    void updatePCLWindow();
    void onCloudLoaded(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const QString& filePath);

private:
    Ui::dianyunClass* ui;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; // 用于点云显示
    pcl::PolygonMesh::Ptr mesh;                   // 用于 STEP 显示
    pcl::visualization::PCLVisualizer::Ptr viewer3D;
    pcl::visualization::PCLVisualizer::Ptr viewerCloud;
    QTimer* timer;
    PlyDirectoryWorker* plyWorker = nullptr;

    std::vector<WeldSeamData> seam_list;
    int seam_counter = 0;

    enum State { STATE_IDLE, STATE_PICK_FACE_A, STATE_PICK_FACE_B };
    State current_state = STATE_IDLE;

    // CAD Data
    std::vector<TopoDS_Face> m_model_faces;
    std::vector<int> m_cell_to_face_map;

    int m_selected_face_id_A = -1;
    int m_selected_face_id_B = -1;

    // Helpers
    void initialWindow();
    void adjustWindowSize();
    void updateStatusUI();
    void appendLog(const QString& msg);

    // Callbacks
    void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* args);
    void highlightFaceMesh(int face_id, std::string id);
    void CalculateSeam_OCCT();
    bool LoadStepFileToPCL(const std::string& filename);
};