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
#include <QThread>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

// OCCT
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <vector>

// Eigen
#include <Eigen/Core>

// 引入海康相机后台拼接工作类
#include "HikCameraWorker.h"

// 引入模型匹配工作类 (内含 MatchResult 等数据结构定义)
#include "ModelMatchWorker.h"

class HikCameraWorker;

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

    // ===== 相机控制新增槽函数 =====
    void on_btnOpenCamera_clicked();
    void onCameraPointCloudReady(pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud);
    // ==============================

    // ===== 模型匹配新增槽函数 =====
    void on_btnOpen_3_clicked();              // 对应 ui 中 btnOpen_3(模型匹配)
    void onMatchProgress(QString stage);      // 接收匹配算法进度文本
    void onProcessedScanReady(pcl::PointCloud<pcl::PointXYZRGB>::Ptr processedCloud);
    void onMatchFinished(MatchResult result); // 接收最新架构的匹配结果对象
    // ==============================

    void on_btnNewSeam_clicked();
    void on_btnCancel_clicked();
    void on_btnDeleteSeam_clicked();
    void on_listWeldSeams_itemClicked(QListWidgetItem* item);
    void updatePCLWindow();

private:
    Ui::dianyunClass* ui;

    // ===== 相机后台处理指针 =====
    HikCameraWorker* m_cameraWorker = nullptr;
    // ============================

    // ===== 模型匹配新增数据与指针 =====
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cadCloud; // 独立的CAD点云副本，防被清空
    MatchResult m_lastMatchResult;                     // 缓存上一次的匹配结果

    QThread* m_matchThread = nullptr;
    ModelMatchWorker* m_matchWorker = nullptr;
    // ================================

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; // 用于点云显示
    pcl::PolygonMesh::Ptr mesh;                   // 用于 STEP 显示
    pcl::visualization::PCLVisualizer::Ptr viewer3D;
    pcl::visualization::PCLVisualizer::Ptr viewerCloud;
    QTimer* timer;

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
