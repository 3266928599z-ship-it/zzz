#include "dianyun.h"
#include <QtWidgets/QApplication>

// 主题与自定义类型注册所需头文件
#include <QFile>
#include <QMetaType>
#include <QPalette>
#include <QStyleFactory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace {

void applyDarkTheme(QApplication& app)
{
    app.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));

    QPalette p;
    p.setColor(QPalette::Window,          QColor(0x1E, 0x1E, 0x1E));
    p.setColor(QPalette::WindowText,      QColor(0xE4, 0xE4, 0xE7));
    p.setColor(QPalette::Base,            QColor(0x1B, 0x1B, 0x1B));
    p.setColor(QPalette::AlternateBase,   QColor(0x1F, 0x1F, 0x22));
    p.setColor(QPalette::Text,            QColor(0xE4, 0xE4, 0xE7));
    p.setColor(QPalette::Button,          QColor(0x2D, 0x2D, 0x30));
    p.setColor(QPalette::ButtonText,      QColor(0xE4, 0xE4, 0xE7));
    p.setColor(QPalette::BrightText,      QColor(0xFF, 0xFF, 0xFF));
    p.setColor(QPalette::Highlight,       QColor(0xF5, 0x7C, 0x00));
    p.setColor(QPalette::HighlightedText, QColor(0xFF, 0xFF, 0xFF));
    p.setColor(QPalette::ToolTipBase,     QColor(0x2D, 0x2D, 0x30));
    p.setColor(QPalette::ToolTipText,     QColor(0xE4, 0xE4, 0xE7));
    p.setColor(QPalette::Link,            QColor(0x29, 0xB6, 0xF6));
    p.setColor(QPalette::PlaceholderText, QColor(0x6B, 0x6B, 0x73));

    p.setColor(QPalette::Disabled, QPalette::WindowText, QColor(0x6B, 0x6B, 0x73));
    p.setColor(QPalette::Disabled, QPalette::Text,       QColor(0x6B, 0x6B, 0x73));
    p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(0x6B, 0x6B, 0x73));
    p.setColor(QPalette::Disabled, QPalette::Highlight,  QColor(0x3F, 0x3F, 0x46));
    app.setPalette(p);

    QFile qss(QStringLiteral(":/dianyun/theme/dark.qss"));
    if (qss.open(QIODevice::ReadOnly | QIODevice::Text)) {
        app.setStyleSheet(QString::fromUtf8(qss.readAll()));
    }
}

} // namespace

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    applyDarkTheme(a);

    // === 在这里注册 PCL 和 Eigen 的自定义类型 ===
    // 必须放在 QApplication 创建之后，主窗口 dianyun 实例化之前！
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("PointCloudSource::Ptr");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");

    dianyun w;
    w.show();
    return a.exec();
}
