#include "dianyun.h"
#include <QtWidgets/QApplication>

// 主题与自定义类型注册所需头文件
#include <QFile>
#include <QDateTime>
#include <QDir>
#include <QMetaType>
#include <QPalette>
#include <QStyleFactory>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <typeinfo>
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

// 未捕获的 C++ 异常会直接走 std::terminate 闪退，现场没有任何线索。
// 这里把异常类型与 what() 写入 exe 同目录的 dianyun_crash.log。
std::string g_crashLogPath;

void logTerminate()
{
    std::string text = "未捕获的异常导致程序终止：";
    try {
        if (const std::exception_ptr ep = std::current_exception()) {
            std::rethrow_exception(ep);
        }
        text += "无活动异常（可能是 abort() 或 noexcept 违规）";
    }
    catch (const std::exception& e) {
        text += std::string(typeid(e).name()) + " : " + e.what();
    }
    catch (...) {
        text += "非 std::exception 类型的异常";
    }

    if (std::FILE* f = std::fopen(g_crashLogPath.c_str(), "ab")) {
        std::fprintf(f, "[%s] %s\n",
            QDateTime::currentDateTime().toString(Qt::ISODate).toLocal8Bit().constData(),
            text.c_str());
        std::fclose(f);
    }
    std::abort();
}

void installTerminateLogger()
{
    g_crashLogPath = QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("dianyun_crash.log")).toLocal8Bit().constData();
    std::set_terminate(logTerminate);
}

} // namespace

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    applyDarkTheme(a);
    installTerminateLogger();

    // === 在这里注册 PCL 和 Eigen 的自定义类型 ===
    // 必须放在 QApplication 创建之后，主窗口 dianyun 实例化之前！
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("PointCloudSource::Ptr");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");

    dianyun w;
    w.show();
    return a.exec();
}
