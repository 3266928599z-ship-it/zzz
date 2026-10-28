# 界面重构实施计划

日期：2026-10-26
设计依据：`docs/superpowers/specs/2026-10-26-ui-redesign-design.md`
视觉样张：`docs/design/ui-mockup.html`
状态：**待确认后执行**

---

## 0. 实施概览

### 0.1 改动文件总表

| 文件 | 类型 | 改动量 | 风险 |
|---|---|---|---|
| `dianyun/dianyun/main.cpp` | 修改 | +32 行 | 低 |
| `dianyun/dianyun/dianyun.ui` | **重写** | 224 → ~430 行 | **高** |
| `dianyun/dianyun/dianyun.h` | 修改 | +18 / −2 行 | 中 |
| `dianyun/dianyun/dianyun.cpp` | 修改 | +190 / −45 行 | **高** |
| `dianyun/dianyun/dianyun.qrc` | 修改 | +9 行 | 低 |
| `dianyun/dianyun/resources/theme/dark.qss` | **新增** | ~300 行 | 低 |
| `dianyun/dianyun/resources/icons/*.svg` | **新增** 7 个 | 各 1~3 行 | 低 |
| `dianyun/dianyun/dianyun.vcxproj` | **不改** | — | — |
| `tests/*` | **不改** | — | — |

### 0.2 不改的东西（明确边界）

- 不动 `ModelMatchWorker` / `ModelMatchController` / `PointCloudLoadController` / `VisualizationManager` / `WeldSeamService` / `WeldSeamMapping` / `HikCameraWorker`。
- 不动 `tests/dianyun.tests.vcxproj`（测试工程只编译 `WeldSeamMapping.cpp`，与 UI 无关）。
- 不改任何 `objectName`，从而保留全部 `on_<name>_<signal>` 自动连接。
- 不引入 QML、不引入第三方 UI 库、不加新的 .cpp 文件（避免改 vcxproj）。

### 0.3 三个已定的技术选型

| 决策 | 选择 | 理由 |
|---|---|---|
| 顶栏实现 | `QWidget#topBar` + `QHBoxLayout` + `QToolButton`，**不用 QToolBar/QAction** | QToolBar 只能放 QAction，会丢失 `on_btnOpen_clicked` 自动连接，被迫改动 9 个槽名。用 QToolButton 则 `QAbstractButton::clicked` 自动连接完全保留 |
| 视口标题 | 保留 `QGroupBox` 标题，用 QSS 美化 | 设计稿原计划用浮动 QLabel 叠加，但 `widget` 带 `WA_PaintOnScreen`，其兄弟控件叠加在原生窗口上有重绘风险。视觉差距小，风险差距大 |
| 主题加载 | `Fusion` + 暗色 `QPalette` + `dark.qss` 三层 | QPalette 负责原生控件（勾选框/下拉箭头/滚动条），QSS 只做品牌层，避免缺图变成空白方块 |

---

## 1. `main.cpp` —— 主题地基

**当前内容**：只有 `QApplication` + 类型注册 + `show()`。

**改后**：

```cpp
#include "dianyun.h"
#include <QtWidgets/QApplication>

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
    app.setStyle(QStyleFactory::create("Fusion"));

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

    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("PointCloudSource::Ptr");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");

    dianyun w;
    w.show();
    return a.exec();
}
```

**注意**：`qRegisterMetaType` 必须在 `QApplication` 之后、`dianyun` 实例化之前（原有约束不变，见 `main.cpp:14-15` 注释）。

---

## 2. `resources/theme/dark.qss` —— 新增

完整内容：

```css
/* ===== 基础 ===== */
QWidget {
    color: #E4E4E7;
    font-family: "Microsoft YaHei UI", "Microsoft YaHei", sans-serif;
    font-size: 9pt;
}
QMainWindow, QWidget#centralWidget {
    background: #1E1E1E;
}
QToolTip {
    background: #2D2D30; color: #E4E4E7;
    border: 1px solid #3F3F46; padding: 4px 7px;
}

/* ===== 顶栏 ===== */
QWidget#topBar {
    background: #2D2D30;
    border-bottom: 1px solid #3F3F46;
}
QWidget#topBar QToolButton {
    color: #C8C8C8;
    background: transparent;
    border: 1px solid transparent;
    border-radius: 4px;
    padding: 4px 8px;
    min-width: 62px;
}
QWidget#topBar QToolButton:hover {
    background: #3A3A3E; color: #FFFFFF;
}
QWidget#topBar QToolButton:pressed {
    background: #45454A;
}
QWidget#topBar QToolButton:disabled {
    color: #6B6B73;
    background: transparent;
}
QFrame#topBarSep {
    background: #3F3F46;
    max-width: 1px; min-width: 1px;
}

/* ===== 告警条 ===== */
QFrame#warningBanner {
    background: #F57C00;
    border: none;
}
QFrame#warningBanner[level="error"] {
    background: #E53935;
}
QFrame#warningBanner QLabel {
    color: #1E1E1E;
    font-weight: 600;
    background: transparent;
}
QFrame#warningBanner[level="error"] QLabel {
    color: #FFFFFF;
}
QLabel#lblAlarmCount {
    background: rgba(0, 0, 0, 0.22);
    border-radius: 8px;
    padding: 1px 7px;
    font-size: 8pt;
}
QFrame#warningBanner QToolButton#btnAckAlarm {
    color: #1E1E1E;
    background: transparent;
    border: 1px solid rgba(0, 0, 0, 0.35);
    border-radius: 3px;
    padding: 3px 12px;
    font-weight: 600;
}
QFrame#warningBanner QToolButton#btnAckAlarm:hover {
    background: rgba(0, 0, 0, 0.14);
}
QFrame#warningBanner[level="error"] QToolButton#btnAckAlarm {
    color: #FFFFFF;
    border-color: rgba(255, 255, 255, 0.45);
}

/* ===== 面板 ===== */
QWidget#seamPanel, QWidget#opsPanel {
    background: #252526;
}
QWidget#seamHeader {
    background: #252526;
    border-bottom: 1px solid #3F3F46;
}
QLabel#lblPanelTitle, QLabel#lblSectionTitle {
    color: #A1A1AA;
    font-weight: 600;
    letter-spacing: 1px;
    font-size: 8pt;
}
QFrame#vsep {
    background: #3F3F46;
    max-width: 1px; min-width: 1px;
}
QFrame#hline {
    background: #3F3F46;
    max-height: 1px; min-height: 1px;
}

/* 图标按钮（+ / ×） */
QWidget#seamHeader QToolButton {
    color: #A1A1AA;
    background: transparent;
    border: 1px solid transparent;
    border-radius: 4px;
    font-size: 12pt;
}
QWidget#seamHeader QToolButton:hover {
    background: #3A3A3E;
    color: #E4E4E7;
    border-color: #3F3F46;
}
QToolButton#btnDeleteSeam:hover {
    color: #E53935;
    border-color: #E53935;
}

/* ===== 焊缝表格 ===== */
QTreeWidget#listWeldSeams {
    background: #1B1B1B;
    alternate-background-color: #1F1F22;
    border: 1px solid #3F3F46;
    border-radius: 4px;
    outline: none;
}
QTreeWidget#listWeldSeams::item {
    padding: 6px 6px;
    border: none;
}
QTreeWidget#listWeldSeams::item:hover {
    background: #2A2A2E;
}
QTreeWidget#listWeldSeams::item:selected {
    background: #F57C00;
    color: #FFFFFF;
}
QHeaderView::section {
    background: #2D2D30;
    color: #A1A1AA;
    padding: 6px;
    border: none;
    border-bottom: 1px solid #3F3F46;
    font-weight: 600;
    font-size: 8pt;
}

/* ===== 状态卡 ===== */
QFrame#statusCard {
    background: #1B1B1B;
    border: 1px solid #3F3F46;
    border-left: 2px solid #F57C00;
    border-radius: 4px;
}
QLabel#lblStatus {
    color: #E4E4E7;
}
QLabel#lblStatusDot {
    color: #F57C00;
    font-size: 10pt;
}
QLabel#lblSeamEmpty {
    color: #6B6B73;
}

/* ===== 视口 ===== */
QGroupBox#groupBox_2, QGroupBox#groupBox_3 {
    border: 1px solid #3F3F46;
    border-radius: 4px;
    margin-top: 20px;
    padding-top: 4px;
    background: #171717;
}
QGroupBox#groupBox_2::title, QGroupBox#groupBox_3::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    color: #A1A1AA;
    font-weight: 600;
}

/* ===== 按钮 ===== */
QPushButton {
    background: #2D2D30;
    color: #E4E4E7;
    border: 1px solid #3F3F46;
    border-radius: 4px;
    padding: 8px 14px;
}
QPushButton:hover {
    background: #3A3A3E;
    border-color: #52525B;
}
QPushButton:pressed {
    background: #232326;
}
QPushButton:disabled {
    background: #2A2A2E;
    color: #6B6B73;
    border-color: #33333A;
}
QPushButton[role="primary"] {
    background: #F57C00;
    color: #FFFFFF;
    border: 1px solid #F57C00;
    font-weight: 600;
}
QPushButton[role="primary"]:hover {
    background: #FB8C00;
    border-color: #FB8C00;
}
QPushButton[role="primary"]:pressed {
    background: #E65100;
    border-color: #E65100;
}
QPushButton[role="primary"]:disabled {
    background: #33333A;
    color: #6B6B73;
    border-color: #33333A;
}
QPushButton[role="ghost"] {
    background: transparent;
    border: 1px solid #3F3F46;
}
QPushButton[role="ghost"]:hover {
    border-color: #F57C00;
    color: #F57C00;
    background: transparent;
}

/* ===== 匹配结果卡 ===== */
QFrame#matchResultCard {
    background: #1B1B1B;
    border: 1px solid #3F3F46;
    border-radius: 4px;
}
QLabel[role="metricKey"] {
    color: #A1A1AA;
    font-size: 8pt;
}
QLabel[role="metricValue"] {
    color: #E4E4E7;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 9pt;
}
QLabel#lblVerdict {
    font-weight: 600;
}
QLabel#lblVerdict[tone="pass"]   { color: #2AC44E; }
QLabel#lblVerdict[tone="review"] { color: #F57C00; }
QLabel#lblVerdict[tone="fail"]   { color: #E53935; }
QLabel#lblVerdict[tone="none"]   { color: #6B6B73; }

/* ===== 日志 / 报警页 ===== */
QTabWidget#logTabs::pane {
    border: none;
    border-top: 1px solid #3F3F46;
    background: #252526;
}
QTabBar::tab {
    background: transparent;
    color: #A1A1AA;
    padding: 8px 18px;
    border: none;
    border-bottom: 2px solid transparent;
}
QTabBar::tab:selected {
    color: #E4E4E7;
    border-bottom-color: #F57C00;
    font-weight: 600;
}
QTabBar::tab:hover:!selected {
    color: #E4E4E7;
}
QTextEdit#textLog, QListWidget#listAlarms {
    background: #1B1B1B;
    border: 1px solid #3F3F46;
    border-radius: 4px;
    selection-background-color: #F57C00;
    selection-color: #FFFFFF;
}
QTextEdit#textLog {
    font-family: "Consolas", "Courier New", monospace;
    font-size: 9pt;
}

/* ===== 分栏手柄 ===== */
QSplitter::handle {
    background: #1E1E1E;
}
QSplitter::handle:horizontal { width: 4px; }
QSplitter::handle:vertical   { height: 4px; }
QSplitter::handle:hover      { background: #F57C00; }

/* ===== 滚动条 ===== */
QScrollBar:vertical {
    background: #1B1B1B; width: 10px; margin: 0; border: none;
}
QScrollBar::handle:vertical {
    background: #3F3F46; min-height: 28px; border-radius: 5px;
}
QScrollBar::handle:vertical:hover { background: #52525B; }
QScrollBar:horizontal {
    background: #1B1B1B; height: 10px; margin: 0; border: none;
}
QScrollBar::handle:horizontal {
    background: #3F3F46; min-width: 28px; border-radius: 5px;
}
QScrollBar::handle:horizontal:hover { background: #52525B; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: none; }

/* ===== 状态栏 ===== */
QStatusBar {
    background: #2D2D30;
    border-top: 1px solid #3F3F46;
    color: #A1A1AA;
}
QStatusBar::item { border: none; }
QLabel[role="statusItem"] { color: #A1A1AA; }
QLabel[role="statusValue"] {
    color: #E4E4E7;
    font-family: "Consolas", "Courier New", monospace;
}

/* ===== 菜单 ===== */
QMenuBar {
    background: #2D2D30;
    border-bottom: 1px solid #3F3F46;
}
QMenuBar::item {
    background: transparent;
    padding: 5px 11px;
    color: #E4E4E7;
}
QMenuBar::item:selected { background: #3A3A3E; }
QMenu {
    background: #2D2D30;
    border: 1px solid #3F3F46;
    padding: 4px;
}
QMenu::item {
    padding: 6px 26px 6px 14px;
    border-radius: 3px;
}
QMenu::item:selected { background: #F57C00; color: #FFFFFF; }
QMenu::separator {
    height: 1px; background: #3F3F46; margin: 4px 8px;
}
```

**关键取舍**：不覆盖 `QCheckBox::indicator` / `QComboBox::drop-down` / `QScrollBar::up-arrow` 等 primitive 指示器——交给 Fusion + QPalette 自动绘制。这样即使没有任何图片资源也不会出现空白方块。

---

## 3. `resources/icons/*.svg` —— 新增 7 个

统一规格：`viewBox="0 0 24 24"`，`fill="none"`，`stroke="#C8C8C8"`，`stroke-width="1.75"`，`stroke-linecap="round"`，`stroke-linejoin="round"`。

| 文件名 | 用途 | `d` 内容 |
|---|---|---|
| `import-cad.svg` | 导入三维模型 | 立方体轮廓 + 内部分面 |
| `import-cloud.svg` | 导入点云模型 | 散点 + 下行箭头 |
| `match.svg` | 模型匹配 | 两错位方框 + 对齐虚线 + 箭头 |
| `camera.svg` | 开始拍摄 | 相机机身 + 镜头圆 |
| `robot-arm.svg` | 机械臂焊接 | 三段折线臂 + 关节圆 |
| `fit-view.svg` | 视图复位 | 四角取景框 + 中心圆 |
| `alarm-clear.svg` | 解除报警 | 铃铛 + 斜杠 |

示例（`import-cad.svg`，其余同规格）：

```svg
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none"
     stroke="#C8C8C8" stroke-width="1.75"
     stroke-linecap="round" stroke-linejoin="round">
  <path d="M12 3 3 7.5v9L12 21l9-4.5v-9L12 3Z"/>
  <path d="M3 7.5 12 12l9-4.5"/>
  <path d="M12 12v9"/>
</svg>
```

**颜色说明**：SVG 内写死 `#C8C8C8`。`QIcon` 会自动生成灰色禁用版本，因此 `:disabled` 状态无需额外资源；hover 变色靠 QSS 背景变化表达，不改图标。

**降级策略**（防 SVG 插件缺失）：构造时检测

```cpp
const QIcon icon(QStringLiteral(":/dianyun/icons/import-cad.svg"));
if (icon.isNull() || icon.availableSizes().isEmpty()) {
    button->setToolButtonStyle(Qt::ToolButtonTextOnly);
    button->setMinimumWidth(84);
}
```

若图标加载失败，自动退化为纯文字按钮，不出现空白图片位。部署时确认 `windeployqt` 已带上 `imageformats/qsvg.dll`。

---

## 4. `dianyun.qrc` —— 修改

**现在**（空壳）：

```xml
<RCC>
    <qresource prefix="dianyun">
    </qresource>
</RCC>
```

**改后**：

```xml
<RCC>
    <qresource prefix="/dianyun">
        <file alias="theme/dark.qss">resources/theme/dark.qss</file>
        <file alias="icons/import-cad.svg">resources/icons/import-cad.svg</file>
        <file alias="icons/import-cloud.svg">resources/icons/import-cloud.svg</file>
        <file alias="icons/match.svg">resources/icons/match.svg</file>
        <file alias="icons/camera.svg">resources/icons/camera.svg</file>
        <file alias="icons/robot-arm.svg">resources/icons/robot-arm.svg</file>
        <file alias="icons/fit-view.svg">resources/icons/fit-view.svg</file>
        <file alias="icons/alarm-clear.svg">resources/icons/alarm-clear.svg</file>
    </qresource>
</RCC>
```

运行时路径：`:/dianyun/theme/dark.qss`、`:/dianyun/icons/camera.svg`。

**vcxproj 无需改动**——`QtRcc` 会监听 qrc 及其引用文件的变化。

---

## 5. `dianyun.ui` —— 重写

### 5.1 objectName 兼容性对照（必须全部保留）

| objectName | 类 | 代码引用次数 | 新位置 |
|---|---|---|---|
| `widget` | QWidget | 7 | 中区 `groupBox_2` 内（VTK 挂载点） |
| `widget_2` | QWidget | 7 | 中区 `groupBox_3` 内（VTK 挂载点） |
| `lblStatus` | QLabel | 20 | 左栏状态卡内 |
| `listWeldSeams` | **QTreeWidget**（原 QListWidget） | 10 | 左栏主体 |
| `textLog` | QTextEdit | 2 | 底部日志页 |
| `ledCapture` | QRoundLed | 2 | 顶栏拍摄按钮右侧 |
| `ledRobotWeld` | QRoundLed | 2 | 顶栏焊接按钮右侧 |
| `groupBoxLog` | — | 2 | **废弃**（见 §7.4，删两行 `setTitle`） |
| `btnOpen` | QToolButton | 2 | 顶栏第 1 组 |
| `btnOpen_2` | QToolButton | 2 | 顶栏第 1 组 |
| `btnOpen_3` | QToolButton | 2 | 顶栏第 2 组 |
| `btnOpenCamera` | QToolButton | 2 | 顶栏第 3 组 |
| `btnMapSeam` | QPushButton | 2 | 右栏 |
| `btnNewSeam` | QToolButton | 自动连接 | 左栏标题行 |
| `btnCancel` | QPushButton | 自动连接 | 右栏 |
| `btnDeleteSeam` | QToolButton | 自动连接 | 左栏标题行 |
| `btnRobotWeld` | QToolButton | 自动连接 | 顶栏第 3 组 |

**自动连接矩阵校验**（`QMetaObject::connectSlotsByName`）：

| 槽 | 期望信号 | 是否成立 |
|---|---|---|
| `on_btnOpen_clicked` | `QToolButton::clicked`（继承自 `QAbstractButton`） | ✅ |
| `on_btnOpen_2_clicked` | 同上 | ✅ |
| `on_btnOpen_3_clicked` | 同上 | ✅ |
| `on_btnOpenCamera_clicked` | 同上 | ✅ |
| `on_btnRobotWeld_clicked` | 同上 | ✅ |
| `on_btnNewSeam_clicked` | 同上 | ✅ |
| `on_btnMapSeam_clicked` | `QPushButton::clicked` | ✅ |
| `on_btnCancel_clicked` | 同上 | ✅ |
| `on_btnDeleteSeam_clicked` | 同上 | ✅ |
| `on_listWeldSeams_currentItemChanged` | `QTreeWidget::currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)` | ✅ 签名一致 |

> `QListWidget` → `QTreeWidget` 的切换**不影响自动连接**，因为两者都有 `currentItemChanged` 且签名相同。这是本次能平滑迁移的关键。

### 5.2 新增 objectName

| objectName | 类 | 用途 |
|---|---|---|
| `centralLayout` | QVBoxLayout | 中央垂直布局 |
| `warningBanner` | QFrame | 告警条容器（`level` 动态属性：`warning`/`error`） |
| `lblAlarmIcon` | QLabel | "⚠" |
| `lblWarningText` | QLabel | 告警正文 |
| `lblAlarmCount` | QLabel | 条数徽标 |
| `btnAckAlarm` | QToolButton | 解除报警 |
| `topBar` | QWidget | 顶栏容器 |
| `topBarSep1` / `topBarSep2` | QFrame | 分组分隔线 |
| `btnFitView` | QToolButton | 视图复位 |
| `mainSplitter` | QSplitter(V) | 工作区 / 日志 |
| `workspaceSplitter` | QSplitter(H) | 左 / 中 / 右 |
| `viewportSplitter` | QSplitter(H) | CAD / 点云 |
| `seamPanel` | QWidget | 左栏 |
| `seamHeader` | QWidget | 左栏标题行 |
| `lblPanelTitle` | QLabel | "焊缝列表" |
| `lblSeamEmpty` | QLabel | 空态提示 |
| `statusCard` | QFrame | 状态卡容器 |
| `lblStatusDot` | QLabel | "●" |
| `opsPanel` | QWidget | 右栏 |
| `matchResultCard` | QFrame | 匹配结果卡（默认隐藏） |
| `lblVerdictDot` | QLabel | 结论圆点 |
| `lblVerdict` | QLabel | 结论文字 |
| `lblOverlap` | QLabel | 覆盖率 |
| `lblRmse` / `lblP95` / `lblNormalConsistency` | QLabel | 三项指标 |
| `lblCadPoints` / `lblCloudPoints` / `lblSeamCount` / `lblMappedCount` | QLabel | 数据概况 |
| `logTabs` | QTabWidget | 日志 / 报警 |
| `tabLog` / `tabAlarm` | QWidget | 两个页 |
| `listAlarms` | QListWidget | 报警记录 |
| `statusStage` / `statusCad` / `statusCloud` / `statusMatch` / `statusSeam` | QLabel | 状态栏常驻项 |

### 5.3 完整 XML

```xml
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>dianyunClass</class>
 <widget class="QMainWindow" name="dianyunClass">
  <property name="geometry">
   <rect><x>0</x><y>0</y><width>1600</width><height>950</height></rect>
  </property>
  <property name="minimumSize">
   <size><width>1280</width><height>760</height></size>
  </property>
  <property name="windowTitle">
   <string>dianyun - 焊接视觉系统</string>
  </property>
  <widget class="QWidget" name="centralWidget">
   <layout class="QVBoxLayout" name="centralLayout" stretch="0,0,1">
    <property name="spacing"><number>0</number></property>
    <property name="leftMargin"><number>0</number></property>
    <property name="topMargin"><number>0</number></property>
    <property name="rightMargin"><number>0</number></property>
    <property name="bottomMargin"><number>0</number></property>

    <!-- ============ 告警条 ============ -->
    <item>
     <widget class="QFrame" name="warningBanner">
      <property name="minimumSize"><size><width>0</width><height>32</height></size></property>
      <property name="maximumSize"><size><width>16777215</width><height>32</height></size></property>
      <property name="level" stdset="0"><string>warning</string></property>
      <layout class="QHBoxLayout" name="bannerLayout">
       <property name="spacing"><number>10</number></property>
       <property name="leftMargin"><number>14</number></property>
       <property name="rightMargin"><number>14</number></property>
       <item><widget class="QLabel" name="lblAlarmIcon">
         <property name="text"><string>⚠</string></property>
       </widget></item>
       <item><widget class="QLabel" name="lblWarningText">
         <property name="text"><string/></property>
         <property name="sizePolicy">
          <sizepolicy hsizetype="Expanding" vsizetype="Preferred">
           <horstretch>1</horstretch><verstretch>0</verstretch>
          </sizepolicy>
         </property>
       </widget></item>
       <item><widget class="QLabel" name="lblAlarmCount">
         <property name="text"><string/></property>
       </widget></item>
       <item><widget class="QToolButton" name="btnAckAlarm">
         <property name="text"><string>解除报警</string></property>
         <property name="cursor"><cursorShape>PointingHandCursor</cursorShape></property>
       </widget></item>
      </layout>
     </widget>
    </item>

    <!-- ============ 顶栏 ============ -->
    <item>
     <widget class="QWidget" name="topBar">
      <property name="minimumSize"><size><width>0</width><height>58</height></size></property>
      <property name="maximumSize"><size><width>16777215</width><height>58</height></size></property>
      <layout class="QHBoxLayout" name="topBarLayout">
       <property name="spacing"><number>2</number></property>
       <property name="leftMargin"><number>8</number></property>
       <property name="rightMargin"><number>8</number></property>
       <property name="topMargin"><number>4</number></property>
       <property name="bottomMargin"><number>4</number></property>
       <item><widget class="QToolButton" name="btnOpen">
         <property name="toolTip"><string>导入 STEP/STP 三维模型 (Ctrl+O)</string></property>
         <property name="text"><string>导入三维模型</string></property>
         <property name="toolButtonStyle">
          <enum>Qt::ToolButtonStyle::ToolButtonTextUnderIcon</enum></property>
         <property name="iconSize"><size><width>22</width><height>22</height></size></property>
       </widget></item>
       <item><widget class="QToolButton" name="btnOpen_2">
         <property name="toolTip"><string>导入 PLY/PCD 点云模型 (Ctrl+Shift+O)</string></property>
         <property name="text"><string>导入点云模型</string></property>
         <property name="toolButtonStyle">
          <enum>Qt::ToolButtonStyle::ToolButtonTextUnderIcon</enum></property>
         <property name="iconSize"><size><width>22</width><height>22</height></size></property>
       </widget></item>
       <item><widget class="QFrame" name="topBarSep1">
         <property name="frameShape"><enum>QFrame::Shape::NoFrame</enum></property>
         <property name="minimumSize"><size><width>1</width><height>34</height></size></property>
         <property name="maximumSize"><size><width>1</width><height>16777215</height></size></property>
       </widget></item>
       <item><widget class="QToolButton" name="btnOpen_3">
         <property name="toolTip"><string>执行 CAD 与点云刚体匹配 (Ctrl+M)</string></property>
         <property name="text"><string>模型匹配</string></property>
         <property name="toolButtonStyle">
          <enum>Qt::ToolButtonStyle::ToolButtonTextUnderIcon</enum></property>
         <property name="iconSize"><size><width>22</width><height>22</height></size></property>
       </widget></item>
       <item><widget class="QFrame" name="topBarSep2">
         <property name="frameShape"><enum>QFrame::Shape::NoFrame</enum></property>
         <property name="minimumSize"><size><width>1</width><height>34</height></size></property>
         <property name="maximumSize"><size><width>1</width><height>16777215</height></size></property>
       </widget></item>
       <item><widget class="QToolButton" name="btnOpenCamera">
         <property name="toolTip"><string>启动海康相机采集</string></property>
         <property name="text"><string>开始拍摄</string></property>
         <property name="toolButtonStyle">
          <enum>Qt::ToolButtonStyle::ToolButtonTextUnderIcon</enum></property>
         <property name="iconSize"><size><width>22</width><height>22</height></size></property>
       </widget></item>
       <item><widget class="QRoundLed" name="ledCapture"/></item>
       <item><widget class="QToolButton" name="btnRobotWeld">
         <property name="toolTip"><string>启动机械臂焊接流程</string></property>
         <property name="text"><string>机械臂焊接</string></property>
         <property name="toolButtonStyle">
          <enum>Qt::ToolButtonStyle::ToolButtonTextUnderIcon</enum></property>
         <property name="iconSize"><size><width>22</width><height>22</height></size></property>
       </widget></item>
       <item><widget class="QRoundLed" name="ledRobotWeld"/></item>
       <item>
        <spacer name="topBarSpacer">
         <property name="orientation">
          <enum>Qt::Orientation::Horizontal</enum></property>
         <property name="sizeHint" stdset="0">
          <size><width>40</width><height>20</height></size></property>
        </spacer>
       </item>
       <item><widget class="QToolButton" name="btnFitView">
         <property name="toolTip"><string>复位两个视口相机 (F5)</string></property>
         <property name="text"><string>视图复位</string></property>
         <property name="toolButtonStyle">
          <enum>Qt::ToolButtonStyle::ToolButtonTextUnderIcon</enum></property>
         <property name="iconSize"><size><width>22</width><height>22</height></size></property>
       </widget></item>
      </layout>
     </widget>
    </item>

    <!-- ============ 主体 ============ -->
    <item>
     <widget class="QSplitter" name="mainSplitter">
      <property name="orientation">
       <enum>Qt::Orientation::Vertical</enum></property>
      <property name="childrenCollapsible"><bool>false</bool></property>

      <widget class="QSplitter" name="workspaceSplitter">
       <property name="orientation">
        <enum>Qt::Orientation::Horizontal</enum></property>
       <property name="childrenCollapsible"><bool>false</bool></property>

       <!-- ---------- 左栏 ---------- -->
       <widget class="QWidget" name="seamPanel">
        <property name="minimumSize"><size><width>280</width><height>0</height></size></property>
        <layout class="QVBoxLayout" name="seamPanelLayout">
         <property name="spacing"><number>8</number></property>
         <property name="leftMargin"><number>8</number></property>
         <property name="topMargin"><number>0</number></property>
         <property name="rightMargin"><number>8</number></property>
         <property name="bottomMargin"><number>8</number></property>
         <item>
          <widget class="QWidget" name="seamHeader">
           <property name="minimumSize"><size><width>0</width><height>38</height></size></property>
           <property name="maximumSize"><size><width>16777215</width><height>38</height></size></property>
           <layout class="QHBoxLayout" name="seamHeaderLayout">
            <property name="spacing"><number>4</number></property>
            <property name="leftMargin"><number>2</number></property>
            <property name="rightMargin"><number>2</number></property>
            <property name="topMargin"><number>0</number></property>
            <property name="bottomMargin"><number>0</number></property>
            <item><widget class="QLabel" name="lblPanelTitle">
              <property name="text"><string>焊缝列表</string></property>
            </widget></item>
            <item><spacer name="seamHeaderSpacer">
              <property name="orientation">
               <enum>Qt::Orientation::Horizontal</enum></property>
              <property name="sizeHint" stdset="0">
               <size><width>20</width><height>20</height></size></property>
            </spacer></item>
            <item><widget class="QToolButton" name="btnNewSeam">
              <property name="toolTip">
               <string>新建焊缝：按住 Ctrl 在三维视图中单击两个面</string></property>
              <property name="text"><string>+</string></property>
            </widget></item>
            <item><widget class="QToolButton" name="btnDeleteSeam">
              <property name="toolTip"><string>删除选中焊缝</string></property>
              <property name="text"><string>×</string></property>
            </widget></item>
           </layout>
          </widget>
         </item>
         <item>
          <widget class="QTreeWidget" name="listWeldSeams">
           <property name="alternatingRowColors"><bool>true</bool></property>
           <property name="rootIsDecorated"><bool>false</bool></property>
           <property name="uniformRowHeights"><bool>true</bool></property>
           <property name="selectionBehavior">
            <enum>QAbstractItemView::SelectionBehavior::SelectRows</enum></property>
           <property name="editTriggers">
            <set>QAbstractItemView::EditTrigger::NoEditTriggers</set></property>
           <column><property name="text"><string>焊缝</string></property></column>
           <column><property name="text"><string>长度</string></property></column>
           <column><property name="text"><string>面</string></property></column>
           <column><property name="text"><string>映射</string></property></column>
          </widget>
         </item>
         <item>
          <widget class="QLabel" name="lblSeamEmpty">
           <property name="text">
            <string>暂无焊缝，点击右上角 + 新建</string></property>
           <property name="alignment">
            <set>Qt::AlignmentFlag::AlignCenter</set></property>
           <property name="wordWrap"><bool>true</bool></property>
          </widget>
         </item>
         <item>
          <widget class="QFrame" name="statusCard">
           <property name="minimumSize"><size><width>0</width><height>36</height></size></property>
           <layout class="QHBoxLayout" name="statusCardLayout">
            <property name="spacing"><number>8</number></property>
            <property name="leftMargin"><number>10</number></property>
            <property name="rightMargin"><number>10</number></property>
            <property name="topMargin"><number>0</number></property>
            <property name="bottomMargin"><number>0</number></property>
            <item><widget class="QLabel" name="lblStatusDot">
              <property name="text"><string>●</string></property>
            </widget></item>
            <item><widget class="QLabel" name="lblStatus">
              <property name="text"><string>就绪</string></property>
              <property name="sizePolicy">
               <sizepolicy hsizetype="Expanding" vsizetype="Preferred">
                <horstretch>1</horstretch><verstretch>0</verstretch>
               </sizepolicy></property>
            </widget></item>
           </layout>
          </widget>
         </item>
        </layout>
       </widget>

       <!-- ---------- 中区：双视口 ---------- -->
       <widget class="QSplitter" name="viewportSplitter">
        <property name="orientation">
         <enum>Qt::Orientation::Horizontal</enum></property>
        <property name="childrenCollapsible"><bool>false</bool></property>

        <widget class="QGroupBox" name="groupBox_2">
         <property name="title"><string>三维模型</string></property>
         <layout class="QVBoxLayout" name="verticalLayout_3">
          <property name="leftMargin"><number>2</number></property>
          <property name="topMargin"><number>2</number></property>
          <property name="rightMargin"><number>2</number></property>
          <property name="bottomMargin"><number>2</number></property>
          <item><widget class="QWidget" name="widget" native="true">
            <property name="sizePolicy">
             <sizepolicy hsizetype="Expanding" vsizetype="Expanding">
              <horstretch>1</horstretch><verstretch>1</verstretch>
             </sizepolicy></property>
            <property name="minimumSize"><size><width>280</width><height>200</height></size></property>
          </widget></item>
         </layout>
        </widget>

        <widget class="QGroupBox" name="groupBox_3">
         <property name="title"><string>点云模型</string></property>
         <layout class="QVBoxLayout" name="verticalLayout_4">
          <property name="leftMargin"><number>2</number></property>
          <property name="topMargin"><number>2</number></property>
          <property name="rightMargin"><number>2</number></property>
          <property name="bottomMargin"><number>2</number></property>
          <item><widget class="QWidget" name="widget_2" native="true">
            <property name="sizePolicy">
             <sizepolicy hsizetype="Expanding" vsizetype="Expanding">
              <horstretch>1</horstretch><verstretch>1</verstretch>
             </sizepolicy></property>
            <property name="minimumSize"><size><width>280</width><height>200</height></size></property>
          </widget></item>
         </layout>
        </widget>
       </widget>

       <!-- ---------- 右栏 ---------- -->
       <widget class="QWidget" name="opsPanel">
        <property name="minimumSize"><size><width>262</width><height>0</height></size></property>
        <property name="maximumSize"><size><width>340</width><height>16777215</height></size></property>
        <layout class="QVBoxLayout" name="opsPanelLayout">
         <property name="spacing"><number>8</number></property>
         <property name="leftMargin"><number>10</number></property>
         <property name="topMargin"><number>0</number></property>
         <property name="rightMargin"><number>10</number></property>
         <property name="bottomMargin"><number>10</number></property>

         <item>
          <widget class="QWidget" name="opsHeader">
           <property name="minimumSize"><size><width>0</width><height>38</height></size></property>
           <property name="maximumSize"><size><width>16777215</width><height>38</height></size></property>
           <layout class="QHBoxLayout" name="opsHeaderLayout">
            <property name="leftMargin"><number>2</number></property>
            <property name="rightMargin"><number>2</number></property>
            <property name="topMargin"><number>0</number></property>
            <property name="bottomMargin"><number>0</number></property>
            <item><widget class="QLabel" name="lblOpsTitle">
              <property name="text"><string>焊缝操作</string></property>
            </widget></item>
           </layout>
          </widget>
         </item>

         <item><widget class="QPushButton" name="btnMapSeam">
           <property name="enabled"><bool>false</bool></property>
           <property name="text"><string>焊缝映射</string></property>
           <property name="role" stdset="0"><string>primary</string></property>
         </widget></item>
         <item><widget class="QPushButton" name="btnCancel">
           <property name="text"><string>取消操作</string></property>
           <property name="role" stdset="0"><string>ghost</string></property>
         </widget></item>

         <item><widget class="QFrame" name="hline1">
           <property name="frameShape"><enum>QFrame::Shape::NoFrame</enum></property>
           <property name="minimumSize"><size><width>0</width><height>1</height></size></property>
           <property name="maximumSize"><size><width>16777215</width><height>1</height></size></property>
         </widget></item>

         <item>
          <widget class="QFrame" name="matchResultCard">
           <layout class="QVBoxLayout" name="matchCardLayout">
            <property name="spacing"><number>6</number></property>
            <property name="leftMargin"><number>12</number></property>
            <property name="topMargin"><number>10</number></property>
            <property name="rightMargin"><number>12</number></property>
            <property name="bottomMargin"><number>12</number></property>

            <item>
             <layout class="QHBoxLayout" name="verdictRow">
              <property name="spacing"><number>7</number></property>
              <item><widget class="QLabel" name="lblVerdictDot">
                <property name="text"><string/></property>
              </widget></item>
              <item><widget class="QLabel" name="lblVerdict">
                <property name="text"><string>—</string></property>
                <property name="tone" stdset="0"><string>none</string></property>
              </widget></item>
              <item><spacer name="verdictSpacer">
                <property name="orientation">
                 <enum>Qt::Orientation::Horizontal</enum></property>
                <property name="sizeHint" stdset="0">
                 <size><width>20</width><height>20</height></size></property>
              </spacer></item>
              <item><widget class="QLabel" name="lblOverlap">
                <property name="text"><string>—</string></property>
                <property name="role" stdset="0"><string>metricValue</string></property>
              </widget></item>
             </layout>
            </item>

            <item><widget class="QFrame" name="hline2">
              <property name="frameShape"><enum>QFrame::Shape::NoFrame</enum></property>
              <property name="minimumSize"><size><width>0</width><height>1</height></size></property>
              <property name="maximumSize"><size><width>16777215</width><height>1</height></size></property>
            </widget></item>

            <item>
             <layout class="QHBoxLayout" name="metricRow1">
              <item><widget class="QLabel" name="lblRmseKey">
                <property name="text"><string>RMSE</string></property>
                <property name="role" stdset="0"><string>metricKey</string></property>
              </widget></item>
              <item><spacer name="s1">
                <property name="orientation">
                 <enum>Qt::Orientation::Horizontal</enum></property>
                <property name="sizeHint" stdset="0">
                 <size><width>20</width><height>20</height></size></property>
              </spacer></item>
              <item><widget class="QLabel" name="lblRmse">
                <property name="text"><string>—</string></property>
                <property name="role" stdset="0"><string>metricValue</string></property>
              </widget></item>
             </layout>
            </item>

            <item>
             <layout class="QHBoxLayout" name="metricRow2">
              <item><widget class="QLabel" name="lblP95Key">
                <property name="text"><string>P95 残差</string></property>
                <property name="role" stdset="0"><string>metricKey</string></property>
              </widget></item>
              <item><spacer name="s2">
                <property name="orientation">
                 <enum>Qt::Orientation::Horizontal</enum></property>
                <property name="sizeHint" stdset="0">
                 <size><width>20</width><height>20</height></size></property>
              </spacer></item>
              <item><widget class="QLabel" name="lblP95">
                <property name="text"><string>—</string></property>
                <property name="role" stdset="0"><string>metricValue</string></property>
              </widget></item>
             </layout>
            </item>

            <item>
             <layout class="QHBoxLayout" name="metricRow3">
              <item><widget class="QLabel" name="lblNormalKey">
                <property name="text"><string>法向一致性</string></property>
                <property name="role" stdset="0"><string>metricKey</string></property>
              </widget></item>
              <item><spacer name="s3">
                <property name="orientation">
                 <enum>Qt::Orientation::Horizontal</enum></property>
                <property name="sizeHint" stdset="0">
                 <size><width>20</width><height>20</height></size></property>
              </spacer></item>
              <item><widget class="QLabel" name="lblNormalConsistency">
                <property name="text"><string>—</string></property>
                <property name="role" stdset="0"><string>metricValue</string></property>
              </widget></item>
             </layout>
            </item>
           </layout>
          </widget>
         </item>

         <item><widget class="QFrame" name="hline3">
           <property name="frameShape"><enum>QFrame::Shape::NoFrame</enum></property>
           <property name="minimumSize"><size><width>0</width><height>1</height></size></property>
           <property name="maximumSize"><size><width>16777215</width><height>1</height></size></property>
         </widget></item>

         <item><widget class="QLabel" name="lblSectionTitle">
           <property name="text"><string>数据概况</string></property>
         </widget></item>

         <item>
          <layout class="QHBoxLayout" name="kvRow1">
           <item><widget class="QLabel" name="lblCadPointsKey">
             <property name="text"><string>CAD 采样点</string></property>
             <property name="role" stdset="0"><string>metricKey</string></property>
           </widget></item>
           <item><spacer name="kv1">
             <property name="orientation">
              <enum>Qt::Orientation::Horizontal</enum></property>
             <property name="sizeHint" stdset="0">
              <size><width>20</width><height>20</height></size></property>
           </spacer></item>
           <item><widget class="QLabel" name="lblCadPoints">
             <property name="text"><string>0</string></property>
             <property name="role" stdset="0"><string>metricValue</string></property>
           </widget></item>
          </layout>
         </item>

         <item>
          <layout class="QHBoxLayout" name="kvRow2">
           <item><widget class="QLabel" name="lblCloudPointsKey">
             <property name="text"><string>点云点数</string></property>
             <property name="role" stdset="0"><string>metricKey</string></property>
           </widget></item>
           <item><spacer name="kv2">
             <property name="orientation">
              <enum>Qt::Orientation::Horizontal</enum></property>
             <property name="sizeHint" stdset="0">
              <size><width>20</width><height>20</height></size></property>
           </spacer></item>
           <item><widget class="QLabel" name="lblCloudPoints">
             <property name="text"><string>0</string></property>
             <property name="role" stdset="0"><string>metricValue</string></property>
           </widget></item>
          </layout>
         </item>

         <item>
          <layout class="QHBoxLayout" name="kvRow3">
           <item><widget class="QLabel" name="lblSeamCountKey">
             <property name="text"><string>焊缝数量</string></property>
             <property name="role" stdset="0"><string>metricKey</string></property>
           </widget></item>
           <item><spacer name="kv3">
             <property name="orientation">
              <enum>Qt::Orientation::Horizontal</enum></property>
             <property name="sizeHint" stdset="0">
              <size><width>20</width><height>20</height></size></property>
           </spacer></item>
           <item><widget class="QLabel" name="lblSeamCount">
             <property name="text"><string>0</string></property>
             <property name="role" stdset="0"><string>metricValue</string></property>
           </widget></item>
          </layout>
         </item>

         <item>
          <layout class="QHBoxLayout" name="kvRow4">
           <item><widget class="QLabel" name="lblMappedCountKey">
             <property name="text"><string>已映射</string></property>
             <property name="role" stdset="0"><string>metricKey</string></property>
           </widget></item>
           <item><spacer name="kv4">
             <property name="orientation">
              <enum>Qt::Orientation::Horizontal</enum></property>
             <property name="sizeHint" stdset="0">
              <size><width>20</width><height>20</height></size></property>
           </spacer></item>
           <item><widget class="QLabel" name="lblMappedCount">
             <property name="text"><string>0</string></property>
             <property name="role" stdset="0"><string>metricValue</string></property>
           </widget></item>
          </layout>
         </item>

         <item><spacer name="opsBottomSpacer">
           <property name="orientation">
            <enum>Qt::Orientation::Vertical</enum></property>
           <property name="sizeHint" stdset="0">
            <size><width>20</width><height>20</height></size></property>
         </spacer></item>
        </layout>
       </widget>
      </widget>

      <!-- ---------- 底部日志 ---------- -->
      <widget class="QTabWidget" name="logTabs">
       <property name="currentIndex"><number>0</number></property>
       <property name="documentMode"><bool>true</bool></property>
       <widget class="QWidget" name="tabLog">
        <attribute name="title"><string>日志</string></attribute>
        <layout class="QVBoxLayout" name="tabLogLayout">
         <property name="spacing"><number>0</number></property>
         <property name="leftMargin"><number>8</number></property>
         <property name="topMargin"><number>8</number></property>
         <property name="rightMargin"><number>8</number></property>
         <property name="bottomMargin"><number>8</number></property>
         <item><widget class="QTextEdit" name="textLog">
           <property name="readOnly"><bool>true</bool></property>
           <property name="acceptRichText"><bool>true</bool></property>
         </widget></item>
        </layout>
       </widget>
       <widget class="QWidget" name="tabAlarm">
        <attribute name="title"><string>报警</string></attribute>
        <layout class="QVBoxLayout" name="tabAlarmLayout">
         <property name="spacing"><number>0</number></property>
         <property name="leftMargin"><number>8</number></property>
         <property name="topMargin"><number>8</number></property>
         <property name="rightMargin"><number>8</number></property>
         <property name="bottomMargin"><number>8</number></property>
         <item><widget class="QListWidget" name="listAlarms"/></item>
        </layout>
       </widget>
      </widget>
     </widget>
    </item>
   </layout>
  </widget>

  <widget class="QMenuBar" name="menuBar">
   <property name="geometry">
    <rect><x>0</x><y>0</y><width>1600</width><height>24</height></rect>
   </property>
  </widget>
  <widget class="QStatusBar" name="statusBar"/>
 </widget>
 <layoutdefault spacing="6" margin="0"/>
 <resources>
  <include location="dianyun.qrc"/>
 </resources>
 <customwidgets>
  <customwidget>
   <class>QRoundLed</class>
   <extends>QWidget</extends>
   <header>QRoundLed.h</header>
  </customwidget>
 </customwidgets>
 <connections/>
</ui>
```

### 5.4 设计稿到实现的偏差说明

| 设计稿 | 实现 | 偏差原因 |
|---|---|---|
| 视口浮动标题标签 | QGroupBox 标题 + QSS | `widget` 带 `WA_PaintOnScreen`，叠加兄弟控件有重绘风险 |
| `QToolBar` | `QWidget#topBar` + `QToolButton` | 保留 `on_btnX_clicked` 自动连接，避免改 9 个槽名 |
| 视口内 FPS 读数 | 仅状态栏 | 源码中不存在 FPS 实现（见 §10 待确认 ②） |

---

## 6. `dianyun.h` —— 修改

### 6.1 头文件包含

```cpp
// 原第 20 行
#include <QListWidget>
// 改为
#include <QTreeWidget>
```

### 6.2 槽签名

```cpp
// 原第 61-62 行
void on_listWeldSeams_currentItemChanged(
    QListWidgetItem* current, QListWidgetItem* previous);
// 改为
void on_listWeldSeams_currentItemChanged(
    QTreeWidgetItem* current, QTreeWidgetItem* previous);
```

### 6.3 新增类型与声明

```cpp
private:
    enum State { STATE_IDLE, STATE_PICK_FACE_A, STATE_PICK_FACE_B };

    // —— 新增 ——
public:
    enum class LogLevel { Normal, Success, Warning, Error, Info };
    enum class AlarmLevel { Info, Warning, Error };

private slots:
    void on_btnAckAlarm_clicked();   // 自动连接
    void on_btnFitView_clicked();    // 自动连接

private:
    // —— 新增方法 ——
    void setupThemeAssets();          // 图标、动态属性初始化
    void setupSplitters();            // 初始比例 + stretch factor
    void setupMenuAndShortcuts();     // 菜单与快捷键
    void initStatusBar();             // 状态栏常驻项

    void setStatusText(const QString& text, AlarmLevel tone = AlarmLevel::Info);
    void showAlarm(AlarmLevel level, const QString& message);
    void clearAlarms();
    void refreshAlarmBanner();

    void showMatchCard(const MatchResult& result);
    void resetMatchCard();
    void refreshDataSummary();

    void appendLog(const QString& message);                    // 保留原签名
    void appendLog(const QString& message, LogLevel level);     // 新增重载

    // —— 新增成员 ——
    QLabel* m_statusStage = nullptr;
    QLabel* m_statusCad = nullptr;
    QLabel* m_statusCloud = nullptr;
    QLabel* m_statusMatch = nullptr;
    QLabel* m_statusSeam = nullptr;

    int m_alarmCount = 0;
    AlarmLevel m_worstAlarmLevel = AlarmLevel::Info;
    QString m_lastAlarmText;
    QSet<QString> m_seenAlarms;   // 去重，避免同一错误刷屏
```

需要新增头文件包含：`#include <QSet>`、`#include <QLabel>`（`ui_dianyun.h` 已包含，但显式包含更安全）。

---

## 7. `dianyun.cpp` —— 修改

### 7.1 构造函数：新增初始化序列

在 `ui->setupUi(this)` 之后、`initializeViewers()` 之前插入：

```cpp
setupThemeAssets();      // 图标 + 动态属性，必须在 setupUi 之后
setupSplitters();
setupMenuAndShortcuts();
initStatusBar();

ui->warningBanner->setVisible(false);
ui->matchResultCard->setVisible(true);   // 卡片常驻，内容为 “—”
ui->lblSeamEmpty->setVisible(true);
```

在 `initializeViewers()` 之后插入：

```cpp
connect(ui->btnAckAlarm, &QToolButton::clicked, this, &dianyun::clearAlarms);
connect(ui->btnFitView, &QToolButton::clicked, this, &dianyun::on_btnFitView_clicked);
```

**删除**原第 73-75 行：

```cpp
if (ui->groupBoxLog) {
    ui->groupBoxLog->setTitle(QString::fromUtf8("操作日志"));
}
```

该 objectName 在新布局中不存在，删掉后 `groupBoxLog` 的 2 处引用归零。

### 7.2 视口背景色对齐主题

| 行 | 原值 | 改为 |
|---|---|---|
| `dianyun.cpp:103` | `setBackgroundColor(0.2, 0.2, 0.2)` | `setBackgroundColor(0.09, 0.09, 0.09)` |
| `dianyun.cpp:123` | `setBackgroundColor(0.1, 0.1, 0.1)` | `setBackgroundColor(0.09, 0.09, 0.09)` |

### 7.3 新增方法实现

```cpp
void dianyun::setupThemeAssets()
{
    struct IconSpec { QToolButton* button; const char* path; };
    const IconSpec icons[] = {
        { ui->btnOpen,       ":/dianyun/icons/import-cad.svg"  },
        { ui->btnOpen_2,     ":/dianyun/icons/import-cloud.svg"},
        { ui->btnOpen_3,     ":/dianyun/icons/match.svg"       },
        { ui->btnOpenCamera, ":/dianyun/icons/camera.svg"      },
        { ui->btnRobotWeld,  ":/dianyun/icons/robot-arm.svg"   },
        { ui->btnFitView,    ":/dianyun/icons/fit-view.svg"    },
    };
    for (const auto& spec : icons) {
        if (!spec.button) continue;
        const QIcon icon(QString::fromLatin1(spec.path));
        if (icon.isNull() || icon.availableSizes().isEmpty()) {
            // SVG 插件缺失时降级为纯文字，不留空白图片位
            spec.button->setToolButtonStyle(Qt::ToolButtonTextOnly);
            spec.button->setMinimumWidth(88);
        }
        else {
            spec.button->setIcon(icon);
        }
    }

    ui->lblSeamEmpty->setVisible(false);
    ui->lblStatusDot->setText(QString::fromUtf8("●"));
}

void dianyun::setupSplitters()
{
    ui->workspaceSplitter->setStretchFactor(0, 0);
    ui->workspaceSplitter->setStretchFactor(1, 1);
    ui->workspaceSplitter->setStretchFactor(2, 0);
    ui->workspaceSplitter->setSizes({ 320, 980, 300 });

    ui->viewportSplitter->setStretchFactor(0, 1);
    ui->viewportSplitter->setStretchFactor(1, 1);
    ui->viewportSplitter->setSizes({ 500, 500 });

    ui->mainSplitter->setStretchFactor(0, 1);
    ui->mainSplitter->setStretchFactor(1, 0);
    ui->mainSplitter->setSizes({ 700, 180 });

    QHeaderView* header = ui->listWeldSeams->header();
    header->setSectionResizeMode(0, QHeaderView::Stretch);
    header->setSectionResizeMode(1, QHeaderView::Fixed);
    header->setSectionResizeMode(2, QHeaderView::Fixed);
    header->setSectionResizeMode(3, QHeaderView::Fixed);
    ui->listWeldSeams->setColumnWidth(1, 68);
    ui->listWeldSeams->setColumnWidth(2, 62);
    ui->listWeldSeams->setColumnWidth(3, 52);
    ui->listWeldSeams->setHeaderHidden(false);
}

void dianyun::setupMenuAndShortcuts()
{
    QMenu* fileMenu = ui->menuBar->addMenu(QString::fromUtf8("文件"));
    fileMenu->addAction(QString::fromUtf8("导入三维模型"),
        this, &dianyun::on_btnOpen_clicked, QKeySequence::Open);
    fileMenu->addAction(QString::fromUtf8("导入点云模型"),
        this, &dianyun::on_btnOpen_2_clicked,
        QKeySequence(QStringLiteral("Ctrl+Shift+O")));
    fileMenu->addAction(QString::fromUtf8("模型匹配"),
        this, &dianyun::on_btnOpen_3_clicked,
        QKeySequence(QStringLiteral("Ctrl+M")));
    fileMenu->addSeparator();
    fileMenu->addAction(QString::fromUtf8("退出"),
        this, &QWidget::close, QKeySequence::Quit);

    QMenu* viewMenu = ui->menuBar->addMenu(QString::fromUtf8("视图"));
    viewMenu->addAction(QString::fromUtf8("视图复位"),
        this, &dianyun::on_btnFitView_clicked,
        QKeySequence(Qt::Key_F5));

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
            QString::fromUtf8("dianyun 焊接视觉系统\nQt 6 / PCL / OpenCascade / VTK"));
    });
}
```

```cpp
void dianyun::initStatusBar()
{
    auto makeLabel = [](const QString& role) {
        auto* label = new QLabel;
        label->setProperty("role", role);
        return label;
    };

    m_statusStage = makeLabel("statusItem");
    m_statusCad   = makeLabel("statusValue");
    m_statusCloud = makeLabel("statusValue");
    m_statusMatch = makeLabel("statusValue");
    m_statusSeam  = makeLabel("statusValue");

    ui->statusBar->addWidget(m_statusStage);
    ui->statusBar->addPermanentWidget(makeLabel("statusItem"));
    // 分隔符用小竖线，简化处理：直接拼接文本
    ui->statusBar->showMessage(QString());
    m_statusStage->setText(QString::fromUtf8("● 就绪"));

    auto* right = new QWidget;
    auto* row = new QHBoxLayout(right);
    row->setContentsMargins(0, 0, 8, 0);
    row->setSpacing(16);
    row->addWidget(m_statusCad);
    row->addWidget(m_statusCloud);
    row->addWidget(m_statusMatch);
    row->addWidget(m_statusSeam);
    ui->statusBar->addPermanentWidget(right);

    refreshDataSummary();
}
```

> 状态栏实现细节（分隔符、FPS 项）在编码时按实际观感微调；上表 `addWidget` + `addPermanentWidget` 的组合保证左侧流程阶段、右侧持久指标。

```cpp
void dianyun::setStatusText(const QString& text, AlarmLevel tone)
{
    // 统一入口：左侧状态卡 + 状态栏同步
    if (ui->lblStatus) {
        ui->lblStatus->setText(text);
    }
    if (m_statusStage) {
        const char* prefix = QString::fromUtf8("● ");
        m_statusStage->setText(prefix + text);
    }
    if (ui->lblStatusDot) {
        const char* color =
            tone == AlarmLevel::Error   ? "#E53935" :
            tone == AlarmLevel::Warning ? "#F57C00" : "#2AC44E";
        ui->lblStatusDot->setStyleSheet(
            QStringLiteral("color: %1; font-size: 10pt;")
                .arg(QString::fromLatin1(color)));
    }
}
```

```cpp
void dianyun::showAlarm(AlarmLevel level, const QString& message)
{
    if (!ui->warningBanner) return;

    const QString key = message;
    if (!m_seenAlarms.contains(key)) {
        m_seenAlarms.insert(key);
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
        // 报警页签加计数
        ui->logTabs->setTabText(1, QString::fromUtf8("报警 (%1)").arg(m_alarmCount));
    }

    m_lastAlarmText = message;
    if (level == AlarmLevel::Error ||
        (level == AlarmLevel::Warning && m_worstAlarmLevel != AlarmLevel::Error)) {
        m_worstAlarmLevel = level;
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
    appendLog(QString::fromUtf8("告警已解除。"));
}
```

```cpp
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
    ui->lblVerdictDot->setStyleSheet(QStringLiteral("color: %1; font-size: 9pt;")
        .arg(tone == QStringLiteral("pass")   ? QStringLiteral("#2AC44E") :
             tone == QStringLiteral("review") ? QStringLiteral("#F57C00") :
                                                QStringLiteral("#E53935")));

    ui->lblOverlap->setText(result.success
        ? QString::fromUtf8("覆盖 %1%").arg(result.overlapRatio * 100.0f, 0, 'f', 1)
        : QStringLiteral("—"));
    ui->lblRmse->setText(result.success
        ? QString::number(result.rmse, 'f', 3) : QStringLiteral("—"));
    ui->lblP95->setText(result.success
        ? QString::number(result.p95Residual, 'f', 3) : QStringLiteral("—"));
    ui->lblNormalConsistency->setText(result.success
        ? QString::number(result.normalConsistency * 100.0, 'f', 1) + "%"
        : QStringLiteral("—"));

    if (m_statusMatch) {
        m_statusMatch->setText(QString::fromUtf8("匹配 ") + verdictText);
    }
}

void dianyun::resetMatchCard()
{
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
    if (ui->lblCadPoints) {
        ui->lblCadPoints->setText(QString::number(
            m_cadModel.sampleCloud ? m_cadModel.sampleCloud->size() : 0));
    }
    if (ui->lblCloudPoints) {
        ui->lblCloudPoints->setText(QString::number(
            m_scanCloud ? m_scanCloud->size() : 0));
    }
    if (ui->lblSeamCount) {
        ui->lblSeamCount->setText(QString::number(m_seams.size()));
    }
    if (ui->lblMappedCount) {
        ui->lblMappedCount->setText(QString::number(m_mappedSeams.size()));
    }
    if (m_statusCad) {
        m_statusCad->setText(QString::fromUtf8("CAD %1 点").arg(
            m_cadModel.sampleCloud ? m_cadModel.sampleCloud->size() : 0));
    }
    if (m_statusCloud) {
        m_statusCloud->setText(QString::fromUtf8("点云 %1 点").arg(
            m_scanCloud ? m_scanCloud->size() : 0));
    }
    if (m_statusSeam) {
        m_statusSeam->setText(QString::fromUtf8("焊缝 %1 / 已映射 %2")
            .arg(m_seams.size()).arg(m_mappedSeams.size()));
    }
    if (ui->lblSeamEmpty) {
        ui->lblSeamEmpty->setVisible(m_seams.empty());
    }
}
```

```cpp
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
    ui->textLog->verticalScrollBar()->setValue(
        ui->textLog->verticalScrollBar()->maximum());
}
```

### 7.4 现有调用点替换清单

#### A. `lblStatus`（20 处 → 全部走 `setStatusText`）

| 行（已核对） | 原代码 | 替换为 |
|---|---|---|
| 137-155 | `updateStatusUI()` 写 HTML | 见下方整段替换 |
| 150-153 | `ui->lblStatus->setText(HTML)` | `setStatusText(status)` |
| 390-391 | `setText("当前状态：正在显示完整点云")` | `setStatusText("正在显示完整点云")` |
| 398-399 | `setText("当前状态：点云导入失败")` | `setStatusText("点云导入失败", AlarmLevel::Error)` |
| 405-406 | `setText("当前状态：正在后台加载点云")` | `setStatusText("正在后台加载点云")` |
| 510-513 | `setText("当前状态：正在显示完整点云 %1%")` | `setStatusText(...)` |
| 519-520 | `setText("当前状态：点云就绪")` | `setStatusText("点云就绪")` |
| 592-593 | `setText("当前状态：匹配中")` | `setStatusText("匹配中")` |
| 600-601 | `setText("当前状态：" + stage)` | `setStatusText(stage)` |
| 624-625 | `setText("当前状态：匹配失败")` | `setStatusText("匹配失败", AlarmLevel::Error)` |
| 636 | 三种 verdict | `setStatusText(status, tone)` |

`updateStatusUI()` 整段改为：

```cpp
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
```

#### B. `listWeldSeams`（QListWidget → QTreeWidget，10 处）

| 行 | 原代码 | 替换为 |
|---|---|---|
| 175 | `ui->listWeldSeams->clear()` | 不变（QTreeWidget 也有 `clear()`） |
| 189-193 | `currentItem()->data(Qt::UserRole)` | `currentItem()->data(0, Qt::UserRole)` |
| 319-323 | `new QListWidgetItem(seam.name)` + `addItem` | 见下方 |
| 471-483 | `currentRow()` + `takeItem(row)` | `indexOfTopLevelItem` + `takeTopLevelItem` |
| 488-499 | 槽签名 | 类型改 `QTreeWidgetItem*` |

`createSelectedSeam()` 中的行创建（原 319-323）：

```cpp
if (ui->listWeldSeams) {
    auto* item = new QTreeWidgetItem;
    item->setText(0, seam.name);
    item->setText(1, QString::number(seam.length, 'f', 1));
    item->setText(2, QStringLiteral("A%1·B%2")
        .arg(m_selectedFaceA + 1).arg(m_selectedFaceB + 1));
    item->setText(3, m_mappedSeams.count(seam.id) ? QString::fromUtf8("●")
                                                  : QString::fromUtf8("○"));
    item->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
    item->setTextAlignment(3, Qt::AlignCenter);
    item->setData(0, Qt::UserRole, QString::fromStdString(seam.id));
    item->setForeground(3, QBrush(QColor(
        m_mappedSeams.count(seam.id) ? 0x2A : 0x4A,
        m_mappedSeams.count(seam.id) ? 0xC4 : 0x4A,
        m_mappedSeams.count(seam.id) ? 0x4E : 0x50)));
    ui->listWeldSeams->addTopLevelItem(item);
}
```

`on_btnDeleteSeam_clicked()` 中的删除（原 471-483）：

```cpp
const QTreeWidgetItem* current = ui->listWeldSeams->currentItem();
if (!current) return;
const int row = ui->listWeldSeams->indexOfTopLevelItem(
    const_cast<QTreeWidgetItem*>(current));
// …原有 m_visualization / m_seams / m_mappedSeams 清理逻辑不变…
delete ui->listWeldSeams->takeTopLevelItem(row);
```

`on_listWeldSeams_currentItemChanged` 签名改为 `QTreeWidgetItem*`，函数体不变（仍取 `data(0, Qt::UserRole)`）。

#### C. 映射状态列刷新

`on_btnMapSeam_clicked()` 结尾（原 455-462）追加：

```cpp
// 刷新列表中该行的映射状态列
for (int i = 0; i < ui->listWeldSeams->topLevelItemCount(); ++i) {
    auto* item = ui->listWeldSeams->topLevelItem(i);
    if (item->data(0, Qt::UserRole).toString().toStdString() == seamId) {
        item->setText(3, QString::fromUtf8("●"));
        item->setForeground(3, QBrush(QColor(0x2A, 0xC4, 0x4E)));
        break;
    }
}
refreshDataSummary();
```

#### D. 匹配结果写入卡片

`onMatchFinished()`（原 613-672）在现有日志输出之后追加：

```cpp
showMatchCard(result);

if (!result.success) {
    showAlarm(AlarmLevel::Error,
        QString::fromUtf8("模型匹配失败：") + result.errorMessage);
}
else if (result.verdict == MatchVerdict::Fail) {
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
```

`on_btnOpen_3_clicked()` 开头（原 577 附近）追加：

```cpp
resetMatchCard();
clearAlarms();   // 新一轮匹配，清掉上一轮告警
```

#### E. 数据变化后刷新概况

以下位置各追加一行 `refreshDataSummary();`：

| 位置 | 场景 |
|---|---|
| `on_btnOpen_clicked()` 结尾 | CAD 导入完成 |
| `onPointCloudLoadFinished()` 结尾 | 点云导入完成 |
| `onCameraPointCloudReady()` 结尾 | 相机采集完成 |
| `createSelectedSeam()` 结尾 | 新建焊缝 |
| `on_btnDeleteSeam_clicked()` 结尾 | 删除焊缝 |
| `clearSeams()` 结尾 | 清空焊缝 |

#### F. 失败路径补告警

| 位置 | 追加 |
|---|---|
| `on_btnOpen_clicked()` 第 342-345 行（STEP 失败） | `showAlarm(AlarmLevel::Error, "三维模型导入失败：" + errorMessage)` |
| `onPointCloudLoadFailed()` | 现有 `setStatusText` + `showAlarm(AlarmLevel::Error, ...)` |
| `on_btnMapSeam_clicked()` 第 450-452 行（映射失败） | `showAlarm(AlarmLevel::Error, "焊缝映射失败：" + errorMessage)` |
| `createSelectedSeam()` 第 296-300 / 310-313 行 | `showAlarm(AlarmLevel::Warning, errorMessage)` |

#### G. 日志分级迁移（36 处 `appendLog`，只改语义明确的）

只改语义明确的调用，其余保持默认色（**零破坏性**）：

| 行（已核对） | 现有文本 | 级别 |
|---|---|---|
| 343 | `三维模型导入失败：` | `Error` |
| 351 | `三维模型导入成功` | `Success` |
| 387 | `完整点云后台加载成功` | `Success` |
| 397 | `点云模型导入失败：` | `Error` |
| 451 | `焊缝映射失败：` | `Error` |
| 458 | `%1映射完成` | `Success` |
| 517 | `完整点云显示完成` | `Success` |
| 590 | `模型匹配开始` | `Info` |
| 627 | `匹配失败：` | `Error` |
| 638 | `模型匹配通过/需确认/质量不足` | `Success` / `Warning` / `Error` |
| 665 | `结果未达到质量门槛` | `Error` |

其余 25 处（提示类、引导类）保持 `Normal`。

#### H. 新增两个槽

```cpp
void dianyun::on_btnFitView_clicked()
{
    if (!m_cadViewer || !m_scanViewer) return;
    m_cadViewer->resetCameraClippingRange();
    m_cadViewer->resetCamera();
    m_scanViewer->resetCameraClippingRange();
    m_scanViewer->resetCamera();
    appendLog(QString::fromUtf8("视图已复位。"));
}
```

`on_btnAckAlarm_clicked()` 不需要单独实现——已在构造函数中用 lambda `connect` 到 `clearAlarms()`。若偏好自动连接，可在 `.ui` 中保留 `btnAckAlarm` 并声明该槽，二选一。

> **实施时统一为自动连接**：删除构造函数里的 lambda，声明 `void on_btnAckAlarm_clicked();`，实现直接调用 `clearAlarms()`。这样与项目现有风格（`on_xxx_clicked`）一致。

#### I. `on_btnOpenCamera_clicked` / `on_btnRobotWeld_clicked`

保持现有 LED 演示行为，各追加一行说明日志（见 §10 待确认 ①）：

```cpp
appendLog(QString::fromUtf8(
    "演示模式：相机采集尚未接入，仅触发状态指示灯。"), LogLevel::Warning);
```

---

## 8. 实施顺序（4 个可独立验证的提交）

### P0 — 主题地基（不碰布局）

1. 新建 `resources/theme/dark.qss`、`resources/icons/*.svg`
2. 改写 `dianyun.qrc`
3. 改 `main.cpp` 加 `applyDarkTheme()`
4. 改 `dianyun.cpp:103,123` 视口背景色

**验证**：编译通过；界面整体变深色；原生控件（滚动条、菜单、下拉）正常，无空白方块。
**风险**：低。此步不改 `.ui`，可单独回滚。

### P1 — 布局重构（高风险，先打 tag）

1. 备份：`git tag ui-redesign-before`
2. 重写 `dianyun.ui`
3. 改 `dianyun.h`：`QListWidget` → `QTreeWidget`、槽签名、新增声明
4. 改 `dianyun.cpp`：§7.1 初始化、§7.4-B `listWeldSeams` 全部替换、§7.3 `setupSplitters()`

**验证清单**：
- [ ] 编译通过（`ui_dianyun.h` 重新生成）
- [ ] 全链路：导入 CAD → 导入点云 → 匹配 → 新建焊缝 → 映射
- [ ] 两个 VTK 视口正常渲染，无黑屏/空白
- [ ] 拖拽分栏正常

### P2 — 状态与告警体系

1. `dianyun.h` 新增 `setStatusText` / `showAlarm` / `clearAlarms` / `refreshAlarmBanner` / `showMatchCard` / `resetMatchCard` / `refreshDataSummary`
2. `dianyun.cpp`：§7.4-A 状态统一、§7.4-C/D/E/F 接入点
3. `setupMenuAndShortcuts()` / `initStatusBar()`

**验证清单**：
- [ ] 走完全链路，`lblStatus` 无样式跳变
- [ ] 制造一次匹配失败（导入不匹配的点云），告警条变红、报警页 +1
- [ ] 点击"解除报警"，告警条消失、页签计数归零
- [ ] 状态栏四项数据随操作更新

### P3 — 日志着色与图标

1. `appendLog(msg, LogLevel)` 重载
2. §7.4-G 逐条迁移
3. `setupThemeAssets()` 图标加载 + 降级

**验证清单**：
- [ ] 日志中成功绿、警告橙、错误红
- [ ] 顶栏 6 个图标显示正常（若空白，检查 `qsvg.dll`，同时确认已自动降级为纯文字）

---

## 9. 构建与验证命令

```bash
# 命令行构建（MSBuild）
"C:/Program Files/Microsoft Visual Studio/2022/Community/MSBuild/Current/Bin/MSBuild.exe" \
  "E:/data document/dianyun/dianyun.sln" \
  -p:Configuration=Release -p:Platform=x64 -m -v:minimal

# 测试工程（UI 改动不影响，但作为回归基线）
"E:/data document/dianyun/tests/x64/Release/dianyun.tests.exe"
```

> 若命令行 MSBuild 路径不同，用 VS 打开 `dianyun.sln` 直接按 F7 亦可。**注意**：改 `.ui` 后必须触发 `QtUic` 重新生成；若 VS 未自动重跑，右键项目 → "重新生成"。

---

## 10. 待确认决策

以下 3 项会改变行为，实施前请确认（括号内为我的推荐）：

### ① 演示按钮处理

`btnOpenCamera` / `btnRobotWeld` 目前只触发 LED 闪烁（`dianyun.cpp:531-543`），`m_cameraWorker` 从未实例化。

- **推荐 B**：保持可点击 + 日志输出"演示模式：相机采集尚未接入，仅触发状态指示灯" → 保留现有演示能力，同时不误导
- A：`setEnabled(false)` + tooltip「相机未连接」→ 最诚实，但演示时按钮是灰的
- C：接入 `HikCameraWorker` 真实实现 → 超出 UI 改造范围

### ② FPS 显示

你截图里两个视口左下角有 `333.3 FPS` / `1000.1 FPS`，但 `grep -rn "FPS"` 全库无匹配——**这段代码不在当前分支**。

- **推荐**：本次不做 FPS，状态栏也不放。等该分支代码合并后再接
- 备选：现在就用 `m_renderTimer`（30ms 周期）算一个全局渲染帧率显示在状态栏

### ③ 焊缝"面"列

设计稿的表格第 3 列显示 `A1·B2`（焊缝来自哪两个 CAD 面）。`WeldSeamData` 当前**不记录**源面索引。

- **推荐**：在 `WeldTypes.h` 给 `WeldSeamData` 加两个字段
  ```cpp
  int faceA = -1;
  int faceB = -1;
  ```
  由 `createSelectedSeam()` 填充。改动 3 行，测试工程不受影响（`WeldSeamMapping.cpp` 不读这两个字段）
- 备选：砍掉该列，表格只留 焊缝 / 长度 / 映射

---

## 11. 回滚方案

| 阶段 | 回滚方式 |
|---|---|
| P0 | 还原 `main.cpp`、`dianyun.qrc`、两行背景色；删除 `resources/` |
| P1 | `git reset --hard ui-redesign-before`（tag） |
| P2 / P3 | `git revert` 对应提交 |

`.ui` 重写是唯一不可逐行 review 的改动，因此 P1 必须独立成一次提交，便于整体回退。
