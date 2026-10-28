# 界面重构设计方案（深色工业风）

日期：2026-10-26
范围：`dianyun.ui` 布局重构 + 深色主题 + 状态/告警体系
状态：**设计待评审，尚未实施**

---

## 1. 目标与非目标

### 目标

1. 修正信息层级：让「数据准备 → 算法匹配 → 焊缝操作 → 日志」形成清晰的操作流。
2. 建立统一的深色工业视觉语言（灰底 + 橙色强调），与 PCL/VTK 视口的暗色背景自洽。
3. 补齐三类缺失的状态反馈：全局状态、匹配质量、异常告警。
4. 消除现有样式不一致缺陷（`lblStatus` 19 处 `setText` 中 1 处用 HTML、9 处用裸文本）。

### 非目标

- 不改动 `ModelMatchWorker` / `WeldSeamService` / `VisualizationManager` 的算法与渲染逻辑。
- 不引入 QML 或第三方 UI 库，保持纯 Qt Widgets。
- 不实现相机采集（见 §9.1 遗留问题），本次只做界面表达。
- 不做多语言 / 多主题切换（预留结构，不实现）。

---

## 2. 现状诊断

### 2.1 栅格配比失衡

```
dianyun.ui:17   horizontalLayout_3  stretch="4,1,0"
```

三维视口 : 焊缝列表 : 日志 = **4 : 1 : 0**。日志被压到最窄，而它恰恰是排障时唯一能读到完整错误行的地方（`dianyun.cpp` 中所有失败路径都只写日志）。

### 2.2 状态标签位置错误 + 样式不一致

- `lblStatus` 放在 `groupBox`（焊缝管理器）内部（`dianyun.ui:121`），但它承载的是**全局**状态。
- `updateStatusUI()`（`dianyun.cpp:137-155`）写入 HTML 粗体居中黑字，且硬编码 `color: black`。
- 其余 9 处直接写入纯文本：`390` `396` `405` `511` `519` `592` `598` `624` `631`。
- 后果：运行时状态栏样式在「粗体居中」和「左对齐小字」之间反复跳变；一旦切深色主题，黑字直接不可见。

### 2.3 顶栏语义断裂

6 个裸 `QPushButton` 平铺（`dianyun.ui:23-62`），把三类完全不同的事物放在同一视觉层级：

| 现有按钮 | 实际语义 |
|---|---|
| 导入三维模型 / 导入点云模型 | 数据准备 |
| 模型匹配 | 算法执行 |
| 开始拍摄 / 机械臂焊接 | 硬件动作 |

无分组、无图标、无主次、无危险操作区分。

### 2.4 焊缝列表信息密度过低

`QListWidget`（`dianyun.ui:128`）单列纯文本，只显示 `seam.name`。`WeldSeamData` 里的 `length`、以及 `m_mappedSeams` 中的映射状态都已存在，但界面完全不展示。

### 2.5 无告警通道

工业软件的核心诉求是「异常一定看得见」。当前所有错误只沉进日志流，操作员不主动翻就看不到。

### 2.6 空置框架

`dianyun.ui:191-209` 声明了 `QMenuBar` / `QToolBar` / `QStatusBar`，全部为空 —— 浪费了 Qt 现成的结构能力。

### 2.7 潜在缺陷（顺带记录）

| 位置 | 问题 |
|---|---|
| `dianyun.ui:131-134` | `dragDropMode = InternalMove` 允许拖拽排序，但 `dianyun.cpp` 从不同步回 `m_seams`，拖拽顺序对业务无任何影响，属于误导性交互 |
| `dianyun.ui:123` | HTML 引用字体 `'Inter'`，Windows 默认未安装，实际回落到系统 sans-serif |
| `dianyun.cpp:73-75` | `groupBoxLog` 标题「Logs」在构造函数里运行时改成「操作日志」，应直接写进 `.ui` |
| `dianyun.h:69` / `dianyun.cpp:217` | `m_cameraWorker` **从未实例化**，`isCapturing()` 恒为 `false`；`onCameraPointCloudReady` 也从未 connect。顶栏「开始拍摄」目前只触发 LED 动画 |

---

## 3. 布局架构

### 3.1 目标结构

```
QMainWindow
├─ QMenuBar            文件 / 视图 / 帮助
├─ QToolBar#mainToolBar   高 54px，图标 + 文字
├─ centralWidget
│   └─ QVBoxLayout (0 边距, 0 间距)
│       ├─ QFrame#warningBanner      高 30px，默认 hidden
│       └─ QSplitter(Vertical)       主分栏
│           ├─ QSplitter(Horizontal) 工作区
│           │   ├─ QWidget#seamPanel       左：焊缝列表   min 280 / def 320
│           │   ├─ QSplitter(Horizontal)   中：双视口     stretch=1
│           │   │   ├─ QGroupBox#groupBox_2   三维模型
│           │   │   └─ QGroupBox#groupBox_3   点云模型
│           │   └─ QWidget#opsPanel        右：操作与结果  min 260 / def 300
│           └─ QTabWidget#logTabs          底：日志 / 报警   def 180
└─ QStatusBar          高 24px，常驻指标
```

### 3.2 关键变化

| 变化 | 理由 |
|---|---|
| **日志从右侧窄列 → 底部通栏** | 单点收益最大。日志需要行宽和行高，横向通栏后不再折行 |
| **焊缝列表独立成左栏** | 数据在选择之前，左→中→右符合操作流 |
| **双视口改为 QSplitter** | 当前 CAD/点云固定 1:1，CAD 场景复杂时无法给主视口让位 |
| **新增告警条** | 参考图的核心设计；错误必须脱离日志浮到顶层 |
| **状态归口到状态栏** | `lblStatus` 只保留「当前流程阶段」；持久事实（点数、匹配结论）移到状态栏 |
| **顶栏三分组 + 分隔线** | 用视觉分隔表达「数据 / 算法 / 硬件」三种心智模型 |

### 3.3 三分栏宽度策略

使用 `QSplitter::setStretchFactor` 而非固定像素：

```cpp
workspaceSplitter->setStretchFactor(0, 0);  // seamPanel   不伸缩
workspaceSplitter->setStretchFactor(1, 1);  // viewports   吸收剩余
workspaceSplitter->setStretchFactor(2, 0);  // opsPanel    不伸缩
workspaceSplitter->setSizes({320, 1000, 300});
```

左右两栏用 `setMinimumWidth` 兜底，避免 1280 宽度下视觉区域被挤死。

---

## 4. 设计令牌

### 4.1 颜色

| 令牌 | 值 | 用途 |
|---|---|---|
| `bg-app` | `#1E1E1E` | 窗口底色 |
| `bg-panel` | `#252526` | 面板背景 |
| `bg-elevated` | `#2D2D30` | 工具栏、表头、卡片 |
| `bg-input` | `#1B1B1B` | 输入框、列表、日志底 |
| `bg-viewport` | `#171717` | VTK 视口（代码侧同步为 RGB 0.09） |
| `border` | `#3F3F46` | 常规分隔 |
| `border-strong` | `#52525B` | 聚焦/强调边框 |
| `text-primary` | `#E4E4E7` | 主文本 |
| `text-secondary` | `#A1A1AA` | 次要文本、面板标题 |
| `text-disabled` | `#6B6B73` | 禁用态 |
| `accent` | `#F57C00` | 橙色强调 |
| `accent-hover` | `#FB8C00` | |
| `accent-pressed` | `#E65100` | |
| `ok` | `#2AC44E` | 通过 / LED 点亮 |
| `warn` | `#F57C00` | 警告 |
| `error` | `#E53935` | 错误 |
| `info` | `#29B6F6` | 提示 |

### 4.2 橙色使用纪律

橙色**只用于**以下 5 种情形，不做装饰：

1. 告警条背景
2. 主操作按钮（同一视图内最多 1 个）
3. 当前选中行 / 选中焊缝
4. 键盘焦点环
5. 活动标签页下划线

理由：橙色在参考图里是唯一的暖色锚点，一旦滥用就失去「注意我」的语义。

### 4.3 字体

| 角色 | 字体 | 字号 | 备注 |
|---|---|---|---|
| 基础 | `Microsoft YaHei UI` | 9pt | 移除 HTML 里的 `Inter` |
| 面板标题 | 同上 | 9pt SemiBold | `letter-spacing: 0.5px`，`text-secondary` |
| 数值读数 | `Consolas` | 10pt | 等宽，避免数字跳动 |
| 日志 | `Consolas` | 9pt | |
| 告警条 | `Microsoft YaHei UI` | 9pt Medium | |

### 4.4 尺寸

| 元素 | 值 |
|---|---|
| 窗口默认 / 最小 | 1600×950 / 1280×760 |
| 工具栏高 | 54 |
| 告警条高 | 30 |
| 状态栏高 | 24 |
| 分栏手柄 | 4（hover 时 `accent`，其余 `border`） |
| 视口最小 | 320×240 |
| 工具栏图标 | 22×22 |
| 圆角 | 面板 4px，按钮 4px，徽标 3px |

所有尺寸用**布局 + 点值**表达，`.ui` 里不写死 `geometry` 宽高（当前 `dianyun.ui:8-11` 是 592×420，在高 DPI 下会错位）。

---

## 5. 主题实现策略（分层）

**核心决策：先设暗色 `QPalette` + Fusion 样式，QSS 只做品牌化。**

这是本方案最重要的一条工程约束。若直接用 QSS 全量重绘，会踩中大量坑：`QComboBox` 下拉箭头、`QCheckBox` 勾选框、`QTreeView` 展开三角、滚动条箭头在 QSS 覆盖后都需要额外图片资源，缺一张就变成空白方块。

三层分工：

**第 1 层 — 样式与调色板**（覆盖所有原生绘制）

```cpp
QApplication::setStyle(QStyleFactory::create("Fusion"));

QPalette p;
p.setColor(QPalette::Window,          QColor("#1E1E1E"));
p.setColor(QPalette::WindowText,      QColor("#E4E4E7"));
p.setColor(QPalette::Base,            QColor("#1B1B1B"));
p.setColor(QPalette::AlternateBase,   QColor("#252526"));
p.setColor(QPalette::Text,            QColor("#E4E4E7"));
p.setColor(QPalette::Button,          QColor("#2D2D30"));
p.setColor(QPalette::ButtonText,      QColor("#E4E4E7"));
p.setColor(QPalette::Highlight,       QColor("#F57C00"));
p.setColor(QPalette::HighlightedText, QColor("#FFFFFF"));
p.setColor(QPalette::ToolTipBase,     QColor("#2D2D30"));
p.setColor(QPalette::ToolTipText,     QColor("#E4E4E7"));
p.setColor(QPalette::Disabled, QPalette::Text,       QColor("#6B6B73"));
p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor("#6B6B73"));
qApp->setPalette(p);
```

这一步完成后，勾选框、下拉箭头、滚动条、禁用态**全部自动变暗**，无需任何图片。

**第 2 层 — QSS**（`resources/theme/dark.qss`，仅结构 + 品牌）

只覆盖：工具栏、面板标题、树行、按钮变体、标签页、告警条、分栏手柄、视口边框、状态栏。

**第 3 层 — QProxyStyle（可选）**：若 Fusion 默认间距不满意，再做微调。本次不引入。

### 5.1 QSS 关键片段

按钮变体用**动态属性**而非列举 objectName，减少耦合：

```css
QPushButton[role="primary"] {
    background: #F57C00; color: #FFFFFF; border: none;
    padding: 7px 16px; border-radius: 4px; font-weight: 600;
}
QPushButton[role="primary"]:hover   { background: #FB8C00; }
QPushButton[role="primary"]:pressed { background: #E65100; }
QPushButton[role="primary"]:disabled{ background: #3F3F46; color: #6B6B73; }

QPushButton[role="ghost"] {
    background: transparent; color: #E4E4E7;
    border: 1px solid #3F3F46; padding: 7px 14px; border-radius: 4px;
}
QPushButton[role="ghost"]:hover { border-color: #F57C00; color: #F57C00; }

QPushButton[role="danger"]:hover { border-color: #E53935; color: #E53935; }

QFrame#warningBanner { background: #F57C00; border: none; }
QFrame#warningBanner QLabel { color: #1E1E1E; font-weight: 600; }
QFrame#warningBanner[level="error"] { background: #E53935; }
QFrame#warningBanner[level="error"] QLabel { color: #FFFFFF; }

QToolBar { background: #2D2D30; border: none; border-bottom: 1px solid #3F3F46; spacing: 2px; }
QToolBar QToolButton {
    color: #C8C8C8; padding: 4px 10px; border-radius: 4px; min-width: 64px;
}
QToolBar QToolButton:hover   { background: #3A3A3E; color: #FFFFFF; }
QToolBar QToolButton:pressed { background: #45454A; }
QToolBar QToolButton:disabled{ color: #6B6B73; }
QToolBar::separator { background: #3F3F46; width: 1px; margin: 8px 6px; height: 36px; }

QTreeView {
    background: #1B1B1B; border: 1px solid #3F3F46; border-radius: 4px;
    alternate-background-color: #1F1F22; outline: none;
}
QTreeView::item { padding: 5px 6px; border: none; }
QTreeView::item:hover    { background: #2A2A2E; }
QTreeView::item:selected { background: #F57C00; color: #FFFFFF; }

QTabWidget::pane  { border: none; border-top: 1px solid #3F3F46; }
QTabBar::tab {
    background: transparent; color: #A1A1AA;
    padding: 7px 18px; border: none; border-bottom: 2px solid transparent;
}
QTabBar::tab:selected    { color: #E4E4E7; border-bottom-color: #F57C00; }
QTabBar::tab:hover:!selected { color: #E4E4E7; }

QSplitter::handle { background: #1E1E1E; }
QSplitter::handle:hover { background: #F57C00; }

QGroupBox {
    border: 1px solid #3F3F46; border-radius: 4px;
    margin-top: 22px; padding-top: 6px;
}
QGroupBox::title {
    subcontrol-origin: margin; subcontrol-position: top left;
    left: 10px; padding: 0 5px;
    color: #A1A1AA; font-weight: 600;
}
```

### 5.2 视口颜色同步（必须改代码）

QSS **无法穿透到 VTK 渲染区域**，必须同步修改：

| 位置 | 现值 | 改为 |
|---|---|---|
| `dianyun.cpp:103` | `setBackgroundColor(0.2, 0.2, 0.2)` | `0.09, 0.09, 0.09` (#171717) |
| `dianyun.cpp:123` | `setBackgroundColor(0.1, 0.1, 0.1)` | `0.09, 0.09, 0.09` |

同时建议：`showCadModel` 里 CAD 网格颜色 `0.8,0.8,0.8`（`VisualizationManager.cpp:73`）在更暗的底上偏亮，可降到 `0.72` 以减少眩光；坐标轴可通过 `setCoordinateSystem` 后的 actor 属性调整，但**本次不强制**（会改动渲染逻辑，超出非目标范围）。

---

## 6. 组件规格

### 6.1 工具栏分组

```
[导入三维模型] [导入点云模型] │ [模型匹配] │ [开始拍摄 ●] [机械臂焊接 ●] ⟵spacer⟶ [视图复位]
```

- 样式：`Qt::ToolButtonTextUnderIcon`（参考图同款，图标在上、文字在下）
- 高度 54，按钮 `min-width: 64`
- LED 紧贴对应按钮右侧，作为按钮的**状态附属物**，不加分隔
- 「模型匹配」设为唯一 `role="primary"` 的工具栏按钮？**否** —— 主操作留给右侧操作面板的「焊缝映射」，工具栏一律 ghost 风格，避免两个橙色焦点

**图标清单**（7 个 SVG，`resources/icons/`，24×24，`stroke="#C8C8C8"` `stroke-width="1.75"` `fill="none"`）：

| 文件 | 图形 |
|---|---|
| `import-cad.svg` | 立方体 + 下行箭头 |
| `import-cloud.svg` | 散点云 + 下行箭头 |
| `match.svg` | 两个错位轮廓 + 对齐虚线 |
| `camera.svg` | 相机机身 + 镜头 |
| `robot-arm.svg` | 三段机械臂 |
| `fit-view.svg` | 四角取景框 |
| `alarm-clear.svg` | 铃铛 + 斜杠 |

图标**只用单色**，hover 时靠 QSS 背景变化表达，不改图标颜色 —— 避免为每个状态准备多份资源。

> 图标方案说明：用户选择「先出设计方案」，故此处仅列出清单与规格，图标资源在实际实施阶段生成。

### 6.2 告警条

**结构**：`QFrame#warningBanner` → `QHBoxLayout`：`QLabel#lblAlarmIcon` | `QLabel#lblWarningText`(stretch) | `QToolButton#btnAckAlarm`("解除报警")

**行为**：
- 默认 `setVisible(false)`
- `showAlarm(level, text)`：设置 `level` 动态属性 → `style()->unpolish/polish` 触发重绘 → 显示
- 多条告警时文字显示最新一条，右侧追加计数徽标 `(3)`
- 「解除报警」仅清空**已确认**的告警，不清除报警页记录

**语义分级**：

| 级别 | 背景 | 触发来源（现有代码） |
|---|---|---|
| `warning` | `#F57C00` | 匹配需人工确认（`NeedsReview`）、点云分批显示延迟 |
| `error` | `#E53935` | 匹配失败（`Fail`）、点云导入失败、焊缝映射失败、CAD 导入失败 |

**新增 API**（`dianyun.h`）：

```cpp
enum class AlarmLevel { Info, Warning, Error };
void showAlarm(AlarmLevel level, const QString& message);
void clearAlarms();
int  m_activeAlarmCount = 0;
```

### 6.3 左栏：焊缝列表

**结构**：`QWidget#seamPanel` → `QVBoxLayout`
1. 标题行：`QLabel`"焊缝列表"（面板标题样式） + `QToolButton#btnNewSeam`(+) + `QToolButton#btnDeleteSeam`(×)
2. 分隔线
3. `QTreeWidget#listWeldSeams`（4 列）
4. 状态卡片：`QLabel#lblStatus`

**表格列定义**：

| 列 | 标题 | 宽度 | 内容 |
|---|---|---|---|
| 0 | 焊缝 | stretch | `seam.name` |
| 1 | 长度 | 70 | `QString::number(seam.length,'f',1)` 右对齐 |
| 2 | 面 | 60 | `A{m_selectedFaceA+1}·B{m_selectedFaceB+1}`；需在 `WeldSeamData` 中新增两个字段记录源面 |
| 3 | 映射 | 56 | 依据 `m_mappedSeams.count(seam.id)` 显示 `●` / `○` |

- 列 2 需要在 `WeldSeamData` 增加 `int faceA = -1; int faceB = -1;`，由 `createSelectedSeam()` 填充（`dianyun.cpp:311` 附近）。**若不希望扩展数据结构，可先砍掉此列。**
- 列 3 图标用文本 `●`/`○` 上色（`ok` / `text-disabled`），无需图片
- 开启 `setAlternatingRowColors(true)`、`setRootIsDecorated(false)`、`setUniformRowHeights(true)`
- **移除** `dragEnabled/InternalMove`（见 §2.7）

**空态**：无焊缝时在树区域居中显示 `QLabel`"暂无焊缝，点击 + 新建"（`text-disabled`）。

### 6.4 中区：双视口

- `QSplitter(Horizontal)`，默认 1:1，`handleWidth: 4`
- 每个 `QGroupBox` 的标题改为右上角浮动标签（`QLabel#lblCadTitle` / `lblCloudTitle`），叠在视口左上角，半透明底 `rgba(45,45,48,0.85)`，避免 GroupBox 边框挤压视口
- 视口右下角叠加小型读数：CAD 显示顶点数 / 点云点数；FPS 由状态栏统一承载
- 保留 `widget` / `widget_2` 作为 VTK 挂载点，**objectName 不改**

### 6.5 右栏：操作与结果

`QWidget#opsPanel` → `QVBoxLayout`，三个分区（用 `QFrame` + 上边距分隔，不用 GroupBox，减少边框噪音）：

**分区 A — 焊缝操作**
- `btnMapSeam` 焊缝映射 → `role="primary"`
- `btnCancel` 取消操作 → `role="ghost"`

**分区 B — 匹配结果**（新增，`QFrame#matchResultCard`）
仅在 `onMatchFinished` 成功后显示：

```
┌─ 匹配结论 ──────────────┐
│  ● 通过          覆盖率 87% │
│  RMSE 0.412   P95 0.913    │
└──────────────────────────┘
```

- 结论徽标 `QLabel#lblVerdict`：通过=`ok` / 需确认=`warn` / 不通过=`error`
- `QLabel#lblOverlap` / `#lblRmse` / `#lblP95`：等宽字体右对齐
- 未匹配时整卡显示 `—`，`text-disabled`
- 这些数据当前**只写入日志**（`dianyun.cpp:638-646`），提到卡片后操作员不必翻日志

**分区 C — 数据概况**（新增，可选）
CAD 采样点数 / 点云点数 / 焊缝数，三行 `QFormLayout`，等宽数值。若嫌拥挤可并入状态栏（§6.7）。

### 6.6 底部：日志 / 报警

- `QTabWidget#logTabs`，两页
- 页 1「日志」：现有 `QTextEdit#textLog`，只读、等宽字体、`#1B1B1B` 底
- 页 2「报警」：新增 `QListWidget#listAlarms`，每条含时间 / 级别徽标 / 文本，最多保留 200 条
- 默认高度 180，最小 100
- **日志着色**（新增能力，收益高）：
  - 普通：`#E4E4E7`
  - 失败/错误关键词：`#E53935`
  - 成功/完成：`#2AC44E`
  - 警告：`#F57C00`
  
  实现方式：`appendLog(msg, LogLevel)` 重载，用 `QTextCharFormat` 上色。现有 20 处调用可按语义逐个迁移，未迁移的保持默认色，**无破坏性**。

### 6.7 状态栏

`QStatusBar` 常驻右侧（`addPermanentWidget`）：

```
[● 就绪]  │  CAD 12,345 点  │  点云 486,210 点  │  匹配 通过  │  30 FPS
```

- 最左为流程阶段（对应 `m_state`：就绪 / 选择面A / 选择面B / 匹配中 / 点云显示中）
- 各段之间用 `│` 分隔（`QLabel` 内容），`text-secondary`
- 悬停提示走 `showMessage()` 临时消息，不占用常驻位

**`lblStatus` 的去向**：保留 objectName（19 处引用），但降级为左栏内的「当前步骤提示卡」，文字统一为纯文本 + 前置状态点，不再用 HTML。

### 6.8 菜单栏

| 菜单 | 项 | 复用现有槽 |
|---|---|---|
| 文件 | 导入三维模型 / 导入点云模型 / ─ / 退出 | `on_btnOpen_clicked` 等 |
| 视图 | 显示/隐藏日志面板 (F9) / 视图复位 (F5) / 重置分栏 | 新增小槽 |
| 帮助 | 操作说明 / 关于 | `QMessageBox` |

快捷键：`Ctrl+O` 导入 CAD，`Ctrl+Shift+O` 导入点云，`Ctrl+M` 匹配，`F5` 复位视图。

---

## 7. 代码改动清单

> 本节仅为**范围与风险评估**，未执行任何修改。

### 7.1 必须改动（否则编译不通过或功能失效）

| 文件 | 位置 | 改动 |
|---|---|---|
| `dianyun.ui` | 全文 | 重写布局；**保留全部现有 objectName** |
| `dianyun.h:20` | `#include <QListWidget>` | → `<QTreeWidget>` |
| `dianyun.h:61-62` | 槽签名 | `QListWidgetItem*` → `QTreeWidgetItem*` |
| `dianyun.cpp:189-200` | `selectedSeam()` | `currentItem()->data(Qt::UserRole)` → `data(0, Qt::UserRole)` |
| `dianyun.cpp:319-323` | 新增焊缝行 | `QListWidgetItem` → `QTreeWidgetItem`，改用 `setText(col, …)` |
| `dianyun.cpp:471-483` | 删除焊缝 | `currentRow()` → `indexOfTopLevelItem()`；`takeItem` → `takeTopLevelItem` |
| `dianyun.cpp:488-499` | `currentItemChanged` | 参数类型随 QTreeWidget 变更 |
| `main.cpp` | 全局 | 设 Fusion 样式 + 暗色 QPalette + 加载 `dark.qss` |
| `dianyun.cpp:103,123` | 视口背景 | 与 `bg-viewport` 对齐 |

### 7.2 建议改动（状态统一）

| 位置 | 改动 |
|---|---|
| `dianyun.cpp:137-155` | `updateStatusUI()` 去掉 HTML，改调 `setStatus(Stage)` 统一入口 |
| `dianyun.cpp:390/396/405/511/519/592/598/624/631` | 9 处裸 `setText` 收敛到 `setStatus()` / 状态栏 |
| `dianyun.cpp:73-75` | 删除运行时改标题，标题写入 `.ui` |
| `dianyun.cpp:638-646` | 匹配指标同时写入结果卡片 |

### 7.3 新增

| 文件 | 内容 |
|---|---|
| `resources/theme/dark.qss` | 主题样式 |
| `resources/icons/*.svg` | 7 个工具图标 |
| `dianyun.qrc` | 注册上述资源（当前为空壳） |
| `dianyun.h/.cpp` | `showAlarm()` / `clearAlarms()` / `setStatus()` / `appendLog(msg, level)` |

### 7.4 objectName 兼容性对照

**必须保留（代码引用）**：
`widget`、`widget_2`、`btnOpen`、`btnOpen_2`、`btnOpen_3`、`btnOpenCamera`、`btnRobotWeld`、`btnNewSeam`、`btnMapSeam`、`btnCancel`、`btnDeleteSeam`、`listWeldSeams`、`lblStatus`、`textLog`、`ledCapture`、`ledRobotWeld`、`groupBoxLog`

**说明**：`btnOpen` / `btnOpen_2` / `btnOpen_3` 语义模糊，但改名会中断 `on_<name>_clicked` 自动连接，**本次不改名**，仅在代码中加注释。若后续要改，需同步 `connect` 并移除自动连接依赖。

---

## 8. 分阶段实施

| 阶段 | 内容 | 可独立验证 |
|---|---|---|
| **P0 主题地基** | Fusion + QPalette + `dark.qss` + qrc；不改布局 | 界面整体变深色，原生控件（勾选框/下拉/滚动条）正常 |
| **P1 布局重构** | 重写 `.ui`；日志移底部；双视口 splitter；顶栏分组；`QListWidget`→`QTreeWidget` | 功能全部可用，objectName 无遗漏 |
| **P2 新增组件** | 告警条 + 报警页；匹配结果卡片；状态栏指标；菜单栏 | 制造一次匹配失败，验证告警条与报警页 |
| **P3 细节** | SVG 图标；日志着色；`lblStatus` 统一；空态提示 | 视觉走查 |

每个阶段结束时都应能**编译 + 跑通「导入 CAD → 导入点云 → 匹配 → 新建焊缝 → 映射」完整链路**。P1 是唯一有回归风险的阶段，建议在此之前打一个 git tag。

---

## 9. 风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| QSS 无法穿透 VTK 视口 | 视口与面板底色割裂 | 代码同步改背景色（§5.2）；接受存在 1 级色差 |
| `QListWidget`→`QTreeWidget` 遗漏调用点 | 编译失败或运行时崩溃 | 已穷举 7 处（§7.1）；改后全文搜索 `listWeldSeams` 复查 |
| 高 DPI 下硬编码尺寸错位 | 布局挤压 | 全部用布局 + 点值，`.ui` 不写 `geometry` 宽高 |
| QSS 覆盖原生指示器导致缺图标 | 勾选框/箭头变空白 | 采用 §5 分层策略，不覆盖 primitive 指示器 |
| 深色下 CAD 模型对比度不足 | 焊缝/缺陷看不清 | 视口降为 `#171717` 后，CAD 网格色从 0.8 降至 0.72 |
| 告警条吞掉纵向空间 | 视口高度减少 30px | 默认隐藏，仅在异常时出现 |
| `m_cameraWorker` 未实例化 | “开始拍摄”只有动画，误导操作员 | **建议**：按钮进入 disabled，或文案改为「拍摄（未连接）」，见 §9.1 |
| 主题改动无自动化测试覆盖 | 回归靠肉眼 | 项目已有 `tests/` 工程，可增加一个「启动后 objectName 存在性」冒烟测试 |

### 9.1 需要产品决策的遗留问题

**「开始拍摄」与「机械臂焊接」当前是纯演示按钮**（`dianyun.cpp:531-543` 只启动 LED 闪烁定时器）。三种处理方式：

- **A**（推荐，最诚实）：按钮保持可见，未连接硬件时 `disabled` + tooltip「相机未连接」，LED 常暗
- **B**：保留可点击，但操作后日志明确输出「演示模式：未执行实际采集」
- **C**：接入 `HikCameraWorker` 真实实现（超出本次 UI 范围）

另外，截图中视口左下角的 `333.3 FPS` / `1000.1 FPS` 在现有源码中**找不到对应实现**（`grep -rn "FPS"` 全库无匹配）。如果这部分代码在未提交的分支里，需要在实施前合并；如果只是设计稿占位，建议改为状态栏统一显示。

---

## 10. 验收标准

**布局**
- [ ] 窗口在 1280×760、1600×950、1920×1080 下均无控件重叠或截断
- [ ] 三栏 + 底部日志均可拖动调整，且重启后尺寸不重置（可选 `QSettings` 持久化）
- [ ] 日志区横向可完整显示一条 `[HH:mm:ss]` + 60 字中文消息而不折行

**功能回归**
- [ ] 导入 STEP → 视口显示 CAD
- [ ] 导入 PCD/PLY → 分批显示进度，`lblStatus` 百分比递增
- [ ] 匹配通过 → 结果卡片显示结论与三项指标
- [ ] 匹配失败 → 告警条以 `error` 色出现，报警页新增一条
- [ ] 新建焊缝（Ctrl+点击两个面）→ 表格新增行，长度/映射列正确
- [ ] 焊缝映射 → 对应行映射列变 `●`
- [ ] 删除焊缝 → 表格行与视口管道同时消失

**视觉**
- [ ] 全界面无浅色残留（除视口内 VTK 自绘内容）
- [ ] 状态文字全路径统一：走完上述链路，`lblStatus` 不出现粗细/对齐跳变
- [ ] 橙色元素在同一屏内不超过 3 处
- [ ] 禁用态按钮与正常按钮可明显区分

---

## 附：目标布局示意

```
┌───────────────────────────────────────────────────────────────────┐
│ 文件   视图   帮助                                        ─ □ ×   │
├───────────────────────────────────────────────────────────────────┤
│  ⬛      ⬛       ⬛      │   ⬛    │  ⬛        ●   ⬛        ●   ⬛ │
│ 导入三维 导入点云  模型匹配 │ 开始拍摄 │ 机械臂焊接        视图复位 │
├───────────────────────────────────────────────────────────────────┤
│ ⚠  匹配质量不足：覆盖率 61%，低于门槛 75%            解除报警 (2) │
├──────────────┬──────────────────────────────┬────────────────────┤
│ 焊缝列表  + × │ 三维模型       │ 点云模型    │ 焊缝操作           │
│ ┌──┬──┬──┬──┐│                             │  ┌──────────────┐  │
│ │焊缝│长度│面│映││                             │  │  焊缝映射    │  │
│ ├──┼──┼──┼──┤│     (VTK 视口)              │  └──────────────┘  │
│ │焊缝-1│42.5│A1·B2│●││                             │  ┌──────────────┐  │
│ │焊缝-2│18.0│A3·B4│○││                             │  │  取消操作    │  │
│ └──┴──┴──┴──┘│                             │  └──────────────┘  │
│              ├──────────────────────────────┤  ── 匹配结论 ──    │
│ ● 就绪       │  ⬛                 ⬛        │  ● 需确认  覆盖 61%│
│              │                             │  RMSE 0.83 P95 1.9 │
├──────────────┴─────────────────────────────┴────────────────────┤
│ 日志 │ 报警                                                       │
│ [13:50:11] 模型匹配开始：全场景自动分割与CAD引导裁剪              │
│ [13:50:14] 模型匹配需要人工确认：覆盖率 61.0%，RMSE 0.831         │
├───────────────────────────────────────────────────────────────────┤
│ ● 需确认 │ CAD 12,345 点 │ 点云 486,210 点 │ 匹配 需确认 │ 30 FPS│
└───────────────────────────────────────────────────────────────────┘
```

配套可交互样张：`docs/design/ui-mockup.html`（浏览器打开预览）
