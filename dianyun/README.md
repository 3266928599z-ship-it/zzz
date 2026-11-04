# dianyun

基于 Qt、PCL、OpenCascade 和海康工业相机 SDK 的三维工件处理软件。目前版本重点实现了 STEP CAD 模型与不完整扫描点云之间的全自动刚体匹配。

## 当前功能

- 导入 STEP/STP 三维模型并显示 CAD 网格。
- 对 CAD 三角面进行均匀采样，生成配准使用的 CAD 点云。
- 导入 PLY/PCD 扫描点云并显示。
- PLY/PCD 在后台线程读取和清理，主界面保持响应。
- 扫描点云按每批十万点渐进渲染，最终显示全部有效点，不做显示降采样。
- 连接海康工业相机，接收并显示采集点云。
- 单帧拍摄，以及「相机持续出流 + 机械臂直线扫描」的整幅深度图拼接采集。
- 连接法奥机械臂控制器，监控运行状态、程序状态、急停和故障码，并下发扫描用的直线运动指令。
- 自动从包含底板、夹具和定位环的扫描场景中提取工件主体。
- 自动完成粗配准、精配准、质量评价和结果判定。
- 将对齐后的半透明 CAD 模型叠加到扫描点云中，便于人工复核。
- 支持在 CAD 上选择两个面，通过 OpenCascade 求交生成和管理焊缝线。
- 将选中的 CAD 焊缝按已通过匹配的刚性变换映射到扫描坐标系，并在两个视图同步高亮。
- 使用独立工作线程执行模型匹配，避免长时间计算阻塞主界面。
- 使用输入版本和匹配任务编号拒绝过期回调，防止旧矩阵用于新模型或新点云。
- 对相机句柄初始化、重复采集和回调点云写入进行状态及线程保护。

当前焊缝映射使用模型匹配输出的 `T_final` 对焊缝采样点和管道线段执行刚性变换。它不会把焊缝吸附到最近点，也不会因扫描缺失而改变焊缝长度或连续性；映射精度由模型匹配精度决定。

## 模型匹配流程

1. 对 CAD 和扫描点云执行体素降采样，并根据 CAD 尺寸自动确定体素大小。
2. 对扫描点云执行统计离群点滤波、支撑平面剔除和欧式聚类。支撑平面在近水平候选平面中选取：几乎没有点在它下方（沿 `MatchConfig::scanUpAxis`，海康线激光为 +Z），并且上方有物体；找不到满足条件的平面就不剔除。如果剔除后的主体明显小于 CAD，则回退到不剔除的结果。
3. 估计法向和曲率，提取高显著性关键点并计算 FPFH 描述子。
4. 使用 FPFH、PPF 和 PCA 生成多个只包含旋转和平移的粗配准候选。
5. 根据扫描覆盖、CAD 反向支持、关键特征、空间结构、法向和截断 RMSE 对候选评分。
6. 对每个候选执行 CAD 引导自动裁剪，并以 6.0、3.5、2.5 倍体素距离逐级收紧工件区域。
7. 每次裁剪后执行鲁棒点到点 ICP，并选择综合质量最好的候选。
8. 对最佳工件点云重新提取特征，再执行带 Huber 鲁棒核的点到面 ICP。
9. 如果最终点到面 ICP 使质量下降，自动回退到上一阶段的最佳姿态。
10. 输出 `T_final`，其方向为 `CAD 坐标系 -> 扫描点云坐标系`。

## 结果指标

- 扫描覆盖率：参与匹配的扫描点中，可以在 CAD 附近找到对应点的比例。
- CAD 反向支持率：完整 CAD 表面中，被当前扫描数据观测到的比例；局部扫描时该值低于扫描覆盖率属于正常现象。
- 关键特征覆盖率：孔、圆弧、边缘和台阶等高辨识度结构的对应程度。
- 空间结构覆盖率：匹配是否覆盖整个工件空间，用于排除只对齐局部区域的假解。
- 法向一致性：对应表面的方向一致程度。
- RMSE：有效对应点的均方根距离误差，单位与 CAD 模型单位一致。
- P95 残差：95% 有效对应点不超过的距离误差，用于观察尾部误差。

最终结果分为“匹配通过”和“匹配质量不足”两种状态。只要质量指标达到门槛就采用得分最高的姿态，并允许焊缝映射。如果存在分数接近的其他候选，或点到面精配准没有收敛，只在日志中提示，不再要求人工确认。

## 主要代码

- `dianyun.cpp/.h`：主窗口事件协调、状态和日志更新。
- `AppTypes.h`：CAD模型与通用点云数据结构。
- `StepModelLoader.cpp/.h`：STEP读取、三角化和CAD表面采样。
- `PointCloudLoader.cpp/.h`：PLY/PCD读取与无效点清理。
- `PointCloudLoadController.cpp/.h`：后台点云加载任务、进度和异常处理。
- `WeldSeamService.cpp/.h`：CAD面求交和焊缝离散采样。
- `WeldSeamMapping.cpp`：将焊缝按 CAD 到扫描的刚性矩阵映射到点云坐标系。
- `WeldTypes.h`：焊缝数据与显示线段结构。
- `MatchRequestTracker.h`：跟踪输入版本和匹配任务，拒绝过期异步结果。
- `CameraCaptureState.h`：维护单次采集状态，防止相机任务重复启动。
- `ModelMatchController.cpp/.h`：匹配线程生命周期和只读点云输入传递。
- `ModelMatchWorker.cpp/.h`：完整模型匹配流水线。
- `MatchTypes.h`：匹配参数、结果和质量判定类型。
- `HikCameraWorker.cpp/.h`：海康相机连接、参数下发、ABC16 轮廓解码与逐帧输出。
- `ScanTypes.h`：轮廓帧与拼接结果的数据结构。
- `ScanSession.cpp/.h`：按行序号补齐 Y 坐标，拼接整幅扫描深度图。
- `ScanController.cpp/.h`：扫描流程编排（定位 → 起流 → 运动 → 收尾 → 拼接）。
- `PoseMath.h`：法奥固定轴 XYZ 位姿与齐次矩阵互转。
- `HandEyeConfig.cpp/.h`：读写 `calibration.json`（工具 1 → 传感器外参、轴映射、时间偏置）。
- `WeldPathPlanner.cpp/.h`：基坐标焊缝折线 → TCP 目标序列。
- `WeldPathController.cpp/.h`：逐段下发 `MoveL` 执行轨迹。
- `RobotWorker.cpp/.h`：机械臂 RPC 连接、状态轮询、故障判定与直线运动指令。
- `RobotTypes.h`：机械臂遥测数据结构，不依赖厂商 SDK 头文件。
- `VisualizationManager.cpp/.h`：主窗口内两个三维显示区域的渲染操作。
- `dianyun.ui`：Qt 主界面布局。
- `tests/dianyun.tests.vcxproj`：不依赖相机硬件的焊缝映射、请求版本、扫描拼接（含基坐标输出）、位姿换算与焊缝轨迹规划测试。

## 机械臂模块

机械臂接入基于法奥（FAIRINO）官方 C++ SDK `fairino-cpp-sdk` 的 tag **3.9.3**（`SDK V2.3.3.0-robot v3.9.3`）：读取机器人状态、程序状态、急停与故障码，并为扫描流程下发直线运动指令。

SDK 版本必须与控制器 web 版本对齐。现场控制器为 FR10 V6.0，web 版本 `v3.9.3.1`。该 SDK 只用 20003 XML-RPC 指令 / 20004 状态流 / 8080 命令，不依赖 CNDE 20005 端口。

- 不要换回 v1.x（`cobotAPI.lib`，SDK 1.0.0.0）：它与 3.9.3 固件的 XML-RPC 协议不匹配，`MoveL` 和 `GetInverseKinRef` 的参数个数不同，响应解析会抛出 C++ 异常。也不要换成带 `FRCNDEClient` 的更新版本，那一版依赖 20005。
- 仓库只收录 vs2022 x86-64 的 Release 库，因此 Debug 配置不定义 `DIANYUN_ROBOT_SDK`，`RobotWorker` 编译为空实现，点「连接机械臂」会直接提示「当前构建未启用机械臂 SDK」。
- 厂商头文件只在 `RobotWorker.cpp` 中出现，`RobotWorker.h` 通过 pimpl（`struct Impl`）隔离，避免 `errno_t` / `uint8_t` 的 typedef 冲突污染主窗口代码。
- 状态由 SDK 内部线程从 20004 端口状态流持续刷新，`RobotWorker` 通过 `GetRobotRealTimeState()` 拷贝状态包。这个调用不做 IO，但会检查 socket 状态，同时用作链路探活，链路断开时返回 `ERR_SOCKET_COM_FAILED`。
- SDK 内部不捕获 XML-RPC 解析异常，`GetInverseKinRef`、`MoveL` 等调用都包在 `try/catch` 里，失败时上报错误而不是闪退。
- `RPC()` 建立的是异步连接，IP 不可达时同样返回 0，因此连接成功以「状态流帧序号开始推进」为准，最多等 3 秒。
- 状态轮询为 100 ms 周期，运行在独立线程；连续失败 5 次才判定链路中断，避免网络抖动误报。若 20004 断流超过 1 秒（帧序号停止推进）同样判定故障。
- `linkStateChanged` 只在「链路状态或 detail」真正变化时发出。稳态 `Online` 不携带 detail，避免每轮轮询都把「机械臂连接成功」重复写进日志。
- 指示灯语义：灰=未连接，灰闪=连接中，绿=已连接，绿闪=机器人运动中，红=故障。
- 运动指令通道用**非阻塞**下发（`MoveL` 的 `blendR = 0`），由独立的 50 ms 定时器轮询 `GetRobotMotionDone()` 判定到位。若用阻塞参数，一条 180 mm 的扫描会占满 worker 线程事件循环，`stopMotion()` 无法及时生效。
- 增量运动默认按**工具坐标系**解算：读出当前 TCP 位姿，把工具坐标系下的增量按当前 RPY 旋转到基坐标系再叠加。扫描因此可以表述成「沿工具 X 走 180 mm」，姿态在直线插补过程中保持不变。

## 扫描采集

单帧拍摄之外，主流程是「相机持续出流 + 机械臂直线行走」的拼接采集，由 `ScanController` 编排：

```
Idle → Positioning(可选) → Starting(起流) → Scanning(MoveL) → Stopping(收尾) → Done
```

设备行为决定了实现方式（皆由实机探针确认，型号 MV-DP2240-01H，SDK 1.3.3.3）：

- `ImageMode = 4` 返回 `Profile/ABC16`（3×int16），A=X 编码值、B 恒 0、C=Z 编码值，空点三分量同时为 `-32768`。按 float32 解读会得到彻底的垃圾数据。
- 编码值 × `LSLProfileCoordXUnit/ZUnit`（本机 4.0 µm/count）= µm。回调里的 `fXScale/fYScale/fZScale` 恒为 0，不可用。
- 设备把多条轮廓打包成一批输出，每批行数由设备自行决定（实测 20~50 行浮动）。批次内没有行坐标，Y 只能由上位机按「全局行序号 × 步距」补齐。
- `AcquisitionFrameRate` 控制的是**轮廓行率**，不是深度图帧率。实测设定 600 Hz 得到约 491 行/s，设备上限约 1120 行/s。
- `LSLStepDistance` 为行间步距，本机 96 µm。

因此 `速度(mm/s) = 步距(µm) × 行率(Hz) / 1000` 是硬约束：96 µm × 491 行/s ≈ 47 mm/s。三者不自洽时点云会在 Y 向被整体拉伸或压缩。

行进方向由 `ScanRequest::scanAxis` 选择工具 X / Y / Z，默认工具 X。这只决定机器人怎么走；传感器点云里的行永远是沿相机自身 Y 轴堆叠的，所以拼接仍按行序号补 Y。该选择成立的前提是「传感器 Y 轴 ≈ 所选工具轴」——本机标定的 rx -3.1° / rz 1.6° 是斜装，行进方向与传感器 Y 轴会有约 3° 夹角，扫描越长，末端横向漂移越大（180 mm 行程约 10 mm），需要时用 `reverseY` 之外的方式补偿或在标定里修正。

扫描流程的几个关键取舍：

- 相机先于运动起流、运动到位后再等 300 ms 才停流，保证行程两端都有轮廓数据。
- 首帧只用于确认流已就绪，**不计入拼接**；机器人位移超过 0.5 mm 后才开始记录，避免把启动加速段的原地轮廓写进点云。
- 拼接时按帧序号检测丢批，缺帧处按已收帧的平均行数补空行，保持 Y 尺度正确（缺失区间内是空洞，不会把后续点云前移）。
- 收尾时用实测 TCP 位移复核 Y 尺度：若等效步距（位移 / 行数）与名义步距偏差超过 5%，采用实测值并写日志。原因是法奥 `MoveL` 的 `vel` 是百分比而非 mm/s，现场速度百分比与实际线速度几乎不可能天然匹配，这条日志就是标定 `scanVelPercent` 的依据。
- 扫描期间激光常亮，任何失败路径都会同时 `StopMotion()` 与 `stopCapture()`，不让激光在无人看管的行程上继续点亮。
- 加载到有效手眼标定（`calibration.json`）时，扫描按行时间戳在 TCP 6D 轨迹上插值（平移线性、姿态 slerp），直接输出**机器人基坐标**点云（`CloudFrame::RobotBase`）；此时匹配得到的 `T_final` 为 CAD → 基座。标定无效或轨迹样本不足时回退为传感器坐标点云（`CloudFrame::Sensor`），文件导入与单帧拍摄也属于传感器坐标。

## 焊缝轨迹执行

只有基坐标点云 + 匹配通过 + 已设起焊点 + 已映射的焊缝才能下发运动。菜单「机械臂」下：

- **移到点选位置上方 30 mm**：Shift+单击扫描视图拾取点后使用，用于核对手眼标定。
- **焊缝空走**：整条轨迹抬高 20 mm（尖端距焊缝约 30 mm）。
- **沿焊缝执行**：TCP 落在焊缝上；工具 1 的 Z=260 mm 已含 10 mm 余量，真实尖端距焊缝 10 mm。
- **停止轨迹运动**（Esc）。

轨迹由 `WeldPathPlanner` 生成：接近点（起点上方 30 mm）→ 下降到起点 → 沿焊缝各折点 → 终点上方退出；合并过近点、删去共线点；姿态全程调平（Rx 180 / Ry 0 / Rz 取当前航向，末端偏离竖直超过 30° 时拒绝）。`WeldPathController` 以工具 1、基坐标逐段下发 `MoveL`，每段到位后再发下一段，失败或取消即 `StopMotion()`。焊缝最高点比当前 TCP 高出 100 mm 以上视为标定方向错误并拒绝执行。未做逆解预检，不可达点由控制器在对应段报错停机。

SDK 的头文件与库已纳入版本库，无需额外下载：

```
dianyun/third_party/fairino-3.9.3/include/robot.h
dianyun/third_party/fairino-3.9.3/include/robot_error.h
dianyun/third_party/fairino-3.9.3/include/robot_types.h
dianyun/third_party/fairino-3.9.3/lib/fairino.lib
dianyun/third_party/fairino-3.9.3/lib/fairino.dll
```

`fairino.dll` 是运行时依赖，Release 构建后由 `DeployFairinoRuntime` 目标自动拷贝到 exe 同目录。库放在 `lib/` 下而不是 `lib/Release/`，因为 `.gitignore` 的 `[Rr]elease/` 规则会排除后者。

## 构建环境

- Visual Studio 2022，x64
- Qt 6.10.1
- PCL 1.15
- OpenCascade
- Eigen
- VTK
- 海康 MVS SDK
- 法奥机器人 C++ SDK（仅 Release）

项目文件中包含本机 SDK 路径。迁移到其他计算机时，需要根据实际安装目录调整 Qt、vcpkg、PCL 和 MVS SDK 的包含目录与库目录。
