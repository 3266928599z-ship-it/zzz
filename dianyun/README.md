# dianyun

基于 Qt、PCL、OpenCascade 和海康工业相机 SDK 的三维工件处理软件。目前版本重点实现了 STEP CAD 模型与不完整扫描点云之间的全自动刚体匹配。

## 当前功能

- 导入 STEP/STP 三维模型并显示 CAD 网格。
- 对 CAD 三角面进行均匀采样，生成配准使用的 CAD 点云。
- 导入 PLY/PCD 扫描点云并显示。
- PLY/PCD 在后台线程读取和清理，主界面保持响应。
- 扫描点云按每批十万点渐进渲染，最终显示全部有效点，不做显示降采样。
- 连接海康工业相机，接收并显示采集点云。
- 连接法奥机械臂控制器，只读监控运行状态、程序状态、急停和故障码（不下发运动指令）。
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
2. 对扫描点云执行最大平面剔除、统计离群点滤波和欧式聚类。
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

最终结果分为“匹配通过”“需要人工确认”和“匹配质量不足”三种状态。

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
- `HikCameraWorker.cpp/.h`：海康相机连接、采集和点云输出。
- `RobotWorker.cpp/.h`：机械臂 RPC 连接、状态轮询和故障判定（只读监控）。
- `RobotTypes.h`：机械臂遥测数据结构，不依赖厂商 SDK 头文件。
- `VisualizationManager.cpp/.h`：主窗口内两个三维显示区域的渲染操作。
- `dianyun.ui`：Qt 主界面布局。
- `tests/dianyun.tests.vcxproj`：不依赖相机硬件的焊缝映射和请求版本测试。

## 机械臂模块

机械臂接入基于法奥（FAIRINO）C++ SDK，当前阶段只做**只读链路监控**：读取机器人状态、程序状态、急停与故障码，不下发任何运动指令。

- `fairino` 仅提供 vs2022 x86-64 的 Release 库，因此 Debug 配置不定义 `DIANYUN_ROBOT_SDK`，`RobotWorker` 编译为空实现，点「连接机械臂」会直接提示「当前构建未启用机械臂 SDK」。
- `robot.h` 只在 `RobotWorker.cpp` 中出现，`RobotWorker.h` 通过 pimpl（`struct Impl`）隔离，避免厂商头文件的 `winsock2.h` / `windows.h` 冲突污染主窗口代码。
- 状态轮询为 100 ms 周期的 `GetRobotRealTimeState()` 同步调用，运行在独立线程；连续失败 5 次才判定链路中断，避免网络抖动误报。
- 指示灯语义：灰=未连接，灰闪=连接中，绿=已连接（只读，未标定），绿闪=机器人运动中，红=故障。

`fairino` 的运行库是厂商二进制，不纳入版本库。编译前需要从 `fairino-cpp-sdk` 复制：

```
fairino-cpp-sdk/windows/libfairino/include/*.h          -> dianyun/third_party/fairino/include/
fairino-cpp-sdk/windows/libfairino/lib/vs2022 x86-64/Release/fairino.lib  -> dianyun/third_party/fairino/lib/
fairino-cpp-sdk/windows/libfairino/lib/vs2022 x86-64/Release/fairino.dll  -> dianyun/third_party/fairino/lib/
```

Release 配置的 PostBuildEvent 会自动把 `fairino.dll` 复制到输出目录。

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
