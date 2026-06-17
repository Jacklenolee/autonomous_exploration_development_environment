# 自主探索开发环境 - ROS 2 Humble 增强版

本仓库基于原始 `autonomous_exploration_development_environment` 改造，保留了其核心能力：

- Gazebo 仿真
- 地形分析
- 局部路径规划
- 路径跟踪控制
- 传感器数据仿真
- RViz 可视化
- 指标日志记录

在此基础上，当前版本新增了 LRAE 场景、更加真实的 Gazebo 环境设置、RViz 中的障碍物约束最短路径显示、路径优化度日志，以及离线可视化分析脚本。

## 1. 与原始仓库相比新增了什么

### 1.1 新增 LRAE 场景

新增启动命令：

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch
ros2 launch vehicle_simulator system_lrae_scene_2.launch
ros2 launch vehicle_simulator system_lrae_scene_3.launch
ros2 launch vehicle_simulator system_lrae_scene_4.launch
```

新增 world 文件：

```text
src/vehicle_simulator/world/lrae_scene_1.world
src/vehicle_simulator/world/lrae_scene_2.world
src/vehicle_simulator/world/lrae_scene_3.world
src/vehicle_simulator/world/lrae_scene_4.world
```

默认初始位姿：

| 启动文件 | 世界文件 | 初始 `(x, y, terrainZ)` |
| --- | --- | --- |
| `system_lrae_scene_1.launch` | `lrae_scene_1.world` | `(-14, -14, 0.3)` |
| `system_lrae_scene_2.launch` | `lrae_scene_2.world` | `(-27, -27, 0.3)` |
| `system_lrae_scene_3.launch` | `lrae_scene_3.world` | `(-18, -20, 0.0)` |
| `system_lrae_scene_4.launch` | `lrae_scene_4.world` | `(0, 0, 0.0)` |

每个 LRAE 场景都额外生成了 preview 点云：

```text
src/vehicle_simulator/mesh/lrae_scene_1/preview/pointcloud.ply
src/vehicle_simulator/mesh/lrae_scene_2/preview/pointcloud.ply
src/vehicle_simulator/mesh/lrae_scene_3/preview/pointcloud.ply
src/vehicle_simulator/mesh/lrae_scene_4/preview/pointcloud.ply
```

这些 preview 点云会被以下功能使用：

- RViz 中的 `/overall_map`
- A* 障碍物约束最短路径 `/shortest_path`
- 路径优化度指标计算

### 1.2 改进 Gazebo 真实感

多个场景从原始的零重力改为了地球重力：

```xml
<gravity>0 0 -9.81</gravity>
```

更新过的 world 包括：

```text
garage.world
indoor.world
tunnel.world
forest.world
campus.world
lrae_scene_1.world
lrae_scene_2.world
lrae_scene_3.world
lrae_scene_4.world
```

`robot.sdf` 也做了增强：

- 增加非静态模型设置
- 增加更合理的质量和惯量
- 增加车体碰撞几何
- 增加轮子碰撞几何
- 增加摩擦和接触参数

注意：

- 这个仓库里的 `vehicleSimulator` 仍然是通过 Gazebo 的 `/set_entity_state` 更新车体、激光雷达和相机位置。
- 因此它更适合做导航、感知、指标评估和算法验证。
- 它不是完整的轮地动力学仿真平台。

### 1.3 新增 RViz 最短路径显示

原始 RViz 主要显示：

- `/path`：实际局部规划路径
- `/trajectory`：实际行驶轨迹
- `/overall_map`：全局 preview 点云

当前版本新增两个最短路径相关可视化：

```text
/shortest_path
/shortest_path_history
```

RViz 中显示名称为：

```text
CurrentShortestPath
ShortestPath
```

- `CurrentShortestPath` 订阅 `/shortest_path`，显示当前 waypoint 对应的 A* 障碍物约束最短路径。
- `ShortestPath` 订阅 `/shortest_path_history`，显示已经规划过的最短路径历史线段。

这两条都不是简单直线。它们基于 preview 点云构建 2D 占据栅格，在考虑障碍物膨胀、地面可通行区域和动态障碍后，用 A* 算法求出可达最短路径。历史显示使用 `MarkerArray/LINE_STRIP`：每次点击 waypoint 并成功规划后，把这一整段理论最短路径追加到红色历史轨迹；下一次 waypoint 再追加下一整段，最终形成一条连续的理论最短路径轨迹，用来和实际 `Trajectory` 对比。历史线还会按 `shortestPathHistoryZOffset` 轻微抬高，并按 `shortestPathHistoryLineWidth` 加粗，避免被当前最短路径或实际轨迹遮住。RViz 的 `Waypoint` 工具一次点击会连续发布两次 `/way_point`，当前版本会用 `shortestPathWaypointDuplicateTime` 和 `shortestPathWaypointDuplicateDistance` 对重复目标点去重，避免第一个 waypoint 就重复追加两条红色历史线。

为了减少远距离 waypoint 因 preview 地面点云稀疏而不显示的问题，当前版本还增加了 `shortestPathUseRelaxedGroundFallback`。A* 会先使用严格的“地面连通 + 障碍膨胀 + 动态障碍”栅格；如果严格栅格无法连通，再尝试“放宽地面连通、仍保留静态/动态障碍膨胀”的兜底栅格。这样可以让远处目标更容易显示参考最短路径，同时不会把已识别的墙体、树木、木板等障碍当成可通行区域。

### 1.4 新增路径优化度日志

每次点击 RViz 的目标点后，系统会记录：

```text
L_actual
L_shortest
L_actual / L_shortest * 100
L_shortest / L_actual * 100
```

输出文件：

```text
src/vehicle_simulator/log/path_metrics_<time>.txt
```

判定方式：

```text
actual_to_shortest_percent = L_actual / L_shortest * 100 < 120
```

等价形式：

```text
shortest_to_actual_percent = L_shortest / L_actual * 100 > 83.33
```

### 1.5 新增离线路径优化度报告脚本

新增脚本：

```text
src/visualization_tools/scripts/analyze_path_optimization.py
```

它会自动读取 `path_metrics_*.txt`，生成：

- `summary.csv`
- `summary.png`
- `segment_*.png`
- `report.html`

最推荐的分析方式是直接打开 HTML 报告，因为它同时包含：

- 总体统计
- 每段路径的通过/失败判定
- 最终柱状图
- 随时间变化的长度与优化度曲线

## 2. 代码结构

关键功能包如下：

```text
src/vehicle_simulator
src/local_planner
src/terrain_analysis
src/terrain_analysis_ext
src/sensor_scan_generation
src/visualization_tools
```

关键代码文件：

```text
src/vehicle_simulator/src/vehicleSimulator.cpp
src/local_planner/src/localPlanner.cpp
src/local_planner/src/pathFollower.cpp
src/terrain_analysis/src/terrainAnalysis.cpp
src/terrain_analysis_ext/src/terrainAnalysisExt.cpp
src/visualization_tools/src/visualizationTools.cpp
src/visualization_tools/scripts/analyze_path_optimization.py
```

## 3. 依赖环境

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Colcon
- PCL
- OpenCV
- xacro

如果需要重新生成 LRAE preview 点云，还需要：

- `assimp`
- Python `trimesh`

常用依赖安装：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-cv-bridge \
  assimp-utils
```

如果缺少 `trimesh`：

```bash
python3 -m pip install trimesh
```

## 4. 编译

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只改了局部功能，快速重编译可用：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vehicle_simulator visualization_tools local_planner
source install/setup.bash
```

## 5. 启动仿真

### 5.1 原始场景

Garage：

```bash
ros2 launch vehicle_simulator system_garage.launch
```

Indoor：

```bash
ros2 launch vehicle_simulator system_indoor.launch
```

Tunnel：

```bash
ros2 launch vehicle_simulator system_tunnel.launch
```

Forest：

```bash
ros2 launch vehicle_simulator system_forest.launch
```

Campus：

```bash
ros2 launch vehicle_simulator system_campus.launch
```

### 5.2 LRAE 场景

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch
ros2 launch vehicle_simulator system_lrae_scene_2.launch
ros2 launch vehicle_simulator system_lrae_scene_3.launch
ros2 launch vehicle_simulator system_lrae_scene_4.launch
```

关闭 Gazebo GUI：

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch gazebo_gui:=false
```

覆盖初始位姿：

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch \
  vehicleX:=-10.0 vehicleY:=-12.0 terrainZ:=0.3 vehicleYaw:=0.0
```

## 6. RViz 可视化怎么用

系统启动后，RViz 会自动打开，并加载 `vehicle_simulator.rviz`。

常用显示项：

| RViz 显示名称 | 话题 | 含义 |
| --- | --- | --- |
| `OverallMap` | `/overall_map` | 预生成全局点云地图 |
| `RegScan` | `/registered_scan` | 已配准到地图坐标系的扫描点云 |
| `TerrainMap` | `/terrain_map` | 局部地形分析结果 |
| `TerrainMapExt` | `/terrain_map_ext` | 扩展地形图 |
| `Path` | `/path` | 实际局部规划器输出路径 |
| `CurrentShortestPath` | `/shortest_path` | 当前 waypoint 的 A* 障碍物约束最短路径 |
| `ShortestPath` | `/shortest_path_history` | A* 最短路径历史线段 |
| `Trajectory` | `/trajectory` | 车辆实际行驶轨迹 |
| `FreePaths` | `/free_paths` | 当前可行候选路径集合 |
| `Waypoint` | `/way_point` | 目标点工具 |
| `Boundary` | `/navigation_boundary` | 人工导航边界 |
| `AddedObstacles` | `/added_obstacles` | 手工增加障碍物 |

推荐使用流程：

1. 启动某个场景。
2. 等 Gazebo、RViz、点云和路径显示都稳定。
3. 在 RViz 左上角切换到 `Waypoint` 工具。
4. 在地图中点击目标点。
5. 系统会发布 `/way_point`。
6. 实际导航路径会显示在 `/path`。
7. 当前 A* 最短路径会显示在 `CurrentShortestPath`。
8. 已规划过的 A* 最短路径历史会累计显示在 `ShortestPath`。
9. 实际车辆轨迹会显示在 `/trajectory`。
10. 路径优化度日志会写入 `path_metrics_<time>.txt`。

你在 RViz 中观察路径优化时，最关键的可视化对象通常是：

- `Path`：算法真实执行的路径
- `CurrentShortestPath`：当前目标的理论最短可达路径
- `ShortestPath`：历史理论最短可达路径，便于和 `Trajectory` 一起累计观察
- `Trajectory`：车辆真实走过的轨迹
- `OverallMap`：路径参考底图

## 7. 路径优化度日志与可视化分析

### 7.1 日志什么时候新建

当前实现是：

- 每次启动 `visualizationTools` 节点时，会创建一份新的 `path_metrics_<time>.txt`
- 同一次 Gazebo/RViz 运行里，如果你点击多个 waypoint，它们会继续写入同一个日志文件
- 关闭后重新启动，才会创建下一份新的日志

### 7.2 如何分析最新一份日志

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py
```

### 7.3 如何分析指定某一次日志

可以直接指定完整路径：

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py \
  src/vehicle_simulator/log/path_metrics_2026-6-15-23-7-42.txt
```

也可以只写文件名：

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py \
  path_metrics_2026-6-15-23-7-42.txt
```

### 7.4 如何一次分析多份日志

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py \
  path_metrics_2026-6-15-22-56-11.txt \
  path_metrics_2026-6-15-23-1-14.txt
```

### 7.5 如何分析所有日志

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py --all
```

### 7.6 分析输出在哪里

输出目录格式：

```text
src/vehicle_simulator/log/path_optimization_report_<time>/
```

输出内容：

```text
report.html
summary.csv
summary.png
segment_*.png
```

建议直接打开：

```text
report.html
```

它会用图形方式展示：

- 每段路径最终是否满足 `< 120%`
- 每段路径的 `L_actual` 与 `L_shortest`
- 每段路径的优化度曲线
- 总体通过率统计

### 7.7 指标怎么理解

本仓库使用两组等价指标：

```text
actual_to_shortest_percent = L_actual / L_shortest * 100
shortest_to_actual_percent = L_shortest / L_actual * 100
```

判定通过：

```text
actual_to_shortest_percent < 120
```

等价于：

```text
shortest_to_actual_percent > 83.33
```

## 8. 日志文件说明

日志目录：

```text
src/vehicle_simulator/log
```

常见文件：

```text
metrics_<time>.txt
trajectory_<time>.txt
path_metrics_<time>.txt
```

`metrics_<time>.txt` 含义：

```text
explored_volume traveling_distance runtime time_duration
```

`trajectory_<time>.txt` 含义：

```text
x y z roll pitch yaw time_duration
```

`path_metrics_<time>.txt` 含义：

```text
time_duration start_x start_y start_z goal_x goal_y goal_z
actual_path_length shortest_path_length
actual_to_shortest_percent shortest_to_actual_percent
```

## 9. LRAE preview 点云重新生成

如果修改了 LRAE world，可重新生成 preview 点云：

```bash
python3 src/vehicle_simulator/scripts/generate_lrae_preview_maps.py
colcon build --packages-select vehicle_simulator
source install/setup.bash
```

## 10. 常见问题

### 10.1 RViz 中没有 OverallMap

先检查对应场景的 preview 点云是否存在：

```bash
ls src/vehicle_simulator/mesh/<world_name>/preview/pointcloud.ply
```

如果是 LRAE 场景，可以重新生成：

```bash
python3 src/vehicle_simulator/scripts/generate_lrae_preview_maps.py
colcon build --packages-select vehicle_simulator
source install/setup.bash
```

### 10.2 RViz 中没有 CurrentShortestPath 或 ShortestPath

出现条件：

- 已在 RViz 中点击 `Waypoint`
- preview 点云可成功加载
- 起点和终点都在地图范围内
- 起点和终点位于同一个可达区域

如果目标点点得比较远，`CurrentShortestPath` 以前更容易不显示。原因不是 RViz 的显示项坏了，而是 A* 只能在 preview 点云生成的占据栅格上搜索：目标点超出 preview 点云范围、起点和终点之间的地面点云过稀疏、地面膨胀后仍然没有形成连续自由区域、或者障碍物膨胀把窄通道封住时，A* 就会认为“没有从起点到终点的可达连通域”，于是发布空路径。

当前版本已增加远距离 waypoint 兜底策略：

- 第一步：使用严格栅格规划。严格栅格要求地面点云连通，并叠加静态障碍、动态障碍和安全膨胀。
- 第二步：如果动态障碍层导致失败，退回静态 preview 地图再规划一次。
- 第三步：如果严格地面连通仍然失败，并且 `shortestPathUseRelaxedGroundFallback=true`，则使用放宽地面连通的兜底栅格。这个兜底只放宽“未知地面是否连通”，仍然保留障碍物膨胀和动态障碍叠加。
- 第四步：如果兜底成功，RViz 会继续显示 `CurrentShortestPath`，终端会输出 warning 说明使用了 fallback。
- 第五步：如果仍然失败，说明目标可能超出地图范围、附近没有自由格、被障碍完全包围，或者 preview 点云没有覆盖该区域。

如果仍然没有路径，可尝试：

- 换一个更近的目标点
- 调整 `visualization_tools.launch` 里的 A* 栅格参数
- 检查 preview 点云是否覆盖该区域
- 调大 `shortestPathNearestFreeRadius`，让系统能在目标点附近搜索更远的自由格
- 适当减小 `shortestPathObstacleInflation`，但不要小到让最短路径贴墙或穿障

第二次或多次点击 waypoint 后，如果 `Path`、`Trajectory`、`ShortestPath` 历史都还在，但 `CurrentShortestPath` 突然不显示，通常表示当前 A* 在“静态地图 + 实时激光动态障碍”的合并栅格上没有找到可达路径。常见原因是车辆附近实时扫描点、墙边点云或窄通道被 `shortestPathDynamicObstacleInflation` 膨胀后临时堵住了起点、终点或通道。

当前版本已增加动态障碍兜底策略：

- 优先使用静态 preview 地图 + 动态障碍层计算最短路径。
- 如果动态障碍层导致 A* 失败，会自动退回静态 preview 地图再计算一次。
- 如果静态地图仍然因为地面点云稀疏而失败，会继续尝试 `shortestPathUseRelaxedGroundFallback`。
- 如果兜底成功，RViz 会继续显示 `CurrentShortestPath`，终端会输出一条 warning 说明触发了哪一级 fallback。
- 如果所有策略都失败，终端会输出具体失败原因，例如目标超出地图、附近没有自由格、A* 无法连通等。

### 10.3 为什么红色 ShortestPath 以前看起来会穿过障碍物

旧版本把历史最短路径也作为 `nav_msgs/msg/Path` 发布。`Path` 在 RViz 中会把所有相邻 pose 自动连线，因此当系统重新规划、切换 waypoint 或替换当前目标的最短路径时，上一段路径的末端和下一段路径的起点会被 RViz 画成一条不存在的直线。这条直线没有经过 A*，所以可能看起来穿过墙体或障碍物。

当前版本已修改为：

- `CurrentShortestPath`：仍使用 `/shortest_path` 的 `nav_msgs/msg/Path`，只显示当前目标的连续 A* 路径。
- `ShortestPath`：改用 `/shortest_path_history` 的 `visualization_msgs/msg/MarkerArray`，内部使用 `LINE_STRIP` 按 waypoint 任务追加整段最短路径，形成连续红色理论最短路径轨迹。
- 重复 waypoint 去重：`Waypoint` 工具一次点击会发布两次 `/way_point`，`visualizationTools` 会在短时间、近距离内只记录一次红色历史。

因此历史最短路径不会再因为 RViz 自动补线而出现假穿障。如果 `CurrentShortestPath` 本身贴近障碍物，可以继续调大 `shortestPathObstacleInflation` 或 `shortestPathLineCheckRadius`。

### 10.4 为什么日志里一个文件会有多段路径

因为一次运行过程中你可能点击了多个目标点。

当前实现中：

- 一个 `path_metrics_<time>.txt` 对应一次 `visualizationTools` 进程生命周期
- 一个文件里可以包含多段 waypoint 任务
- 离线分析脚本会自动把这些段拆分出来并分别统计

### 10.5 多个仿真实例同时运行怎么办

建议设置不同的 `ROS_DOMAIN_ID`：

```bash
export ROS_DOMAIN_ID=120
```

尽量不要使用大于 `232` 的 ID，否则 DDS 端口映射可能出现问题。

## 11. 算法原理文档

本仓库额外提供了一份中文原理文档，详细说明：

- 车辆如何从目标点导航到终点
- 实际导航路径和 A* 最短路径的区别
- 各节点之间的话题流
- 对应代码文件和关键函数
- A* 的建图、障碍膨胀、搜索和简化过程

文档路径：

```text
docs/navigation_and_path_optimization_zh.md
```

## 12. 原始项目

原始项目主页：

```text
https://www.cmu-exploration.com
```

当前增强版仓库在保留原始自主探索链路的基础上，增加了：

- LRAE 场景适配
- Gazebo 环境增强
- RViz 最短路径显示
- 路径优化度日志
- 路径优化度离线可视化报告
