# 导航与路径优化原理说明

## 1. 文档目的

本文档详细说明本仓库中“车辆如何从起点导航到目标点”以及“如何计算障碍物约束下的理论最短路径并评估路径优化度”。

重点回答四个问题：

1. 系统接收到 RViz 目标点以后，整体数据流是什么？
2. 实际导航路径是怎么规划出来的？
3. A* 最短路径是在什么地图上、按什么规则规划出来的？
4. 路径优化度日志和可视化报告是怎么生成的？

## 2. 整体节点与话题关系

### 2.1 主要节点

系统中与导航和路径优化最相关的节点有：

- `vehicleSimulator`
  文件：[vehicleSimulator.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/vehicle_simulator/src/vehicleSimulator.cpp:1)
- `terrainAnalysis`
  文件：[terrainAnalysis.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/terrain_analysis/src/terrainAnalysis.cpp:1)
- `terrainAnalysisExt`
  文件：[terrainAnalysisExt.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/terrain_analysis_ext/src/terrainAnalysisExt.cpp:1)
- `localPlanner`
  文件：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:1)
- `pathFollower`
  文件：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:1)
- `visualizationTools`
  文件：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:1)
- `sensor_scan_generation`
  文件：[sensorScanGeneration.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/sensor_scan_generation/src/sensorScanGeneration.cpp:1)

### 2.2 关键话题

核心话题链如下：

- `/way_point`
  RViz 目标点输入
- `/registered_scan`
  已注册到地图坐标系的激光点云
- `/terrain_map`
  基础地形分析结果
- `/terrain_map_ext`
  扩展地形结果
- `/path`
  实际局部规划器输出路径
- `/cmd_vel`
  路径跟踪器输出的速度指令
- `/state_estimation`
  车辆状态估计
- `/trajectory`
  实际轨迹
- `/shortest_path`
  A* 障碍物约束最短路径
- `/overall_map`
  preview 全局点云地图

## 3. 从目标点到车辆运动的全流程

### 3.1 目标点输入

当用户在 RViz 中使用 `Waypoint` 工具点击地图时，会发布：

```text
/way_point
```

这个话题会被两个节点同时使用：

- `localPlanner`
- `visualizationTools`

对应代码：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:593)
- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:791)

### 3.2 点云与地形输入

导航器并不是在静态图像上规划，而是依赖实时点云和地形图。

来源如下：

1. `vehicleSimulator` 发布 `/registered_scan`
   文件：[vehicleSimulator.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/vehicle_simulator/src/vehicleSimulator.cpp:382)
2. `terrainAnalysis` 从 `/registered_scan` 生成 `/terrain_map`
   文件：[terrainAnalysis.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/terrain_analysis/src/terrainAnalysis.cpp:260)
3. `terrainAnalysisExt` 从 `/registered_scan` 和 `/terrain_map` 生成 `/terrain_map_ext`
   文件：[terrainAnalysisExt.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/terrain_analysis_ext/src/terrainAnalysisExt.cpp:219)

### 3.3 局部路径规划

`localPlanner` 会订阅：

- `/state_estimation`
- `/registered_scan`
- `/terrain_map`
- `/way_point`
- `/navigation_boundary`
- `/added_obstacles`

对应代码：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:587)

它输出：

```text
/path
```

对应代码：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:607)

### 3.4 路径跟踪控制

`pathFollower` 订阅 `/path` 和 `/state_estimation`，把局部路径转换成速度控制命令：

```text
/cmd_vel
```

对应代码：

- 订阅 `/path`：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:239)
- 发布 `/cmd_vel`：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:245)

### 3.5 车辆执行

`vehicleSimulator` 订阅 `/cmd_vel`：

- [vehicleSimulator.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/vehicle_simulator/src/vehicleSimulator.cpp:360)

然后在主循环里更新：

- 车辆姿态
- 车辆位置
- Gazebo 中车体、雷达、相机状态
- `/state_estimation`

因此，实际运动闭环是：

```text
Waypoint -> localPlanner -> /path -> pathFollower -> /cmd_vel -> vehicleSimulator -> /state_estimation
```

## 4. 实际导航路径的规划原理

### 4.1 实际路径不是 A*

这一点非常重要：

- 实际车辆走的路径 `/path` 不是 A* 直接生成的。
- 实际车辆走的路径来自 `localPlanner` 的“候选路径族 + 障碍物筛选 + 分组评分”机制。
- A* 只用于生成“评估参考最短路径” `/shortest_path`。

所以这个系统里有两条路径：

1. 实际执行路径 `/path`
2. 评估参考路径 `/shortest_path`

### 4.2 候选路径族来源

`localPlanner` 在初始化时会读取预生成路径文件：

- `startPaths.ply`
- `paths.ply`
- `pathList.ply`
- `correspondences.txt`

对应函数：

- [readStartPaths()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:349)
- [readPaths()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:383)
- [readPathList()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:424)
- [readCorrespondences()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:462)

其中：

- `pathNum = 343`
- `groupNum = 7`

含义可以理解为：

- 系统预先离线生成了很多局部轨迹模板
- 运行时不从零连续优化整条曲线
- 而是从这些模板里快速筛选一个当前最合适的

### 4.3 点云转到车体局部坐标系

在主循环中，`localPlanner` 先把障碍点云、边界点云、手工障碍物点云都转换到车辆局部坐标系。

对应代码：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:671)

这样做的目的：

- 所有候选路径模板本来就是在车辆局部前方定义的
- 障碍物也必须转换到同一个局部坐标系中，才能判断哪条路径被挡住

### 4.4 如何决定当前目标方向

如果处于自主模式，规划器会根据当前车辆位置和目标点位置计算相对目标方向：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:732)

核心思想：

- 把目标点从世界坐标转换到车体坐标
- 计算目标相对方向 `joyDir`
- 后续路径评分时，更偏向朝向目标方向的路径

### 4.5 候选路径与障碍物的碰撞判断

这一部分是 `localPlanner` 的核心。

系统使用一个离散体素网格来记录“某个空间体素会阻挡哪些路径”：

- 网格参数：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:94)
- 对应表加载：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:462)

工作机制：

1. 将障碍点投影到局部体素网格
2. 查 `correspondences[ind]`
3. 找到会被这个体素挡住的路径编号
4. 给这些路径增加“被阻挡计数”或“代价值”

关键代码：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:781)

这里的设计非常高效，因为它避免了“逐点逐路径逐几何体精确碰撞检测”的昂贵计算。

### 4.6 路径评分与分组选择

对没有被阻挡的路径，系统会计算一个分数：

- 越接近目标方向，分数越高
- 越少碰到高代价地形，分数越高
- 分组得分越高，越容易被选中

对应代码：

- 路径评分：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:823)
- 分组累计得分：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:849)
- 选最大分组：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:855)

### 4.7 输出最终局部路径

选中某个分组后，系统从 `startPaths[groupID]` 中取出一条基础轨迹，再按当前旋转角和尺度进行变换，最终发布为 `/path`。

对应代码：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:870)

如果当前完全找不到可行路径，则发布一个只含原点的退化路径：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:955)

## 5. 路径跟踪器如何把路径变成控制命令

### 5.1 接收路径

`pathFollower` 收到 `/path` 后会：

- 保存这条路径
- 记录当前车体参考位姿
- 把路径点索引重置到起点

对应代码：

- [pathHandler()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:113)

### 5.2 前视点跟踪

`pathFollower` 使用典型的 look-ahead 跟踪方式：

1. 计算车辆相对路径坐标
2. 沿路径向前寻找前视距离 `lookAheadDis`
3. 使用该点计算路径方向
4. 根据方向误差输出角速度

关键代码：

- 寻找前视点：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:271)
- 计算方向误差：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:286)
- 角速度控制：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:307)

### 5.3 速度控制

线速度控制策略包括：

- 根据 `autonomySpeed` 设定目标速度
- 靠近终点时减速
- 方向误差大时减速
- 根据最大加速度平滑增减速
- 必要时完全停车

关键代码：

- 终点减速：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:341)
- 加速度限制：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:352)
- 安全停车：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:362)

最终发布：

```text
/cmd_vel
```

对应代码：

- [pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:369)

## 6. A* 最短路径是怎么计算的

### 6.1 A* 的用途

A* 在本仓库中的作用不是直接开车，而是做“理论参考路径”：

- 用于 RViz 显示 `/shortest_path`
- 用于日志中的 `L_shortest`
- 用于计算路径优化度

代码文件：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:1)

### 6.2 A* 使用什么地图

A* 不直接用实时雷达点云，而是用场景的 preview 地图：

- `mesh/<world_name>/preview/pointcloud.ply`

由 `visualization_tools.launch` 传入：

- [visualization_tools.launch](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/launch/visualization_tools.launch:7)

这样做的优点：

- 起点和终点间的全局信息更完整
- 不依赖局部实时扫描的瞬时覆盖范围
- 适合做“理论最短路径”评估

### 6.3 A* 之前如何建图

#### 6.3.1 计算栅格范围

系统先遍历 `overallMapCloud`，找出：

- `minX`
- `maxX`
- `minY`
- `maxY`
- `minZ`

然后根据 `shortestPathGridResolution` 构建二维栅格。

代码：

- [buildShortestPathGrid()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:236)

#### 6.3.2 计算每个栅格的局部地面高度

对于每个格子，先记录该格子中的最低点 `cellMinZ`：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:274)

这样设计的意义是：

- 不直接用全局统一高度判断障碍物
- 而是按“每个局部格子的地面”来判断
- 对起伏地形更稳健

#### 6.3.3 区分地面和障碍物

对每个点，根据它相对 `cellMinZ` 的高度分两类：

1. 可通行地面：
   `point.z` 落在
   `[localGroundZ + shortestPathGroundMinZ, localGroundZ + shortestPathGroundMaxZ]`
2. 障碍物：
   `point.z` 落在
   `[localGroundZ + shortestPathObstacleMinZ, localGroundZ + shortestPathObstacleMaxZ]`

代码：

- 地面判断：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:296)
- 障碍物判断：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:301)

默认参数来自：

- [visualization_tools.launch](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/launch/visualization_tools.launch:15)

默认值含义：

- `shortestPathGroundMinZ = -0.3`
- `shortestPathGroundMaxZ = 0.3`
- `shortestPathObstacleMinZ = 0.2`
- `shortestPathObstacleMaxZ = 2.0`

即：

- 地面附近 `[-0.3m, +0.3m]` 被看作可支撑可通行区域
- 高于地面 `0.2m ~ 2.0m` 的点被认为是障碍

#### 6.3.4 地面膨胀

为了让地面栅格更连通，系统会对地面做一次膨胀：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:307)

作用：

- 让稀疏点云导致的小空洞不至于把路面割裂

#### 6.3.5 障碍物膨胀

系统还会对障碍物做膨胀：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:331)

默认障碍膨胀半径：

- `shortestPathObstacleInflation = 0.6`

作用：

- 给障碍物增加安全边界
- 防止最短路径贴着墙或树边走

### 6.4 A* 搜索过程

真正的 A* 在：

- [computeObstacleAwareShortestPath()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:370)

#### 6.4.1 起点终点处理

系统先将起点和终点从世界坐标映射到栅格：

- [worldToGrid()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:141)

如果起点或终点落在障碍物上，不会直接失败，而是会在一定半径内搜索最近的自由格：

- [findNearestFreeCell()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:162)

这是一个很实用的鲁棒性处理，避免：

- RViz 点击时刚好点到障碍边缘
- 起点正好落在一个膨胀障碍格内

#### 6.4.2 八邻域搜索

A* 使用八邻域扩展：

- 上下左右
- 四个对角

代码：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:418)

代价：

- 直连步长代价 = `gridResolution`
- 对角步长代价 = `gridResolution * sqrt(2)`

#### 6.4.3 禁止穿墙角

如果尝试走对角线，但横向或纵向相邻格被障碍占据，则这条对角扩展会被禁止：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:441)

这样做的作用：

- 防止路径从两个障碍格对角之间“挤过去”
- 让栅格搜索结果更符合实际车辆宽度要求

#### 6.4.4 启发函数

启发函数使用当前格到目标格的欧氏距离：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:404)

因此这里是标准的基于欧氏启发的 A*。

### 6.5 路径回溯与简化

A* 得到的是一条栅格路径，通常锯齿较多。系统做了两步后处理：

1. 从终点回溯 `cameFrom`
2. 用 line-of-sight 检查做折线简化

对应代码：

- 回溯：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:468)
- 简化：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:485)

line-of-sight 检查函数：

- [hasLineOfSight()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:193)

这个步骤的效果是：

- 去掉不必要的折点
- 使 `/shortest_path` 更平滑
- 让 `L_shortest` 更接近连续空间中的可达最短折线

### 6.6 发布最短路径

最终结果以 `nav_msgs/msg/Path` 发布：

- [publishShortestPath()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:522)

话题：

```text
/shortest_path
```

## 7. 路径优化度日志是怎么生成的

### 7.1 记录起点和终点

每次收到 `/way_point`，`visualizationTools` 会记录：

- 当前车辆位置作为起点
- 点击的 waypoint 作为终点

对应代码：

- [waypointHandler()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:569)

### 7.2 计算最短路径长度

在 `waypointHandler()` 中会调用：

- [computeObstacleAwareShortestPath()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:370)

然后调用：

- [computePolylineLength()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:358)

得到：

```text
shortestPathDis = L_shortest
```

### 7.3 实际路径长度如何累加

每次收到新的 `/state_estimation`，系统会计算当前里程增量 `dis`，并累计：

```text
pathActualDis += dis
```

对应代码：

- [odometryHandler()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:634)

即：

```text
pathActualDis = L_actual
```

### 7.4 优化度公式

更新公式在：

- [updatePathOptimization()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:542)

即：

```text
actualToShortestRatio = 100 * L_actual / L_shortest
pathOptimization = 100 * L_shortest / L_actual
```

### 7.5 写入日志

最终每次 odometry 更新时，会把当前值写入：

```text
path_metrics_<time>.txt
```

对应代码：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:662)

因此一个日志文件里会记录：

- 同一次运行中的多次 waypoint 任务
- 每个 waypoint 任务在时间推进过程中的连续状态

## 8. 离线分析脚本是怎么工作的

离线脚本文件：

- [analyze_path_optimization.py](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/scripts/analyze_path_optimization.py:1)

它的工作过程是：

1. 读取一个或多个 `path_metrics_*.txt`
2. 过滤注释行
3. 解析 11 列数值
4. 根据起点/终点变化和 `actual_path_length` 重置自动分段
5. 对每一段取最后一行作为该段最终结果
6. 生成：
   - `summary.csv`
   - `summary.png`
   - `segment_*.png`
   - `report.html`

分段逻辑在：

- [split_segments()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/scripts/analyze_path_optimization.py:104)

最终指标提取在：

- [final_metrics()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/scripts/analyze_path_optimization.py:130)

## 9. 为什么系统同时保留“实际路径”和“A* 最短路径”

这是整个评估体系的核心设计。

### 9.1 实际路径 `/path`

它代表：

- 真实局部规划器在当前传感器约束、障碍物检测、候选路径模板和控制约束下的输出

优点：

- 实时性高
- 适合在线控制
- 与车辆运动学约束更贴近

### 9.2 A* 最短路径 `/shortest_path`

它代表：

- 在同一场景地图和障碍物假设下，一个更理想、更全局化的可达参考路径

优点：

- 适合做“理论可达最短路径”基准
- 便于定义量化指标

### 9.3 路径优化度的意义

如果只看 `L_actual`，无法判断路径到底是“正常绕障”还是“绕得太冗余”。

如果同时有 `L_shortest`，就能评价：

- 实际规划是否接近理论最优可达解
- 算法是否存在明显绕路

因此：

```text
路径优化度 = L_shortest / L_actual * 100%
```

越高越好；

```text
L_actual / L_shortest * 100%
```

越低越好。

## 10. 结论

本仓库的导航与路径优化体系，本质上由两套互补算法组成：

1. 在线局部规划与跟踪
   对应 `localPlanner + pathFollower`
   负责真正把车开到目标点

2. 离线/在线评估参考最短路径
   对应 `visualizationTools` 中的 A*
   负责给出障碍物约束下的理论最短路径，并计算路径优化度

因此当你在 RViz 中点击一个 waypoint 时，系统实际上会同时做两件事：

- 生成一条可执行的局部导航路径 `/path`
- 生成一条用于评估的理论最短路径 `/shortest_path`

两者再结合实际轨迹 `/trajectory` 和日志 `path_metrics_<time>.txt`，就构成了完整的导航评估闭环。
