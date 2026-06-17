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
- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:1035)

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

`localPlanner` 不是每一帧从零开始求一条连续曲线，而是先离线生成一套路径模板，运行时在这些模板中快速筛选。

离线生成脚本是：

- [path_generator.m](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/paths/path_generator.m:1)

运行时读取的文件是：

- `startPaths.ply`
- `paths.ply`
- `pathList.ply`
- `correspondences.txt`

对应读取函数：

- [readStartPaths()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:349)
- [readPaths()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:383)
- [readPathList()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:424)
- [readCorrespondences()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:462)

这四个文件的作用不同：

| 文件 | 主要内容 | 运行时用途 |
| --- | --- | --- |
| `startPaths.ply` | 7 组短起步路径点，每个点带 `group_id` | 最终选中某个组以后，从这里取对应组的点，旋转、缩放、裁剪后发布为 `/path` |
| `paths.ply` | 343 条完整候选路径的采样点，每个点带 `path_id` 和 `group_id` | 离线阶段用于生成 `correspondences.txt`；运行时在宏开关 `PLOTPATHSET == 1` 时读取，用于发布 `/free_paths` 可视化 |
| `pathList.ply` | 每条候选路径的终点、`path_id`、`group_id` | 建立“343 条详细路径 -> 7 个执行组”的映射，并计算每条详细路径的末端方向 |
| `correspondences.txt` | 每个局部体素对应会被该体素阻挡的路径编号列表 | 运行时快速判断某个障碍点会影响哪些候选路径 |

`pathNum = 343` 和 `groupNum = 7` 定义在：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:92)

具体含义是：

- `pathNum = 343`
  表示有 343 条详细候选路径。它们参与障碍物碰撞计数、地形代价计算和方向评分。
- `groupNum = 7`
  表示最终可执行的短起步路径分成 7 个组。343 条详细路径会按 `pathList[pathID]` 投票累加到这 7 个组里，最终 `/path` 输出的是某个 `startPaths[groupID]`。

为什么刚好是 343 和 7：

- `path_generator.m` 里第一段转向 `shift1` 从 `-27` 到 `27`，步长 `9`，一共 7 种，因此形成 7 个 `groupID`。
- 每个 `groupID` 下继续枚举第二段 `shift2` 的 7 种变化和第三段 `shift3` 的 7 种变化。
- 总数就是 `7 * 7 * 7 = 343` 条详细候选路径。

运行时还会把这 343 条路径按 36 个旋转方向复用：

```text
rotDir = 0 ... 35
rotAng = 10 * rotDir - 180 degrees
```

因此实际参与评分的组合是：

```text
36 * 343 = 12348 个“旋转方向 + 详细路径”组合
```

但最终分组得分只有：

```text
36 * 7 = 252 个“旋转方向 + 执行组”组合
```

这样做的好处是：详细路径很多，利于判断局部可行性；最终执行组较少，利于输出稳定、连续、可跟踪的局部路径。

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

局部体素参数是：

```text
gridVoxelSize = 0.02
searchRadius = 0.45
gridVoxelOffsetX = 3.2
gridVoxelOffsetY = 4.5
gridVoxelNumX = 161
gridVoxelNumY = 451
```

离线阶段，`path_generator.m` 会先生成这个二维体素网格，然后用 `rangesearch()` 查找每个体素附近 `searchRadius = 0.45m` 内有哪些路径采样点：

- 体素生成：[path_generator.m](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/paths/path_generator.m:113)
- 路径-体素对应关系生成：[path_generator.m](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/paths/path_generator.m:143)

也就是说，`correspondences.txt` 本质上是一张离线预计算的反向查表：

```text
体素编号 ind -> 经过或靠近这个体素的 pathID 列表
```

运行时工作机制：

1. 将障碍点投影到局部体素网格
2. 查 `correspondences[ind]`
3. 找到会被这个体素挡住的路径编号
4. 给这些路径增加“被阻挡计数”或“代价值”

关键代码：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:781)

这里的设计非常高效，因为它避免了“逐点逐路径逐几何体精确碰撞检测”的昂贵计算。

#### 4.5.1 障碍物是不是按高度判断

是按“相对局部地面的高度差”判断，而不是简单按世界坐标 `z` 判断。

当 `useTerrainAnalysis = true` 时，`localPlanner` 使用 `/terrain_map`。`terrainAnalysis` 会先估计局部地面高度 `planarVoxelElev`，然后把点相对局部地面的高度差写入 `point.intensity`：

- 高度差计算：[terrainAnalysis.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/terrain_analysis/src/terrainAnalysis.cpp:591)
- 写入 `intensity`：[terrainAnalysis.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/terrain_analysis/src/terrainAnalysis.cpp:600)

因此在 `localPlanner` 中：

```text
h = plannerCloudCrop[i].intensity
```

这里的 `h` 不是原始反射强度，也不是世界坐标高度，而是“该点高出局部地面的高度差”。

默认阈值在：

- [local_planner.launch](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch:23)

默认值是：

```text
useTerrainAnalysis = true
obstacleHeightThre = 0.15
groundHeightThre = 0.10
costHeightThre = 0.10
useCost = false
pointPerPathThre = 2
```

判断逻辑在：

- [terrainCloudHandler()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:186)
- [碰撞计数与代价更新](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:793)

规则可以写成：

```text
如果 h > obstacleHeightThre：
    认为这是硬障碍
    clearPathList[rotDir, pathID] += 1

否则如果 h > groundHeightThre：
    认为这是较高但未达到硬障碍的地形代价
    pathPenaltyList[rotDir, pathID] = max(pathPenaltyList[rotDir, pathID], h)
```

需要注意一个参数细节：

- 默认 `useCost = false` 时，`terrainCloudHandler()` 只把 `point.intensity > obstacleHeightThre` 的点送入局部规划器，所以默认更偏向“硬障碍避障”。
- 如果把 `useCost = true`，低于硬障碍阈值但高于地面阈值的点也会进入规划器，`pathPenaltyList` 的软代价作用会更明显。
- 如果 `useTerrainAnalysis = false`，则局部规划器使用原始点云，满足局部高度窗口的点会被直接当作障碍处理。

所以你问的“是不是 `correspondences[ind]` 给对应路径编号增加代价”，答案是：是的，但分两种情况。

- 硬障碍：对 `correspondences[ind]` 中每个 `pathID` 增加阻挡计数 `clearPathList`。
- 软地形代价：对 `correspondences[ind]` 中每个 `pathID` 更新最大地形代价 `pathPenaltyList`。

### 4.6 路径评分与分组选择

对没有被硬障碍阻挡的路径，系统会计算一个分数。

- 越接近目标方向，分数越高
- 越少碰到高代价地形，分数越高
- 分组得分越高，越容易被选中

对应代码：

- 路径评分：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:823)
- 分组累计得分：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:849)
- 选最大分组：[localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:855)

具体流程如下。

第一步，判断路径是否硬阻挡：

```text
如果 clearPathList[rotDir, pathID] < pointPerPathThre：
    这条路径仍被认为可用
否则：
    这条路径被认为被障碍阻挡
```

默认 `pointPerPathThre = 2`，意思是同一条路径至少被 2 个障碍体素命中才会被判定为不可行。这样可以减少单个噪声点导致整条路径失效。

第二步，计算地形代价分数：

```text
penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre
penaltyScore = max(penaltyScore, costScore)
```

含义：

- `pathPenaltyList[i]` 越大，说明这条路径附近越可能有凸起、台阶、低矮障碍等高代价地形。
- `penaltyScore` 越小，这条路径得分越低。
- `costScore` 是下限，避免软代价把得分直接压成负数。

第三步，计算目标方向误差：

```text
dirDiff = abs(joyDir - endDirPathList[pathID] - rotDeg)
```

其中：

- `joyDir` 是当前车辆指向目标点的方向。
- `endDirPathList[pathID]` 是该条详细候选路径终点方向，由 `pathList.ply` 的终点坐标计算得到。
- `rotDeg = 10 * rotDir - 180` 是当前路径模板被旋转到的方向。

`dirDiff` 会被归一化到 `0 ~ 180` 度。越小，说明这条候选路径越朝向目标。

第四步，计算旋转方向权重：

```text
if rotDir < 18:
    rotDirW = abs(abs(rotDir - 9) + 1)
else:
    rotDirW = abs(abs(rotDir - 27) + 1)
```

这个权重是原始局部规划器里的方向偏置项。它会让某些旋转方向更容易累积分数，特别是前进/后退附近的方向比纯侧向方向更强。

第五步，计算详细路径得分：

```text
score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW^4 * penaltyScore
```

代码中只有 `score > 0` 时才会计入分组。

这表示：

- `dirDiff` 越小，方向项越接近 1，分数越高。
- `pathPenaltyList` 越小，`penaltyScore` 越接近 1，分数越高。
- 如果方向差太大，方向项会变成 0 或负数，该路径不会给分组贡献得分。

第六步，把详细路径分数累加到执行组：

```text
groupID = pathList[pathID]
clearPathPerGroupScore[rotDir, groupID] += score
```

这一步非常关键：系统不是直接选 343 条详细路径中的某一条来执行，而是让 343 条详细路径给 7 个组投票。某个组里越多详细路径可行、越朝向目标、地形代价越低，该组的累计分数越高。

最后，系统在 `36 * 7` 个“旋转方向 + 执行组”里选最大得分：

```text
selected = argmax clearPathPerGroupScore[rotDir, groupID]
```

如果开启 `checkRotObstacle`，还会额外检查原地转向时车体附近障碍物是否会碰撞；当前默认 `checkRotObstacle = false`。

### 4.7 输出最终局部路径

选中某个分组后，系统从 `startPaths[groupID]` 中取出一条基础轨迹，再按当前旋转角和尺度进行变换，最终发布为 `/path`。

对应代码：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:870)

输出公式可以理解为：

```text
x_out = pathScale * (cos(rotAng) * x - sin(rotAng) * y)
y_out = pathScale * (sin(rotAng) * x + cos(rotAng) * y)
z_out = pathScale * z
```

并且会按两个条件裁剪：

```text
路径点距离 <= pathRange
路径点距离 <= relativeGoalDis
```

也就是说，如果目标点很近，`/path` 不会继续发布超过目标点的后续模板点。

`/path` 的坐标系是：

```text
frame_id = vehicle
```

所以 `/path` 是车辆局部坐标系下的短期执行路径。它不是从起点到终点的一整条全局路径，而是局部规划器不断重算、不断发布的局部轨迹段。

如果当前完全找不到可行路径，则发布一个只含原点的退化路径：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:955)

在完全失败之前，规划器还会逐步缩小 `pathScale` 和 `pathRange` 后重试：

- [localPlanner.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp:943)

这表示：如果远距离、大尺度模板找不到可行路径，就尝试更短、更保守的局部路径。

## 5. 路径跟踪器如何把路径变成控制命令

### 5.1 接收路径

`pathFollower` 收到 `/path` 后会：

- 保存这条路径
- 记录当前车体参考位姿
- 把路径点索引重置到起点

对应代码：

- [pathHandler()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:113)

这里有一个很重要的设计：

```text
vehicleXRec, vehicleYRec, vehicleYawRec
```

记录的是“收到这条 `/path` 时车辆的位姿”。因为 `/path` 是 `vehicle` 坐标系下发布的局部路径，`pathFollower` 后续要把车辆实时运动量换算回“这条路径刚发布时的局部坐标系”，才能知道车辆相对这条路径走到了哪里。

### 5.2 前视点跟踪

`pathFollower` 使用典型的 look-ahead 跟踪方式：

1. 计算车辆相对路径坐标
2. 沿路径向前寻找前视距离 `lookAheadDis`
3. 使用该点计算路径方向
4. 根据方向误差输出角速度

关键代码：

- 计算车辆相对路径坐标：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:277)
- 寻找前视点：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:288)
- 角速度控制：[pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:329)

具体计算过程如下。

第一步，计算车辆相对路径坐标：

```text
vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec)
            + sin(vehicleYawRec) * (vehicleY - vehicleYRec)

vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec)
            + cos(vehicleYawRec) * (vehicleY - vehicleYRec)
```

对应代码：

- [pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:277)

这一步把当前车辆位置转换到“收到 `/path` 那一刻的车辆局部坐标系”。

第二步，沿路径点向前找前视点：

```text
while pathPointID < pathSize - 1:
    dis = distance(path[pathPointID], vehicleRel)
    if dis < lookAheadDis:
        pathPointID += 1
    else:
        break
```

默认前视距离：

```text
lookAheadDis = 0.5m
```

对应参数：

- [local_planner.launch](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch:61)

含义是：路径点如果已经离车辆太近，就继续向前找，直到找到一个距离车辆至少约 `0.5m` 的目标点。

第三步，计算车辆指向前视点的方向：

```text
pathDir = atan2(disY, disX)
```

对应代码：

- [pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:300)

第四步，计算方向误差：

```text
dirDiff = vehicleYaw - vehicleYawRec - pathDir
```

并归一化到 `[-pi, pi]`。

对应代码：

- [pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:305)

第五步，根据方向误差输出角速度：

```text
if abs(vehicleSpeed) 很小:
    vehicleYawRate = -stopYawRateGain * dirDiff
else:
    vehicleYawRate = -yawRateGain * dirDiff
```

然后限制到：

```text
[-maxYawRate, maxYawRate]
```

对应代码：

- [pathFollower.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp:329)

默认参数：

```text
yawRateGain = 7.5
stopYawRateGain = 7.5
maxYawRate = 90 deg/s
```

因此它的控制思想是：车辆朝向偏离前视点方向越大，输出的角速度越大；接近目标或路径退化时，角速度会被置零或限制。

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

A* 的基础地图来自场景的 preview 点云：

- `mesh/<world_name>/preview/pointcloud.ply`

由 `visualization_tools.launch` 传入：

- [visualization_tools.launch](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/launch/visualization_tools.launch:7)

同时，当前版本会把 `/registered_scan` 中的实时障碍叠加到最短路径占据栅格上：

- 动态障碍更新：[laserCloudHandler()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:873)
- 静态/动态栅格合并：[updateCombinedShortestPathGrid()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:198)

这样做的意义：

- 起点和终点间的全局信息更完整
- preview 地图提供全局可达区域
- 实时扫描补充 preview 地图中漏掉、过稀疏或运行时才出现的障碍物
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

当前默认：

```text
shortestPathGridResolution = 0.2m
```

代码：

- [buildShortestPathGrid()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:346)

#### 6.3.2 计算局部地面高度

系统先记录每个格子的最低点 `cellMinZ`：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:388)

然后在 `shortestPathGroundSearchRadius` 邻域内搜索局部最低地面高度：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:398)

这样设计的意义是：

- 不直接用全局统一高度判断障碍物
- 也不只用“当前格子自己的最低点”判断
- 对起伏地形更稳健
- 对薄墙、木板、立柱这类格子内缺少地面点的障碍更稳健

#### 6.3.3 区分地面和障碍物

对每个点，根据它相对邻域局部地面 `localGroundZ` 的高度分两类：

1. 可通行地面：
   `point.z` 落在
   `[localGroundZ + shortestPathGroundMinZ, localGroundZ + shortestPathGroundMaxZ]`
2. 障碍物：
   `point.z` 落在
   `[localGroundZ + shortestPathObstacleMinZ, localGroundZ + shortestPathObstacleMaxZ]`

代码：

- 地面判断：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:433)
- 障碍物判断：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:439)

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

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:445)

作用：

- 让稀疏点云导致的小空洞不至于把路面割裂

#### 6.3.5 障碍物膨胀

系统还会对障碍物做膨胀：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:457)

默认障碍膨胀半径：

- `shortestPathObstacleInflation = 0.75`

作用：

- 给障碍物增加安全边界
- 防止最短路径贴着墙或树边走

#### 6.3.6 动态障碍叠加

静态 preview 地图只能代表启动时加载的全局参考地图。如果 preview 点云过稀疏，或者 RViz 中能看到的实时扫描障碍没有被 preview 正确表达，最短路径就可能看起来穿过障碍物。

当前版本增加了动态障碍层：

1. 订阅 `/registered_scan`
2. 过滤车辆附近、相对车辆高度在 `[0.2m, 2.0m]` 的点
3. 将这些点投影到最短路径栅格
4. 按 `shortestPathDynamicObstacleInflation = 0.75m` 膨胀
5. 与静态栅格合并为最终 A* 占据栅格

关键代码：

- 动态障碍提取：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:873)
- 动态障碍膨胀：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:895)
- 栅格合并：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:198)

如果动态障碍挡住了当前 `/shortest_path`，系统会按 `shortestPathReplanInterval` 节流自动重算：

- [replanIfDynamicObstaclesBlockShortestPath()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:742)

#### 6.3.7 动态障碍失败兜底

第二次或多次点击 waypoint 时，`CurrentShortestPath` 有时会消失，而 `Path`、`Trajectory` 和 `ShortestPath` 历史仍然正常。这个现象通常不是 RViz 显示插件坏了，而是当前 A* 计算失败了。

原因是第二次点击时车辆已经移动，实时 `/registered_scan` 中会出现新的近距离点云。动态障碍层会把这些点投影到 A* 栅格并按 `shortestPathDynamicObstacleInflation` 膨胀。如果车辆贴近墙体、目标点在墙边、通道较窄，动态膨胀可能会把起点附近、目标附近或中间通道临时堵住。此时合并栅格上不存在从起点到终点的连通自由区域，`/shortest_path` 就会被发布为空路径。

当前版本采用两级计算：

1. 优先使用“静态 preview 地图 + 动态障碍层”的合并栅格计算 A*。
2. 如果合并栅格失败，并且确实存在动态障碍层，则自动退回静态 preview 地图再计算一次。
3. 如果静态地图能找到路，继续发布 `CurrentShortestPath`，并在终端输出 warning，说明动态栅格临时堵住了路径。
4. 如果静态地图也失败，则保留上一条有效最短路径，并输出具体失败原因，例如目标超出地图、起终点附近没有自由格、A* 无法连通等。

这样处理的目的不是忽略障碍物，而是避免瞬时激光噪声或临时膨胀把评估用最短路径完全清空。真正用于评估的 `L_shortest` 仍然来自障碍物约束 A*；当动态层可靠时优先使用动态层，当动态层导致不可达时才退回静态地图基准。

### 6.4 A* 搜索过程

真正的 A* 在：

- [computeObstacleAwareShortestPath()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:486)

#### 6.4.1 起点终点处理

系统先将起点和终点从世界坐标映射到栅格：

- [worldToGrid()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:141)

如果起点或终点落在障碍物上，不会直接失败，而是会在一定半径内搜索最近的自由格：

- [findNearestFreeCell()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:217)

这是一个很实用的鲁棒性处理，避免：

- RViz 点击时刚好点到障碍边缘
- 起点正好落在一个膨胀障碍格内

同时当前版本不再无条件把路径首尾点拉回原始点击坐标。只有当“原始坐标到最近自由格”的连接线也通过安全检查时，才恢复原始坐标：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:629)

这避免了目标点点在障碍边缘时，最后一段红线被强行拉进障碍物。

#### 6.4.2 八邻域搜索

A* 使用八邻域扩展：

- 上下左右
- 四个对角

代码：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:555)

代价：

- 直连步长代价 = `gridResolution`
- 对角步长代价 = `gridResolution * sqrt(2)`

#### 6.4.3 禁止穿墙角

如果尝试走对角线，但横向或纵向相邻格被障碍占据，则这条对角扩展会被禁止：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:562)

这样做的作用：

- 防止路径从两个障碍格对角之间“挤过去”
- 让栅格搜索结果更符合实际车辆宽度要求

#### 6.4.4 启发函数

启发函数使用当前格到目标格的欧氏距离：

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:524)

因此这里是标准的基于欧氏启发的 A*。

### 6.5 路径回溯与简化

A* 得到的是一条栅格路径，通常锯齿较多。系统做了两步后处理：

1. 从终点回溯 `cameFrom`
2. 用带安全半径的 line-of-sight 检查做折线简化

对应代码：

- 回溯：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:588)
- 简化：[visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:602)

line-of-sight 检查函数：

- [hasLineOfSight()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:303)

原先如果只按栅格 Bresenham 直线检查，折线简化有可能把 A* 栅格路径拉成一条穿过障碍边缘的长直线。当前版本改为：

```text
shortestPathLineCheckResolution = 0.05m
shortestPathLineCheckRadius = 0.15m
```

也就是沿候选直线每 5cm 采样一次，并检查采样点周围 15cm 半径内是否全是自由格。

这个步骤的效果是：

- 去掉不必要的折点
- 使 `/shortest_path` 更平滑
- 让 `L_shortest` 更接近连续空间中的可达最短折线
- 避免为了过度平滑而穿过障碍物

### 6.6 发布最短路径

当前最短路径和历史最短路径分开发布：

- [publishShortestPath()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:651)

当前 waypoint 的最短路径：

```text
/shortest_path
```

消息类型为 `nav_msgs/msg/Path`，RViz 中显示名称为 `CurrentShortestPath`。它只表示当前目标点的 A* 结果，因此同一条消息内部可以按顺序连成连续折线。

历史最短路径：

```text
/shortest_path_history
```

消息类型为 `visualization_msgs/msg/MarkerArray`，RViz 中显示名称为 `ShortestPath`。内部使用 `Marker::LINE_LIST`，每两个点组成一条独立线段。这样可以累计显示多次 waypoint 或多次重规划的最短路径历史，同时不会让 RViz 把上一段路径终点和下一段路径起点自动连成一条不存在的直线。

这项修改非常关键：旧版本如果把历史最短路径也作为 `nav_msgs/msg/Path` 发布，RViz 会强制连接所有相邻 pose。重新规划或切换目标后，显示层会出现一条没有经过 A* 检查的假直线，看起来就像“红色最短路径穿过障碍物”。当前版本用 `LINE_LIST` 后，历史显示只画真实 A* 段内的线，不再跨段补线。

## 7. 路径优化度日志是怎么生成的

### 7.1 记录起点和终点

每次收到 `/way_point`，`visualizationTools` 会记录：

- 当前车辆位置作为起点
- 点击的 waypoint 作为终点

对应代码：

- [waypointHandler()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:756)

### 7.2 计算最短路径长度

在 `waypointHandler()` 中会调用：

- [computeObstacleAwareShortestPath()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:486)

然后调用：

- [computePolylineLength()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:474)

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

- [odometryHandler()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:789)

即：

```text
pathActualDis = L_actual
```

### 7.4 优化度公式

更新公式在：

- [updatePathOptimization()](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:675)

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

- [visualizationTools.cpp](/home/gh/Explore_Report/autonomous_exploration_development_environment/src/visualization_tools/src/visualizationTools.cpp:847)

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
