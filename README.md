# Autonomous Exploration Development Environment - Enhanced ROS 2 Humble Version

This repository is based on the original
`autonomous_exploration_development_environment` and keeps its core ground robot
autonomous navigation stack: Gazebo simulation, terrain analysis, local path
planning, waypoint following, sensor simulation, RViz visualization, and metric
logging.

This enhanced version adds LRAE scenes, more realistic Gazebo contact settings,
RViz obstacle-aware shortest-path visualization, and path-planning optimality
metrics.

## What Changed Compared With The Original Repository

### 1. Added LRAE Simulation Scenes

New launch files:

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch
ros2 launch vehicle_simulator system_lrae_scene_2.launch
ros2 launch vehicle_simulator system_lrae_scene_3.launch
ros2 launch vehicle_simulator system_lrae_scene_4.launch
```

New world files:

```text
src/vehicle_simulator/world/lrae_scene_1.world
src/vehicle_simulator/world/lrae_scene_2.world
src/vehicle_simulator/world/lrae_scene_3.world
src/vehicle_simulator/world/lrae_scene_4.world
```

Default LRAE initial poses:

| Launch file | World | Initial `(x, y, terrainZ)` |
| --- | --- | --- |
| `system_lrae_scene_1.launch` | `lrae_scene_1.world` | `(-14, -14, 0.3)` |
| `system_lrae_scene_2.launch` | `lrae_scene_2.world` | `(-27, -27, 0.3)` |
| `system_lrae_scene_3.launch` | `lrae_scene_3.world` | `(-18, -20, 0.0)` |
| `system_lrae_scene_4.launch` | `lrae_scene_4.world` | `(0, 0, 0.0)` |

Each LRAE scene also has a generated preview point cloud:

```text
src/vehicle_simulator/mesh/lrae_scene_1/preview/pointcloud.ply
src/vehicle_simulator/mesh/lrae_scene_2/preview/pointcloud.ply
src/vehicle_simulator/mesh/lrae_scene_3/preview/pointcloud.ply
src/vehicle_simulator/mesh/lrae_scene_4/preview/pointcloud.ply
```

These files are required by `/overall_map` and by the obstacle-aware A*
shortest-path calculation.

### 2. Improved Gazebo Realism

The original simulator worlds used zero gravity in several scenes. This version
sets Earth gravity:

```xml
<gravity>0 0 -9.81</gravity>
```

Updated worlds include:

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

The robot SDF was also improved:

- added non-static robot model setting
- added realistic mass and inertia
- added body collision geometry
- added wheel collision geometry
- added wheel/body friction and contact parameters

Important note: the simulator still drives `robot`, `lidar`, and `camera`
poses through Gazebo entity state updates. It is more realistic than the
original zero-gravity setup, but it is still not a full wheel-joint dynamics
controller.

### 3. More Robust Gazebo Launch Sequence

`vehicle_simulator.launch` now:

- starts `gzserver` and `gzclient` separately
- supports `gui:=true/false`
- waits until Gazebo `/model_states` is ready
- spawns robot, camera, and lidar in order
- starts `vehicleSimulator` only after models are spawned
- spawns models at `terrainZ + vehicleHeight` instead of directly at terrain
  height

This prevents common startup problems where the vehicle appears floating,
spawns too early, or sensor models fail to attach correctly.

### 4. Added Obstacle-Aware Shortest Path In RViz

Original RViz already showed:

- `/path`: planner path
- `/trajectory`: actual traveled trajectory
- `/overall_map`: global preview map

This version adds:

```text
/shortest_path
```

RViz display name:

```text
ShortestPath
```

It is drawn as an orange path in RViz.

The shortest path is not a simple straight line. It is computed by A* on a
2D occupancy grid built from the preview point cloud:

- local ground cells define traversable support
- obstacle-height points define blocked cells
- obstacles are inflated by a safety radius
- A* searches the shortest reachable path from the vehicle pose to the waypoint
- the result is published as `nav_msgs/msg/Path` on `/shortest_path`

### 5. Added Path Planning Optimality Metrics

After selecting a waypoint, `visualization_tools` records:

```text
L_actual
L_shortest
L_actual / L_shortest * 100
L_shortest / L_actual * 100
```

Output file:

```text
src/vehicle_simulator/log/path_metrics_<time>.txt
```

Columns:

```text
time_duration
start_x start_y start_z
goal_x goal_y goal_z
actual_path_length
shortest_path_length
actual_to_shortest_percent
shortest_to_actual_percent
```

For the common requirement "actual path length should be lower than 120% of the
shortest path", use:

```text
actual_to_shortest_percent = L_actual / L_shortest * 100 < 120
```

Equivalently:

```text
shortest_to_actual_percent = L_shortest / L_actual * 100 > 83.33
```

### 6. Added LRAE Preview Map Generator

New script:

```text
src/vehicle_simulator/scripts/generate_lrae_preview_maps.py
```

It parses the LRAE `.world` files, samples collision geometry, and regenerates:

```text
mesh/lrae_scene_*/preview/pointcloud.ply
```

Use it after editing any LRAE world:

```bash
python3 src/vehicle_simulator/scripts/generate_lrae_preview_maps.py
colcon build --packages-select vehicle_simulator
```

## Repository Structure

Important packages:

```text
src/vehicle_simulator
src/visualization_tools
src/local_planner
src/terrain_analysis
src/terrain_analysis_ext
src/sensor_scan_generation
```

Important modified files:

```text
src/vehicle_simulator/launch/vehicle_simulator.launch
src/vehicle_simulator/launch/system_lrae_scene_*.launch
src/vehicle_simulator/urdf/robot.sdf
src/vehicle_simulator/rviz/vehicle_simulator.rviz
src/visualization_tools/src/visualizationTools.cpp
src/visualization_tools/launch/visualization_tools.launch
```

## Dependencies

Recommended environment:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Colcon
- PCL
- OpenCV
- xacro
- `assimp` and Python `trimesh` only if regenerating LRAE preview maps

Install common dependencies:

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

If `trimesh` is missing:

```bash
python3 -m pip install trimesh
```

## Build

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

For faster rebuilds after changing only these packages:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vehicle_simulator visualization_tools
source install/setup.bash
```

## Run Existing Scenes

Garage:

```bash
ros2 launch vehicle_simulator system_garage.launch
```

Indoor:

```bash
ros2 launch vehicle_simulator system_indoor.launch
```

Tunnel:

```bash
ros2 launch vehicle_simulator system_tunnel.launch
```

Forest:

```bash
ros2 launch vehicle_simulator system_forest.launch
```

Campus:

```bash
ros2 launch vehicle_simulator system_campus.launch
```

## Run LRAE Scenes

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch
ros2 launch vehicle_simulator system_lrae_scene_2.launch
ros2 launch vehicle_simulator system_lrae_scene_3.launch
ros2 launch vehicle_simulator system_lrae_scene_4.launch
```

Disable Gazebo GUI if needed:

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch gazebo_gui:=false
```

Override initial pose:

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch \
  vehicleX:=-10.0 vehicleY:=-12.0 terrainZ:=0.3 vehicleYaw:=0.0
```

## RViz Usage

When a system launch file starts, RViz opens with the configured displays.

Useful displays:

| RViz Display | Topic | Meaning |
| --- | --- | --- |
| `OverallMap` | `/overall_map` | Preview point-cloud map |
| `Path` | `/path` | Planner output path |
| `ShortestPath` | `/shortest_path` | A* obstacle-aware shortest path |
| `Trajectory` | `/trajectory` | Actual traveled trajectory |
| `Waypoint` | `/way_point` | Clicked target point |

Workflow:

1. Start one scene.
2. Wait for RViz and Gazebo to finish loading.
3. Select the `Waypoint` tool in RViz.
4. Click a target point in the map.
5. The planner publishes `/path`.
6. The vehicle trajectory is recorded on `/trajectory`.
7. The obstacle-aware ideal shortest path is published on `/shortest_path`.
8. The path optimality metrics are written to `path_metrics_<time>.txt`.

## How The Shortest Path Is Computed

The shortest-path reference is generated in
`src/visualization_tools/src/visualizationTools.cpp`.

Main steps:

1. Load `mesh/<world_name>/preview/pointcloud.ply`.
2. Build a 2D occupancy grid.
3. Estimate local ground from the lowest point in each grid cell.
4. Mark traversable ground cells.
5. Mark obstacle cells from points above local ground.
6. Inflate obstacles by `shortestPathObstacleInflation`.
7. When `/way_point` is received, record current vehicle pose as start.
8. Run A* from start to goal.
9. Publish the resulting `nav_msgs/msg/Path` on `/shortest_path`.
10. Accumulate actual traveled distance from odometry and write metrics.

Important parameters are in:

```text
src/visualization_tools/launch/visualization_tools.launch
```

Defaults:

```xml
<param name="shortestPathGridResolution" value="0.25" />
<param name="shortestPathObstacleInflation" value="0.6" />
<param name="shortestPathObstacleMinZ" value="0.2" />
<param name="shortestPathObstacleMaxZ" value="2.0" />
<param name="shortestPathGroundMinZ" value="-0.3" />
<param name="shortestPathGroundMaxZ" value="0.3" />
<param name="shortestPathGroundInflation" value="0.3" />
<param name="shortestPathNearestFreeRadius" value="3.0" />
```

## Logs

Logs are saved under:

```text
src/vehicle_simulator/log
```

Files:

```text
metrics_<time>.txt
trajectory_<time>.txt
path_metrics_<time>.txt
```

`metrics_<time>.txt`:

```text
explored_volume traveling_distance runtime time_duration
```

`trajectory_<time>.txt`:

```text
x y z roll pitch yaw time_duration
```

`path_metrics_<time>.txt`:

```text
time_duration start_x start_y start_z goal_x goal_y goal_z
actual_path_length shortest_path_length
actual_to_shortest_percent shortest_to_actual_percent
```

## Path Optimization Report

For requirement 5, the recommended workflow is:

1. Run one simulation scene.
2. Click one or more waypoints in RViz.
3. Let the robot move so `path_metrics_<time>.txt` is recorded.
4. Run the offline analyzer to generate figures and an HTML report.

The report uses the obstacle-aware shortest path already computed by
`visualization_tools` on the preview map. This means the "theoretical shortest
path" is not a straight line through walls or obstacles; it is the shortest
reachable path under the same traversability assumptions used in RViz.

Run the analyzer on the latest path metric log:

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py
```

Analyze one specific log:

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py \
  src/vehicle_simulator/log/path_metrics_2026-6-15-23-7-42.txt
```

Analyze several specific logs together:

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py \
  path_metrics_2026-6-15-22-56-11.txt \
  path_metrics_2026-6-15-23-1-14.txt
```

Analyze all matching logs:

```bash
python3 src/visualization_tools/scripts/analyze_path_optimization.py --all
```

Outputs are written to:

```text
src/vehicle_simulator/log/path_optimization_report_<time>/
```

Generated files:

```text
report.html
summary.csv
summary.png
segment_*.png
```

Interpretation:

- `actual_to_shortest_percent = L_actual / L_shortest * 100`
- pass condition: `actual_to_shortest_percent < 120`
- equivalently: `shortest_to_actual_percent > 83.33`

The HTML report shows:

- summary statistics across all path segments
- pass/fail status for each waypoint segment
- bar charts for final ratios
- per-segment time-history plots of actual path length, shortest path length,
  and path optimization

## Troubleshooting

### RViz Has No OverallMap

Check whether the preview point cloud exists:

```bash
ls src/vehicle_simulator/mesh/<world_name>/preview/pointcloud.ply
```

For LRAE scenes, regenerate maps:

```bash
python3 src/vehicle_simulator/scripts/generate_lrae_preview_maps.py
colcon build --packages-select vehicle_simulator
source install/setup.bash
```

### RViz Has No ShortestPath

`ShortestPath` appears only after clicking a target with the RViz `Waypoint`
tool. It also requires:

- `/overall_map` can be loaded
- start and goal are inside the preview map
- start and goal are in the same traversable connected region

If no path is found, try a closer waypoint or adjust obstacle/grid parameters
in `visualization_tools.launch`.

### Gazebo Starts But Vehicle Pose Looks Wrong

Check `terrainZ` and `vehicleHeight`:

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch terrainZ:=0.3
```

The model spawn height is:

```text
terrainZ + vehicleHeight
```

### Multiple Simulations On One Machine

Use a separate ROS domain ID. Keep it no higher than 232 because DDS port
mapping can fail above that range:

```bash
export ROS_DOMAIN_ID=120
```

## Original Project

The original project page is:

```text
https://www.cmu-exploration.com
```

This enhanced repository keeps the original autonomous exploration pipeline and
adds the LRAE scenes, Gazebo realism improvements, obstacle-aware shortest-path
visualization, and path-planning optimality metric support described above.
