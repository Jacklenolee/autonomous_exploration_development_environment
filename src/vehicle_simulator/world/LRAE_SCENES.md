LRAE scenes port
================

The `lrae_scene_*.world` files are adapted copies of the LRAE `simworld`
`map_scene_*.world` files. They are kept under `vehicle_simulator/world` so the
existing autonomous exploration launch stack can load them through
`world_name`, for example:

```bash
ros2 launch vehicle_simulator system_lrae_scene_1.launch
```

The launch wrappers intentionally reuse `system_garage.launch`; only the world
name and initial pose are changed. The vehicle model, sensors, local planner,
terrain analysis, visualization, and other algorithm nodes still come from this
project.

Compatibility changes made to the copied worlds:

- added the `gazebo_ros_state` world plugin required by `vehicleSimulator`
- kept the original LRAE scenes compatible with this project's kinematic
  `vehicleSimulator`; all LRAE worlds now use Earth gravity (`0 0 -9.81`) for
  a more realistic Gazebo environment
- added explicit terrain/collision friction where the copied world files left
  contact friction unspecified
- renamed copied collision elements with a `_collision` suffix to avoid Gazebo
  duplicate-name warnings when a link had visual and collision elements sharing
  a name
- copied the referenced LRAE terrain meshes and Gazebo model assets into
  `vehicle_simulator/mesh`
- generated `mesh/lrae_scene_*/preview/pointcloud.ply` maps. These preview
  point clouds are the source for RViz `/overall_map` and the obstacle-aware
  A* `/shortest_path` reference path used by the path optimality metric.

The vehicle visualization model has simple collision geometry and contact
friction, but the default simulator still drives `robot`, `lidar`, and `camera`
poses through `/set_entity_state` rather than through wheel joints and motor
controllers. It is therefore suitable for navigation and perception testing,
not full wheel-ground dynamics validation.

Default initial poses follow the LRAE scene notes:

| Launch file | World | Initial `(x, y, terrainZ)` |
| --- | --- | --- |
| `system_lrae_scene_1.launch` | `lrae_scene_1.world` | `(-14, -14, 0.3)` |
| `system_lrae_scene_2.launch` | `lrae_scene_2.world` | `(-27, -27, 0.3)` |
| `system_lrae_scene_3.launch` | `lrae_scene_3.world` | `(-18, -20, 0.0)` |
| `system_lrae_scene_4.launch` | `lrae_scene_4.world` | `(0, 0, 0.0)` |

If a LRAE world is edited, regenerate its RViz/metric preview maps with:

```bash
python3 src/vehicle_simulator/scripts/generate_lrae_preview_maps.py
colcon build --packages-select vehicle_simulator
```
