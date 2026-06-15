Exploration metrics and vehicle trajectory are saved in this folder.

'metric_xxx.txt' contains four columns, respectively explored volume (m^3), traveling distance (m), algorithm runtime (second), and time duration from start of the run (second). Note that the runtime is recorded by receiving numbers as 'std_msgs::Float32' typed messages on ROS topic '/runtime'.

'trajectory_xxx.txt' contains seven columns, respectively x (m), y (m), z (m), roll (rad), pitch (rad), yaw (rad), and time duration from start of the run (second).

'path_metrics_xxx.txt' contains the path planning optimality metrics after a waypoint is selected. Columns are time duration (s), start x/y/z (m), goal x/y/z (m), actual path length L_actual (m), obstacle-aware shortest path length L_shortest (m), actual-to-shortest ratio L_actual/L_shortest*100 (%), and path optimization L_shortest/L_actual*100 (%). The obstacle-aware shortest path is computed with A* on the preview pointcloud occupancy grid and published in RViz on '/shortest_path'. By default, each grid cell uses its local lowest point as ground; points from -0.3 m to 0.3 m above that ground define traversable support, and points from 0.2 m to 2.0 m above that ground are treated as obstacles and inflated by 0.6 m.
