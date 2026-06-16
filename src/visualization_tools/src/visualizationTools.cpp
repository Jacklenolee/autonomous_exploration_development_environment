#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <geometry_msgs/msg/point_stamped.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

const double PI = 3.1415926;

string metricFile;
string trajFile;
string pathMetricFile;
string mapFile;
double overallMapVoxelSize = 0.5;
double exploredAreaVoxelSize = 0.3;
double exploredVolumeVoxelSize = 0.5;
double transInterval = 0.2;
double yawInterval = 10.0;
double shortestPathGridResolution = 0.2;
double shortestPathObstacleInflation = 0.75;
double shortestPathObstacleMinZ = 0.2;
double shortestPathObstacleMaxZ = 2.0;
double shortestPathGroundMinZ = -0.3;
double shortestPathGroundMaxZ = 0.3;
double shortestPathGroundInflation = 0.3;
double shortestPathGroundSearchRadius = 0.6;
double shortestPathNearestFreeRadius = 3.0;
double shortestPathLineCheckResolution = 0.05;
double shortestPathLineCheckRadius = 0.15;
bool shortestPathUseDynamicObstacles = true;
double shortestPathDynamicObstacleMinZ = 0.2;
double shortestPathDynamicObstacleMaxZ = 2.0;
double shortestPathDynamicObstacleInflation = 0.75;
double shortestPathDynamicObstacleRange = 12.0;
double shortestPathReplanInterval = 1.0;
int overallMapDisplayInterval = 2;
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;
int exploredAreaDisplayCount = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = false;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;
float pathStartX = 0, pathStartY = 0, pathStartZ = 0;
float pathGoalX = 0, pathGoalY = 0, pathGoalZ = 0;
float pathActualDis = 0, shortestPathDis = 0;
float actualToShortestRatio = 0, pathOptimization = 0;
bool shortestPathInited = false;
bool shortestPathGridReady = false;
int shortestPathGridWidth = 0, shortestPathGridHeight = 0;
float shortestPathGridMinX = 0, shortestPathGridMinY = 0;
float shortestPathFloorZ = 0;
vector<unsigned char> shortestPathGrid;
vector<unsigned char> shortestPathStaticGrid;
vector<unsigned char> shortestPathDynamicObstacleGrid;
vector<geometry_msgs::msg::Point> shortestPathPoints;
vector<geometry_msgs::msg::Point> shortestPathHistoryLinePoints;
size_t shortestPathCurrentHistoryStart = 0;
bool shortestPathCurrentHistoryActive = false;
bool shortestPathDynamicGridDirty = false;
double shortestPathLastReplanTime = 0;

pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

sensor_msgs::msg::PointCloud2 overallMap2;

shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubExploredAreaPtr;

shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubTrajectoryPtr;

shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pubShortestPathPtr;

shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pubShortestPathHistoryPtr;

shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubExploredVolumePtr;

shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubTravelingDisPtr;

shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubTimeDurationPtr;

shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubPathOptimizationPtr;

shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubActualToShortestRatioPtr;

FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;
FILE *pathMetricFilePtr = NULL;

void redirectInstallPathToSource(string& filePath)
{
  size_t installPos = filePath.find("/install/");
  if (installPos != string::npos) {
    filePath.replace(installPos, 8, "/src");
  }
}

int gridIndex(int ix, int iy)
{
  return iy * shortestPathGridWidth + ix;
}

bool isInsideGrid(int ix, int iy)
{
  return ix >= 0 && ix < shortestPathGridWidth && iy >= 0 && iy < shortestPathGridHeight;
}

bool worldToGrid(float x, float y, int& ix, int& iy)
{
  ix = static_cast<int>(floor((x - shortestPathGridMinX) / shortestPathGridResolution));
  iy = static_cast<int>(floor((y - shortestPathGridMinY) / shortestPathGridResolution));
  return isInsideGrid(ix, iy);
}

geometry_msgs::msg::Point gridToWorld(int ix, int iy, float z)
{
  geometry_msgs::msg::Point point;
  point.x = shortestPathGridMinX + (ix + 0.5) * shortestPathGridResolution;
  point.y = shortestPathGridMinY + (iy + 0.5) * shortestPathGridResolution;
  point.z = z;
  return point;
}

bool isFreeCell(int ix, int iy)
{
  return isInsideGrid(ix, iy) && shortestPathGrid[gridIndex(ix, iy)] == 0;
}

void setInflatedGridCells(vector<unsigned char>& grid, int cx, int cy, int radiusCells, unsigned char value)
{
  int radiusCells2 = radiusCells * radiusCells;
  for (int dy = -radiusCells; dy <= radiusCells; dy++) {
    for (int dx = -radiusCells; dx <= radiusCells; dx++) {
      if (dx * dx + dy * dy > radiusCells2) {
        continue;
      }

      int nx = cx + dx;
      int ny = cy + dy;
      if (isInsideGrid(nx, ny)) {
        grid[gridIndex(nx, ny)] = value;
      }
    }
  }
}

void updateCombinedShortestPathGrid()
{
  if (shortestPathStaticGrid.empty()) {
    shortestPathGrid.clear();
    return;
  }

  shortestPathGrid = shortestPathStaticGrid;
  if (!shortestPathUseDynamicObstacles || shortestPathDynamicObstacleGrid.empty()) {
    return;
  }

  for (size_t i = 0; i < shortestPathGrid.size(); i++) {
    if (shortestPathDynamicObstacleGrid[i] != 0) {
      shortestPathGrid[i] = 1;
    }
  }
}

bool findNearestFreeCell(int& ix, int& iy)
{
  if (isFreeCell(ix, iy)) {
    return true;
  }

  int maxRadius = max(1, static_cast<int>(ceil(shortestPathNearestFreeRadius / shortestPathGridResolution)));
  int bestX = ix;
  int bestY = iy;
  int bestDist2 = std::numeric_limits<int>::max();

  for (int radius = 1; radius <= maxRadius; radius++) {
    for (int dx = -radius; dx <= radius; dx++) {
      for (int dy = -radius; dy <= radius; dy++) {
        if (abs(dx) != radius && abs(dy) != radius) {
          continue;
        }

        int nx = ix + dx;
        int ny = iy + dy;
        if (!isFreeCell(nx, ny)) {
          continue;
        }

        int dist2 = dx * dx + dy * dy;
        if (dist2 < bestDist2) {
          bestDist2 = dist2;
          bestX = nx;
          bestY = ny;
        }
      }
    }

    if (bestDist2 != std::numeric_limits<int>::max()) {
      ix = bestX;
      iy = bestY;
      return true;
    }
  }

  return false;
}

bool isWorldCircleFree(float x, float y, float radius)
{
  int ix, iy;
  if (!worldToGrid(x, y, ix, iy)) {
    return false;
  }

  int radiusCells = max(0, static_cast<int>(ceil(radius / shortestPathGridResolution)));
  int radiusCells2 = radiusCells * radiusCells;
  for (int dy = -radiusCells; dy <= radiusCells; dy++) {
    for (int dx = -radiusCells; dx <= radiusCells; dx++) {
      if (dx * dx + dy * dy > radiusCells2) {
        continue;
      }

      if (!isFreeCell(ix + dx, iy + dy)) {
        return false;
      }
    }
  }

  return true;
}

bool hasWorldLineOfSight(float x0, float y0, float x1, float y1, float radius)
{
  float dx = x1 - x0;
  float dy = y1 - y0;
  float length = sqrt(dx * dx + dy * dy);
  int sampleNum = max(1, static_cast<int>(ceil(length / shortestPathLineCheckResolution)));

  for (int i = 0; i <= sampleNum; i++) {
    float ratio = static_cast<float>(i) / static_cast<float>(sampleNum);
    float x = x0 + ratio * dx;
    float y = y0 + ratio * dy;
    if (!isWorldCircleFree(x, y, radius)) {
      return false;
    }
  }

  return true;
}

bool hasLineOfSight(int x0, int y0, int x1, int y1)
{
  geometry_msgs::msg::Point start = gridToWorld(x0, y0, shortestPathFloorZ);
  geometry_msgs::msg::Point goal = gridToWorld(x1, y1, shortestPathFloorZ);
  return hasWorldLineOfSight(start.x, start.y, goal.x, goal.y, shortestPathLineCheckRadius);
}

void appendShortestPathToHistory()
{
  if (shortestPathPoints.size() < 2) {
    return;
  }

  if (!shortestPathCurrentHistoryActive) {
    shortestPathCurrentHistoryStart = shortestPathHistoryLinePoints.size();
    shortestPathCurrentHistoryActive = true;
  } else if (shortestPathCurrentHistoryStart < shortestPathHistoryLinePoints.size()) {
    shortestPathHistoryLinePoints.resize(shortestPathCurrentHistoryStart);
  }

  for (size_t i = 1; i < shortestPathPoints.size(); i++) {
    shortestPathHistoryLinePoints.push_back(shortestPathPoints[i - 1]);
    shortestPathHistoryLinePoints.push_back(shortestPathPoints[i]);
  }
}

void buildShortestPathGrid()
{
  shortestPathGridReady = false;
  shortestPathPoints.clear();

  if (overallMapCloud->points.empty()) {
    return;
  }

  float minX = overallMapCloud->points[0].x;
  float maxX = overallMapCloud->points[0].x;
  float minY = overallMapCloud->points[0].y;
  float maxY = overallMapCloud->points[0].y;
  float minZ = overallMapCloud->points[0].z;

  for (const auto& point : overallMapCloud->points) {
    minX = min(minX, point.x);
    maxX = max(maxX, point.x);
    minY = min(minY, point.y);
    maxY = max(maxY, point.y);
    minZ = min(minZ, point.z);
  }

  shortestPathFloorZ = minZ;
  const float mapPadding = max(1.0, shortestPathNearestFreeRadius);
  shortestPathGridMinX = minX - mapPadding;
  shortestPathGridMinY = minY - mapPadding;
  shortestPathGridWidth = static_cast<int>(ceil((maxX - minX + 2.0 * mapPadding) / shortestPathGridResolution));
  shortestPathGridHeight = static_cast<int>(ceil((maxY - minY + 2.0 * mapPadding) / shortestPathGridResolution));

  if (shortestPathGridWidth <= 0 || shortestPathGridHeight <= 0) {
    return;
  }

  shortestPathGrid.assign(shortestPathGridWidth * shortestPathGridHeight, 1);
  vector<unsigned char> rawObstacleGrid(shortestPathGridWidth * shortestPathGridHeight, 0);
  vector<unsigned char> rawGroundGrid(shortestPathGridWidth * shortestPathGridHeight, 0);
  vector<float> cellMinZ(shortestPathGridWidth * shortestPathGridHeight,
                         std::numeric_limits<float>::infinity());
  vector<float> localGroundZ(shortestPathGridWidth * shortestPathGridHeight,
                             std::numeric_limits<float>::infinity());

  for (const auto& point : overallMapCloud->points) {
    int ix, iy;
    if (!worldToGrid(point.x, point.y, ix, iy)) {
      continue;
    }

    int index = gridIndex(ix, iy);
    cellMinZ[index] = min(cellMinZ[index], point.z);
  }

  int groundSearchCells = max(0, static_cast<int>(ceil(shortestPathGroundSearchRadius / shortestPathGridResolution)));
  for (int y = 0; y < shortestPathGridHeight; y++) {
    for (int x = 0; x < shortestPathGridWidth; x++) {
      float minLocalZ = std::numeric_limits<float>::infinity();
      for (int dy = -groundSearchCells; dy <= groundSearchCells; dy++) {
        for (int dx = -groundSearchCells; dx <= groundSearchCells; dx++) {
          if (dx * dx + dy * dy > groundSearchCells * groundSearchCells) {
            continue;
          }

          int nx = x + dx;
          int ny = y + dy;
          if (!isInsideGrid(nx, ny)) {
            continue;
          }

          minLocalZ = min(minLocalZ, cellMinZ[gridIndex(nx, ny)]);
        }
      }

      localGroundZ[gridIndex(x, y)] = minLocalZ;
    }
  }

  for (const auto& point : overallMapCloud->points) {
    int ix, iy;
    if (!worldToGrid(point.x, point.y, ix, iy)) {
      continue;
    }

    int index = gridIndex(ix, iy);
    if (!std::isfinite(localGroundZ[index])) {
      continue;
    }

    float groundZ = localGroundZ[index];
    if (point.z >= groundZ + shortestPathGroundMinZ &&
        point.z <= groundZ + shortestPathGroundMaxZ) {
      rawGroundGrid[index] = 1;
    }

    if (point.z >= groundZ + shortestPathObstacleMinZ &&
        point.z <= groundZ + shortestPathObstacleMaxZ) {
      rawObstacleGrid[index] = 1;
    }
  }

  int groundInflationCells = max(0, static_cast<int>(ceil(shortestPathGroundInflation / shortestPathGridResolution)));
  for (int y = 0; y < shortestPathGridHeight; y++) {
    for (int x = 0; x < shortestPathGridWidth; x++) {
      if (rawGroundGrid[gridIndex(x, y)] == 0) {
        continue;
      }

      setInflatedGridCells(shortestPathGrid, x, y, groundInflationCells, 0);
    }
  }

  int inflationCells = max(0, static_cast<int>(ceil(shortestPathObstacleInflation / shortestPathGridResolution)));
  for (int y = 0; y < shortestPathGridHeight; y++) {
    for (int x = 0; x < shortestPathGridWidth; x++) {
      if (rawObstacleGrid[gridIndex(x, y)] == 0) {
        continue;
      }

      setInflatedGridCells(shortestPathGrid, x, y, inflationCells, 1);
    }
  }

  shortestPathStaticGrid = shortestPathGrid;
  shortestPathDynamicObstacleGrid.assign(shortestPathGridWidth * shortestPathGridHeight, 0);
  updateCombinedShortestPathGrid();
  shortestPathGridReady = true;
}

float computePolylineLength(const vector<geometry_msgs::msg::Point>& points)
{
  float length = 0;
  for (size_t i = 1; i < points.size(); i++) {
    float dx = points[i].x - points[i - 1].x;
    float dy = points[i].y - points[i - 1].y;
    float dz = points[i].z - points[i - 1].z;
    length += sqrt(dx * dx + dy * dy + dz * dz);
  }
  return length;
}

bool computeObstacleAwareShortestPath()
{
  shortestPathPoints.clear();
  shortestPathDis = 0;

  if (!shortestPathGridReady) {
    return false;
  }

  int originalStartX, originalStartY, originalGoalX, originalGoalY;
  if (!worldToGrid(pathStartX, pathStartY, originalStartX, originalStartY) ||
      !worldToGrid(pathGoalX, pathGoalY, originalGoalX, originalGoalY)) {
    return false;
  }

  int startX = originalStartX;
  int startY = originalStartY;
  int goalX = originalGoalX;
  int goalY = originalGoalY;
  if (!findNearestFreeCell(startX, startY) || !findNearestFreeCell(goalX, goalY)) {
    return false;
  }

  const int cellCount = shortestPathGridWidth * shortestPathGridHeight;
  const float inf = std::numeric_limits<float>::infinity();
  vector<float> gScore(cellCount, inf);
  vector<int> cameFrom(cellCount, -1);
  vector<unsigned char> closed(cellCount, 0);

  struct SearchNode {
    int index;
    float priority;
    bool operator<(const SearchNode& other) const
    {
      return priority > other.priority;
    }
  };

  auto heuristic = [&](int x, int y) {
    float dx = static_cast<float>(x - goalX);
    float dy = static_cast<float>(y - goalY);
    return static_cast<float>(shortestPathGridResolution * sqrt(dx * dx + dy * dy));
  };

  int startIndex = gridIndex(startX, startY);
  int goalIndex = gridIndex(goalX, goalY);
  priority_queue<SearchNode> openSet;
  gScore[startIndex] = 0;
  openSet.push({startIndex, heuristic(startX, startY)});

  const int neighborDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int neighborDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  while (!openSet.empty()) {
    int currentIndex = openSet.top().index;
    openSet.pop();

    if (closed[currentIndex]) {
      continue;
    }

    if (currentIndex == goalIndex) {
      break;
    }

    closed[currentIndex] = 1;
    int currentX = currentIndex % shortestPathGridWidth;
    int currentY = currentIndex / shortestPathGridWidth;

    for (int i = 0; i < 8; i++) {
      int nx = currentX + neighborDx[i];
      int ny = currentY + neighborDy[i];
      if (!isFreeCell(nx, ny)) {
        continue;
      }

      if (neighborDx[i] != 0 && neighborDy[i] != 0 &&
          (!isFreeCell(currentX + neighborDx[i], currentY) ||
           !isFreeCell(currentX, currentY + neighborDy[i]))) {
        continue;
      }

      int neighborIndex = gridIndex(nx, ny);
      if (closed[neighborIndex]) {
        continue;
      }

      float stepCost = static_cast<float>((neighborDx[i] == 0 || neighborDy[i] == 0) ?
                       shortestPathGridResolution : shortestPathGridResolution * sqrt(2.0));
      float tentativeG = gScore[currentIndex] + stepCost;
      if (tentativeG < gScore[neighborIndex]) {
        cameFrom[neighborIndex] = currentIndex;
        gScore[neighborIndex] = tentativeG;
        openSet.push({neighborIndex, tentativeG + heuristic(nx, ny)});
      }
    }
  }

  if (!std::isfinite(gScore[goalIndex])) {
    return false;
  }

  vector<int> rawPath;
  for (int current = goalIndex; current != -1; current = cameFrom[current]) {
    rawPath.push_back(current);
    if (current == startIndex) {
      break;
    }
  }

  if (rawPath.empty() || rawPath.back() != startIndex) {
    return false;
  }

  reverse(rawPath.begin(), rawPath.end());

  vector<int> simplifiedPath;
  size_t anchor = 0;
  simplifiedPath.push_back(rawPath.front());
  while (anchor < rawPath.size() - 1) {
    size_t best = anchor + 1;
    int anchorX = rawPath[anchor] % shortestPathGridWidth;
    int anchorY = rawPath[anchor] / shortestPathGridWidth;
    for (size_t candidate = rawPath.size() - 1; candidate > anchor; candidate--) {
      int candidateX = rawPath[candidate] % shortestPathGridWidth;
      int candidateY = rawPath[candidate] / shortestPathGridWidth;
      if (hasLineOfSight(anchorX, anchorY, candidateX, candidateY)) {
        best = candidate;
        break;
      }
    }
    simplifiedPath.push_back(rawPath[best]);
    anchor = best;
  }

  for (size_t i = 0; i < simplifiedPath.size(); i++) {
    int ix = simplifiedPath[i] % shortestPathGridWidth;
    int iy = simplifiedPath[i] / shortestPathGridWidth;
    float z = pathStartZ + (pathGoalZ - pathStartZ) * static_cast<float>(i) /
              static_cast<float>(max<size_t>(1, simplifiedPath.size() - 1));
    shortestPathPoints.push_back(gridToWorld(ix, iy, z));
  }

  if (!shortestPathPoints.empty()) {
    if (isFreeCell(originalStartX, originalStartY) &&
        hasWorldLineOfSight(pathStartX, pathStartY, shortestPathPoints.front().x, shortestPathPoints.front().y,
                            shortestPathLineCheckRadius)) {
      shortestPathPoints.front().x = pathStartX;
      shortestPathPoints.front().y = pathStartY;
      shortestPathPoints.front().z = pathStartZ;
    }

    if (isFreeCell(originalGoalX, originalGoalY) &&
        hasWorldLineOfSight(shortestPathPoints.back().x, shortestPathPoints.back().y, pathGoalX, pathGoalY,
                            shortestPathLineCheckRadius)) {
      shortestPathPoints.back().x = pathGoalX;
      shortestPathPoints.back().y = pathGoalY;
      shortestPathPoints.back().z = pathGoalZ;
    }
  }

  shortestPathDis = computePolylineLength(shortestPathPoints);
  return shortestPathPoints.size() >= 2 && shortestPathDis > 1e-3;
}

void publishShortestPath(const builtin_interfaces::msg::Time& stamp)
{
  nav_msgs::msg::Path shortestPath;
  shortestPath.header.stamp = stamp;
  shortestPath.header.frame_id = "map";
  if (shortestPathInited && shortestPathPoints.size() >= 2) {
    shortestPath.poses.resize(shortestPathPoints.size());

    for (size_t i = 0; i < shortestPathPoints.size(); i++) {
      shortestPath.poses[i].header = shortestPath.header;
      shortestPath.poses[i].pose.position = shortestPathPoints[i];
      shortestPath.poses[i].pose.orientation.w = 1.0;
    }
  }
  pubShortestPathPtr->publish(shortestPath);

  visualization_msgs::msg::MarkerArray shortestPathHistoryMsg;
  visualization_msgs::msg::Marker historyMarker;
  historyMarker.header = shortestPath.header;
  historyMarker.ns = "shortest_path_history";
  historyMarker.id = 0;
  historyMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
  historyMarker.action = visualization_msgs::msg::Marker::ADD;
  historyMarker.pose.orientation.w = 1.0;
  historyMarker.scale.x = 0.08;
  historyMarker.color.r = 1.0;
  historyMarker.color.g = 0.31;
  historyMarker.color.b = 0.0;
  historyMarker.color.a = 1.0;
  historyMarker.points = shortestPathHistoryLinePoints;
  shortestPathHistoryMsg.markers.push_back(historyMarker);
  pubShortestPathHistoryPtr->publish(shortestPathHistoryMsg);
}

void updatePathOptimization()
{
  if (!shortestPathInited || shortestPathDis < 1e-3 || pathActualDis < 1e-3) {
    actualToShortestRatio = 0;
    pathOptimization = 0;
    return;
  }

  actualToShortestRatio = 100.0 * pathActualDis / shortestPathDis;
  pathOptimization = 100.0 * shortestPathDis / pathActualDis;
}

bool replanShortestPath(const char* reason)
{
  if (!shortestPathGridReady || !shortestPathInited) {
    return false;
  }

  vector<geometry_msgs::msg::Point> previousShortestPathPoints = shortestPathPoints;
  float previousShortestPathDis = shortestPathDis;
  bool replanned = computeObstacleAwareShortestPath();
  if (!replanned) {
    shortestPathPoints = previousShortestPathPoints;
    shortestPathDis = previousShortestPathDis;
    RCLCPP_WARN(rclcpp::get_logger("visualizationTools"),
                "Failed to replan obstacle-aware shortest path after %s. Keeping previous path.", reason);
    return false;
  }

  appendShortestPathToHistory();
  updatePathOptimization();
  return true;
}

void publishPathMetrics()
{
  if (!shortestPathInited) {
    return;
  }

  std_msgs::msg::Float32 pathOptimizationMsg;
  pathOptimizationMsg.data = pathOptimization;
  pubPathOptimizationPtr->publish(pathOptimizationMsg);

  std_msgs::msg::Float32 actualToShortestRatioMsg;
  actualToShortestRatioMsg.data = actualToShortestRatio;
  pubActualToShortestRatioPtr->publish(actualToShortestRatioMsg);
}

bool isShortestPathCollisionFree()
{
  if (!shortestPathInited || shortestPathPoints.size() < 2) {
    return true;
  }

  for (size_t i = 1; i < shortestPathPoints.size(); i++) {
    if (!hasWorldLineOfSight(shortestPathPoints[i - 1].x, shortestPathPoints[i - 1].y,
                             shortestPathPoints[i].x, shortestPathPoints[i].y,
                             shortestPathLineCheckRadius)) {
      return false;
    }
  }

  return true;
}

void replanIfDynamicObstaclesBlockShortestPath()
{
  if (!shortestPathInited || !shortestPathDynamicGridDirty ||
      systemTime - shortestPathLastReplanTime < shortestPathReplanInterval) {
    return;
  }

  if (!isShortestPathCollisionFree() &&
      replanShortestPath("dynamic obstacle update")) {
    shortestPathLastReplanTime = systemTime;
  }

  shortestPathDynamicGridDirty = false;
}

void waypointHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr waypoint)
{
  pathStartX = vehicleX;
  pathStartY = vehicleY;
  pathStartZ = vehicleZ;
  pathGoalX = waypoint->point.x;
  pathGoalY = waypoint->point.y;
  pathGoalZ = waypoint->point.z;
  pathActualDis = 0;
  size_t previousShortestPathHistoryStart = shortestPathCurrentHistoryStart;
  bool previousShortestPathHistoryActive = shortestPathCurrentHistoryActive;
  shortestPathCurrentHistoryStart = shortestPathHistoryLinePoints.size();
  shortestPathCurrentHistoryActive = false;
  vector<geometry_msgs::msg::Point> previousShortestPathPoints = shortestPathPoints;
  float previousShortestPathDis = shortestPathDis;
  bool previousShortestPathInited = shortestPathInited;
  shortestPathInited = computeObstacleAwareShortestPath();

  if (!shortestPathInited) {
    RCLCPP_WARN(rclcpp::get_logger("visualizationTools"),
                "Failed to compute obstacle-aware shortest path from waypoint. Check map bounds, obstacle filters, or goal reachability.");
    shortestPathPoints = previousShortestPathPoints;
    shortestPathDis = previousShortestPathDis;
    shortestPathInited = previousShortestPathInited;
    shortestPathCurrentHistoryStart = previousShortestPathHistoryStart;
    shortestPathCurrentHistoryActive = previousShortestPathHistoryActive;
  } else {
    appendShortestPathToHistory();
    shortestPathLastReplanTime = systemTime;
  }

  updatePathOptimization();
  publishShortestPath(waypoint->header.stamp);
  publishPathMetrics();
}

void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  systemTime = rclcpp::Time(odom->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI  - dYaw;

  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  if (systemInited) {
    timeDuration = systemTime - systemInitTime;
    
    std_msgs::msg::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  replanIfDynamicObstaclesBlockShortestPath();

  if (dis < transInterval && dYaw < yawInterval) {
    return;
  }

  if (!systemInited) {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  travelingDis += dis;
  if (shortestPathInited) {
    pathActualDis += dis;
    updatePathOptimization();
  }

  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);

  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  sensor_msgs::msg::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);

  publishShortestPath(odom->header.stamp);
  publishPathMetrics();

  if (pathMetricFilePtr != NULL && shortestPathInited) {
    fprintf(pathMetricFilePtr, "%f %f %f %f %f %f %f %f %f %f %f\n",
            timeDuration, pathStartX, pathStartY, pathStartZ, pathGoalX, pathGoalY, pathGoalZ,
            pathActualDis, shortestPathDis, actualToShortestRatio, pathOptimization);
  }
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  if (shortestPathUseDynamicObstacles && shortestPathGridReady &&
      !shortestPathStaticGrid.empty() && !laserCloud->points.empty()) {
    shortestPathDynamicObstacleGrid.assign(shortestPathGridWidth * shortestPathGridHeight, 0);
    int dynamicInflationCells = max(0, static_cast<int>(ceil(shortestPathDynamicObstacleInflation /
                                                            shortestPathGridResolution)));

    for (const auto& point : laserCloud->points) {
      float relX = point.x - vehicleX;
      float relY = point.y - vehicleY;
      float relZ = point.z - vehicleZ;
      float disXY = sqrt(relX * relX + relY * relY);
      if (disXY > shortestPathDynamicObstacleRange ||
          relZ < shortestPathDynamicObstacleMinZ ||
          relZ > shortestPathDynamicObstacleMaxZ) {
        continue;
      }

      int ix, iy;
      if (worldToGrid(point.x, point.y, ix, iy)) {
        setInflatedGridCells(shortestPathDynamicObstacleGrid, ix, iy, dynamicInflationCells, 1);
      }
    }

    updateCombinedShortestPathGrid();
    shortestPathDynamicGridDirty = true;
  }

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::msg::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  std_msgs::msg::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::msg::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}

void runtimeHandler(const std_msgs::msg::Float32::ConstSharedPtr runtimeIn)
{
  runtime = runtimeIn->data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("visualizationTools");

  nh->declare_parameter<std::string>("metricFile", metricFile);
  nh->declare_parameter<std::string>("trajFile", trajFile);
  nh->declare_parameter<std::string>("pathMetricFile", pathMetricFile);
  nh->declare_parameter<std::string>("mapFile", mapFile);
  nh->declare_parameter<double>("overallMapVoxelSize", overallMapVoxelSize);
  nh->declare_parameter<double>("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nh->declare_parameter<double>("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nh->declare_parameter<double>("transInterval", transInterval);
  nh->declare_parameter<double>("yawInterval", yawInterval);
  nh->declare_parameter<double>("shortestPathGridResolution", shortestPathGridResolution);
  nh->declare_parameter<double>("shortestPathObstacleInflation", shortestPathObstacleInflation);
  nh->declare_parameter<double>("shortestPathObstacleMinZ", shortestPathObstacleMinZ);
  nh->declare_parameter<double>("shortestPathObstacleMaxZ", shortestPathObstacleMaxZ);
  nh->declare_parameter<double>("shortestPathGroundMinZ", shortestPathGroundMinZ);
  nh->declare_parameter<double>("shortestPathGroundMaxZ", shortestPathGroundMaxZ);
  nh->declare_parameter<double>("shortestPathGroundInflation", shortestPathGroundInflation);
  nh->declare_parameter<double>("shortestPathGroundSearchRadius", shortestPathGroundSearchRadius);
  nh->declare_parameter<double>("shortestPathNearestFreeRadius", shortestPathNearestFreeRadius);
  nh->declare_parameter<double>("shortestPathLineCheckResolution", shortestPathLineCheckResolution);
  nh->declare_parameter<double>("shortestPathLineCheckRadius", shortestPathLineCheckRadius);
  nh->declare_parameter<bool>("shortestPathUseDynamicObstacles", shortestPathUseDynamicObstacles);
  nh->declare_parameter<double>("shortestPathDynamicObstacleMinZ", shortestPathDynamicObstacleMinZ);
  nh->declare_parameter<double>("shortestPathDynamicObstacleMaxZ", shortestPathDynamicObstacleMaxZ);
  nh->declare_parameter<double>("shortestPathDynamicObstacleInflation", shortestPathDynamicObstacleInflation);
  nh->declare_parameter<double>("shortestPathDynamicObstacleRange", shortestPathDynamicObstacleRange);
  nh->declare_parameter<double>("shortestPathReplanInterval", shortestPathReplanInterval);
  nh->declare_parameter<int>("overallMapDisplayInterval", overallMapDisplayInterval);
  nh->declare_parameter<int>("exploredAreaDisplayInterval", exploredAreaDisplayInterval);

  nh->get_parameter("metricFile", metricFile);
  nh->get_parameter("trajFile", trajFile);
  nh->get_parameter("pathMetricFile", pathMetricFile);
  nh->get_parameter("mapFile", mapFile);
  nh->get_parameter("overallMapVoxelSize", overallMapVoxelSize);
  nh->get_parameter("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nh->get_parameter("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nh->get_parameter("transInterval", transInterval);
  nh->get_parameter("yawInterval", yawInterval);
  nh->get_parameter("shortestPathGridResolution", shortestPathGridResolution);
  nh->get_parameter("shortestPathObstacleInflation", shortestPathObstacleInflation);
  nh->get_parameter("shortestPathObstacleMinZ", shortestPathObstacleMinZ);
  nh->get_parameter("shortestPathObstacleMaxZ", shortestPathObstacleMaxZ);
  nh->get_parameter("shortestPathGroundMinZ", shortestPathGroundMinZ);
  nh->get_parameter("shortestPathGroundMaxZ", shortestPathGroundMaxZ);
  nh->get_parameter("shortestPathGroundInflation", shortestPathGroundInflation);
  nh->get_parameter("shortestPathGroundSearchRadius", shortestPathGroundSearchRadius);
  nh->get_parameter("shortestPathNearestFreeRadius", shortestPathNearestFreeRadius);
  nh->get_parameter("shortestPathLineCheckResolution", shortestPathLineCheckResolution);
  nh->get_parameter("shortestPathLineCheckRadius", shortestPathLineCheckRadius);
  nh->get_parameter("shortestPathUseDynamicObstacles", shortestPathUseDynamicObstacles);
  nh->get_parameter("shortestPathDynamicObstacleMinZ", shortestPathDynamicObstacleMinZ);
  nh->get_parameter("shortestPathDynamicObstacleMaxZ", shortestPathDynamicObstacleMaxZ);
  nh->get_parameter("shortestPathDynamicObstacleInflation", shortestPathDynamicObstacleInflation);
  nh->get_parameter("shortestPathDynamicObstacleRange", shortestPathDynamicObstacleRange);
  nh->get_parameter("shortestPathReplanInterval", shortestPathReplanInterval);
  nh->get_parameter("overallMapDisplayInterval", overallMapDisplayInterval);
  nh->get_parameter("exploredAreaDisplayInterval", exploredAreaDisplayInterval);

  // No direct replacement present for $(find pkg) in ROS2. Edit file path.
  redirectInstallPathToSource(metricFile);
  redirectInstallPathToSource(trajFile);
  redirectInstallPathToSource(pathMetricFile);

  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odometryHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", 5, laserCloudHandler);

  auto subRuntime = nh->create_subscription<std_msgs::msg::Float32>("/runtime", 5, runtimeHandler);

  auto subWaypoint = nh->create_subscription<geometry_msgs::msg::PointStamped>("/way_point", 5, waypointHandler);

  auto pubOverallMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/overall_map", 5);

  pubExploredAreaPtr = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/explored_areas", 5);

  pubTrajectoryPtr = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/trajectory", 5);

  pubShortestPathPtr = nh->create_publisher<nav_msgs::msg::Path>("/shortest_path", 5);
  pubShortestPathHistoryPtr = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/shortest_path_history", 5);
  
  pubExploredVolumePtr = nh->create_publisher<std_msgs::msg::Float32>("/explored_volume", 5);

  pubTravelingDisPtr = nh->create_publisher<std_msgs::msg::Float32>("/traveling_distance", 5);

  pubTimeDurationPtr = nh->create_publisher<std_msgs::msg::Float32>("/time_duration", 5);

  pubPathOptimizationPtr = nh->create_publisher<std_msgs::msg::Float32>("/path_optimization", 5);

  pubActualToShortestRatioPtr = nh->create_publisher<std_msgs::msg::Float32>("/actual_to_shortest_ratio", 5);

  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  pcl::PLYReader ply_reader;
  if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
    RCLCPP_INFO(nh->get_logger(), "Couldn't read pointcloud.ply file.");
  }

  buildShortestPathGrid();
  if (!shortestPathGridReady) {
    RCLCPP_WARN(nh->get_logger(), "Failed to build obstacle grid for shortest path.");
  }

  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);
  overallMapCloud->clear();

  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + to_string(ltm->tm_mday) + "-" +
                      to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

  metricFile += "_" + timeString + ".txt";
  trajFile += "_" + timeString + ".txt";
  pathMetricFile += "_" + timeString + ".txt";
  metricFilePtr = fopen(metricFile.c_str(), "w");
  trajFilePtr = fopen(trajFile.c_str(), "w");
  pathMetricFilePtr = fopen(pathMetricFile.c_str(), "w");
  if (pathMetricFilePtr != NULL) {
    fprintf(pathMetricFilePtr,
            "# time_duration start_x start_y start_z goal_x goal_y goal_z actual_path_length shortest_path_length actual_to_shortest_percent shortest_to_actual_percent\n");
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);
    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval) {
      overallMap2.header.stamp = rclcpp::Time(static_cast<uint64_t>(systemTime * 1e9));
      overallMap2.header.frame_id = "map";
      pubOverallMap->publish(overallMap2);

      overallMapDisplayCount = 0;
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  fclose(metricFilePtr);
  fclose(trajFilePtr);
  if (pathMetricFilePtr != NULL) {
    fclose(pathMetricFilePtr);
  }

  RCLCPP_INFO(nh->get_logger(), "Exploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.");

  return 0;
}
