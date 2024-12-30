// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#include <open3d/Open3D.h>

#include <Eigen/Dense>
#include <vector>

#include "CollisionMetric.h"
#include "GripperPaths.h"
#include "PointCloud.h"

class CollisionAvoidanceMetric {
 public:
  /**
   * @brief Initialize CollisionAvoidanceMetric
   *
   * @param gripper_z_tolerances List of gripper z tolerances to be used in mm.
   * The output metrics will be computed for each tolerance value
   * @param directions List of 3D vectors representing directions for collision
   * calculation
   * @param outlier_threshold Number of points to be considered as outliers
   * before registering a collision (default: 5)
   * @param gripper_size Size of the gripper in millimeters (default: 10.0)
   * @param gripper_step Step size for gripper movement along specified
   * directions in mm (default: 5.0)
   */
  CollisionAvoidanceMetric(std::vector<double> gripper_z_tolerances,
                           std::vector<Eigen::Vector3d> directions,
                           int outlier_threshold, double gripper_size,
                           int gripper_step);

  /**
   * @brief Compute collision metric for a specific gripper direction
   *
   * @param query_point_cloud Query point cloud to be evaluated
   * @param gt_point_cloud Ground truth point cloud
   * @param gripper Gripper to be used for metrics calculation
   * @param direction Direction of the gripper
   * @return CollisionMetricsResults Results of the collision metrics
   * computation
   */
  CollisionMetricsResults ComputeCollisionForGripperDirection(
      PointCloud& query_point_cloud, PointCloud& gt_point_cloud,
      const GripperPaths& gripper_paths, const Eigen::Vector3d& direction);

  /**
   * @brief Compute collision metric for a specific z_tolerance of the gripper
   *
   * @param query_point_cloud Query point cloud to evaluate
   * @param gt_point_cloud Ground truth point cloud
   * @param gripper_z_tolerance Z-axis tolerance for collision checking
   * @return std::vector<double> Returns {fp_averaged, fn_averaged} where:
   *         - fp_averaged: False Positives averaged over multiple directions
   *         - fn_averaged: False Negatives averaged over multiple directions
   */
  std::vector<double> ComputeCollisionMetricForZTolerance(
      PointCloud& query_point_cloud, PointCloud& gt_point_cloud,
      double gripper_z_tolerance);

  /**
   * @brief Compute collision metrics for all z_tolerances of the gripper
   *
   * @param query_point_cloud Query point cloud to evaluate
   * @param gt_point_cloud Ground truth point cloud
   * @return std::unordered_map<double, std::unordered_map<std::string, double>>
   * Returns a map of z_tolerances to their corresponding collision metrics
   */
  std::unordered_map<double, std::unordered_map<std::string, double>>
  ComputeCollisionMetrics(open3d::geometry::PointCloud& query_point_cloud,
                          open3d::geometry::PointCloud& gt_point_cloud);

 public:
  std::vector<double> gripper_z_tolerances;
  std::vector<Eigen::Vector3d> directions;
  int outlier_threshold;
  double gripper_size;
  int gripper_step;
};