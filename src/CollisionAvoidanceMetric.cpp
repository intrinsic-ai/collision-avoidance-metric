// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#include "CollisionAvoidanceMetric.h"

#include "CollisionMetric.h"
#include "PointCloud.h"
#include "Utils.h"

CollisionAvoidanceMetric::CollisionAvoidanceMetric(
    std::vector<double> gripper_z_tolerances,
    std::vector<Eigen::Vector3d> directions, int outlier_threshold,
    double gripper_size, int gripper_step)
    : gripper_z_tolerances(gripper_z_tolerances),
      directions(directions),
      outlier_threshold(outlier_threshold),
      gripper_size(gripper_size),
      gripper_step(gripper_step) {}

CollisionMetricsResults
CollisionAvoidanceMetric::ComputeCollisionForGripperDirection(
    PointCloud &query_point_cloud, PointCloud &gt_point_cloud,
    const GripperPaths &gripper, const Eigen::Vector3d &direction) {
  gt_point_cloud.RotateAndUpdateAABB(direction, std::nullopt);
  query_point_cloud.RotateAndUpdateAABB(direction,
                                        gt_point_cloud.rotated_point_cloud);
  xt::xarray<double> gt_collision_map =
      gt_point_cloud.GenerateCollisionMap(gripper, outlier_threshold);
  xt::xarray<double> query_collision_map =
      query_point_cloud.GenerateCollisionMap(gripper, outlier_threshold);
  auto collision_metrics_results = EvaluateCollisionMap(
      gt_collision_map, query_collision_map, gripper, false);
  return collision_metrics_results;
}

std::vector<double>
CollisionAvoidanceMetric::ComputeCollisionMetricForZTolerance(
    PointCloud &query_point_cloud, PointCloud &gt_point_cloud,
    double gripper_z_tolerance) {
  std::vector<double> results;
  GripperPaths gripper_paths(gripper_size, gripper_step, gripper_z_tolerance);
  double fp_final = 0;
  double fn_final = 0;
  double total_paths_final = 0;
  for (int i = 0; i < directions.size(); i++) {
    Eigen::Vector3d direction = directions[i];
    CollisionMetricsResults collision_metrics =
        ComputeCollisionForGripperDirection(query_point_cloud, gt_point_cloud,
                                            gripper_paths, direction);
    fp_final += collision_metrics.fp;
    fn_final += collision_metrics.fn;
    total_paths_final += collision_metrics.total_paths;
  }
  double fp_averaged = fp_final / total_paths_final;
  double fn_averaged = fn_final / total_paths_final;
  return {fp_averaged, fn_averaged};
}

std::unordered_map<double, std::unordered_map<std::string, double>>
CollisionAvoidanceMetric::ComputeCollisionMetrics(
    open3d::geometry::PointCloud &query_point_cloud,
    open3d::geometry::PointCloud &gt_point_cloud) {
  std::unordered_map<double, std::unordered_map<std::string, double>>
      collision_metrics;
  PointCloud query_point_cloud_obj(query_point_cloud);
  PointCloud gt_point_cloud_obj(gt_point_cloud);
  for (int j = 0; j < gripper_z_tolerances.size(); j++) {
    double gripper_z_tolerance = gripper_z_tolerances[j];
    std::vector<double> results = ComputeCollisionMetricForZTolerance(
        query_point_cloud_obj, gt_point_cloud_obj, gripper_z_tolerance);
    collision_metrics[gripper_z_tolerance] = {
        {"FP", results[0]},
        {"FN", results[1]},
    };
  }
  return collision_metrics;
}