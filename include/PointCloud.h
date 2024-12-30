// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <Eigen/Core>
#include <memory>
#include <optional>
#include <vector>

#include "GripperPaths.h"
#include "open3d/geometry/PointCloud.h"
#include "xtensor/xarray.hpp"

/**
 * @brief A class to handle point cloud operations for collision detection
 *
 * This class wraps Open3D point cloud functionality and provides additional
 * methods for collision detection and manipulation. It supports various input
 * formats and provides methods for rotation, collision map generation, and
 * bounding box calculations.
 */
class PointCloud {
 public:
  /**
   * @brief Construct a new Point Cloud object
   *
   * @param point_cloud Input point cloud from Open3D
   */
  PointCloud(open3d::geometry::PointCloud point_cloud_);

  /**
   * @brief Rotate point cloud and update its axis-aligned bounding box
   *
   * @param direction Direction vector for rotation
   * @param check_point_cloud Optional second point cloud to ensure bounding box
   * contains both clouds
   */
  void RotateAndUpdateAABB(
      const Eigen::Vector3d& direction,
      const std::optional<open3d::geometry::PointCloud>& check_point_cloud);

  /**
   * @brief Generate a collision map for the point cloud
   *
   * Creates a 2D collision map representing z-axis coordinates of collision
   * points for each gripper path. Uses voxel grid for efficient point cloud
   * cropping.
   *
   * @param gripper Gripper configuration for collision detection
   * @param outlier_threshold Number of points to be considered as outliers (not
   * causing collision)
   * @return xt::xarray<double> 2D array containing collision points'
   * z-coordinates
   */
  xt::xarray<double> GenerateCollisionMap(const GripperPaths& gripper,
                                          int outlier_threshold);

 public:
  /// Original point cloud data
  open3d::geometry::PointCloud point_cloud;
  /// Rotated version of the point cloud after applying transformation
  open3d::geometry::PointCloud rotated_point_cloud;
  /// Axis-aligned bounding box for the point cloud
  open3d::geometry::AxisAlignedBoundingBox aabb;
  /// Collision map for the point cloud
  xt::xarray<double> collision_map;
  /// Gripper paths for collision detection
  xt::xarray<double> paths;
};

#endif  // POINT_CLOUD_H