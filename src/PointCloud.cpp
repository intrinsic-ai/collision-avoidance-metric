// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#include "PointCloud.h"

#include <GripperPaths.h>

#include <cmath>
#include <iostream>
#include <optional>
#include <xtensor/xadapt.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xindex_view.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xview.hpp>

using namespace xt::placeholders;

double _DetectCollisionPoint(xt::xarray<double> point_cloud,
                             double gripper_size, int outlier_threshold) {
  auto collision_point = std::numeric_limits<double>::quiet_NaN();
  auto z_points = xt::flatten(point_cloud);
  // Remove NaNs and make them infinity, so sorting works properly and replace
  // them back
  z_points = xt::where(xt::isnan(z_points),
                       std::numeric_limits<double>::infinity(), z_points);
  z_points = xt::sort(z_points);
  z_points = xt::where(xt::isinf(z_points),
                       std::numeric_limits<double>::quiet_NaN(), z_points);
  auto z_points_filtered = xt::filter(z_points, !xt::isnan(z_points));
  if (z_points_filtered.size() <= outlier_threshold) {
    return collision_point;
  }
  auto diff = xt::view(z_points, xt::range(outlier_threshold, _)) -
              xt::view(z_points, xt::range(0, -outlier_threshold));
  auto collisions = diff < gripper_size;
  return z_points(xt::argmax(collisions)(0));
}

PointCloud::PointCloud(open3d::geometry::PointCloud point_cloud_)
    : point_cloud(point_cloud_) {}

void PointCloud::RotateAndUpdateAABB(
    const Eigen::Vector3d &direction,
    const std::optional<open3d::geometry::PointCloud> &check_point_cloud) {
  auto mesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame();
  auto rotation_matrix = mesh->GetRotationMatrixFromXYZ(direction);
  rotated_point_cloud = open3d::geometry::PointCloud(point_cloud);
  rotated_point_cloud.Rotate(rotation_matrix, Eigen::Vector3d(0.0, 0.0, 0.0));
  if (check_point_cloud.has_value()) {
    aabb = check_point_cloud->GetAxisAlignedBoundingBox();
    aabb.min_bound_ = aabb.GetMinBound() - Eigen::Vector3d(0.0, 0.0, 100.0);
    rotated_point_cloud = *rotated_point_cloud.Crop(aabb);
  } else {
    aabb = rotated_point_cloud.GetAxisAlignedBoundingBox();
    aabb.min_bound_ = aabb.GetMinBound() - Eigen::Vector3d(0.0, 0.0, 100.0);
  }
  return;
}

xt::xarray<double> PointCloud::GenerateCollisionMap(const GripperPaths &gripper,
                                                    int outlier_threshold) {
  xt::xarray<double> paths_ = gripper.GeneratePaths(aabb);

  std::array<std::size_t, 2> collision_map_shape = {paths_.shape(0),
                                                    paths_.shape(1)};
  xt::xarray<double> collision_map_ = xt::zeros<double>(collision_map_shape);

  std::vector<Eigen::Vector3d> points_o3d = rotated_point_cloud.points_;
  std::vector<double> flat_buffer;
  for (const auto &vec : points_o3d) {
    flat_buffer.push_back(vec[0]);
    flat_buffer.push_back(vec[1]);
    flat_buffer.push_back(vec[2]);
  }
  std::vector<std::size_t> points_shape = {points_o3d.size(), 3};
  xt::xarray<double> points = xt::adapt(flat_buffer, points_shape);

  xt::xarray<double> aabb_extent = {aabb.GetExtent()[0], aabb.GetExtent()[1],
                                    aabb.GetExtent()[2]};
  xt::xarray<double> abb_min_bound = {
      aabb.GetMinBound()[0], aabb.GetMinBound()[1], aabb.GetMinBound()[2]};
  xt::xarray<double> voxel_grid =
      xt::zeros<double>({aabb_extent[1] + 1, aabb_extent[0] + 1});

  xt::xarray<double> points_rounded;
  points_rounded = xt::round(points - abb_min_bound);
  auto third_column = xt::col(points_rounded, 2);
  auto sorted_indices = xt::flip(xt::argsort(third_column), {0});
  // Reorder the rows based on sorted indices
  xt::xarray<double> sorted_points =
      xt::view(points_rounded, xt::keep(sorted_indices), xt::all());
  xt::xarray<int> x_indices = xt::col(sorted_points, 0);
  xt::xarray<int> y_indices = xt::col(sorted_points, 1);
  xt::xarray<double> z_values = xt::col(sorted_points, 2);
  // Update voxel_grid at the given (y, x) coordinates of the sorted points with
  // z values
  using index_type = std::array<int, 2>;
  std::vector<index_type> indices;
  for (int i = 0; i < x_indices.size(); i++) {
    indices.push_back({y_indices[i], x_indices[i]});
  }
  auto voxel_grid_view = xt::index_view(voxel_grid, indices);
  voxel_grid_view = z_values;
  voxel_grid = xt::where(xt::equal(voxel_grid, 0.0),
                         std::numeric_limits<double>::quiet_NaN(), voxel_grid);

  // iterate through the paths
  for (int y = 0; y < paths_.shape(0); y++) {
    for (int x = 0; x < paths_.shape(1); x++) {
      int min_x =
          std::round(paths_(y, x, 1) - abb_min_bound(1) - gripper.size / 2);
      int max_x =
          std::round(paths_(y, x, 1) - abb_min_bound(1) + gripper.size / 2);
      int min_y =
          std::round(paths_(y, x, 0) - abb_min_bound(0) - gripper.size / 2);
      int max_y =
          std::round(paths_(y, x, 0) - abb_min_bound(0) + gripper.size / 2);
      xt::xarray<double> pcd_crop = xt::view(
          voxel_grid, xt::range(min_x, max_x), xt::range(min_y, max_y));
      double collision_point =
          _DetectCollisionPoint(pcd_crop, gripper.size, outlier_threshold);
      collision_map_(y, x) = collision_point;
    }
  }

  collision_map_ =
      xt::where(xt::equal(collision_map_, 0.0),
                std::numeric_limits<double>::quiet_NaN(), collision_map_);
  collision_map = collision_map_;
  paths = paths_;
  return collision_map;
}