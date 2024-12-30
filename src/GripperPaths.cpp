// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#include "GripperPaths.h"

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <xtensor/xarray.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xeval.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

GripperPaths::GripperPaths(double size, double step, double z_tolerance)
    : size(size), step(step), z_tolerance(z_tolerance) {}

xt::xarray<double> GripperPaths::GeneratePaths(
    const open3d::geometry::AxisAlignedBoundingBox& axis_aligned_bounding_box)
    const {
  auto min_bound = axis_aligned_bounding_box.GetMinBound();
  auto max_bound = axis_aligned_bounding_box.GetMaxBound();
  xt::xarray<double> init_point = {min_bound[0] + size / 2.0,
                                   min_bound[1] + size / 2.0};

  auto x = xt::linspace<double>(
      init_point(0), max_bound(0),
      static_cast<size_t>(std::ceil((max_bound(0) - init_point(0)) / step)));
  auto y = xt::linspace<double>(
      init_point(1), max_bound(1),
      static_cast<size_t>(std::ceil((max_bound(1) - init_point(1)) / step)));

  auto [X, Y] = xt::meshgrid(x, y);
  std::array<std::size_t, 3> shape = {Y.shape(0), X.shape(1), 2};
  xt::xarray<double> paths = xt::zeros<double>(shape);
  // Assign meshgrid values to paths
  xt::view(paths, xt::all(), xt::all(), 0) = X;
  xt::view(paths, xt::all(), xt::all(), 1) = Y;

  return paths;
}
