// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#ifndef COLLISION_METRIC_H
#define COLLISION_METRIC_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <xtensor/xarray.hpp>

/**
 * @brief Struct to store results for collision metric computation.
 */
struct CollisionMetricsResults {
  double fp;                 /// False Positive collisions
  double fn;                 /// False Negative collisions
  int total_paths;           /// Total gripper paths
  xt::xarray<double> fpmap;  /// False Positive collisions map
  xt::xarray<double> fnmap;  /// False Negative collisions map

  /**
   * @brief Constructor to initialize the struct.
   *
   * @param fp False Positive collisions
   * @param fn False Negative collisions
   * @param total_paths Total gripper paths
   * @param fpmap False Positive collisions map
   * @param fnmap False Negative collisions map
   */
  CollisionMetricsResults(double fp, double fn, int total_paths,
                          xt::xarray<double> fpmap, xt::xarray<double> fnmap)
      : fp(fp), fn(fn), total_paths(total_paths), fpmap(fpmap), fnmap(fnmap) {}
};

#endif  // COLLISION_METRIC_H