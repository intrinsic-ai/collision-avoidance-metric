// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#ifndef UTILS_H
#define UTILS_H

/**
 * @brief Generates a tolerance map for XY coordinates
 *
 * @param map1 First input map to compare
 * @param map2 Second input map to compare
 * @return xt::xarray<double> A map containing tolerance values for XY
 * coordinates
 */
xt::xarray<double> xyToleranceMapGenerator(const xt::xarray<double>& map1,
                                           const xt::xarray<double>& map2);

/**
 * @brief Evaluates collision metrics between ground truth and query maps
 *
 * @param ground_truth_map Reference map representing ground truth collision
 * states
 * @param query_map Query map to evaluate against ground truth
 * @param gripper Gripper paths configuration for collision checking
 * @param incomplete_gt Flag indicating if ground truth map is incomplete
 * @return CollisionMetricsResults Contains:
 *         - fp: False positive collision count
 *         - fn: False negative collision count
 *         - total_paths: Total number of paths evaluated
 */
CollisionMetricsResults EvaluateCollisionMap(
    xt::xarray<double>& ground_truth_map, xt::xarray<double>& query_map,
    const GripperPaths& gripper, bool incomplete_gt);

#endif  // UTILS_H
