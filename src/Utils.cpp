// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <vector>
#include <xtensor/xadapt.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xpad.hpp>
#include <xtensor/xview.hpp>

#include "CollisionMetric.h"
#include "GripperPaths.h"

xt::xarray<double> xyToleranceMapGenerator(const xt::xarray<double>& map1,
                                           const xt::xarray<double>& map2) {
  auto shiftedmap1 = xt::pad(map2, {{0, 1}, {0, 1}}, xt::pad_mode::constant,
                             std::numeric_limits<double>::quiet_NaN());
  auto shiftedmap2 = xt::pad(map2, {{1, 0}, {1, 0}}, xt::pad_mode::constant,
                             std::numeric_limits<double>::quiet_NaN());
  auto diff1 = map1 - map2;
  auto diff2 = map1 - xt::view(shiftedmap1, xt::range(0, -1), xt::range(1, _));
  auto diff3 = map1 - xt::view(shiftedmap1, xt::range(1, _), xt::range(0, -1));
  auto diff4 = map1 - xt::view(shiftedmap2, xt::range(0, -1), xt::range(1, _));
  auto diff5 = map1 - xt::view(shiftedmap2, xt::range(1, _), xt::range(0, -1));
  auto xy_tolerance_map =
      xt::stack(xt::xtuple(diff1, diff2, diff3, diff4, diff5), 2);

  return xy_tolerance_map;
}

xt::xarray<double> minOrMaxReduction(const xt::xarray<double>& array,
                                     bool min) {
  int dim_1 = array.size() / array.shape()[2];
  int dim_2 = array.shape()[2];
  auto array_flatten = xt::reshape_view(array, {dim_1, dim_2});

  std::vector<double> reduced_vec;
  for (std::size_t i = 0; i < array_flatten.shape()[0]; ++i) {
    auto row = xt::view(array_flatten, i, xt::all());
    auto has_nan = xt::any(xt::isnan(row));
    if (has_nan) {
      reduced_vec.push_back(std::numeric_limits<double>::quiet_NaN());
    } else {
      double val = *((min) ? std::min_element(row.begin(), row.end())
                           : std::max_element(row.begin(), row.end()));
      reduced_vec.push_back(val);
    }
  }

  return xt::adapt(reduced_vec, {array.shape()[0], array.shape()[1]});
}

CollisionMetricsResults EvaluateCollisionMap(
    xt::xarray<double>& ground_truth_map, xt::xarray<double>& query_map,
    const GripperPaths& gripper, bool incomplete_gt) {
  int total_paths = static_cast<int>(xt::sum(ground_truth_map > 0)());

  query_map = xt::where(xt::isnan(query_map),
                        std::numeric_limits<double>::infinity(), query_map);
  if (!incomplete_gt) {
    ground_truth_map =
        xt::where(xt::isnan(ground_truth_map),
                  std::numeric_limits<double>::infinity(), ground_truth_map);
  }

  auto non_inf_mask = !(xt::isinf(ground_truth_map));
  auto points = xt::argwhere(non_inf_mask);
  std::vector<cv::Point> points_vec;
  for (int i = 0; i < points.size(); i++) {
    auto point = points[i];
    points_vec.push_back(cv::Point(point[0], point[1]));
  }

  std::vector<cv::Point> hull;
  cv::convexHull(points_vec, hull);
  cv::Mat mask = cv::Mat::zeros(
      cv::Size(ground_truth_map.shape()[1], ground_truth_map.shape()[0]),
      CV_8UC1);
  // Fill the convex polygon on the mask
  cv::fillConvexPoly(mask, hull, 1);

  // Custom min/max function ensures that when there is a NaN an array, the min
  // or max is NaN. This is not the default behavior of xtensor, but is for
  // numpy hence the need for a custom function (Looked into using xt::reduce
  // and xt::accumulate, but couldnt get it to work).
  auto max_tbe_gt_diff = minOrMaxReduction(
      xyToleranceMapGenerator(query_map, ground_truth_map), false);
  auto min_tbe_gt_diff = minOrMaxReduction(
      -xyToleranceMapGenerator(ground_truth_map, query_map), true);

  xt::xarray<double> fpmap = (max_tbe_gt_diff < -gripper.z_tolerance);
  xt::xarray<double> fnmap = min_tbe_gt_diff > gripper.z_tolerance;

  double fp = static_cast<double>(xt::sum(fpmap)());
  double fn = static_cast<double>(xt::sum(fnmap)());
  return CollisionMetricsResults(fp, fn, total_paths, fpmap, fnmap);
}