# Copyright (c) Intrinsic Innovation LLC
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


import numpy as np
import open3d as o3d
import sys

from collision_avoidance_metric import collision_metric_evaluator

if __name__ == "__main__":
    assert len(sys.argv) == 3, "Please pass query and gt point cloud paths"
    query = sys.argv[1]
    gt = sys.argv[2]
    evaluator = collision_metric_evaluator.CollisionAvoidanceMetric(
        gripper_z_tolerances=[10.0],
        directions=[[0, 0, 0]],
        outlier_threshold=5,
    )

    # Example usage with path to file input.
    collision_metrics = evaluator.compute_collision_metrics(
        query_point_cloud=query,
        gt_point_cloud=gt,
    )
    print(f"\n{collision_metrics}")

    # Example usage with open3D Point Cloud.
    query_o3d = o3d.io.read_point_cloud(query)
    gt_o3d = o3d.io.read_point_cloud(gt)
    collision_metrics = evaluator.compute_collision_metrics(
        query_point_cloud=query_o3d,
        gt_point_cloud=gt_o3d,
    )
    print(f"\n{collision_metrics}")

    # Example usage with numpy array.
    query_array = np.array(query_o3d.points)
    gt_array = np.array(gt_o3d.points)
    collision_metrics = evaluator.compute_collision_metrics(
        query_point_cloud=query_array,
        gt_point_cloud=gt_array,
    )
    print(f"\n{collision_metrics}")
