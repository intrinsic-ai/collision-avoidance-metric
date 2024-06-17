import dataclasses

import numpy as np
import cv2

from collision_avoidance_metric.data_types import gripper
from collision_avoidance_metric.data_types import collision_metric


def _xy_tolerance_map_generator(map1: np.ndarray, map2: np.ndarray):
    """Generate the collison point difference map with displacements of one collision map against another one.

    Args:
        map1 (np.array): the fixed map, MxN
        map2 (np.array): the shifted map, MxN
    Returns:
        xy_tolerance_map (np.array): the map of the difference between the collision points, MxNx5.
    """
    shiftedmap1 = np.pad(
        map2,
        ((0, 1), (0, 1)),
        mode='constant',
        constant_values=np.nan
    )
    shiftedmap2 = np.pad(
        map2,
        ((1, 0), (1, 0)),
        mode='constant',
        constant_values=np.nan
    )

    xy_tolerance_map = np.stack(
        (
            map1 - map2,
            map1 - shiftedmap1[0:-1, 1:],
            map1 - shiftedmap1[1:, 0:-1],
            map1 - shiftedmap2[0:-1, 1:],
            map1 - shiftedmap2[1:, 0:-1]
        ),
        axis=2
    )
    return xy_tolerance_map


def evaluate_collision_map(
        ground_truth_map: np.ndarray,
        query_map: np.ndarray,
        gripper: gripper.GripperPaths,
        incomplete_gt=False
    ):
    """Evaluate the collision map given the ground truth and query collision maps.

    Args:
        ground_truth_map (np.array): the ground truth point cloud collision map.
        query_map (np.array): the query point cloud collision map.
        gripper (Gripper): the gripper object.
    Returns:
        collision_metric.CollisionMetricsResults
    """

    total_paths = np.sum(ground_truth_map > 0)

    query_map[np.isnan(query_map)] = np.inf
    if not incomplete_gt:
        ground_truth_map[np.isnan(ground_truth_map)] = np.inf

    y_coords, x_coords = np.where(~np.isinf(ground_truth_map))
    points = np.array(list(zip(x_coords, y_coords)))

    hull = cv2.convexHull(points)
    mask = np.zeros_like(ground_truth_map)
    cv2.fillConvexPoly(mask, hull, 1)

    max_tbe_gt_diff = np.max(
        _xy_tolerance_map_generator(query_map, ground_truth_map),
        axis=2,
    )
    min_tbe_gt_diff = np.min(
        -_xy_tolerance_map_generator(ground_truth_map, query_map),
        axis=2
    )

    fpmap = (max_tbe_gt_diff < -gripper.z_tolerance) * mask
    fpmap = fpmap.astype(np.bool_)
    fnmap = min_tbe_gt_diff > gripper.z_tolerance

    fp = np.sum(fpmap)
    fn = np.sum(fnmap)
    return collision_metric.CollisionMetricsResults(fp, fn, total_paths, fpmap, fnmap)
