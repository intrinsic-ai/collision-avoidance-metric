import copy
import os
from typing import Optional, Union

import numpy as np
import open3d as o3d

from collision_avoidance_metric.data_types import gripper


PointCloudAllowedTypes = Union[str, np.ndarray, o3d.geometry.PointCloud]


def _detect_collision_point(
        point_cloud: np.ndarray,
        gripper_size: float,
        outlier_threshold: int
    ) -> float:
    """Detect the collision point between point cloud and gripper.
    Args:
        point_cloud (np.array): point cloud in the defined gripper direction.
        size (float): the size of the gripper
        outlier_threshold: int, the number of points to be considered as outliers.
    Returns:
        float, the collision coordinate in z-axis.
    """
    collision_point = np.nan
    z_points = point_cloud.flatten()
    z_points = np.sort(z_points)
    if len(z_points[~np.isnan(z_points)]) <= outlier_threshold:
        return collision_point
    diff = z_points[outlier_threshold:] - z_points[:-outlier_threshold]
    collisions = diff < gripper_size
    return z_points[np.argmax(collisions)]


class PointCloud:

    def __init__(self, point_cloud: PointCloudAllowedTypes):
        """Initialize PointCloud.

        Args:
            point_cloud (str, np.ndarray, o3d.geometry.PointCloud): input point cloud.

        Raises:
            ValueError: If numpy array point cloud does not have the right shape. Allowed shapes are
                [N, 6] or [N, 3].
            ValueError: If path to point cloud does not exist.
        """
        if isinstance(point_cloud, np.ndarray):
            self.point_cloud_array = point_cloud
            if self.point_cloud_array.shape[1] not in [3, 6]:
                raise ValueError(f"Point Cloud should be of shape [N, 6] or [N, 3], got {self.point_cloud_array.shape}")

            self.point_cloud = o3d.geometry.PointCloud()
            self.point_cloud.points = o3d.utility.Vector3dVector(
                copy.deepcopy(self.point_cloud_array[:, :3])
            )
        elif isinstance(point_cloud, str):
            if not os.path.exists(point_cloud):
                raise ValueError(f"Please pass a valid path. {point_cloud} does not exists.")
            self.point_cloud = o3d.io.read_point_cloud(point_cloud)
            self.point_cloud_array = np.array(self.point_cloud.points)
        else:
            self.point_cloud_array = np.array(point_cloud.points)
            self.point_cloud = point_cloud
        self.rotated_point_cloud = None
        self.abb = None

    def rotate_and_update_aabb(
            self,
            direction,
            check_point_cloud: Optional[o3d.geometry.PointCloud] = None,
        ):
        """Rotate the point cloud and update the axis aligned bounding box.

        Args:
            check_point_cloud: open3d.geometry.PointCloud, the second point cloud. It is important
                to ensure that the bounding box is containing both point clouds.
            direction: np.array, the direction of gripper
        """
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        rotation_matrix = mesh.get_rotation_matrix_from_xyz(direction)
        self.rotated_point_cloud = copy.deepcopy(self.point_cloud).rotate(rotation_matrix, center=(0, 0, 0))
        if check_point_cloud is not None:
            self.aabb = check_point_cloud.get_axis_aligned_bounding_box()
            self.aabb.min_bound = self.aabb.min_bound - np.array([0, 0, 100])
            self.rotated_point_cloud = self.rotated_point_cloud.crop(self.aabb)
        else:
            self.aabb = self.rotated_point_cloud.get_axis_aligned_bounding_box()
            self.aabb.min_bound = self.aabb.min_bound - np.array([0, 0, 100])

    def generate_collision_map(self, gripper: gripper.GripperPaths, outlier_threshold: int) -> np.ndarray:
        """ Generate the collision map.

        Args:
            gripper (Gripper): the gripper object to be used.
            outlier_threshold (int): Outlier threshold represents the number of points to be considered
                as outliers, so not causing collision to 5.
        Returns:
            np.array, the collision map - the z-axis coordinate of the collision point for each gripper path
        """
        # generate the paths
        paths = gripper.generate_paths(self.aabb)

        collision_map = np.zeros((paths.shape[0], paths.shape[1]))

        points = np.array(self.rotated_point_cloud.points)

        # generate a 2D voxel grid for faster point cloud cropping
        aabb_extent = self.aabb.get_extent()

        abb_min_bound = self.aabb.get_min_bound()
        voxel_grid = np.zeros(
            (
                np.round(aabb_extent[1]).astype(int) + 1,
                np.round(aabb_extent[0]).astype(int) + 1
            )
        )

        points_rounded = np.round(points - abb_min_bound).astype(int)
        points_rounded = points_rounded[points_rounded[:, 2].argsort()[::-1]]
        voxel_grid[points_rounded[:, 1], points_rounded[:, 0]] = points_rounded[:, 2]
        voxel_grid[voxel_grid == 0] = np.nan

        # iterate through the paths
        for y in range(paths.shape[0]):
            for x in range(paths.shape[1]):
                # crop the point cloud that is within the gripper path
                min_x = np.round(
                    paths[y, x, 1] - abb_min_bound[1] - gripper.size / 2
                ).astype(int)
                max_x = np.round(
                    paths[y, x, 1] - abb_min_bound[1] + gripper.size / 2
                ).astype(int)
                min_y = np.round(
                    paths[y, x, 0] - abb_min_bound[0] - gripper.size / 2
                ).astype(int)
                max_y = np.round(
                    paths[y, x, 0] - abb_min_bound[0] + gripper.size / 2
                ).astype(int)
                pcd_crop = voxel_grid[min_x:max_x, min_y:max_y]

                # compute the collision point for each path; collision point is
                # defined as the first moment of intersection of N points,
                # where N > outlier_threshold
                collision_point = _detect_collision_point(pcd_crop, gripper.size, outlier_threshold)
                collision_map[y, x] = collision_point

        collision_map[collision_map == 0] = np.nan
        self.collision_map = collision_map
        self.paths = paths
        return collision_map
