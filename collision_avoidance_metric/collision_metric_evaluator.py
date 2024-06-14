from typing import Dict, List, Tuple, Union

from collision_avoidance_metric import utils
from collision_avoidance_metric.data_types import point_cloud
from collision_avoidance_metric.data_types import gripper


class CollisionAvoidanceMetric():
    def __init__(
        self,
        gripper_z_tolerances: Union[float, List[float]] = 10.0,
        directions: List[Tuple[float, float, float]] = [(0, 0, 0)],
        outlier_threshold: int = 5,
        gripper_size: float = 10.0,
        gripper_step: int = 5.0,
    ):
        """Initialize CollisionAvoidanceMetric.

        Args:
            gripper_z_tolerances: (Union[float, List[float]], optional): List of gripper z
                tolerances to be used in mm. The output of compute_collision_metrics will be a dictionary
                containing false positives and false negatives collision for each gripper tolerance.
                Defaults to 10.0.
            directions (List[Tuple[float, float, float]], optional): List of directions to be used
                to calculate the collisions of the gripper with the point cloud.
                Defaults to [(0, 0, 0)] namely perpendicular to the pointcloud x-y plane.
            outlier_threshold (int, optional): Outlier threshold represents the
                number of points to be considered as outliers, so not causing collision to 5.
            gripper_size (float): size of the gripper used in mm. Default 10.0mm.
            gripper_step (float): step for each gripper movement along the passed directions. In mm.
                Default 5.0mm
        """
        self.gripper_z_tolerances = gripper_z_tolerances
        if isinstance(self.gripper_z_tolerances, int):
            self.gripper_z_tolerances = [gripper_z_tolerances]
        self.directions = directions
        self.outlier_threshold = outlier_threshold
        self.gripper_size = gripper_size
        self.gripper_step = gripper_step

    def _compute_collision_for_gripper_direction(
            self,
            query_point_cloud: point_cloud.PointCloud,
            gt_point_cloud: point_cloud.PointCloud,
            gripper: gripper.GripperPaths,
            direction: Tuple[float, float, float],
        ) -> utils.CollisionMetricsResults:
        """Compute collision metric for a specific gripper direction.

        Args:
            query_point_cloud (point_cloud.PointCloud): query point cloud.
            gt_point_cloud (point_cloud.PointCloud): ground truth point cloud.
            gripper (gripper.GripperPaths): gripper to be used for metrics calculation.
            direction: direction of the gripper.

        Returns:
            Results of the collision metrics computation.
        """
        gt_point_cloud.rotate_and_update_aabb(direction)
        query_point_cloud.rotate_and_update_aabb(direction, gt_point_cloud.rotated_point_cloud)
        gt_collision_map = gt_point_cloud.generate_collision_map(gripper, self.outlier_threshold)
        query_collision_map = query_point_cloud.generate_collision_map(gripper, self.outlier_threshold)
        collision_metrics_results = utils.evaluate_collision_map(
            gt_collision_map,
            query_collision_map,
            gripper,
            incomplete_gt=False
        )
        return collision_metrics_results

    def _compute_collision_metric_for_z_tolerance(
            self,
            query_point_cloud: point_cloud.PointCloud,
            gt_point_cloud: point_cloud.PointCloud,
            gripper_z_tolerance: float
        ) -> Tuple[float, float]:
        """Compute collision metric for a specific z_tolerance of the gripper.

        Args:
            query_point_cloud: query point cloud.
            gt_point_cloud: ground truth point cloud.
        Returns:
            False Positives and False Negatives collisions averaged over multiple directions.
        """
        gripper_paths = gripper.GripperPaths(
            size=self.gripper_size,
            step=self.gripper_step,
            z_tolerance=gripper_z_tolerance
        )
        fp_final = 0
        fn_final = 0
        total_paths_final = 0
        for direction in self.directions:
            eval_results = self._compute_collision_for_gripper_direction(
                gt_point_cloud=gt_point_cloud,
                query_point_cloud=query_point_cloud,
                gripper=gripper_paths,
                direction=direction,
            )
            fp_final += eval_results.fp
            fn_final += eval_results.fn
            total_paths_final += eval_results.total_paths
        fp_averaged = fp_final / total_paths_final
        fn_averaged = fn_final / total_paths_final
        return fp_averaged, fn_averaged

    def compute_collision_metrics(
        self,
        query_point_cloud: point_cloud.PointCloudAllowedTypes,
        gt_point_cloud: point_cloud.PointCloudAllowedTypes,
        ) -> Dict[str, Dict[str, float]]:
        """Compute collision metrics for given query and ground truth point cloud.

        Args:
            query_point_cloud (str, np.ndarray, o3d.geometry.PointCloud): query point cloud to
                be evaluated. Can be a path to a .ply file, a [N, 3] or [N, 6] numpy array or an
                open3D PointCloud.
            gt_point_cloud (str, np.ndarray, o3d.geometry.PointCloud): ground truth point cloud to
                compare. Can be a path to a .ply file, a [N, 3] or [N, 6] numpy array or an
                open3D PointCloud.

        Returns:
            Dictionary that has the following data:
                - Keys: keys are the different gripper_z_tolerances set at initialization time.
                - Values: values are dictionaries contaning two "FP" and "FN" keys.
        """
        assert type(query_point_cloud) == type(gt_point_cloud), "Query and GT point cloud should be of the same type."
        gt_point_cloud = point_cloud.PointCloud(gt_point_cloud)
        query_point_cloud = point_cloud.PointCloud(query_point_cloud)
        collision_metrics = {}
        for gripper_z_tolerance in self.gripper_z_tolerances:
            false_positives, false_negatives = self._compute_collision_metric_for_z_tolerance(
                query_point_cloud,
                gt_point_cloud,
                gripper_z_tolerance,

            )
            collision_metrics[gripper_z_tolerance] = {}
            collision_metrics[gripper_z_tolerance]["FP"] = false_positives
            collision_metrics[gripper_z_tolerance]["FN"] = false_negatives
        return collision_metrics
