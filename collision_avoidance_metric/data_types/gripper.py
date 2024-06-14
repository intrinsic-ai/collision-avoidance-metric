import numpy as np
import open3d as o3d


class GripperPaths():

    def __init__(self, size: float, step: float, z_tolerance: float):
        """Initialize GripperPaths

        Args:
            size (float): size of the gripper used in mm. Default 10.0mm.
            step (float): step for each gripper movement along the passed directions. In mm.
            z_tolerance (float): z tolerance.
        """
        self.size = size
        self.step = step
        self.z_tolerance = z_tolerance

    def generate_paths(
            self,
            axis_aligned_bounding_box: o3d.geometry.AxisAlignedBoundingBox
        ) -> np.ndarray:
        """ Generate the gripper paths.

        Args:
            axis_aligned_bounding_box: open3d.geometry.AxisAlignedBoundingBox, the bounding box
                of the point cloud.
        Returns:
            np.array, the gripper paths defined as the center point of the gripper in
                each descent path
        """
        # generate the gripper paths
        min_bound = axis_aligned_bounding_box.get_min_bound()
        max_bound = axis_aligned_bounding_box.get_max_bound()
        init_point = np.array([min_bound[0], min_bound[1]]) + self.size / 2

        x = np.linspace(
            init_point[0],
            max_bound[0],
            np.int16(np.ceil((max_bound[0] - init_point[0]) / self.step))
        )
        y = np.linspace(
            init_point[1],
            max_bound[1],
            np.int16(np.ceil((max_bound[1] - init_point[1]) / self.step))
        )
        paths = np.zeros((len(y), len(x), 2))
        paths[:, :, 0], paths[:, :, 1] = np.meshgrid(x, y)
        return paths
