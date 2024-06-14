import dataclasses
import numpy as np


@dataclasses.dataclass
class CollisionMetricsResults:
    """Results for the collision metric computation.

    Args:
        fp (float): False Positive collisions.
        fn (float): False Negative collisions.
        fpmap (np.ndarray): False Positive collisions map
            for each gripper paths.
        fnmap (np.ndarray): False Negative collisions map
            for each gripper paths.

    """
    fp: float
    fn: float
    total_paths: int
    fpmap: np.ndarray
    fnmap: np.ndarray
