import dataclasses
import numpy as np


@dataclasses.dataclass
class CollisionMetricsResults:
    fp: float
    fn: float
    total_paths: int
    fpmap: np.ndarray
    fnmap: np.ndarray
