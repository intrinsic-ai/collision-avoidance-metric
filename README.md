![alt text](images/logo.png)

# Collision Avoidance Metrics for 3D Camera Evaluation (CPP)

This repository contains the implementation of our paper:
> **Collision Avoidance Metric for 3D Camera Evaluation**\
> Vage Taamazyan, Alberto Dall'olio, Agastya Kalra \
> CVPR Workshop 2024 (Vision and Language for Autonomous Driving and Robotics - VLADR - **Oral**)

**[Paper](https://arxiv.org/pdf/2405.09755)  | [ArXiv](https://arxiv.org/abs/2405.09755) | [BibTeX](#citation)** 
![alt text](images/collision_metric_viz.png)


## Abstract
3D cameras have emerged as a critical source of information for applications in robotics and autonomous driving. These cameras provide robots with the ability to capture and utilize point clouds, enabling them to navigate their surroundings and avoid collisions with other objects. However, current standard camera evaluation metrics often fail to consider the specific application context. These metrics typically focus on measures like Chamfer distance (CD) or Earth Mover’s Distance (EMD), which may not directly translate to performance in real-world scenarios. To address this limitation, we propose a novel metric for point cloud evaluation, specifically designed to assess the suitability of 3D cameras for the critical task of collision avoidance. This metric incorporates application-specific considerations and provides a more accurate measure of a camera's effectiveness in ensuring safe robot navigation.

## This Repo
This repo contains the implementation of the Collision Avoidance Metrics. Example usage can be found in [example.py](example.py).

Example usage:

```sh
    python3 example.py <PATH-TO-QUERY-POINTCLOUD> <PATH-TO-GT-POINTCLOUD>
```

### Requirements and setup

- Numpy
- Open3D
- OpenCV

With Pyhthon 3.10 you can run:
```
    pip install -r requirements.txt
```

## Citation
If you want to cite our paper:

```
@misc{taamazyan2024collision,
      title={Collision Avoidance Metric for 3D Camera Evaluation}, 
      author={Vage Taamazyan and Alberto Dall'olio and Agastya Kalra},
      year={2024},
      eprint={2405.09755},
      archivePrefix={arXiv},
      primaryClass={id='cs.CV' full_name='Computer Vision and Pattern Recognition' is_active=True alt_name=None in_archive='cs' is_general=False description='Covers image processing, computer vision, pattern recognition, and scene understanding. Roughly includes material in ACM Subject Classes I.2.10, I.4, and I.5.'}
}
```

### Owners
- vage@google.com
- dallolio@google.com