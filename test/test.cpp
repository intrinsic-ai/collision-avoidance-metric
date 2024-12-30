#include "CollisionAvoidanceMetric.h"

int main(int argc, char** argv) {
  CollisionAvoidanceMetric evaluator({10}, {Eigen::Vector3d(0.0, 0.0, 0.0)},
                                     5.0, 10.0, 5.0);
  auto query_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
  auto gt_point_cloud = std::make_shared<open3d::geometry::PointCloud>();

  // Read the PLY file
  open3d::io::ReadPointCloud(
      "/home/sujay/Code/collision-avoidance-metric/test/data_qp.ply",
      *query_point_cloud);
  open3d::io::ReadPointCloud(
      "/home/sujay/Code/collision-avoidance-metric/test/data_gt.ply",
      *gt_point_cloud);
  auto collision_metrics =
      evaluator.ComputeCollisionMetrics(*query_point_cloud, *gt_point_cloud);
  for (const auto& outer_pair : collision_metrics) {
    std::cout << outer_pair.first << "\n";
    for (const auto& inner_pair : outer_pair.second) {
      std::cout << "   " << inner_pair.first << " : " << inner_pair.second
                << "\n";
    }
  }

  return 0;
}
