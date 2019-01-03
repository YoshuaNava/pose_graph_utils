#pragma once

// standard lib
#include <memory>

// eigen
#include <Eigen/Dense>

// g2o
#include <g2o/core/sparse_optimizer.h>

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
// class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
}  // namespace g2o

namespace pose_graph_utils {

class PoseGraphG2O {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PoseGraphG2O>;

  PoseGraphG2O();
  ~PoseGraphG2O();

  /**
   * @brief add a SE3 node to the graph
   * @param pose
   * @return registered node
   */
  g2o::VertexSE3* addSe3Node(const Eigen::Isometry3d& pose);

  /**
   * @brief add a point_xyz node to the graph
   * @param xyz
   * @return registered node
   */
  g2o::VertexPointXYZ* addPointXyzNode(const Eigen::Vector3d& xyz);

  /**
   * @brief add an edge between SE3 nodes
   * @param v1  node1
   * @param v2  node2
   * @param relative_pose  relative pose between node1 and node2
   * @param information_matrix  information matrix (it must be 6x6)
   * @return registered edge
   */
  g2o::EdgeSE3* addSe3Edge(
      g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief add an edge between an SE3 node and a point_xyz node
   * @param v_se3        SE3 node
   * @param v_xyz        point_xyz node
   * @param xyz          xyz coordinate
   * @param information  information_matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3PointXYZ* addSe3PointXyzEdge(
      g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief perform graph optimization
   */
  void optimize(const bool& verbose = false);

  /**
   * @brief save the pose graph
   * @param filename  output filename
   */
  void save(const std::string& filename);

 public:
  std::unique_ptr<g2o::SparseOptimizer> optimizer;  // g2o graph
};

}  // namespace pose_graph_utils
