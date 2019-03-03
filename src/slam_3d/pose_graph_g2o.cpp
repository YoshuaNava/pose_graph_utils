
#include "slam_3d/pose_graph_g2o.hpp"

// standard library
#include <chrono>

// boost
#include <boost/format.hpp>

// g2o
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>

// Slam 3D types
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace pose_graph_utils {

G2O_USE_OPTIMIZATION_LIBRARY(csparse)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)

/**
 * @brief constructor
 */
PoseGraphG2O::PoseGraphG2O() {
  optimizer.reset(new g2o::SparseOptimizer());

  std::string g2o_solver_name = "lm_var";

  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(g2o_solver_name, solver_property);
  optimizer->setAlgorithm(solver);

  if (!optimizer->solver()) {
    std::cerr << std::endl;
    std::cerr << "Error: g2o failed to allocate solver!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cin.ignore(1);
    return;
  }
}

/**
 * @brief destructor
 */
PoseGraphG2O::~PoseGraphG2O() {
  optimizer.reset();
}

g2o::VertexSE3* PoseGraphG2O::addSe3Node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(optimizer->vertices().size());
  vertex->setEstimate(pose);

  if (optimizer->vertices().size() == 0) {
    vertex->setFixed(true);
  }

  optimizer->addVertex(vertex);

  return vertex;
}

g2o::VertexPointXYZ* PoseGraphG2O::addPointXyzNode(const Eigen::Vector3d& xyz) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(optimizer->vertices().size());
  vertex->setEstimate(xyz);
  optimizer->addVertex(vertex);

  return vertex;
}

g2o::EdgeSE3* PoseGraphG2O::addSe3Edge(
    g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3* edge = new g2o::EdgeSE3();
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  optimizer->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PointXYZ* PoseGraphG2O::addSe3PointXyzEdge(
    g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  optimizer->addEdge(edge);

  return edge;
}

void PoseGraphG2O::optimize(const bool& verbose) {
  if (optimizer->edges().size() < 1) {
    return;
  }

  optimizer->setVerbose(verbose);
  optimizer->verifyInformationMatrices();
  optimizer->initializeOptimization();

  double initial_chi2 = optimizer->chi2();

  const auto t_start = std::chrono::high_resolution_clock::now();
  int iterations = optimizer->optimize(100);
  const auto t_end = std::chrono::high_resolution_clock::now();

  if (verbose) {
    std::cout << "--- Pose optimizer optimization ---" << std::endl;
    std::cout << " Nodes: " << optimizer->vertices().size() << "   edges: " << optimizer->edges().size() << std::endl;
    std::cout << " Iterations: " << iterations << std::endl;
    std::cout << " Chi2: (before) " << initial_chi2 << "  ->  (after) " << optimizer->chi2() << std::endl;
    const auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    std::cout << "  Time: " << boost::format("%.8f") % time_elapsed << "[sec]" << std::endl;
  }
}

void PoseGraphG2O::save(const std::string& filename) {
  std::ofstream ofs(filename);
  optimizer->save(ofs);
}

}  // namespace pose_graph_utils
