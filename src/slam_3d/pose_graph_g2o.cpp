
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
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace pose_graph_utils {

G2O_USE_OPTIMIZATION_LIBRARY(csparse)

/**
 * @brief constructor
 */
PoseGraphG2O::PoseGraphG2O() {
  graph.reset(new g2o::SparseOptimizer());

  std::string g2o_solver_name = "lm_var";
  
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(g2o_solver_name, solver_property);
  graph->setAlgorithm(solver);

  if (!graph->solver()) {
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
  graph.reset();
}

g2o::VertexSE3* PoseGraphG2O::addSe3Node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(graph->vertices().size());
  vertex->setEstimate(pose);

  if (graph->vertices().size() == 0)
    vertex->setFixed(true);

  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexPointXYZ* PoseGraphG2O::addPointXyzNode(const Eigen::Vector3d& xyz) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(graph->vertices().size());
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);

  return vertex;
}

g2o::EdgeSE3* PoseGraphG2O::addSe3Edge(
    g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PointXYZ* PoseGraphG2O::addSe3PointXyzEdge(
    g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  graph->addEdge(edge);

  return edge;
}

void PoseGraphG2O::optimize(const bool& verbose) {
  if (graph->edges().size() < 1) {
    return;
  }

  graph->initializeOptimization();
  graph->setVerbose(verbose);

  double initial_chi2 = graph->chi2();

  const auto t_start = std::chrono::high_resolution_clock::now();
  int iterations = graph->optimize(1024);
  const auto t_end = std::chrono::high_resolution_clock::now();

  if (verbose) {
    std::cout << "--- Pose graph optimization ---" << std::endl;
    std::cout << " Nodes: " << graph->vertices().size() << "   edges: " << graph->edges().size() << std::endl;
    std::cout << " Iterations: " << iterations << std::endl;
    std::cout << " Chi2: (before) " << initial_chi2 << "  ->  (after) " << graph->chi2() << std::endl;
    const auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    std::cout << "  Time: " << boost::format("%.8f") % time_elapsed << "[sec]" << std::endl;
  }
}

void PoseGraphG2O::save(const std::string& filename) {
  std::ofstream ofs(filename);
  graph->save(ofs);
}

}  // namespace pose_graph_utils
