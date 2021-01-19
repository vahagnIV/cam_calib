//
// Created by vahagn on 1/10/21.
//

#include "intrinsic_solver.h"
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>


namespace g2o_learning {

void IntrinsicSolver::Calbirate(const std::vector<std::vector<Eigen::Vector2d>> & points,
                                const std::vector<std::vector<Eigen::Vector2d>> & original_points,
                                Matx33d & out_intrinsic_matrix,
                                TVertexCamera::DistCoeffs_t & out_distortion_coefficients) const {

  g2o::SparseOptimizer optimizer;

  assert(points.size() == original_points.size());
  std::vector<Matx33d> homographies(points.size());
  for (int i = 0; i < points.size(); ++i) {
    Find3HomographyFromPlanar4Points(points[i], original_points[i], homographies[i]);
  }
  TVertexCamera::Estimate_t camera_matrix_params_estimate;
  GetCameraMatrixInitialEstimate(homographies, camera_matrix_params_estimate);

  // Camera Matrix
  TVertexCamera *camera_params = new TVertexCamera();
  camera_params->setId(0);
  camera_params->setEstimate(camera_matrix_params_estimate);
  optimizer.addVertex(camera_params);

  Matx33d K;
  K << camera_matrix_params_estimate[0], 0, camera_matrix_params_estimate[2],
      0, camera_matrix_params_estimate[1], camera_matrix_params_estimate[3],
      0, 0, 1;

  Matx33d Kinv = K.inverse();
  for (size_t measurement_id = 0; measurement_id < points.size(); ++measurement_id) {
    Matx33d rotation_matrix_mu;
    Eigen::Vector3d translation_vector_mu;

    ComputeRotationMatrix(homographies[measurement_id], Kinv, rotation_matrix_mu, translation_vector_mu);
    g2o::SE3Quat pose(rotation_matrix_mu, translation_vector_mu);
    pose.normalizeRotation();

    g2o::VertexSE3Expmap *pose_exp_map_mu = new g2o::VertexSE3Expmap();
    pose_exp_map_mu->setId(measurement_id + 1);
    pose_exp_map_mu->setEstimate(pose);

    optimizer.addVertex(pose_exp_map_mu);

    for (int i = 0; i < points[i].size(); ++i) {
      TEdgeError *e = new TEdgeError();
      e->setOriginalPoint(original_points[measurement_id][i]);
      e->setMeasurement(points[measurement_id][i]);
      e->setVertex(0, camera_params);
      e->setVertex(1, pose_exp_map_mu);
      e->setInformation(Eigen::Matrix2d::Identity());
      e->setId((measurement_id + 1) * points.size() + i + 2);
      if (!optimizer.addEdge(e))
        throw std::runtime_error("Could not add edge");
    }
  }


  // I do not really get what am I doing here
  typedef g2o::BlockSolverX BalBlockSolver;
  typedef g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType> BalLinearSolver;
  std::unique_ptr<g2o::LinearSolver<BalBlockSolver::PoseMatrixType>> linearSolver;
  auto cholesky = g2o::make_unique<BalLinearSolver>();
  cholesky->setBlockOrdering(true);
  linearSolver = std::move(cholesky);
  g2o::OptimizationAlgorithmLevenberg *optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BalBlockSolver>(std::move(linearSolver)));

//  optimizer.setVerbose(true);
  optimizer.setAlgorithm(optimizationAlgorithm);

  optimizer.initializeOptimization();
  optimizer.optimize(1000);

  out_distortion_coefficients
      << camera_params->estimate()[4], camera_params->estimate()[5], camera_params->estimate()[6], camera_params->estimate()[7],
      camera_params->estimate()[8], camera_params->estimate()[9], camera_params->estimate()[10], camera_params->estimate()[11];

  out_intrinsic_matrix
      << camera_params->estimate()[0], 0, camera_params->estimate()[2], 0, camera_params->estimate()[1], camera_params->estimate()[3], 0, 0, 1;

}

void IntrinsicSolver::ComputeRotationMatrix(const IntrinsicSolver::Matx33d & h,
                                            const IntrinsicSolver::Matx33d & Kinv,
                                            IntrinsicSolver::Matx33d & out_rotM,
                                            Eigen::Vector3d & out_T) const {
  Eigen::Vector3d h1, h2, h3, r1, r2, r3;

  h1 << h(0, 0), h(1, 0), h(2, 0);
  h2 << h(0, 1), h(1, 1), h(2, 1);
  h3 << h(0, 2), h(1, 2), h(2, 2);

  r1 = Kinv * h1;
  double norm = std::sqrt(r1.dot(r1));
  r1 = r1 / norm;

  r2 = Kinv * h2 / norm;
  r3 = r1.cross(r2);

  out_rotM << r1[0], r2[0], r3[0],
      r1[1], r2[1], r3[1],
      r1[2], r2[2], r3[2];

  out_T = Kinv * h3 / norm;
}

void IntrinsicSolver::GetCameraMatrixInitialEstimate(const std::vector<Matx33d> & homographies,
                                                     TVertexCamera::Estimate_t & out_projection_matrix) const {

  Eigen::Matrix<double, 6, 6> v;
  v.resize(6, Eigen::NoChange);
  for (int i = 0; i < 3; ++i) {
    const Matx33d & h = homographies[i];

    v(2 * i, 0) = h(0, 0) * h(0, 1);
    v(2 * i, 1) = h(0, 1) * h(1, 0) + h(0, 0) * h(1, 1);
    v(2 * i, 2) = h(1, 0) * h(1, 1);
    v(2 * i, 3) = h(0, 1) * h(2, 0) + h(0, 0) * h(2, 1);
    v(2 * i, 4) = h(2, 0) * h(1, 1) + h(1, 0) * h(2, 1);
    v(2 * i, 5) = h(2, 0) * h(2, 1);

    v(2 * i + 1, 0) = h(0, 0) * h(0, 0) - h(0, 1) * h(0, 1);
    v(2 * i + 1, 1) = 2 * h(0, 0) * h(1, 0) - 2 * h(0, 1) * h(1, 1);
    v(2 * i + 1, 2) = h(1, 0) * h(1, 0) - h(1, 1) * h(1, 1);
    v(2 * i + 1, 3) = 2 * h(2, 0) * h(0, 0) - 2 * h(0, 1) * h(2, 1);
    v(2 * i + 1, 4) = 2 * h(1, 0) * h(2, 0) - 2 * h(1, 1) * h(2, 1);
    v(2 * i + 1, 5) = h(2, 0) * h(2, 0) - h(2, 1) * h(2, 1);
  }

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(v, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix<double, 6, 6> & right_singular_matrix = svd.matrixV();

  const double b11 = right_singular_matrix(0, 5);
  const double b12 = right_singular_matrix(1, 5);
  const double b22 = right_singular_matrix(2, 5);
  const double b13 = right_singular_matrix(3, 5);
  const double b23 = right_singular_matrix(4, 5);
  const double b33 = right_singular_matrix(5, 5);

  double cy = (b12 * b13 - b11 * b23) / (b11 * b22 - b12 * b12);
  double lambda = std::abs(b33 - (b13 * b13 + cy * (b12 * b13 - b11 * b23)) / b11);
  double fx = std::sqrt(lambda / b11);
  double fy = std::sqrt(lambda / b22);
  double cx = -b13 * fx * fx / lambda;


  out_projection_matrix.setZero();

  out_projection_matrix[0] = fx;
  out_projection_matrix[1] = fy;
  out_projection_matrix[2] = cx;
  out_projection_matrix[3] = cy;
  std::cout << "lambda = " << lambda << std::endl;
  std::cout << "cx = " << cx << std::endl;
  std::cout << "cy = " << cy << std::endl;
  std::cout << "fx = " << fx << std::endl;
  std::cout << "fy = " << fy << std::endl;

}

void IntrinsicSolver::Find3HomographyFromPlanar4Points(const std::vector<Eigen::Vector2d> & points_to,
                                                       const std::vector<Eigen::Vector2d> & points_from,
                                                       Matx33d & out_homography) const {

  if (points_to.size() < 4 || points_from.size() < 4)
    throw std::runtime_error("FindHomographyFromFirst4Points: there should be at least 4 point correspondences. ");
  Eigen::Matrix<double, Eigen::Dynamic, 9> L;
  L.resize(points_to.size() * 2, Eigen::NoChange);

  for (size_t i = 0; i < points_to.size(); ++i) {
    const Eigen::Vector2d & U = points_to[i];
    const Eigen::Vector2d & X = points_from[i];

    L(2 * i, 0) = X[0];
    L(2 * i, 1) = X[1];
    L(2 * i, 2) = 1;
    L(2 * i, 3) = 0;
    L(2 * i, 4) = 0;
    L(2 * i, 5) = 0;
    L(2 * i, 6) = -U[0] * X[0];
    L(2 * i, 7) = -U[0] * X[1];
    L(2 * i, 8) = -U[0];

    L(2 * i + 1, 0) = 0;
    L(2 * i + 1, 1) = 0;
    L(2 * i + 1, 2) = 0;
    L(2 * i + 1, 3) = X[0];
    L(2 * i + 1, 4) = X[1];
    L(2 * i + 1, 5) = 1;
    L(2 * i + 1, 6) = -U[1] * X[0];
    L(2 * i + 1, 7) = -U[1] * X[1];
    L(2 * i + 1, 8) = -U[1];
  }

  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.computeV())
    throw std::runtime_error("Could not estimate homography");

  const Eigen::Matrix<double, 9, 9> & v = svd.matrixV();

  out_homography << v(0, 8), v(1, 8), v(2, 8),
      v(3, 8), v(4, 8), v(5, 8),
      v(6, 8), v(7, 8), v(8, 8);
  out_homography /= out_homography(2, 2);
}

int IntrinsicSolver::FindIntrinsicParameters() {
  return 0;
}

}