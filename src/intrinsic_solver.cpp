//
// Created by vahagn on 1/10/21.
//

#include "intrinsic_solver.h"
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include "vertex_camera.h"
namespace g2o_learning {

void IntrinsicSolver::Calbirate(const std::vector<std::vector<Eigen::Vector2d>> & points,
                                const std::vector<std::vector<Eigen::Vector2d>> & original_points) {
  assert(points.size() == original_points.size());
  std::vector<Matx33d> homographies(points.size());
  for (int i = 0; i < points.size(); ++i) {
    std::vector<Eigen::Vector2d> pts = {points[i][0], points[i][7], points[i][12 * 8], points[i][12 * 8 + 4]};
    std::vector<Eigen::Vector2d> pts1 =
        {original_points[i][0], original_points[i][7], original_points[i][12 * 8], original_points[i][12 * 8 + 4]};
    Find3HomographyFromPlanar4Points(pts, pts1, homographies[i]);
  }
  VertexCamera::EstimateType camera_matrix_params_estimate;
  double lambda = GetCameraMatrixInitialEstimate(homographies, camera_matrix_params_estimate);

  // Camera Matrix
  VertexCamera * camera_params = new VertexCamera();
  camera_params->setId(0);
  camera_params->setEstimate(camera_matrix_params_estimate);
  optimizer_.addVertex(camera_params);

  //
  Matx33d K;
  K << camera_matrix_params_estimate[0], 0, camera_matrix_params_estimate[2],
      0, camera_matrix_params_estimate[1], camera_matrix_params_estimate[3],
      0, 0, 1;

  std::cout << "K = " << K << std::endl;
  Matx33d Kinv = K.inverse();

  for (size_t measurement_id = 0; measurement_id < points.size(); ++measurement_id) {

    g2o::VertexSE3Expmap * rot_matrix_mu = new g2o::VertexSE3Expmap();
    rot_matrix_mu->setId(2 * measurement_id + 1);
    Matx33d rotation_matrix_mu;
    Eigen::Vector3d translation_vector_mu;


    ComputeRotationMatrix(homographies[measurement_id], Kinv, rotation_matrix_mu, translation_vector_mu);
    Eigen::Quaterniond rquat(rotation_matrix_mu);

    // TODO: initialize rotation matrix

    g2o::VertexPointXYZ * translation_vector = new g2o::VertexPointXYZ();
    translation_vector->setId(2 * measurement_id + 2);
    // TODO: initizlize translation vector

  }
}

void IntrinsicSolver::ComputeRotationMatrix(const IntrinsicSolver::Matx33d & h,
                                            const IntrinsicSolver::Matx33d & Kinv,
                                            IntrinsicSolver::Matx33d & out_rotM,
                                            Eigen::Vector3d & out_T) {
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

//  std::cout << "Rotation matrix = " << out_rotM << std::endl;
//  std::cout << "R R^T = " << out_rotM.transpose() * out_rotM << std::endl;
//  std::cout << "T = " << out_T << std::endl;

}

double IntrinsicSolver::GetCameraMatrixInitialEstimate(const std::vector<Matx33d> & homographies,
                                                       Eigen::VectorXd & out_projection_matrix) const {

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

  out_projection_matrix.resize(9, Eigen::NoChange);
  out_projection_matrix << fx, fy, cx, cy, 0, 0, 0, 0, 0;

  std::cout << "lambda = " << lambda << std::endl;
  std::cout << "cx = " << cx << std::endl;
  std::cout << "cy = " << cy << std::endl;
  std::cout << "fx = " << fx << std::endl;
  std::cout << "fy = " << fy << std::endl;

  return lambda;

}

void IntrinsicSolver::Find3HomographyFromPlanar4Points(const std::vector<Eigen::Vector2d> & points_to,
                                                       const std::vector<Eigen::Vector2d> & points_from,
                                                       Matx33d & out_homography) const {

  if (points_to.size() < 4 || points_from.size() < 4)
    throw std::runtime_error("FindHomographyFromFirst4Points: there should be at least 4 point correspondences. ");

  const Eigen::Vector2d & U0 = points_to[0];
  const Eigen::Vector2d & X0 = points_from[0];

  const Eigen::Vector2d & U1 = points_to[1];
  const Eigen::Vector2d & X1 = points_from[1];

  const Eigen::Vector2d & U2 = points_to[2];
  const Eigen::Vector2d & X2 = points_from[2];

  const Eigen::Vector2d & U3 = points_to[3];
  const Eigen::Vector2d & X3 = points_from[3];
  Eigen::Matrix<double, 8, 9> L;

  L << X0[0], X0[1], 1, 0, 0, 0, -U0[0] * X0[0], -U0[0] * X0[1], -U0[0],
      0, 0, 0, X0[0], X0[1], 1, -U0[1] * X0[0], -U0[1] * X0[1], -U0[1],
      X1[0], X1[1], 1, 0, 0, 0, -U1[0] * X1[0], -U1[0] * X1[1], -U1[0],
      0, 0, 0, X1[0], X1[1], 1, -U1[1] * X1[0], -U1[1] * X1[1], -U1[1],
      X2[0], X2[1], 1, 0, 0, 0, -U2[0] * X2[0], -U2[0] * X2[1], -U2[0],
      0, 0, 0, X2[0], X2[1], 1, -U2[1] * X2[0], -U2[1] * X2[1], -U2[1],
      X3[0], X3[1], 1, 0, 0, 0, -U3[0] * X3[0], -U3[0] * X3[1], -U3[0],
      0, 0, 0, X3[0], X3[1], 1, -U3[1] * X3[0], -U3[1] * X3[1], -U3[1];

  Eigen::JacobiSVD<Eigen::Matrix<double, 8, 9>> svd(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
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