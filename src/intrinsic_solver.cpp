//
// Created by vahagn on 1/10/21.
//

#include "intrinsic_solver.h"
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include "vertex_camera.h"
namespace g2o_learning {

void IntrinsicSolver::Calbirate(const std::vector<std::vector<Eigen::Vector2d>> &points,
                                const std::vector<std::vector<Eigen::Vector2d>> &original_points) {
  assert(points.size() == original_points.size());
  std::vector<Matx33d> homographies(points.size());
  for (int i = 0; i < points.size(); ++i) {
    Find3HomographyFromPlanar4Points(points[i], original_points[i], homographies[i]);
  }
  Matx33d projection_matrix;
  GetEstimate(homographies, projection_matrix);

  VertexCamera *camera_params = new VertexCamera();
  camera_params->setId(0);
  // TODO:: Initialize camera_matrix
  // camera_params->setEstimate()
  optimizer_.addVertex(camera_params);

  for (size_t measurement_id = 0; measurement_id < points.size(); ++measurement_id) {
    g2o::VertexSE3Expmap *rot_matrix_mu = new g2o::VertexSE3Expmap();
    rot_matrix_mu->setId(2 * measurement_id + 1);
    // TODO: initialize rotation matrix

    g2o::VertexPointXYZ *translation_vector = new g2o::VertexPointXYZ();
    translation_vector->setId(2 * measurement_id + 2);
    // TODO: initizlize translation vector

  }
}

void IntrinsicSolver::GetEstimate(const std::vector<Matx33d> &homographies,
                                  Matx33d &projection_matrix) const {
  // Separate rotation and projection matrixes
  Eigen::Matrix<double, 6, 6> v;
  v.resize(6, Eigen::NoChange);
  for (int i = 0; i < 3; ++i) {
    const Matx33d &h = homographies[i];

    std::cout << h << std::endl << std::endl;
    v(2 * i, 0) = h(0, 0) * h(1, 0);
    v(2 * i, 1) = h(0, 0) * h(1, 1) + h(0, 1) * h(1, 0);
    v(2 * i, 2) = h(0, 1) * h(1, 1);
    v(2 * i, 3) = h(0, 2) * h(1, 0) + h(0, 0) * h(1, 2);
    v(2 * i, 4) = h(0, 2) * h(1, 1) + h(0, 1) * h(1, 2);
    v(2 * i, 5) = h(0, 2) * h(1, 2);

    v(2 * i + 1, 0) = h(0, 0) * h(0, 0) - h(1, 1) * h(1, 1);
    v(2 * i + 1, 1) = 2 * h(0, 0) * h(0, 1) - 2 * h(1, 0) * h(1, 1);
    v(2 * i + 1, 2) = h(0, 1) * h(0, 1) - h(1, 1) * h(1, 1);
    v(2 * i + 1, 3) = 2 * h(0, 2) * h(0, 0) - 2 * h(1, 0) * h(1, 2);
    v(2 * i + 1, 4) = 2 * h(0, 2) * h(0, 1) - 2 * h(1, 2) * h(1, 1);
    v(2 * i + 1, 5) = h(0, 2) * h(0, 2) - h(1, 2) * h(1, 2);
  }



  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(v, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix<double, 6, 6> &right_singular_matrix = svd.matrixV();
  for (int j = 0; j < 6; ++j) {
    std::cout << right_singular_matrix(5, j) / right_singular_matrix(5, 5)  << std::endl;
  }

  //(B 01 B 02 − B 00 B 12 )/(B 00 B 11 − B 01
  //)
  std::cout << (right_singular_matrix(5, 1) * right_singular_matrix(5, 2)  - right_singular_matrix(5, 0) * right_singular_matrix(5, j))
//  std::cout << right_singular_matrix(0, 5) << " " << right_singular_matrix(1, 5) << " " << right_singular_matrix(2, 5)
//            << " " << right_singular_matrix(3, 5) << " " << right_singular_matrix(4, 5) << " "
//            << right_singular_matrix(5, 5) << std::endl;

}

void IntrinsicSolver::Find3HomographyFromPlanar4Points(const std::vector<Eigen::Vector2d> &points_to,
                                                       const std::vector<Eigen::Vector2d> &points_from,
                                                       Matx33d &out_homography) const {

  if (points_to.size() < 4 || points_from.size() < 4)
    throw std::runtime_error("FindHomographyFromFirst4Points: there should be at least 4 point correspondences. ");

  Eigen::Matrix<double, 8, 9> L;

  const Eigen::Vector2d &U0 = points_to[0];
  const Eigen::Vector2d &X0 = points_from[0];

  const Eigen::Vector2d &U1 = points_to[1];
  const Eigen::Vector2d &X1 = points_from[1];

  const Eigen::Vector2d &U2 = points_to[2];
  const Eigen::Vector2d &X2 = points_from[2];

  const Eigen::Vector2d &U3 = points_to[3];
  const Eigen::Vector2d &X3 = points_from[3];

  L << X0[0], X0[1], 1, 0, 0, 0, -U0[0] * X0[0], -U0[0] * X0[1], -U0[0],
      0, 0, 0, X0[0], X0[1], 1, -U0[1] * X0[0], -U0[1] * X0[1], -U0[1],
      X1[0], X1[1], 1, 0, 0, 0, -U1[0] * X1[0], -U1[0] * X1[1], -U1[0],
      0, 0, 0, X1[0], X1[1], 1, -U1[1] * X1[0], -U1[1] * X1[1], -U1[1],
      X2[0], X2[1], 1, 0, 0, 0, -U2[0] * X2[0], -U2[0] * X2[1], -U2[0],
      0, 0, 0, X2[0], X2[1], 1, -U2[1] * X2[0], -U2[1] * X2[1], -U2[1],
      X3[0], X3[1], 1, 0, 0, 0, -U3[0] * X3[0], -U3[0] * X3[1], -U3[0],
      0, 0, 0, X3[0], X3[1], 1, -U3[1] * X3[0], -U3[1] * X3[1], -U3[1];

  Eigen::JacobiSVD<Eigen::Matrix<double, 8, 9>> svd(L, Eigen::ComputeFullU | Eigen::ComputeFullV);

  const Eigen::Matrix<double, 9, 9> &right_singluar_values = svd.matrixV();

  out_homography << right_singluar_values(0, 8), right_singluar_values(1, 8), right_singluar_values(2, 8),
      right_singluar_values(3, 8), right_singluar_values(4, 8), right_singluar_values(5, 8),
      right_singluar_values(6, 8), right_singluar_values(7, 8), right_singluar_values(8, 8);
}

int IntrinsicSolver::FindIntrinsicParameters() {
  return 0;
}

}