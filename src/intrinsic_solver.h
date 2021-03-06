//
// Created by vahagn on 1/10/21.
//

#ifndef G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_
#define G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_

#include <g2o/core/sparse_optimizer.h>
#include "vertex_camera.h"
#include "edge_error.h"

namespace g2o_learning {

//const int DistortionCoefficientSize   = 5;
#define DistortionCoefficientSize    5

typedef VertexCamera<DistortionCoefficientSize> TVertexCamera;
typedef EdgeError<DistortionCoefficientSize> TEdgeError;
typedef VertexCamera<DistortionCoefficientSize>::DistCoeffs_t TDistortionCoefficients;

class IntrinsicSolver {
 public:
  typedef Eigen::Matrix<double, 3, 3> Matx33d;

  IntrinsicSolver() = default;
  int FindIntrinsicParameters();
  void Calbirate(const std::vector<std::vector<Eigen::Vector2d>> & points,
                 const std::vector<std::vector<Eigen::Vector2d>> & original_points,
                 Matx33d & out_intrinsic_matrix,
                 TVertexCamera::DistCoeffs_t & out_distortion_coefficients) const;

 private:
  void Find3HomographyFromPlanar4Points(const std::vector<Eigen::Vector2d> & points_to,
                                        const std::vector<Eigen::Vector2d> & points_from,
                                        Matx33d & out_homography) const;
  void ComputeRotationMatrix(const Matx33d & homogrpaphy,
                             const Matx33d & Kinv,
                             Matx33d & out_rotM,
                             Eigen::Vector3d & out_T) const;
  void GetCameraMatrixInitialEstimate(const std::vector<Matx33d> & homographies,
                                      TVertexCamera::Estimate_t & out_projection_matrix) const;

};

}

#endif //G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_
