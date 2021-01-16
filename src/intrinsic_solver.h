//
// Created by vahagn on 1/10/21.
//

#ifndef G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_
#define G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_

#include <g2o/core/sparse_optimizer.h>

namespace g2o_learning {

class IntrinsicSolver {
 public:
  typedef Eigen::Matrix<double, 3, 3> Matx33d;
  IntrinsicSolver() = default;
  int FindIntrinsicParameters();
  void Calbirate(const std::vector<std::vector<Eigen::Vector2d>> &points,
                 const std::vector<std::vector<Eigen::Vector2d>> &original_points);


  void Find3HomographyFromPlanar4Points(const std::vector<Eigen::Vector2d> &points_to,
                                        const std::vector<Eigen::Vector2d> &points_from,
                                        Matx33d &out_homography) const;
 private:
  void GetEstimate(const std::vector<Matx33d> &homographies,
                   Matx33d &projection_matrix) const;
  g2o::SparseOptimizer optimizer_;
};

}

#endif //G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_
