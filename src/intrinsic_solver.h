//
// Created by vahagn on 1/10/21.
//

#ifndef G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_
#define G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_

#include <g2o/core/sparse_optimizer.h>

namespace g2o_learning {

class IntrinsicSolver {
 public:
  IntrinsicSolver(const std::vector<std::vector<Eigen::Vector2d>> & points,
                  const std::vector<std::vector<Eigen::Vector3d>> & original_points);
  int FindIntrinsicParameters();
 private:
  g2o::SparseOptimizer optimizer_;
};

}

#endif //G2O_BA_TEST_SRC_INTRINSIC_SOLVER_H_
