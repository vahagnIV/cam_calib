//
// Created by vahagn on 1/10/21.
//

#ifndef G2O_BA_TEST_SRC_VERTEX_CAMERA_H_
#define G2O_BA_TEST_SRC_VERTEX_CAMERA_H_

#include <g2o/core/base_vertex.h>

namespace g2o_learning {
/*!
 * Represents the camera matrix with 9 parameters
 * f_x, f_y, c_x, x_y - optical parameters
 * k_1, k_2, p_1, p_2, k_3 - distortion parameters
 */

class VertexCameraBAL : public g2o::BaseVertex<9, Eigen::VectorXd> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexCameraBAL() = default;

  bool read(std::istream & /*is*/) override {
    assert(!"Read is not Implemented yet");
    return false;
  }

  bool write(std::ostream & /*os*/) const override {
    assert(!"Write is not Implemented yet");
    return false;
  }

  void setToOriginImpl() override {
    assert(!"Set to Origin is not Implemented yet");
  }

  void oplusImpl(const double * update) override {
    Eigen::VectorXd::ConstMapType v(update, VertexCameraBAL::Dimension);
    _estimate += v;
  }
};

}
#endif //G2O_BA_TEST_SRC_VERTEX_CAMERA_H_
