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

class VertexCamera : public g2o::BaseVertex<9, Eigen::VectorXd> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexCamera() = default;

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

  void oplusImpl(const double *update) override {
    Eigen::VectorXd::ConstMapType v(update, VertexCamera::Dimension);
    _estimate += v;
  }

 public:
  Eigen::Vector2d map(const Eigen::Vector3d & vector) {
    const double & fx = _estimate[0];
    const double & fy = _estimate[1];
    const double & cx = _estimate[2];
    const double & cy = _estimate[3];
    const double & k1 = _estimate[4];
    const double & k2 = _estimate[5];
    const double & p1 = _estimate[6];
    const double & p2 = _estimate[7];
    const double & k3 = _estimate[8];

    Eigen::Vector2d result;
    double z_inv = 1 / vector[2];
    double x = vector[0] * z_inv;
    double y = vector[1] * z_inv;

    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;

    double cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
    double a1 = 2 * x * y;


    double xd = x * cdist +  p1 * a1 + p2 * (r2 + 2 * x * x);
    double yd = y * cdist +  p2 * a1 + p1 * (r2 + 2 * y * y);

    result[0] = xd * fx + cx;
    result[1] = yd * fy + cy;
    return result;
  }
};

}
#endif //G2O_BA_TEST_SRC_VERTEX_CAMERA_H_
