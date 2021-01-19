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

template<int DistCoeffsLength>
class VertexCamera : public g2o::BaseVertex<DistCoeffsLength + 4, Eigen::VectorXd> {
 public:
  typedef Eigen::Matrix<double, DistCoeffsLength + 4, 1> Estimate_t;
  typedef Eigen::Matrix<double, DistCoeffsLength, 1> DistCoeffs_t;
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
    this->_estimate += v;
  }

 public:
  Eigen::Vector2d map(const Eigen::Vector3d & vector) {
    const double & fx =  this->_estimate[0];
    const double & fy =  this->_estimate[1];
    const double & cx =  this->_estimate[2];
    const double & cy =  this->_estimate[3];

    Eigen::Vector2d result;
    double z_inv = 1 / vector[2];
    double x = vector[0] * z_inv;
    double y = vector[1] * z_inv;

    double xd, yd;
    ComputeDistortion(x, y, xd, yd);

    result[0] = xd * fx + cx;
    result[1] = yd * fy + cy;
    return result;
  }

 private:
  void ComputeDistortion(const double x, const double y, double & xd, double & yd);
};



}
#endif //G2O_BA_TEST_SRC_VERTEX_CAMERA_H_
