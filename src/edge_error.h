//
// Created by vahagn on 1/17/21.
//

#ifndef G2O_BA_TEST_SRC_EDGE_ERROR_H_
#define G2O_BA_TEST_SRC_EDGE_ERROR_H_


#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/sba/vertex_se3_expmap.h>

namespace g2o_learning {

template<int CameraDistortionCoefficientSize>
class EdgeError : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCamera<CameraDistortionCoefficientSize>, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeError() {
    this->error()[0] = this->error()[1] = 0;
  }
 public:


  void computeError() override {

    const Eigen::Vector3d
        transformed = dynamic_cast<g2o::VertexSE3Expmap *>(this->vertex(1))->estimate().map(pattern_point_);

    this->error() = dynamic_cast<VertexCamera<CameraDistortionCoefficientSize> *>(this->vertex(0))->map(transformed) - this->_measurement;



  }
  bool read(std::istream & is) override {
    assert(!"EdgeError: Read is not implemented yet");
    return false;
  }
  bool write(std::ostream & os) const override {
    assert(!"EdgeError: Write is not implemented yet");
    return false;
  }

  void setOriginalPoint(const Eigen::Vector2d & pattern_point) {
    pattern_point_[0] = pattern_point[0];
    pattern_point_[1] = pattern_point[1];
    pattern_point_[2] = 0;
  }
 private:
  Eigen::Vector3d pattern_point_;
};
}
#endif //G2O_BA_TEST_SRC_EDGE_ERROR_H_
