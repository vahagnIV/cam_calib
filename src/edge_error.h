//
// Created by vahagn on 1/17/21.
//

#ifndef G2O_BA_TEST_SRC_EDGE_ERROR_H_
#define G2O_BA_TEST_SRC_EDGE_ERROR_H_

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include "vertex_camera.h"

namespace g2o_learning {

class EdgeError : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCamera, g2o::VertexSE3Expmap> {
 public:
  EdgeError(const Eigen::Vector2d & pattern_point) : pattern_point_(pattern_point[0], pattern_point[1], 1) {}
 public:
  void computeError() override {

    const Eigen::Vector3d
        transformed = dynamic_cast<g2o::VertexSE3Expmap *>(this->vertex(1))->estimate().map(pattern_point_);
    _error = dynamic_cast<VertexCamera *>(this->vertex(0))->map(transformed) - _measurement;
  }
  bool read(std::istream & is) override {
    assert(!"EdgeError: Read is not implemented yet");
    return false;
  }
  bool write(std::ostream & os) const override {
    assert(!"EdgeError: Write is not implemented yet");
    return false;
  }
 private:
  Eigen::Vector3d pattern_point_;
};
}
#endif //G2O_BA_TEST_SRC_EDGE_ERROR_H_
