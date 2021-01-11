//
// Created by vahagn on 1/10/21.
//

#include "intrinsic_solver.h"
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include "vertex_camera.h"
namespace g2o_learning {

IntrinsicSolver::IntrinsicSolver(const std::vector<std::vector<Eigen::Vector2d>> & points,
                                 const std::vector<std::vector<Eigen::Vector3d>> & original_points) {
  assert(points.size() == original_points.size());

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

int IntrinsicSolver::FindIntrinsicParameters() {
  return 0;
}

}