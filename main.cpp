// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <Eigen/StdVector>
#include <iostream>
#include <stdint.h>

#include <unordered_set>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

using namespace Eigen;
using namespace std;

class Sample {
 public:
  static int uniform(int from, int to);
  static double uniform();
  static double gaussian(double sigma);
};

static double uniform_rand(double lowerBndr, double upperBndr) {
  return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

static double gauss_rand(double mean, double sigma) {
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

int Sample::uniform(int from, int to) {
  return static_cast<int>(uniform_rand(from, to));
}

double Sample::uniform() {
  return uniform_rand(0., 1.);
}

double Sample::gaussian(double sigma) {
  return gauss_rand(0., sigma);
}

const int NUMBER_OF_3D_POINTS = 500;
const int NUMBER_OF_POSES = 15;
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
const double FOCAL_LENGTH = 1000.;
const double PRINCIPAL_POINT_X = 320;
const double PRINCIPAL_POINT_Y = 240;

double PIXEL_NOISE;
double OUTLIER_RATIO;
bool ROBUST_KERNEL;

g2o::CameraParameters *GetCameraParameters() {

  Vector2d principal_point(PRINCIPAL_POINT_X, PRINCIPAL_POINT_Y);

  return new g2o::CameraParameters(FOCAL_LENGTH, principal_point, 0.);
}

void GenerateTruePoints(vector<Eigen::Vector3d> &out_true_points) {
  for (size_t i = 0; i < NUMBER_OF_3D_POINTS; ++i) {
    out_true_points.push_back(Vector3d((Sample::uniform() - 0.5) * 3,
                                       Sample::uniform() - 0.5,
                                       Sample::uniform() + 3));
  }
}

typedef vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > TruePoses_t;

/*!
 * Fills the argument vector with poses.
 * In current implementation simply translates the camera along the x axis
 * @param true_poses
 * @return
 */
int GenerateTruePoses(TruePoses_t &true_poses) {
  for (size_t i = 0; i < NUMBER_OF_POSES; ++i) {
    Eigen::Quaterniond q;
    q.setIdentity();

    Vector3d translation(i * 0.04 - 1., 0, 0);

    g2o::SE3Quat pose(q, translation);

    true_poses.push_back(pose);
  }
  return NUMBER_OF_POSES;
}

void AddPosesToOptimizer(const TruePoses_t &true_poses, g2o::SparseOptimizer &optimizer) {
  for (size_t vertex_id = 0; vertex_id < true_poses.size(); ++vertex_id) {
    g2o::VertexSE3Expmap *v_se3 = new g2o::VertexSE3Expmap();
    v_se3->setId(vertex_id);
    if (vertex_id < 2) {
      v_se3->setFixed(true);
    }
    v_se3->setEstimate(true_poses[vertex_id]);
    optimizer.addVertex(v_se3);
  }
}

Eigen::Vector3d DistortPoint3D(const Eigen::Vector3d &true_point) {
  return true_point
      + Eigen::Vector3d(Sample::gaussian(1),
                        Sample::gaussian(1),
                        Sample::gaussian(1));
}

Eigen::Vector2d DistortPoint2D(const Eigen::Vector2d &point, double noise) {
  return point
      + Eigen::Vector2d(Sample::gaussian(noise),
                        Sample::gaussian(noise));
}

bool IsValidPoint(const Vector2d &z) {
  return z[0] >= 0 && z[1] >= 0 && z[0] < IMAGE_WIDTH && z[1] < IMAGE_HEIGHT;
}

int ProjectPointOnPoses(const g2o::CameraParameters *camera_parameters,
                        const Eigen::Vector3d &point_3d,
                        const TruePoses_t &poses,
                        std::vector<Eigen::Vector2d> &out_projected_points) {
  int num_obs = 0;
  for (const g2o::SE3Quat &pose: poses) {
    out_projected_points.push_back(camera_parameters->cam_map(pose.map(point_3d)));
    if (IsValidPoint(out_projected_points.back()))
      ++num_obs;
  }
  return num_obs;
}

void InitializeEstimates(const std::vector<Eigen::Vector3d> &true_points,
                         const TruePoses_t &true_poses,
                         const g2o::CameraParameters *camera_parameters,
                         g2o::SparseOptimizer &optimizer,
                         unordered_map<int, int> &pointid_2_trueid,
                         unordered_set<int> &inliers,
                         int point_id,
                         int &point_num,
                         double &sum_diff2) {

  std::srand(52265554);
  for (size_t i = 0; i < true_points.size(); ++i) {

    std::vector<Eigen::Vector2d> projected_points;
    int num_obs = ProjectPointOnPoses(camera_parameters, true_points.at(i), true_poses, projected_points);


    if (num_obs < 2) {
//      DistortPoint(true_points[i]);
      continue;
    }
    g2o::VertexSBAPointXYZ *v_p = new g2o::VertexSBAPointXYZ();
    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->setEstimate(DistortPoint3D(true_points[i]));


    optimizer.addVertex(v_p);

    bool inlier = true;
    for (size_t j = 0; j < true_poses.size(); ++j) {
      const Vector2d &z = projected_points[j];

      if (IsValidPoint(z)) {
        double sam = Sample::uniform();

        g2o::EdgeProjectXYZ2UV *error
            = new g2o::EdgeProjectXYZ2UV();
        error->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(v_p));
        error->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertices().find(j)->second));

        if (sam < OUTLIER_RATIO) {
          inlier = false;
          error->setMeasurement(Vector2d(Sample::uniform(0, IMAGE_WIDTH),
                                         Sample::uniform(0, IMAGE_HEIGHT)));
        } else {
          error->setMeasurement(DistortPoint2D(z, PIXEL_NOISE));
        }

        error->information() = Matrix2d::Identity();
        if (ROBUST_KERNEL) {
          g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
          error->setRobustKernel(rk);
        }
        error->setParameterId(0, 0);
        optimizer.addEdge(error);
      }
    }

    if (inlier) {
      inliers.insert(point_id);
      Vector3d diff = v_p->estimate() - true_points[i];

      sum_diff2 += diff.dot(diff);
    }
    pointid_2_trueid.insert(make_pair(point_id, i));
    ++point_id;
    ++point_num;

  }
}

int main(int argc, const char *argv[]) {
  Eigen::initParallel();
  if (argc < 2) {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)" << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
    cout << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)"
         << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
    cout << endl;
    exit(0);
  }

  PIXEL_NOISE = atof(argv[1]);
  OUTLIER_RATIO = 0.0;

  if (argc > 2) {
    OUTLIER_RATIO = atof(argv[2]);
  }

  ROBUST_KERNEL = false;
  if (argc > 3) {
    ROBUST_KERNEL = atoi(argv[3]) != 0;
  }
  bool STRUCTURE_ONLY = false;
  if (argc > 4) {
    STRUCTURE_ONLY = atoi(argv[4]) != 0;
  }

  bool DENSE = false;
  if (argc > 5) {
    DENSE = atoi(argv[5]) != 0;
  }

  cout << "PIXEL_NOISE: " << PIXEL_NOISE << endl;
  cout << "OUTLIER_RATIO: " << OUTLIER_RATIO << endl;
  cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
  cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY << endl;
  cout << "DENSE: " << DENSE << endl;

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
  if (DENSE) {
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
  } else {
    linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
  }

  g2o::OptimizationAlgorithmLevenberg *p_algorithm_levenberg = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
  );
  optimizer.setAlgorithm(p_algorithm_levenberg);

  g2o::CameraParameters *cam_params = GetCameraParameters();
  cam_params->setId(0);

  if (!optimizer.addParameter(cam_params)) {
    assert(false);
  }
  vector<Vector3d> true_points;
  GenerateTruePoints(true_points);

  TruePoses_t true_poses;
  int point_id = GenerateTruePoses(true_poses);

  AddPosesToOptimizer(true_poses, optimizer);

  int point_num = 0;
  double sum_diff2 = 0;

  cout << endl;
  unordered_map<int, int> pointid_2_trueid;
  unordered_set<int> inliers;
  InitializeEstimates(true_points,
                      true_poses,
                      cam_params,
                      optimizer,
                      pointid_2_trueid,
                      inliers,
                      point_id,
                      point_num,
                      sum_diff2);


  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  if (STRUCTURE_ONLY) {
    g2o::StructureOnlySolver<3> structure_only_ba;
    cout << "Performing structure-only BA:" << endl;
    g2o::OptimizableGraph::VertexContainer points;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin();
         it != optimizer.vertices().end(); ++it) {
      g2o::OptimizableGraph::Vertex *v = static_cast<g2o::OptimizableGraph::Vertex *>(it->second);
      if (v->dimension() == 3)
        points.push_back(v);
    }
    structure_only_ba.calc(points, 10);
  }
  //optimizer.save("test.g2o");
  cout << endl;
  cout << "Performing full BA:" << endl;
  optimizer.optimize(30);
  cout << endl;
  cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2 / inliers.size()) << endl;
  point_num = 0;
  sum_diff2 = 0;
//  for (int i = 0; i < NUMBER_OF_POSES; ++i) {
//    g2o::VertexSE3Expmap * pose = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(i));
//    if (nullptr == pose)
//      std::cerr << "Vertex " << i << " is not a pose" << std::endl;
//    g2o::SE3Quat q_estimated = pose->estimate();
//    g2o::SE3Quat q_original = true_poses[i];
//    std::cout << q_estimated.rotation().vec() << std::endl;
//    std::cout << q_original.rotation().vec() << std::endl;
//    std::cout << std::endl;
//    std::cout << std::endl;
//  }

  for (unordered_map<int, int>::iterator it = pointid_2_trueid.begin();
       it != pointid_2_trueid.end(); ++it) {
    g2o::HyperGraph::VertexIDMap::iterator v_it
        = optimizer.vertices().find(it->first);
    if (v_it == optimizer.vertices().end()) {
      cerr << "Vertex " << it->first << " not in graph!" << endl;
      exit(-1);
    }
    g2o::VertexSBAPointXYZ *v_p
        = dynamic_cast< g2o::VertexSBAPointXYZ * > (v_it->second);
    if (v_p == nullptr) {
      cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
      exit(-1);
    }
    Vector3d diff = v_p->estimate() - true_points[it->second];
    if (inliers.find(it->first) == inliers.end())
      continue;
    sum_diff2 += diff.dot(diff);
    ++point_num;
  }
  cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2 / inliers.size()) << endl;
  cout << endl;
}
