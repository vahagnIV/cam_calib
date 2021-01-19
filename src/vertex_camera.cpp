//
// Created by vahagn on 19/01/21.
//

#include "vertex_camera.h"
namespace g2o_learning {
template<>
void VertexCamera<5>::ComputeDistortion(const double x, const double y, double & xd, double & yd) {
  const double & k1 = _estimate[4];
  const double & k2 = _estimate[5];
  const double & p1 = _estimate[6];
  const double & p2 = _estimate[7];
  const double & k3 = _estimate[8];

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
  double a1 = 2 * x * y;

  xd = x * cdist + p1 * a1 + p2 * (r2 + 2 * x * x);
  yd = y * cdist + p2 * a1 + p1 * (r2 + 2 * y * y);
};

template<>
void VertexCamera<8>::ComputeDistortion(const double x, const double y, double & xd, double & yd) {
  const double & k1 = _estimate[4];
  const double & k2 = _estimate[5];
  const double & p1 = _estimate[6];
  const double & p2 = _estimate[7];
  const double & k3 = _estimate[8];
  const double & k4 = _estimate[9];
  const double & k5 = _estimate[10];
  const double & k6 = _estimate[11];

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
  double icdist2 = 1. / (1 + k4 * r2 + k5 * r4 + k6 * r6);
  double a1 = 2 * x * y;

  xd = x * cdist * icdist2 + p1 * a1 + p2 * (r2 + 2 * x * x);
  yd = y * cdist * icdist2 + p2 * a1 + p1 * (r2 + 2 * y * y);
}
}