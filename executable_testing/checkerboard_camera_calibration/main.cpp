//
// Created by vahagn on 1/10/21.
//

#include <boost/filesystem.hpp>
#include <intrinsic_solver.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

void ListDirectory(const std::string &path, std::vector<std::string> &out_files) {
  boost::filesystem::path dir(path);
  boost::filesystem::directory_iterator it(path);
  for (; it != boost::filesystem::directory_iterator(); ++it) {
    if (boost::filesystem::is_regular_file(it->path()))
      out_files.push_back(it->path().string());
  }
}

void ExtractCorners(const std::vector<std::string> &imnames,
                    std::vector<std::vector<Eigen::Vector2d>> &out_corners,
                    const cv::Size &pattern_size) {
  out_corners.reserve(imnames.size());
  for (const std::string &s: imnames) {
    cv::Mat image = cv::imread(s, cv::IMREAD_GRAYSCALE);
    std::vector<cv::Point2f> corners;
    if (cv::findChessboardCorners(image, pattern_size, corners)) {
      cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
      out_corners.resize(out_corners.size() + 1);

      for (const cv::Point2f &corner: corners) {
        out_corners.back().push_back(Eigen::Vector2d(corner.x, corner.y));
      }
    }

  }
}

void ComputeOriginalPoints(const cv::Size &pattern_size, std::vector<Eigen::Vector2d> &out_points) {
  for (int i = 0; i < pattern_size.width; ++i) {
    for (int j = 0; j < pattern_size.height; ++j) {
      out_points.push_back(Eigen::Vector2d(i, j));
    }
  }
}
/*
void TestHomography(g2o_learning::IntrinsicSolver & solver){
  Eigen::Matrix<double, 3, 3> H_org;
  H_org << 1, 5, 3, 4, 5, 6, 7.569, 8, 2;

  std::cout << H_org << std::endl;
  std::cout << H_org.determinant() << std::endl;
  std::vector<Eigen::Vector2d> original_point(4);
  std::vector<Eigen::Vector2d> transformed_point(4);
  for (int i = 0; i < 4; ++i) {

    Eigen::Vector2d & X = original_point[i];
    X << static_cast<double>(std::rand()) / RAND_MAX, static_cast<double>(std::rand()) / RAND_MAX;

    transformed_point[i] << (H_org(0, 0) * X[0] + H_org(0, 1) * X[1] + H_org(0, 2))
        / (H_org(2, 0) * X[0] + H_org(2, 1) * X[1] + H_org(2, 2)),
        (H_org(1, 0) * X[0] + H_org(1, 1) * X[1] + H_org(1, 2))
            / (H_org(2, 0) * X[0] + H_org(2, 1) * X[1] + H_org(2, 2));
  }

  Eigen::Matrix<double, 3, 3> h;

  solver.Find3HomographyFromPlanar4Points(transformed_point, original_point, h);
  std::cout << h / h(0, 0) << std::endl << std::endl << std::endl;
}*/

int main(int argc, char *argv[]) {

  const cv::Size pattern_size(8, 13);
  std::vector<std::string> files;
  ListDirectory("../../../data/calib", files);
  std::vector<std::vector<Eigen::Vector2d>> corners;
  ExtractCorners(files, corners, pattern_size);
  std::vector<Eigen::Vector2d> pattern_points_2d;
  ComputeOriginalPoints(pattern_size, pattern_points_2d);

  g2o_learning::IntrinsicSolver
      solver;
  solver.Calbirate(corners, std::vector<std::vector<Eigen::Vector2d> >(corners.size(), pattern_points_2d));
  //solver.FindIntrinsicParameters();


  return 0;
}