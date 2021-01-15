//
// Created by vahagn on 1/10/21.
//

#include <boost/filesystem.hpp>
#include <intrinsic_solver.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

void ListDirectory(const std::string & path, std::vector<std::string> & out_files) {
  boost::filesystem::path dir(path);
  boost::filesystem::directory_iterator it(path);
  for (; it != boost::filesystem::directory_iterator(); ++it) {
    if (boost::filesystem::is_regular_file(it->path()))
      out_files.push_back(it->path().string());
  }
}

void ExtractCorners(const std::vector<std::string> & imnames,
                    std::vector<std::vector<Eigen::Vector2d>> & out_corners,
                    const cv::Size & pattern_size) {
  out_corners.reserve(imnames.size());
  for (const std::string & s: imnames) {
    cv::Mat image = cv::imread(s, cv::IMREAD_GRAYSCALE);
    std::vector<cv::Point2f> corners;
    if (cv::findChessboardCorners(image, pattern_size, corners)) {
      cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
      out_corners.resize(out_corners.size() + 1);

      for (const cv::Point2f & corner: corners) {
        out_corners.back().push_back(Eigen::Vector2d(corner.x, corner.y));
      }
    }

  }
}

void ComputeOriginalPoints(const cv::Size & pattern_size, std::vector<Eigen::Vector3d> & out_points) {
  for (int i = 0; i < pattern_size.width; ++i) {
    for (int j = 0; j < pattern_size.height; ++j) {
      out_points.push_back(Eigen::Vector3d(i, j, 0));
    }
  }
}

int main(int argc, char *argv[]) {

  const cv::Size pattern_size(8, 13);
  std::vector<std::string> files;
  ListDirectory("/data/git/g2o_learning/data/calib", files);
  std::vector<std::vector<Eigen::Vector2d>> corners;
  ExtractCorners(files, corners, pattern_size);
  std::vector<Eigen::Vector3d> pattern_points_3d;
  ComputeOriginalPoints(pattern_size, pattern_points_3d);

  g2o_learning::IntrinsicSolver
      solver(corners, std::vector<std::vector<Eigen::Vector3d> >(corners.size(), pattern_points_3d));
  solver.FindIntrinsicParameters();

  Eigen::Matrix<double, 3, 3> H_org;
  H_org << 1, 5, 3, 4, 5, 6, 7, 8, 2;

  std::cout << H_org << std::endl;
  std::cout << H_org.determinant() << std::endl;
  std::vector<Eigen::Vector3d> original_point(4);
  std::vector<Eigen::Vector3d> transformed_point(4);
  for (int i = 0; i < 4; ++i) {
//    original_point[i] << i * 0.1, i * 0.2, i * 0.3 + 1;
    original_point[i] << static_cast<double>(std::rand()) / RAND_MAX, static_cast<double>(std::rand()) / RAND_MAX, static_cast<double>(std::rand()) / RAND_MAX + 1;
    transformed_point[i] = H_org * original_point[i];
  }

  Eigen::Matrix<double, 3, 3> h;

//  solver.FindHomographyFromFirst4Points(original_point, transformed_point, h);
//  std::cout << h << std::endl << std::endl;

  solver.FindHomographyFromFirst4Points(transformed_point, original_point, h);
  std::cout << h / h(0,0) << std::endl << std::endl << std::endl;
  return 0;
}