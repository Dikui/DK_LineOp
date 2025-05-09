#ifndef LINE_INTERFACE_H
#define LINE_INTERFACE_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"

namespace dk_line_op {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;

struct Polyline {
  Vec4 f_yx = Vec4::Zero();
  Vec4 f_zx = Vec4::Zero();
  Mat3 R = Mat3::Zero();
};

class Line {
 public:
  virtual ~Line() = default;

  virtual bool setPoints(const double* const* i_pts,
                         const size_t i_pts_size) = 0;

 protected:
  std::vector<Vec3> pts_;
  Polyline polyline_;
};

double intPow(const double& a, const int& b);

std::vector<double> linspace(const double& start, const double& end,
                             const size_t& num);

std::vector<Vec3> uniformSample(const std::vector<double>& dists,
                                const std::vector<Vec3>& lists,
                                const double& step);

Eigen::VectorXd robustPoly1d(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                             int order);

bool fitting(Polyline& i_polyline, const std::vector<Vec3>& i_pts);

// <内点，外点>
std::pair<std::optional<Vec3>, std::optional<Vec3>> find_border_point(
    const Vec3& query, const std::vector<Vec3>& i_pts);

}  // namespace dk_line_op

#endif  // LINE_INTERFACE_H