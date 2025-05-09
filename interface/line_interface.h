#ifndef LINE_INTERFACE_H
#define LINE_INTERFACE_H

#include <numeric>
#include <vector>

#include "Eigen/Core"

namespace dk_line_op {

using Vec3 = Eigen::Matrix<double, 3, 1>;

struct Polyline {};

class Line {
 public:
  virtual ~Line() = default;

  virtual bool setPoints(const double* const* i_pts,
                         const size_t i_pts_size) = 0;

 protected:
  std::vector<Vec3> pts_;
};

std::vector<double> linspace(const double& start, const double& end,
                             const size_t& num);

std::vector<Vec3> uniformSample(const std::vector<double>& dists,
                                const std::vector<Vec3>& lists,
                                const double& step);

}  // namespace dk_line_op

#endif  // LINE_INTERFACE_H