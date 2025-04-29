#ifndef LINE_INTERFACE_H
#define LINE_INTERFACE_H

#include <vector>

#include "Eigen/Core"

namespace dk_line_op {

template <int Dim>
class Line {
 public:
  using VecD = Eigen::Matrix<double, Dim, 1>;

  virtual ~Line() = default;

  static Line* createLine(const double& i_lower_range,
                          const double& i_upper_range);

  virtual bool setPoints(const double* const* i_pts,
                         const size_t i_pts_size) = 0;

 protected:
  std::vector<VecD> pts_;
};

}  // namespace dk_line_op

#endif  // LINE_INTERFACE_H