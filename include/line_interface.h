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

  static Line* createLine(const double& lower_range, const double& upper_range);

  virtual bool setControlPoints(const double* const* control_pts,
                                const size_t pts_size) = 0;

 protected:
  int ctrl_pts_num_;
  std::vector<VecD> ctrl_pts_;
};

}  // namespace dk_line_op

#endif  // LINE_INTERFACE_H