#ifndef LINE_INTERFACE_H
#define LINE_INTERFACE_H

#include <Eigen/Core>
#include <vector>

namespace dk_line_op {

class Line {
  using VecD = Eigen::Matrix<double, Dim, 2>;

 public:
  virtual ~Line() = default;
  static Line* createLine(const double lower_range, const double upper_range);
  virtual bool setControlPoints(const double* const* control_pts,
                                const size_t pts_size) = 0;

 private:
  int ctrl_pts_num;
  std::vector<VecD> ctrl_pts;
};

}  // namespace dk_line_op

#endif  // LINE_INTERFACE_H