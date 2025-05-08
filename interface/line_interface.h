#ifndef LINE_INTERFACE_H
#define LINE_INTERFACE_H

#include <vector>

#include "Eigen/Core"

namespace dk_line_op {

template <int Dim>
using VecD = Eigen::Matrix<double, Dim, 1>;

struct PolyLine {
  Eigen::Matrix4d
};

template <int Dim>
class Line {
  using Vec = VecD<Dim>;

 public:
  virtual ~Line() = default;

  virtual bool setPoints(const double* const* i_pts,
                         const size_t i_pts_size) = 0;

 protected:
  std::vector<Vec> pts_;
};

}  // namespace dk_line_op

#endif  // LINE_INTERFACE_H