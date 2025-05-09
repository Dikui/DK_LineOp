#ifndef LINES_MANAGER_H
#define LINES_MANAGER_H

#include "line_interface.h"

namespace dk_line_op {

// 整条曲线包含size-3个CRSpline
class Spline : public Line {
 public:
  Spline() {};
  Spline(const double* const* i_pts, const size_t& i_pts_size) {
    if (!setPoints(i_pts, i_pts_size)) {
      this->reset();
    }
  }

  void reset() { this->pts_.clear(); }

  bool setPoints(const double* const* i_pts, const size_t& i_pts_size) {
    this->pts_.resize(i_pts_size);
    for (size_t i = 0; i < i_pts_size; ++i) {
      this->pts_[i] = Eigen::Map<Vec3 const>(i_pts[i]);
    }
    return true;
  }

  const std::vector<Vec3>& getCtrlPts() const { return this->pts_; }

  // 找出向外生长的点
  std::vector<Vec3> getPtstoAdd(const std::vector<Vec3>& i_origin_pts);

  // 生长
  bool growLine(std::optional<Vec3> i_inital_pt,
                const std::vector<Vec3>& i_origin_pts,
                const Polyline& i_polyline);

 private:
};

}  // namespace dk_line_op

#endif  // LINES_MANAGER_H