#ifndef LINES_MANAGER_H
#define LINES_MANAGER_H

#include "line_interface.h"

namespace dk_line_op {

// 整条曲线包含size-3个CRSpline
class Spline : public Line<3> {
  using Vec = typename Line<3>::Vec;

 public:
  Spline() {};
  Spline(const double* const* i_pts, const size_t& i_pts_size) {
    if (!setPoints(i_pts, i_pts_size)) {
      this->reset();
    }
  }

  void reset() {
    this->pts_.clear();
    num_pts_size_ = 0;
  }

  bool setPoints(const double* const* i_pts, const size_t& i_pts_size) {
    this->pts_.resize(i_pts_size);
    for (size_t i = 0; i < i_pts_size; ++i) {
      this->pts_[i] = Eigen::Map<Vec const>(i_pts[i]);
    }
    num_pts_size_ = this->pts_.size();
    return true;
  }

  const std::vector<Vec>& getCtrlPts() const { return this->pts_; }

  bool getSkeleton(const Vec& i_inital_pt,
                   const std::vector<Vec>& i_origin_pts);

 private:
  int num_pts_size_ = 0;
};

}  // namespace dk_line_op

#endif  // LINES_MANAGER_H