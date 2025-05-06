#ifndef LINES_MANAGER_H
#define LINES_MANAGER_H

#include "line_interface.h"

namespace dk_line_op {

// 整条曲线包含size-3个CRSpline
template <int Dim>
class Spline : public Line<Dim> {
  using Vec = typename Line<Dim>::Vec;

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

  // 均匀重采样
  bool samplePts(const double& i_step) {
    if (this->pts_.size() <= 1) {
      return false;
    }
    // get dists
    std::vector<double> dists;
    for (size_t i = 0; i < this->pts_.size(); ++i) {
      if (i == 0) {
        dists.emplace_back(0.0);
      }
      dists.emplace_back(dists.back() +
                         (this->pts_[i] - this->pts_[i - 1]).norm());
    }
    // resample
    this->pts_ = uniformSample(dists, this->pts_, i_step);
    return true;
  }

 private:
  int num_pts_size_ = 0;
};

}  // namespace dk_line_op

#endif  // LINES_MANAGER_H