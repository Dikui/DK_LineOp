#ifndef CRSPLINE_H
#define CRSPLINE_H

#include "line_interface.h"

namespace dk_line_op {

template <int Dim>
class CRSpline : public Line<Dim> {
 public:
  using Base = Line<Dim>;

  CRSpline() {};
  CRSpline(const double* const* i_ctrl_pts, const size_t& i_pts_size,
           const double& i_tau = 0.5) {
    if (!setControlPoints(i_ctrl_pts, i_pts_size) || !setTau(i_tau)) {
      this->reset();
    }
  }

  void reset() {
    this->ctrl_pts_num_ = 0;
    this->ctrl_pts_.clear();
    this->tau_ = 0.5;
    M_.setZero();
  }

  bool setControlPoints(const double* const* i_ctrl_pts,
                        const size_t& i_pts_size) {
    this->ctrl_pts_num_ = i_pts_size;
    this->ctrl_pts_.resize(i_pts_size);
    for (size_t i = 0; i < i_pts_size; ++i) {
      this->ctrl_pts_[i] = Eigen::Map<typename Base::VecD const>(i_ctrl_pts[i]);
    }
    return true;
  }

  bool setTau(const double& i_tau) {
    if (tau_ != i_tau) {
      tau_ = i_tau;
      M_ << 0, 1, 0, 0, -tau_, 0, tau_, 0, 2 * tau_, tau_ - 3, 3 - 2 * tau_,
          -tau_, -tau_, 2 - tau_, tau_ - 2, tau_;
    }
    return true;
  }

 private:
  double tau_ = 0.5;
  Eigen::Matrix4d M_ = Eigen::Matrix4d::Zero();
};

}  // namespace dk_line_op

#endif  // CRSPLINE_H