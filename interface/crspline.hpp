#ifndef CRSPLINE_HPP
#define CRSPLINE_HPP

#include "line_interface.h"
#include "utils.hpp"

namespace dk_line_op {

// 只有四个控制点组成，且类设计上只支持均匀分布
template <int Dim>
class CRSpline {
 public:
  using VecD = typename Line<Dim>::VecD;

  CRSpline(const double& i_tau = 0.5) {
    M_ << 0, 1, 0, 0, -tau_, 0, tau_, 0, 2 * tau_, tau_ - 3, 3 - 2 * tau_,
        -tau_, -tau_, 2 - tau_, tau_ - 2, tau_;
  }

  bool setTau(const double& i_tau) {
    if (tau_ != i_tau) {
      tau_ = i_tau;
      M_ << 0, 1, 0, 0, -tau_, 0, tau_, 0, 2 * tau_, tau_ - 3, 3 - 2 * tau_,
          -tau_, -tau_, 2 - tau_, tau_ - 2, tau_;
    }
    return true;
  }

  // 采样
  std::vector<VecD> samplePts(const std::vector<VecD>& i_ctrl_pts,
                              const int& i_num_pts) {
    std::vector<VecD> o_pts;
    if (i_ctrl_pts.size() != 4) {
      return o_pts;
    }
    o_pts.reserve(i_num_pts);
    std::vector<double> u_list = linspace(0, 1, i_num_pts);

    // 构造矩阵
    Eigen::Matrix<double, 4, Dim> ctrl_mat;
    for (size_t i = 0; i < 4; ++i) {
      ctrl_mat.rows(i) = i_ctrl_pts[i].transpose();
    }

    // 采样
    for (auto& u : u_list) {
      Eigen::RowVector4d u_vec(1.0, u, u * u, u * u * u);
      Eigen::Matrix<double, 1, Dim> pt = u_vec * M_ * ctrl_mat;
      o_pts.emplace_back(pt.transpose());
    }
    return o_pts;
  }

  // 求解
  VecD getPt(const std::vector<VecD>& i_ctrl_pts, const double& u) {
    VecD o_pt = Eigen::Matrix<double, Dim, 1>::Zero();
    if (u < 0.0 || u > 1.0 || i_ctrl_pts.size() != 4) {
      return o_pt;
    }
    Eigen::Matrix<double, 4, Dim> ctrl_mat;
    for (size_t i = 0; i < 4; ++i) {
      ctrl_mat.rows(i) = i_ctrl_pts[i].transpose();
    }
    Eigen::RowVector4d u_vec(1.0, u, u * u, u * u * u);
    o_pt = (u_vec * M_ * ctrl_mat).transpose();
    return o_pt;
  }

 private:
  double tau_ = 0.5;
  Eigen::Matrix4d M_ = Eigen::Matrix4d::Zero();
};

// 整条曲线包含size-3个CRSpline
template <int Dim>
class CRSplineList : public Line<Dim> {
 public:
  using VecD = typename Line<Dim>::VecD;

  CRSplineList() {};
  CRSplineList(const double* const* i_pts, const size_t& i_pts_size,
               const double& i_tau = 0.5) {
    if (!setPoints(i_pts, i_pts_size) || !setTau(i_tau)) {
      this->reset();
    }
  }

  void reset() {
    this->pts_.clear();
    num_pts_size = 0;
    tau_ = 0.5;
  }

  bool setPoints(const double* const* i_pts, const size_t& i_pts_size) {
    this->pts_.resize(i_pts_size);
    for (size_t i = 0; i < i_pts_size; ++i) {
      this->pts_[i] = Eigen::Map<VecD const>(i_pts[i]);
    }
    num_pts_size = this->pts_.size();
    return true;
  }

  bool setTau(const double& i_tau) {
    if (tau_ != i_tau) {
      tau_ = i_tau;
    }
    return true;
  }

  const std::vector<VecD>& getCtrlPts() const { return this->pts_; }
  const double& getTau() const { return tau_; }

  // 重采样
  std::vector<VecD> samplePts(const int& i_step_size) {}

 private:
  int num_pts_size = 0;
  double tau_ = 0.5;
};

}  // namespace dk_line_op

#endif  // CRSPLINE_HPP