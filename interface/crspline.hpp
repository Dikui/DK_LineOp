#ifndef CRSPLINE_HPP
#define CRSPLINE_HPP

#include "line_interface.h"

namespace dk_line_op {

// 只有四个控制点组成，且类设计上只支持均匀分布
class CRSpline {
 public:
  CRSpline(const double& i_tau = 0.5) {
    tau_ = i_tau;
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
  std::vector<Vec3> samplePts(const std::vector<Vec3>& i_ctrl_pts,
                              const int& i_num_pts) const {
    std::vector<Vec3> o_pts;
    if (i_ctrl_pts.size() != 4) {
      return o_pts;
    }
    o_pts.reserve(i_num_pts);
    std::vector<double> u_list = linspace(0, 1, i_num_pts);

    // 构造矩阵
    Eigen::Matrix<double, 4, 3> ctrl_mat;
    for (size_t i = 0; i < 4; ++i) {
      ctrl_mat.row(i) = i_ctrl_pts[i].transpose();
    }

    // 采样
    for (auto& u : u_list) {
      Eigen::RowVector4d u_vec(1.0, u, u * u, u * u * u);
      Eigen::Matrix<double, 1, 3> pt = u_vec * M_ * ctrl_mat;
      o_pts.emplace_back(pt.transpose());
    }
    return o_pts;
  }

  // 求解
  Vec3 getPt(const std::vector<Vec3>& i_ctrl_pts, const double& u) const {
    Vec3 o_pt = Eigen::Matrix<double, 3, 1>::Zero();
    if (u < 0.0 || u > 1.0 || i_ctrl_pts.size() != 4) {
      return o_pt;
    }
    Eigen::Matrix<double, 4, 3> ctrl_mat;
    for (size_t i = 0; i < 4; ++i) {
      ctrl_mat.row(i) = i_ctrl_pts[i].transpose();
    }
    Eigen::RowVector4d u_vec(1.0, u, u * u, u * u * u);
    o_pt = (u_vec * M_ * ctrl_mat).transpose();
    return o_pt;
  }

 private:
  double tau_ = 0.5;
  Mat4 M_ = Mat4::Zero();
};

class CentripetalCRSpline {
 public:
  CentripetalCRSpline(const double& i_alpha = 0.0) { alpha_ = i_alpha; }
  bool setAlpha(const double& i_alpha) { alpha_ = i_alpha; }

  double uj(const double& ui, const Vec3& pt_i, const Vec3& pt_j) const {
    return ui + std::pow((pt_j - pt_i).norm(), alpha_);
  }

  // 采样
  std::vector<Vec3> samplePts(const std::vector<Vec3>& i_ctrl_pts,
                              const int& i_num_pts) const {
    std::vector<Vec3> o_pts;
    if (i_ctrl_pts.size() != 4) {
      return o_pts;
    }
    o_pts.reserve(i_num_pts);
    std::vector<double> u_list = linspace(0, 1, i_num_pts);

    // 构造参数
    const double u0 = 0.0;
    const double u1 = uj(u0, i_ctrl_pts[0], i_ctrl_pts[1]);
    const double u2 = uj(u1, i_ctrl_pts[1], i_ctrl_pts[2]);
    const double u3 = uj(u2, i_ctrl_pts[2], i_ctrl_pts[3]);

    // 采样
    for (auto& u : u_list) {
      const Vec3 A1 = (u1 - u) / (u1 - u0) * i_ctrl_pts[0] +
                      (u - u0) / (u1 - u0) * i_ctrl_pts[1];
      const Vec3 A2 = (u2 - u) / (u2 - u1) * i_ctrl_pts[1] +
                      (u - u1) / (u2 - u1) * i_ctrl_pts[2];
      const Vec3 A3 = (u3 - u) / (u3 - u2) * i_ctrl_pts[2] +
                      (u - u2) / (u3 - u2) * i_ctrl_pts[3];
      const Vec3 B1 = (u2 - u) / (u2 - u0) * A1 + (u - u0) / (u2 - u0) * A2;
      const Vec3 B2 = (u3 - u) / (u3 - u1) * A2 + (u - u1) / (u3 - u1) * A3;
      const Vec3 o_pt = (u2 - u) / (u2 - u1) * B1 + (u - u1) / (u2 - u1) * B2;
      o_pts.emplace_back(o_pt);
    }
    return o_pts;
  }

  // 求解
  Vec3 getPt(const std::vector<Vec3>& i_ctrl_pts, const double& u) const {
    Vec3 o_pt = Eigen::Matrix<double, 3, 1>::Zero();
    if (u < 0.0 || u > 1.0 || i_ctrl_pts.size() != 4) {
      return o_pt;
    }

    // 构造参数
    const double u0 = 0.0;
    const double u1 = uj(u0, i_ctrl_pts[0], i_ctrl_pts[1]);
    const double u2 = uj(u1, i_ctrl_pts[1], i_ctrl_pts[2]);
    const double u3 = uj(u2, i_ctrl_pts[2], i_ctrl_pts[3]);

    // 求解
    const Vec3 A1 = (u1 - u) / (u1 - u0) * i_ctrl_pts[0] +
                    (u - u0) / (u1 - u0) * i_ctrl_pts[1];
    const Vec3 A2 = (u2 - u) / (u2 - u1) * i_ctrl_pts[1] +
                    (u - u1) / (u2 - u1) * i_ctrl_pts[2];
    const Vec3 A3 = (u3 - u) / (u3 - u2) * i_ctrl_pts[2] +
                    (u - u2) / (u3 - u2) * i_ctrl_pts[3];
    const Vec3 B1 = (u2 - u) / (u2 - u0) * A1 + (u - u0) / (u2 - u0) * A2;
    const Vec3 B2 = (u3 - u) / (u3 - u1) * A2 + (u - u1) / (u3 - u1) * A3;
    o_pt = (u2 - u) / (u2 - u1) * B1 + (u - u1) / (u2 - u1) * B2;

    return o_pt;
  }

 private:
  double alpha_ = 0.0;
};

}  // namespace dk_line_op

#endif  // CRSPLINE_HPP