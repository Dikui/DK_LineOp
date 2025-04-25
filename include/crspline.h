#ifndef CRSPLINE_H
#define CRSPLINE_H

#include "line_interface.h"

namespace dk_line_op {

template <int, Dim>
class CRSpline : public Line<Dim> {
 public:
  CRSpline() {};
  bool setControlPoints(const double* const* i_ctrl_pts, const size_t pts_size);
  bool setTau(const double& i_tau);

 private:
  double tau = 0.5;
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
};

}  // namespace dk_line_op

#endif  // CRSPLINE_H

// CatMullRom_Spline() { ctrl_pts.reserve(20); }
// CatMullRom_Spline(const std::vector<Point2DInfo>& i_ctrl_pts,
//                   const double& i_tau = 0.5)
//     : ctrl_pts(std::move(i_ctrl_pts)), tau(i_tau) {
//   num_ctrl_pts = ctrl_pts.size();
//   M << 0, 1, 0, 0, -tau, 0, tau, 0, 2 * tau, tau - 3, 3 - 2 * tau, -tau,
//   -tau,
//       2 - tau, tau - 2, tau;
// }

// void setTau(const double& i_tau) const {
//   if (tau != i_tau) {
//     tau = i_tau;
//     dirty = true;
//   }
// };

// const Eigen::Matrix4d& getM() const {
//   if (dirty) {
//     M << 0, 1, 0, 0, -tau, 0, tau, 0, 2 * tau, tau - 3, 3 - 2 * tau,
//     -tau, -tau,
//         2 - tau, tau - 2, tau;
//     dirty = false;
//   }
//   return M;
// }

// private:
// mutable bool dirty{true};
// mutable double tau = 0.5;
// int num_ctrl_pts = 0;
// std::vector<Point2DInfo> ctrl_pts;
// mutable Eigen::Matrix4d M = Eigen::Matrix4d::Zero();