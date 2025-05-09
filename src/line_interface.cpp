#include "../interface/line_interface.h"

namespace dk_line_op {

double intPow(const double& a, const int& b) {
  double result = 1.0;
  if (b == 0) {
    result = 1;
  } else {
    for (size_t i = 0; i < b; ++i) {
      result = result * a;
    }
  }
  return result;
}

// 均匀采样一维点
std::vector<double> linspace(const double& start, const double& end,
                             const size_t& num) {
  std::vector<double> o_u_list;
  if (num <= 1) {
    return o_u_list;
  }
  o_u_list.reserve(num);
  double step = (end - start) / (num - 1);
  for (size_t i = 0; i < num; ++i) {
    o_u_list.emplace_back(start + step * i);
  }
  return o_u_list;
}

// 均匀采样3维点
std::vector<Vec3> uniformSample(const std::vector<double>& dists,
                                const std::vector<Vec3>& lists,
                                const double& step) {
  std::vector<Vec3> samples;
  // check
  if (dists.empty() || lists.empty() || step <= 0.0 ||
      dists.size() != lists.size()) {
    return samples;
  }

  size_t idx = 0;

  for (double s = dists.front(); s <= dists.back(); s += step) {
    while (idx + 1 < dists.size() && dists[idx + 1] < s) {
      ++idx;
    }
    if (idx + 1 == dists.size()) {
      const double d_end_0 = dists[idx - 1];
      const double d_end_1 = dists[idx];
      const Vec3 v_end_0 = lists[idx - 1];
      const Vec3 v_end_1 = lists[idx];
      samples.emplace_back(v_end_1 + (s - d_end_1) * (v_end_1 - v_end_0) /
                                         (d_end_1 - d_end_0));
      continue;
    }
    const double d0 = dists[idx];
    const double d1 = dists[idx + 1];
    const double t = (d1 > d0 ? (s - d0) / (d1 - d0) : 0.0);
    const Vec3 v0 = lists[idx];
    const Vec3 v1 = lists[idx + 1];
    samples.emplace_back(v0 + t * (v1 - v0));
  }

  return samples;
}

// 计算多项式系数
Eigen::VectorXd robustPoly1d(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                             int order) {
  int fyx_order = order;
  if (x.size() <= 3) {
    fyx_order = x.size() - 1;
  }
  double y_max = y.maxCoeff();
  double y_min = y.minCoeff();
  if (y_max - y_min < 0.1) {
    fyx_order = 0;
  } else if (y_max - y_min < 1.0) {
    fyx_order = 2;
    if (x.size() <= 2) {
      fyx_order = x.size() - 1;
    }
  }
  // 借助Eigen库进行多项式拟合
  Eigen::MatrixXd A(x.size(), fyx_order + 1);
  for (int i = 0; i < x.size(); ++i) {
    for (int j = 0; j <= fyx_order; ++j) {
      A(i, j) = intPow(x(i), j);
    }
  }

  Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(y);
  return coeffs;
}

// 计算多项式系数
bool fitting(Polyline& i_polyline, const std::vector<Vec3>& i_pts) {
  if (i_pts.size() <= 1) {
    return false;
  }
  // 设置多项式阶数
  int order = 3;
  if (i_pts.size() <= 3) {
    order = i_pts.size() - 1;
  }
  // 计算旋转向量
  Vec2 principal_axis = (i_pts.back() - i_pts.front()).head<2>();
  Vec2 expected_axis(1.0, 0.0);
  double angle = std::acos(principal_axis.dot(expected_axis) /
                           (principal_axis.norm() * expected_axis.norm()));

  if ((principal_axis.x() * expected_axis.y() -
       principal_axis.y() * expected_axis.x()) < 0) {
    angle = -angle;
  }
  Vec3 rotation_vector(0.0, 0.0, angle);
  Eigen::AngleAxisd rotation(rotation_vector.norm(),
                             rotation_vector.normalized());
  // 求解旋转矩阵
  i_polyline.R = rotation.toRotationMatrix();

  // 求解旋转后的系数
  Eigen::VectorXd x(i_pts.size());
  Eigen::VectorXd y(i_pts.size());
  Eigen::VectorXd z(i_pts.size());

  for (size_t i = 0; i < i_pts.size(); ++i) {
    const Vec3 roted_pt = i_polyline.R * i_pts[i];
    x(i) = roted_pt.x();
    y(i) = roted_pt.y();
    z(i) = roted_pt.z();
  }
  i_polyline.f_yx = robustPoly1d(x, y, order);
  i_polyline.f_zx = robustPoly1d(x, z, order);

  return true;
}

// 遍历方式查询内点外点
std::pair<std::optional<Vec3>, std::optional<Vec3>> find_border_point(
    const Vec3& query, const std::vector<Vec3>& i_pts) {
  // <内点，外点>
  std::pair<std::optional<Vec3>, std::optional<Vec3>> result;
  double min_dist = std::numeric_limits<double>::min();  // 最小正数
  double max_dist = 0.0;
  for (size_t i = 0; i < i_pts.size(); ++i) {
    double dist = (i_pts[i] - query).norm();
    if (dist < 3.0) {
    }
  }
}

}  // namespace dk_line_op