#include "../interface/line_manager.h"

namespace dk_line_op {

std::vector<Vec3> Spline::getPtstoAdd(const std::vector<Vec3>& i_origin_pts) {
  std::vector<Vec3> o_pts(i_origin_pts.size());
  int num_pts = this->pts_.size();
  if (num_pts < 2) {
    return o_pts;
  }
  Vec3 normal_a = this->pts_.front() - this->pts_[1];
  Vec3 normal_b = this->pts_.back() - this->pts_[num_pts - 2];
  for (auto& i_pt : i_origin_pts) {
    Vec3 d_a = i_pt - this->pts_.front();
    Vec3 d_b = i_pt - this->pts_.back();
    double cos_a = d_a.dot(normal_a) / (d_a.norm() * normal_a.norm());
    double cos_b = d_b.dot(normal_b) / (d_b.norm() * normal_b.norm());
    double thd = std::cos(M_PI / 2);
    if (cos_a > thd or cos_b > thd) {
      o_pts.emplace_back(i_pt);
    }
  }
  return o_pts;
}

bool Spline::growLine(std::optional<Vec3> i_inital_pt,
                      const std::vector<Vec3>& i_origin_pts,
                      const Polyline& i_polyline) {
  if (!i_inital_pt.has_value()) {
    i_inital_pt = i_origin_pts.front();
    this->pts_.emplace_back(i_inital_pt);
  }
  int num = 0;

  std::vector<Vec3> origin_pts = i_origin_pts;

  // 循环生长点
  while (1) {
    num += 1;
    origin_pts = getPtstoAdd(origin_pts);
    if (origin_pts.empty()) {
      return false;
    }
    // 搜索最近点
    if (origin_pts.size() < 15) {
    }
  }
}

}  // namespace dk_line_op