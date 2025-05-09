#include "../interface/line_interface.h"

namespace dk_line_op {

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

}  // namespace dk_line_op