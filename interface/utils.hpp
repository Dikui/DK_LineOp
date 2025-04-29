#ifndef UTILS_HPP
#define UTILS_HPP

#include "line_interface.h"

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
}  // namespace dk_line_op

#endif  // UTILS_HPP