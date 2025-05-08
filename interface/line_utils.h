#ifndef UTILS_HPP
#define UTILS_HPP

#include <numeric>

#include "line_interface.h"

namespace dk_line_op {

std::vector<double> linspace(const double& start, const double& end,
                             const size_t& num) {};

template <int Dim>
std::vector<VecD<Dim>> uniformSample(const std::vector<double>& dists,
                                     const std::vector<VecD<Dim>>& lists,
                                     const double& step) {};

}  // namespace dk_line_op

#endif  // UTILS_HPP