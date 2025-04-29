#include "../interface/crspline.hpp"
#include "../interface/line_interface.h"

namespace dk_line_op {

template <int Dim>
Line<Dim>* Line<Dim>::createLine(const double& i_lower_range,
                                 const double& i_upper_range) {
  return CRSplineList<Dim>();
}
}  // namespace dk_line_op