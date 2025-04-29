#include "../include/crspline.h"
#include "../include/line_interface.h"

namespace dk_line_op {

template <int Dim>
Line<Dim>* Line<Dim>::createLine(const double& lower_range,
                                 const double& upper_range) {
  return CRSpline<Dim>();
}
}  // namespace dk_line_op