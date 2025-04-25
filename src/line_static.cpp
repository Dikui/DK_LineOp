#include "../include/crspline.h"
#include "../include/line_interface.h"

namespace dk_line_op {

Line* createLine(const double lower_range, const double upper_range) {
  return CRSpline();
}
}  // namespace dk_line_op