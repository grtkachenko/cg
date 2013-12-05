#pragma once
#include <cg/primitives/point.h>

namespace cg {
   template<class Scalar>
   bool is_inside(point_2t<Scalar> const & a, point_2t<Scalar> const & b, point_2t<Scalar> const & c, point_2t<Scalar> const & d) {
      // TODO: make it exact (!)
      double a11 = a.x - d.x, a12 = a.y - d.y, a13 = (a.x * a.x - d.x * d.x) + (a.y * a.y - d.y * d.y);
      double a21 = b.x - d.x, a22 = b.y - d.y, a23 = (b.x * b.x - d.x * d.x) + (b.y * b.y - d.y * d.y);
      double a31 = c.x - d.x, a32 = c.y - d.y, a33 = (c.x * c.x - d.x * d.x) + (c.y * c.y - d.y * d.y);
      return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31) > 0;
   }
}
