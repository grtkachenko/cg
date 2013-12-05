#pragma once
#include "cg/primitives/point.h"
#include <boost/numeric/interval.hpp>
#include <gmpxx.h>

#include <boost/optional.hpp>

namespace cg {
   struct is_inside_d {
      boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const {
         double a00 = a.x - d.x, a01 = a.y - d.y, a02 = (a.x * a.x - d.x * d.x) + (a.y * a.y - d.y * d.y);
         double a10 = b.x - d.x, a11 = b.y - d.y, a12 = (b.x * b.x - d.x * d.x) + (b.y * b.y - d.y * d.y);
         double a20 = c.x - d.x, a21 = c.y - d.y, a22 = (c.x * c.x - d.x * d.x) + (c.y * c.y - d.y * d.y);
         double m1 = a00 * a11 * a22, m2 = a01 * a12 * a20, m3 = a02 * a10 * a21, m4=a20 * a11 * a02, m5=a21 * a12 * a00, m6=a01 * a10 * a22;
         double det =  m1 + m2 + m3 - m4 - m5 - m6;
         double sum = fabs(m1) + fabs(m2) + fabs(m3) + fabs(m4) + fabs(m5) + fabs(m6);
         double eps = sum * 16 * std::numeric_limits<double>::epsilon();
         if (det > eps)
             return true;
         if (det < -eps)
             return false;
         return boost::none;
      }
   };

   struct is_inside_i {
      boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const {
         typedef boost::numeric::interval_lib::unprotect<boost::numeric::interval<double> >::type interval;
         boost::numeric::interval<double>::traits_type::rounding _;
         interval a00 = interval(a.x) - d.x, a01 = interval(a.y) - d.y, a02 = (interval(a.x) * a.x - interval(d.x) * d.x) + (interval(a.y) * a.y - interval(d.y) * d.y);
         interval a10 = interval(b.x) - d.x, a11 = interval(b.y) - d.y, a12 = (interval(b.x) * b.x - interval(d.x) * d.x) + (interval(b.y) * b.y - interval(d.y) * d.y);
         interval a20 = interval(c.x) - d.x, a21 = interval(c.y) - d.y, a22 = (interval(c.x) * c.x - interval(d.x) * d.x) + (interval(c.y) * c.y - interval(d.y) * d.y);
         interval det =  a00 * a11 * a22 + a01 * a12 * a20 + a02 * a10 * a21 -
                        (a20 * a11 * a02 + a21 * a12 * a00 + a01 * a10 * a22);
         if (det.lower() > 0)
            return true;
         if (det.upper() < 0)
                  return false;
         return boost::none;
      }
   };

   struct is_inside_r {
      boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const {
         mpq_class a00 = mpq_class(a.x) - d.x, a01 = mpq_class(a.y) - d.y, a02 = (mpq_class(a.x) * a.x - mpq_class(d.x) * d.x) + (mpq_class(a.y) * a.y - mpq_class(d.y) * d.y);
         mpq_class a10 = mpq_class(b.x) - d.x, a11 = mpq_class(b.y) - d.y, a12 = (mpq_class(b.x) * b.x - mpq_class(d.x) * d.x) + (mpq_class(b.y) * b.y - mpq_class(d.y) * d.y);
         mpq_class a20 = mpq_class(c.x) - d.x, a21 = mpq_class(c.y) - d.y, a22 = (mpq_class(c.x) * c.x - mpq_class(d.x) * d.x) + (mpq_class(c.y) * c.y - mpq_class(d.y) * d.y);
         mpq_class det =  a00 * a11 * a22 + a01 * a12 * a20 + a02 * a10 * a21 -
         (a20 * a11 * a02 + a21 * a12 * a00 + a01 * a10 * a22);

         return det > 0;
      }
   };

   bool is_inside(point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) {
      if (boost::optional<bool> v = is_inside_d()(a, b, c, d))
         return *v;

      if (boost::optional<bool> v = is_inside_i()(a, b, c, d))
         return *v;

      return *is_inside_r()(a, b, c, d);
   }
}
