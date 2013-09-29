#pragma once

#include "cg/primitives/point.h"
#include "cg/primitives/vector.h"
#include <math.h>
#include <algorithm>

namespace cg
{

   template<class OutIt>
   void write_point_to_out(OutIt out, point_2 p)
   {
      *out = p;
      out++;
   }

	template<class BidIt, class OutIt>
   OutIt simplify(BidIt start, BidIt end, double eps, OutIt out, bool write_last = true)
   {
		if (start == end) {
			return out;
		}
		if (start + 1 == end) {
			write_point_to_out<OutIt>(out, *start);
			return out;
		}

      double dmax = -1;
      BidIt max_point;
      vector_2 vect = vector_2((*(end - 1)).x - (*start).x, (*(end - 1)).y - (*start).y);

      for (BidIt it = start + 1; it != end - 1; it++)
      {
         vector_2 cur_sq((*it).x - (*start).x, (*it).y - (*start).y);
			double koef = cur_sq * vect / (vect * vect), cur_d;
			if (0 <= koef && koef <= 1) {
				cur_d = abs((cur_sq ^ vect) / sqrt(vect * vect));
			} else {
				vector_2 sec_cur_sq((*it).x - (*(end - 1)).x, (*it).y - (*(end - 1)).y);
				cur_d = sqrt(fmin(sec_cur_sq * sec_cur_sq, cur_sq * cur_sq));
			}

         if (cur_d > dmax)
         {
            dmax = cur_d;
            max_point = it;
         }
      }

      if (dmax > eps)
      {
         OutIt tmp_out = simplify(start, max_point + 1, eps, out, false);
         simplify(max_point, end, eps, tmp_out, write_last);
      }
      else
      {
         write_point_to_out<OutIt>(out, *start);
         if (write_last)
         {
            write_point_to_out<OutIt>(out, *(end - 1));
         }
      }

      return out;
   }


}
