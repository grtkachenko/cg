#pragma once

#include <cg/convex_hull/graham.h>
#include <cg/primitives/point.h>
#include <cg/primitives/vector.h>
#include <math.h>


namespace cg
{
	template <class Scalar>
	double distance(point_2t<Scalar> a, point_2t<Scalar> b) {
		vector_2t<Scalar> vect(a.x - b.x, a.y - b.y);
		return sqrt(vect * vect);
	}

	template <class BidIter>
	std::pair<BidIter, BidIter> get_diameter(BidIter begin, BidIter end) {
		if (begin == end || begin + 1 == end) {
			return std::make_pair(begin, begin);
		}
		typedef typename BidIter::value_type::value_type Scalar;
//		BidIter tmp = contour_graham_hull<BidIter>(begin, end);
		BidIter ans1, ans2;
		double dmax = -1;
		for (BidIter it = begin; it != end; it++) {
			for (BidIter it1 = begin; it1 != end; it1++) {
				if (dmax < distance(*it, *it1)) {
					dmax = distance(*it, *it1);
					ans1 = it;
					ans2 = it1;
				}
			}
		}
		return std::make_pair(ans1, ans2);
	}
}
