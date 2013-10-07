#pragma once

#include <gmpxx.h>
#include <cg/convex_hull/andrew.h>
#include <cg/primitives/point.h>
#include <cg/primitives/vector.h>
#include <math.h>
#include <boost/numeric/interval.hpp>


namespace cg
{
	typedef boost::numeric::interval_lib::unprotect<boost::numeric::interval<double> >::type interval;
	template <typename T>
	T vector_mul_len(point_2t<T> a, point_2t<T> b, point_2t<T> c)
	{
		return abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
	}

	bool is_near(point_2 & seg1, point_2 & seg2, point_2 & p1, point_2 & p2)
	{
		double len1 = vector_mul_len(seg1, seg2, p1), len2 = vector_mul_len(seg1, seg2, p2);
		double eps = (len1 + len2) * std::numeric_limits<double>::epsilon() * 8;

		if (abs(len1 - len2) > eps)
			return len1 < len2;

		interval len1i(vector_mul_len<interval>(seg1, seg2, p1)), len2i(vector_mul_len<interval>(seg1, seg2, p2));

		if (len1i.lower() >= len2i.upper())
			return false;
		if (len1i.upper() < len2i.lower())
			return true;

		mpq_class len1m(vector_mul_len<mpq_class>(seg1, seg2, p1)), len2m(vector_mul_len<mpq_class>(seg1, seg2, p2));
		return len1m < len2m;
	}

	template <class Scalar>
	double sqr_distance(point_2t<Scalar> a, point_2t<Scalar> b)
	{
		vector_2t<Scalar> vect(a.x - b.x, a.y - b.y);
		return vect * vect;
	}

	bool make_max(double& val, double candidate)
	{
		if (val > candidate)
			return false;

		val = candidate;
		return true;
	}

	template <class BigIter>
	BigIter circ_next(BigIter cur, BigIter begin, BigIter end)
	{
		if (cur + 1 == end)
			return begin;

		return cur + 1;
	}

	template <class Scalar, class PairPoint>
	void relax_ans(double& val, point_2t<Scalar> p, PairPoint& pair, PairPoint& ans)
	{
		if (make_max(val, sqr_distance(p, pair.first)))
			ans = std::make_pair(p, pair.first);

		if (make_max(val, sqr_distance(p, pair.second)))
			ans = std::make_pair(p, pair.second);
	}

	template <class BidIter>
	std::pair<BidIter, BidIter> get_diameter(BidIter begin, BidIter end)
	{
		typedef typename BidIter::value_type::value_type Scalar;
		typedef typename BidIter::value_type Point;
		typedef std::pair<Point, Point> PairPoint;

		if (begin == end || begin + 1 == end)
			return std::make_pair(begin, begin);

		BidIter old_begin = begin, old_end = end;
		std::vector<Point> copy_v(begin, end);
		begin = copy_v.begin();
		end = andrew_hull(copy_v.begin(), copy_v.end());

		PairPoint edge = std::make_pair(*begin, *(begin + 1)), ans;
		BidIter cur_point = begin + 1;
		double dmax = -1;

		for (BidIter it = begin; it != end; it++)
		{
			edge = std::make_pair(*it, *(circ_next(it, begin, end)));

			while (is_near(edge.first, edge.second, *cur_point, *(circ_next(cur_point, begin, end))))
				cur_point = circ_next(cur_point, begin, end);

			relax_ans(dmax, *cur_point, edge, ans);
		}

		return std::make_pair(std::find(old_begin, old_end, ans.first), std::find(old_begin, old_end, ans.second));
	}
}
