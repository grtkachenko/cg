#pragma once

#include <cg/convex_hull/andrew.h>
#include <cg/primitives/point.h>
#include <cg/primitives/vector.h>
#include <math.h>


namespace cg
{
   template <class Scalar>
   double sqr_distance(point_2t<Scalar> a, point_2t<Scalar> b)
   {
      vector_2t<Scalar> vect(a.x - b.x, a.y - b.y);
      return vect * vect;
   }

   template <class Scalar>
   double distance_edge_ver_not_norm(point_2t<Scalar> a, std::pair<point_2t<Scalar>, point_2t<Scalar> > edge)
   {
      vector_2t<Scalar> first(edge.first.x - a.x, edge.first.y - a.y), second(edge.second.x - edge.first.x, edge.second.y - edge.first.y);
      return (first ^ second) * (first ^ second);
   }

   bool make_max(double& val, double candidate)
   {
      if (val > candidate)
      {
         return false;
      }

      val = candidate;
      return true;
   }

   template <class BigIter>
   BigIter circ_next(BigIter cur, BigIter begin, BigIter end)
   {
      if (cur + 1 == end)
      {
         return begin;
      }

      return cur + 1;
   }

   template <class Scalar, class PairPoint>
   void relax_ans(double& val, point_2t<Scalar> p, PairPoint& pair, PairPoint& ans)
   {
      if (make_max(val, sqr_distance(p, pair.first)))
      {
         ans = std::make_pair(p, pair.first);
      }

      if (make_max(val, sqr_distance(p, pair.second)))
      {
         ans = std::make_pair(p, pair.second);
      }
   }

   template <class BidIter>
   std::pair<BidIter, BidIter> get_diameter(BidIter begin, BidIter end)
   {
      typedef typename BidIter::value_type::value_type Scalar;
      typedef typename BidIter::value_type Point;
      typedef std::pair<Point, Point> PairPoint;

      if (begin == end || begin + 1 == end)
      {
         return std::make_pair(begin, begin);
      }

      BidIter old_begin = begin, old_end = end;
      std::vector<Point> copy_v(begin, end);
      begin = copy_v.begin();
      end = andrew_hull(copy_v.begin(), copy_v.end());

      PairPoint edge = std::make_pair(*begin, *(begin + 1)), ans;

      BidIter cur_point = std::max_element(begin, end, [&edge](point_2t<Scalar>& a, point_2t<Scalar>& b)
      {
         return distance_edge_ver_not_norm(a, edge) - distance_edge_ver_not_norm(b, edge);
      });

      double dmax = -1;
      relax_ans(dmax, *cur_point, edge, ans);

      for (BidIter it = begin + 1; it != end; it++)
      {
         edge = std::make_pair(*it, *(circ_next(it, begin, end)));

         while (distance_edge_ver_not_norm(*cur_point, edge) < distance_edge_ver_not_norm(*(circ_next(cur_point, begin, end)), edge))
         {
            cur_point = circ_next(cur_point, begin, end);
         }

         relax_ans(dmax, *cur_point, edge, ans);
      }

      return std::make_pair(std::find(old_begin, old_end, ans.first), std::find(old_begin, old_end, ans.second));
   }
}
