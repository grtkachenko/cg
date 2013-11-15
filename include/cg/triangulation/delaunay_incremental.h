#pragma once

#include <algorithm>
#include <cg/operations/orientation.h>
#include <cg/primitives/triangle.h>
#include <cg/primitives/segment.h>
#include <cg/io/segment.h>
#include <set>
#include <map>
#include <boost/optional.hpp>

namespace cg
{

   template <class Scalar>
   struct edge_less {
      bool operator() (segment_2t<Scalar> const & a, segment_2t<Scalar> const & b) const {
         return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
      }
   };

   template <class Scalar>
   void update_edge(segment_2t<Scalar> & seg, point_2t<Scalar> & opposite_point,
                    std::set<segment_2t<Scalar>, edge_less<Scalar> > & border,
                    std::map<segment_2t<Scalar>, point_2t<Scalar>, edge_less<Scalar> > & point_by_edge) {
      if (border.count(seg) != 0) {
         border.erase(seg);
      } else {
         std::swap(seg[0], seg[1]);
         border.insert(seg);
         point_by_edge[seg] = opposite_point;
      }
   }

   template <class Scalar>
   point_2t<Scalar> mid_ort_intersect(point_2t<Scalar> & a, point_2t<Scalar> & b, point_2t<Scalar> & c) {
      vector_2t<Scalar> q1(b.x - a.x, b.y - a.y), q2(c.x - b.x, c.y - b.y);
      point_2t<Scalar> m1((a.x + b.x) / 2, (a.y + b.y) / 2), m2((c.x + b.x) / 2, (c.y + b.y) / 2);
      Scalar a1 = q1.x, b1 = q1.y, c1 = -(q1.x * m1.x + q1.y * m1.y), a2 = q2.x, b2 = q2.y, c2 = -(q2.x * m2.x + q2.y * m2.y);
      return point_2t<Scalar>((b1 * c2 - b2 * c1) / (b2 * a1 - b1 * a2), (a1 * c2 - a2 * c1) / (-b2 * a1 + b1 * a2));
   }

   template <class Scalar>
   bool less_ort(segment_2t<Scalar> & e, point_2t<Scalar> & a, point_2t<Scalar> & b) {
      point_2t<Scalar> i1 = mid_ort_intersect(a, e[1], e[0]), i2 = mid_ort_intersect(b, e[1], e[0]);
      return cg::orientation(e[1], i1, i2) == cg::CG_LEFT;
   }


   template <class Scalar>
   std::vector<triangle_2t<Scalar> > get_delaunay_triangulation(std::vector<point_2t<Scalar> > const & initial_points) {
      typedef segment_2t<Scalar> Segment;
      typedef triangle_2t<Scalar> Triangle;
      typedef point_2t<Scalar> Point;
      using std::vector;
      vector<Triangle> res;

      if (initial_points.size() <= 2) {
         return res;
      }

      vector<Point> points(initial_points);
      std::set<Segment, edge_less<Scalar> > border;
      std::sort(points.begin(), points.end());
      Point end = *std::min_element(std::next(points.begin()), points.end(),
         [&points] (point_2 const & a, point_2 const & b)
         {
            return cg::orientation(points[0], a, b) == cg::CG_RIGHT;
         }
      );
      border.insert(Segment(points[0], end));
      std::map<Segment, Point, edge_less<Scalar> > point_by_edge;

      while (!border.empty()) {
         Segment e = *border.begin();
         border.erase(border.begin());
         boost::optional<Point> best_point;
         for (int i = 0; i < points.size(); i++) {
            Point p = points[i];
            if (e[0] != p && e[1] != p && cg::orientation(e[0], e[1], p) == cg::CG_RIGHT) {
               if ((point_by_edge.count(e) == 0 || less_ort(e, point_by_edge[e], p)) &&
                       (!best_point|| less_ort(e, p, *best_point))) {
                  best_point =  points[i];
               }
            }
         }

         if (best_point) {
            res.push_back(Triangle(e[0], e[1], *best_point));
            Segment first(e[1], *best_point), second(*best_point, e[0]);
            update_edge(first, e[0], border, point_by_edge);
            update_edge(second, e[1], border, point_by_edge);
         }
      }

      return res;
   }
}
