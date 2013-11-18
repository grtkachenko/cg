#pragma once

#include <algorithm>
#include <cg/operations/orientation.h>
#include <cg/primitives/triangle.h>
#include <cg/primitives/segment.h>
#include <cg/io/segment.h>
#include <set>
#include <map>
#include <memory>
#include <boost/optional.hpp>



namespace cg
{
   template <class Scalar> struct face;
   template <class Scalar> struct vertex;
   template <class Scalar> struct edge;

   template <class Scalar>
   using Vertex = std::shared_ptr<vertex<Scalar> >;
   template <class Scalar>
   using Edge = std::shared_ptr<edge<Scalar> >;
   template <class Scalar>
   using Face = std::shared_ptr<face<Scalar> >;

   template <class Scalar>
   struct edge {
      edge(Vertex<Scalar> start) : start(start) {}

      void init_members(Edge<Scalar> _twin_edge, Edge<Scalar> _next_edge, Face<Scalar> _inc_face) {
         twin_edge = _twin_edge;
         next_edge = _next_edge;
         inc_face = _inc_face;
      }

      //members
      Vertex<Scalar> start;
      Edge<Scalar> twin_edge, next_edge;
      Face<Scalar> inc_face;
   };


   template <class Scalar>
   struct vertex : point_2t<Scalar> {
      vertex(Scalar a, Scalar b) : point_2t<Scalar>(a, b) {}


      point_2t<Scalar> to_point() {
         return point_2t<Scalar>(this->x, this->y);
      }

      //members
      Edge<Scalar> inc_edge;
   };


   template <class Scalar>
   struct face {
      //members
      Edge<Scalar> inc_edge;
   };

   template <class Scalar>
   struct cell {

      Face<Scalar> find_face(point_2t<Scalar> const & p) {
         for (auto f : faces) {
            Edge<Scalar> start_edge = f.inc_edge;
            Edge<Scalar> cur = start_edge;
            bool ok = true;
            while (true) {
               if (cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) == CG_RIGHT) {
                  ok = false;
                  break;
               }
               cur = cur->next_edge;
               if (cur == start_edge) break;
            }
            if (ok) return f;
         }
//         return 0;
      }

      void add_vertex(point_2t<Scalar> const & p) {
         auto p_face = find_face(p);
         face<Scalar> a, b, c;
      }

      // it will be removed after modification
      void init_with_triangle() {
         face<Scalar> * inf_face = new face<Scalar>();
         Face<Scalar> inf_face_ptr(inf_face);


         vertex<Scalar> * a = new vertex<Scalar>(-100, 0);
         vertex<Scalar> * b = new vertex<Scalar>(100, 0);
         vertex<Scalar> * c = new vertex<Scalar>(0, 80);
         Vertex<Scalar> a_ptr(a), b_ptr(b), c_ptr(c);

         edge<Scalar> * a_inside = new edge<Scalar>(a_ptr);
         edge<Scalar> * b_inside = new edge<Scalar>(b_ptr);
         edge<Scalar> * c_inside = new edge<Scalar>(c_ptr);
         Edge<Scalar> a_inside_ptr(a_inside), b_inside_ptr(b_inside), c_inside_ptr(c_inside);

         a_inside->init_members(nullptr, b_inside_ptr, inf_face_ptr);
         b_inside->init_members(nullptr, c_inside_ptr, inf_face_ptr);
         c_inside->init_members(nullptr, a_inside_ptr, inf_face_ptr);

         inf_face->inc_edge = a_inside_ptr;
         faces.push_back(inf_face_ptr);
      }


      std::vector<triangle_2t<Scalar> > get_triangulation() {
         std::vector<triangle_2t<Scalar> > res;
         for (int i = 0; i < faces.size(); i++) {
            Face<Scalar> cur_face = faces[i];
            res.push_back(triangle_2t<Scalar>(cur_face->inc_edge->start->to_point(),
                                              cur_face->inc_edge->next_edge->start->to_point(), cur_face->inc_edge->next_edge->next_edge->start->to_point()));
         }
         return res;
      }

   private:
      std::vector<Face<Scalar> > faces;
   };

   template <class Scalar>
   struct delaunay_triangulation
   {

      delaunay_triangulation() {
         tr_cell.init_with_triangle();
      }

      void add_point(point_2t<Scalar> p)
      {
         tr_cell.add_vertex(p);
      }

      std::vector<triangle_2t<Scalar> > get_delaunay_triangulation() {
         return tr_cell.get_triangulation();
      }

   private:
      std::vector<point_2t<Scalar> > points;
      std::vector<triangle_2t<Scalar> > res;
      cell<Scalar> tr_cell;
   };




}
