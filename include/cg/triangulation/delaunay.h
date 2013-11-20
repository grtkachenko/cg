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
      vertex(point_2t<Scalar> p) : point_2t<Scalar>(p) {}

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

      // returns index in array
      std::pair<int, int> find_face(point_2t<Scalar> const & p) {
         std::pair<int, int> res(-1, -1);
         for (int i = 0; i < faces.size(); i++) {
            auto f = faces[i];
            Edge<Scalar> cur = f->inc_edge;
            bool ok = true;
            for (int j = 0; j < 3; j++) {
               if (cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) == CG_RIGHT) {
                  ok = false;
                  break;
               }
               cur = cur->next_edge;
            }
            if (ok) {
               if (res.first == -1) {
                  res.first = i;
               } else {
                  res.second = i;
               }
            }

         }
         return res;
      }

      void add_vertex(point_2t<Scalar> const & p) {
         auto index = find_face(p);
         if (index.first == -1) {
            return;
         }
         if (index.second != -1) {
            // on the edge
            Edge<Scalar> common_edge;

            auto cur_edge = faces[index.first]->inc_edge;
            for (int i = 0; i < 3; i++) {
               auto cur_second_edge = faces[index.second]->inc_edge;
               for (int j = 0; j < 3; j++) {
                  if (cur_second_edge->twin_edge == cur_edge) {
                     common_edge = cur_edge;
                  }
                  cur_second_edge = cur_second_edge->next_edge;
               }
               cur_edge = cur_edge->next_edge;
            }
            std::cout << common_edge->start->to_point() << std::endl;
         }
         // inside the face
         Face<Scalar> p_face = faces[index.first];
         vertex<Scalar> * a = new vertex<Scalar>(p);
         Vertex<Scalar> a_ptr(a);

         Edge<Scalar> new_edges_ptr[3];

         // creating new edges, their twins
         auto cur_edge = p_face->inc_edge;
         for (int i = 0; i < 3; i++) {
            edge<Scalar> * new_edge = new edge<Scalar>(a_ptr);
            edge<Scalar> * twin = new edge<Scalar>(cur_edge->start);
            Edge<Scalar> new_edge_ptr(new_edge), twin_ptr(twin);
            set_twins(new_edge_ptr, twin_ptr);
            new_edge->next_edge = cur_edge;
            cur_edge = cur_edge->next_edge;
            new_edges_ptr[i] = new_edge_ptr;
         }

         Face<Scalar> new_faces[3];
         new_faces[0] = p_face;
         cur_edge = p_face->inc_edge->next_edge;
         for (int i = 1; i < 3; i++) {
            face<Scalar> * cur_face = new face<Scalar>();
            new_faces[i] = Face<Scalar>(cur_face);
            cur_face->inc_edge = cur_edge;
            cur_edge->inc_face = new_faces[i];
            cur_edge = cur_edge->next_edge;
            faces.push_back(new_faces[i]);
         }

         cur_edge = p_face->inc_edge->next_edge->next_edge;
         for (int i = 0; i < 3; i++) {
            auto tmp = cur_edge->next_edge;
            cur_edge->next_edge = new_edges_ptr[i]->twin_edge;
            new_edges_ptr[i]->twin_edge->next_edge = new_edges_ptr[(i + 2) % 3];
            cur_edge = tmp;
            new_edges_ptr[i]->inc_face = new_faces[i];
            new_edges_ptr[i]->twin_edge->inc_face = new_faces[(i + 2) % 3];
         }


      }

      void set_twins(Edge<Scalar> e1, Edge<Scalar> e2) {
         e1->twin_edge = e2;
         e2->twin_edge = e1;
      }

      // it will be removed after modification
      void init_with_triangle() {
         face<Scalar> * inf_face = new face<Scalar>();
         Face<Scalar> inf_face_ptr(inf_face);

         vertex<Scalar> * a = new vertex<Scalar>(-400, -300);
         vertex<Scalar> * b = new vertex<Scalar>(400, -300);
         vertex<Scalar> * c = new vertex<Scalar>(0, 300);
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
