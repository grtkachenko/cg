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
         // vertex is common for both cases
         vertex<Scalar> * a = new vertex<Scalar>(p);
         Vertex<Scalar> a_ptr(a);
         vertexes.push_back(a_ptr);

         // common as well
         Edge<Scalar> first_edge;
         int total_vertexes;

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

            faces.erase(faces.begin() + std::max(index.first, index.second));
            faces.erase(faces.begin() + std::min(index.first, index.second));

            // made fake next
            common_edge->next_edge->next_edge->next_edge = common_edge->twin_edge->next_edge;
            common_edge->twin_edge->next_edge->next_edge->next_edge = common_edge->next_edge;
            first_edge = common_edge->next_edge;

            total_vertexes = 4;
         } else {
            total_vertexes = 3;
            first_edge = faces[index.first]->inc_edge;
            faces.erase(faces.begin() + index.first);
         }

         Edge<Scalar> new_edges_ptr[4];
         Face<Scalar> new_faces[4];
         auto cur_edge = first_edge;

         // completing faces; creating edges + twins
         for (int i = 0; i < total_vertexes; i++) {
            edge<Scalar> * new_edge = new edge<Scalar>(a_ptr);
            edge<Scalar> * twin = new edge<Scalar>(cur_edge->start);
            Edge<Scalar> new_edge_ptr(new_edge), twin_ptr(twin);
            set_twins(new_edge_ptr, twin_ptr);
            new_edges_ptr[i] = new_edge_ptr;

            face<Scalar> * cur_face = new face<Scalar>();
            new_faces[i] = Face<Scalar>(cur_face);
            cur_face->inc_edge = cur_edge;
            faces.push_back(new_faces[i]);

            cur_edge = cur_edge->next_edge;
         }

         // completing edges
         cur_edge = first_edge;
         for (int i = 0; i < total_vertexes; i++) {
            // backup next_edge
            auto tmp = cur_edge->next_edge;

            // nexts
            new_edges_ptr[i]->next_edge = cur_edge;
            new_edges_ptr[i]->twin_edge->next_edge = new_edges_ptr[(i + total_vertexes - 1) % total_vertexes];
            cur_edge->next_edge = new_edges_ptr[(i + 1) % total_vertexes]->twin_edge;

            // faces
            new_edges_ptr[i]->inc_face = new_faces[i];
            new_edges_ptr[i]->twin_edge->inc_face = new_faces[(i + total_vertexes - 1) % total_vertexes];
            cur_edge->inc_face = new_faces[i];

            cur_edge = tmp;
         }

         // fixes edges
         cur_edge = first_edge;
         std::cout << "__________________" << std::endl;
         for (int i = 0; i < total_vertexes; i++) {
            std::cout << cur_edge->start->to_point() << " " << cur_edge->next_edge->start->to_point() << std::endl;
            fix_edge(cur_edge);
            cur_edge = cur_edge->next_edge->twin_edge->next_edge;
         }
      }

      void fix_edge(Edge<Scalar> e) {
         // TODO: remove it
         if (e->twin_edge == nullptr) {
            return;
         }
         std::cout << "Fix edge : " << e->start->to_point() << " " << e->next_edge->start->to_point() << " " << e->next_edge->next_edge->start->to_point();

         if (is_edge_bad(e)) {
            std::cout << " it's BAD edge" << std::endl;

            // flip and fix new edges
            auto edge_to_fix1 = e->twin_edge->next_edge, edge_to_fix2 = e->twin_edge->next_edge->next_edge;

            //creating edge
            edge<Scalar> * new_edge = new edge<Scalar>(e->next_edge->next_edge->start);
            edge<Scalar> * twin = new edge<Scalar>(e->twin_edge->next_edge->next_edge->start);
            Edge<Scalar> new_edge_ptr(new_edge), twin_ptr(twin);
            set_twins(new_edge_ptr, twin_ptr);

            //completed faces
            auto first_face = e->inc_face, second_face = e->twin_edge->inc_face;

            first_face->inc_edge = e->next_edge;
            second_face->inc_edge = e->twin_edge->next_edge;

            e->next_edge->next_edge->inc_face = second_face;
            e->twin_edge->next_edge->next_edge->inc_face = first_face;

            new_edge_ptr->inc_face = first_face;
            twin_ptr->inc_face = second_face;

            //completed nexts
            new_edge_ptr->next_edge = e->twin_edge->next_edge->next_edge;
            twin_ptr->next_edge = e->next_edge->next_edge;

            e->next_edge->next_edge->next_edge = e->twin_edge->next_edge;
            e->next_edge->next_edge = new_edge_ptr;
            e->twin_edge->next_edge->next_edge->next_edge = e->next_edge;
            e->twin_edge->next_edge->next_edge = twin_ptr;

            //start new fixes!!!
            fix_edge(edge_to_fix1);
            fix_edge(edge_to_fix2);
         } else {
            std::cout << " it's OK edge" << std::endl;
         }
      }

      bool is_edge_bad(Edge<Scalar> e, bool is_twin = false) {
         // TODO: make it fast (maybe you shouldn't check all the points)
         for (auto v : vertexes) {
            if (e->start == v || e->next_edge->start == v
                || e->next_edge->next_edge->start == v) continue;

            if (is_inside(e->start, e->next_edge->start, e->next_edge->next_edge->start, v)) {
               return true;
            }
         }
         if (!is_twin) {
            return is_edge_bad(e->twin_edge, true);
         }

         return false;
      }

      bool is_inside(Vertex<Scalar> va, Vertex<Scalar> vb, Vertex<Scalar> vc, Vertex<Scalar> vd) {
         // TODO: make it exact (!)
         point_2t<Scalar> a = va->to_point(), b = vb->to_point(), c = vc->to_point(), d = vd->to_point();
         double a11 = a.x - d.x, a12 = a.y - d.y, a13 = (a.x * a.x - d.x * d.x) + (a.y * a.y - d.y * d.y);
         double a21 = b.x - d.x, a22 = b.y - d.y, a23 = (b.x * b.x - d.x * d.x) + (b.y * b.y - d.y * d.y);
         double a31 = c.x - d.x, a32 = c.y - d.y, a33 = (c.x * c.x - d.x * d.x) + (c.y * c.y - d.y * d.y);
         return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31) > 0;
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

      bool contains_point(point_2t<Scalar> const & p) {
         for (auto v : vertexes) {
            if (v->to_point() == p) {
               return true;
            }
         }
         return false;
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
      std::vector<Face<Scalar>> faces;
      std::vector<Vertex<Scalar>> vertexes;
   };

   template <class Scalar>
   struct delaunay_triangulation
   {

      delaunay_triangulation() {
         tr_cell.init_with_triangle();
      }

      void add_point(point_2t<Scalar> p)
      {
         if (!tr_cell.contains_point(p)) tr_cell.add_vertex(p);
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
