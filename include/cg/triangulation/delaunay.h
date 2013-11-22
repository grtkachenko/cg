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
#include <exception>
#include <cg/operations/has_intersection/segment_segment.h>

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

      void init_members(Edge<Scalar> _next_edge, Face<Scalar> _inc_face) {
         next_edge = _next_edge;
         inc_face = _inc_face;
      }

      segment_2t<Scalar> to_segment() {
         return segment_2t<Scalar>(start->to_point(), next_edge->start->to_point());
      }

      //members
      Vertex<Scalar> start;
      Edge<Scalar> twin_edge, next_edge;
      Face<Scalar> inc_face;
   };

   template <class Scalar>
   bool less(Edge<Scalar> e1, Edge<Scalar> e2) {
      point_2t<Scalar> e1_start = e1->start->to_point(), e1_end = e1->next_edge->start->to_point();
      point_2t<Scalar> e2_start = e2->start->to_point(), e2_end = e2->next_edge->start->to_point();
      auto res = cg::orientation(e1_start, e1_end, e1_end + (e2_end - e2_start));
      if (res != cg::CG_COLLINEAR) {
         return res == cg::CG_RIGHT;
      } else {
         return (e1_end - e1_start) * (e2_start - e1_start) > 0;
      }
   }

   template <class Scalar>
   struct vertex : point_2t<Scalar> {
      vertex(bool is_inf_point) : is_inf_point(is_inf_point) {}
      vertex(Scalar a, Scalar b) : point_2t<Scalar>(a, b), is_inf_point(false) {}
      vertex(point_2t<Scalar> p) : point_2t<Scalar>(p), is_inf_point(false) {}

      point_2t<Scalar> to_point() {
         return point_2t<Scalar>(this->x, this->y);
      }

      //members
      Edge<Scalar> inc_edge;
      bool is_inf_point;
   };


   template <class Scalar>
   struct face {
      face() : is_inf_face(false) {}

      //members
      Edge<Scalar> inc_edge;
      bool is_inf_face;
   };

   template <class Scalar>
   struct cell {

      cell() {
         vertex<Scalar> * inf_p = new vertex<Scalar>(true);
         vertexes.push_back(Vertex<Scalar>(inf_p));
      }



      // returns vector of indexes + pair of min and max edge (in case of point out of polygon)
      std::pair<std::vector<int>, std::pair<Edge<Scalar>, Edge<Scalar>>> find_face(point_2t<Scalar> const & p, bool on_border = false) {
         std::vector<int> found_faces;
         std::pair<Edge<Scalar>, Edge<Scalar>> min_max_edge;

         for (int i = 0; i < faces.size(); i++) {
            auto f = faces[i];
            f->is_inf_face = false;
            auto cur = f->inc_edge;
            bool ok = true;
            for (int j = 0; j < 3; j++) {
               if (cur->start->is_inf_point || cur->next_edge->start->is_inf_point) {
                  cur = cur->next_edge;
                  f->is_inf_face = true;
                  continue;
               }

               if ((!on_border && cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) != CG_LEFT) ||
               (on_border && cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) == CG_RIGHT)) {
                  ok = false;
                  break;
               }
               cur = cur->next_edge;
            }

            if (f->is_inf_face) {
               cur = f->inc_edge;
               for (int j = 0; j < 3; j++) {
                  if (!cur->start->is_inf_point && !cur->next_edge->start->is_inf_point &&
                      ((!on_border && cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) == CG_LEFT) ||
                                     (on_border && cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) != CG_RIGHT))) {
                     if (min_max_edge.second == nullptr) {
                        min_max_edge.first = cur;
                         min_max_edge.second = cur;
                     } else {
                        if (less(cur, min_max_edge.first)) min_max_edge.first = cur;
                        if (less(min_max_edge.second, cur)) min_max_edge.second = cur;
                     }
                     break;
                  }
                  cur = cur->next_edge;
               }
            }


            if (ok) {
               found_faces.push_back(i);
            }

         }
         return std::pair<std::vector<int>, std::pair<Edge<Scalar>, Edge<Scalar>>>(found_faces, min_max_edge);
      }

      void add_vertex(point_2t<Scalar> const & p) {
         // for splitting cases in debug
         std::cout << "__________________" << std::endl;

         // vertex is common for both cases
         vertex<Scalar> * a = new vertex<Scalar>(p);
         Vertex<Scalar> a_ptr(a);
         vertexes.push_back(a_ptr);
         if (vertexes.size() <= 3) {
            if (vertexes.size() == 3) {
               init_with_triangle();
            }
            return;
         }

         auto find_face_res = find_face(p);

         // it shouldn't be empty
         auto index = find_face_res.first;
         assert(!index.empty());

         // min and max edges
         auto min_max_edge = find_face_res.second;

         // common as well (first edge of out chain and number of vertexes in chain
         Edge<Scalar> first_edge;
         int total_vertexes;

         if (faces[index[0]]->is_inf_face) {
            std::cout << "Selected point is OUT of the polygon" << std::endl;
            auto cur_edge = min_max_edge.first;
            if (index.size() != 1) {
               min_max_edge.second->next_edge->next_edge = min_max_edge.first->next_edge->next_edge;

               for (int i = 0; i < index.size() - 1; i++) {
                  cur_edge->next_edge = cur_edge->next_edge->twin_edge->next_edge;
                  cur_edge = cur_edge->next_edge;
               }
            }

            first_edge = min_max_edge.first;
            total_vertexes = index.size() + 2;
            std::vector<bool> need_to_delete(faces.size(), false);
            std::vector<Face<Scalar>> new_faces_vector;
            for (int cur_index : index) {
               need_to_delete[cur_index] = true;
            }

            for (int i = 0; i < faces.size(); i++) {
               if (!need_to_delete[i]) new_faces_vector.push_back(faces[i]);
            }
            faces.swap(new_faces_vector);
         } else {
            std::cout << "Selected point is IN the polygon" << std::endl;
            if (index.size() == 2) {
               // on the edge
               Edge<Scalar> common_edge;

               auto cur_edge = faces[index[0]]->inc_edge;
               for (int i = 0; i < 3; i++) {
                  auto cur_second_edge = faces[index[1]]->inc_edge;
                  for (int j = 0; j < 3; j++) {
                     if (cur_second_edge->twin_edge == cur_edge) {
                        common_edge = cur_edge;
                     }
                     cur_second_edge = cur_second_edge->next_edge;
                  }
                  cur_edge = cur_edge->next_edge;
               }

               faces.erase(faces.begin() + *std::max_element(index.begin(), index.end()));
               faces.erase(faces.begin() + *std::min_element(index.begin(), index.end()));

               // made fake next
               common_edge->next_edge->next_edge->next_edge = common_edge->twin_edge->next_edge;
               common_edge->twin_edge->next_edge->next_edge->next_edge = common_edge->next_edge;
               first_edge = common_edge->next_edge;
               total_vertexes = 4;
            } else {
               total_vertexes = 3;
               first_edge = faces[index[0]]->inc_edge;
               faces.erase(faces.begin() + index[0]);
            }
         }

         std::cout << "First edge in chain is " << first_edge->start->to_point() << " " << first_edge->next_edge->start->to_point() << std::endl;

         std::vector<Edge<Scalar>> new_edges_ptr(total_vertexes);
         std::vector<Face<Scalar>> new_faces(total_vertexes);
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

         std::cout << "Total vertexes in chain: " << total_vertexes << std::endl;
         for (int i = 0; i < total_vertexes; i++) {
            fix_edge(cur_edge);
            cur_edge = cur_edge->next_edge->twin_edge->next_edge;
         }
      }

      void fix_edge(Edge<Scalar> e) {
         std::cout << "Fix edge : " << e->start->to_point() << " " << e->next_edge->start->to_point() << " " << e->next_edge->next_edge->start->to_point();

         if (e->twin_edge->start->is_inf_point || e->start->is_inf_point ||
             e->next_edge->next_edge->start->is_inf_point ||
             (!constraints.empty() && (e == constraints.back() || e->next_edge == constraints.back() || e->next_edge->next_edge == constraints.back() ||
                                       e == constraints.back()->twin_edge || e->next_edge == constraints.back()->twin_edge || e->next_edge->next_edge == constraints.back()->twin_edge))) {
            std::cout << std::endl;
            return;
         }

         std::cout << " (it passes first check) ";


         if (is_edge_bad(e)) {
            std::cout << " it's BAD edge" << std::endl;

            // flip and fix new edges
            auto edge_to_fix1 = e->twin_edge->next_edge, edge_to_fix2 = e->twin_edge->next_edge->next_edge;

            flip(e);

            //start new fixes
            fix_edge(edge_to_fix1);
            fix_edge(edge_to_fix2);
         } else {
            std::cout << " it's OK edge" << std::endl;
         }
      }

      bool more_than_pi(point_2t<Scalar> const & a, point_2t<Scalar> const & b, point_2t<Scalar> const & c) {
         return orientation(a, b, c) != CG_RIGHT;
      }

      bool flip(Edge<Scalar> e) {
         // check for flip correctness
         for (int i = 0; i < 2; i++) {
            if (more_than_pi(e->next_edge->next_edge->start->to_point(), e->next_edge->start->to_point(), e->twin_edge->next_edge->next_edge->start->to_point())) return false;
            e = e->twin_edge;
         }

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

         return true;
      }

      bool is_edge_bad(Edge<Scalar> e, bool is_twin = false) {
         // TODO: make it fast (maybe you shouldn't check all the points)
         for (auto v : vertexes) {
            if (e->start == v || e->next_edge->start == v
                || e->next_edge->next_edge->start == v || v->is_inf_point || e->next_edge->next_edge->start->is_inf_point) continue;

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
         return is_inside(va->to_point(), vb->to_point(), vc->to_point(), vd->to_point());
      }

      bool is_inside(point_2t<Scalar> a, point_2t<Scalar> b, point_2t<Scalar> c, point_2t<Scalar> d) {
         // TODO: make it exact (!)
         double a11 = a.x - d.x, a12 = a.y - d.y, a13 = (a.x * a.x - d.x * d.x) + (a.y * a.y - d.y * d.y);
         double a21 = b.x - d.x, a22 = b.y - d.y, a23 = (b.x * b.x - d.x * d.x) + (b.y * b.y - d.y * d.y);
         double a31 = c.x - d.x, a32 = c.y - d.y, a33 = (c.x * c.x - d.x * d.x) + (c.y * c.y - d.y * d.y);
         return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31) > 0;
      }

      void set_twins(Edge<Scalar> e1, Edge<Scalar> e2) {
         e1->twin_edge = e2;
         e2->twin_edge = e1;
      }

      void init_with_triangle() {
         auto face1 = make_face(vertexes[1], vertexes[2], vertexes[0]), face2 = make_face(vertexes[2], vertexes[1], vertexes[0]);
         set_twins(face1->inc_edge, face2->inc_edge);
         set_twins(face1->inc_edge->next_edge, face2->inc_edge->next_edge->next_edge);
         set_twins(face1->inc_edge->next_edge->next_edge, face2->inc_edge->next_edge);
      }

      // it creates faces, but (!!!) it doesn't set twins (because in such a method we don't know them)
      Face<Scalar> make_face(Vertex<Scalar> a, Vertex<Scalar> b, Vertex<Scalar> c) {
         Vertex<Scalar> vertexes [3] = {a, b, c};
         Face<Scalar> cur_face_ptr(new face<Scalar>());
         Edge<Scalar> edges[3];
         edge<Scalar> * edges_p[3];
         for (int i = 0; i < 3; i++) {
            edges_p[i] = new edge<Scalar>(vertexes[i]);
            edges[i] = Edge<Scalar>(edges_p[i]);
         }
         for (int i = 0; i < 3; i++) {
            edges[i]->init_members(edges[(i + 1) % 3], cur_face_ptr);
         }
         cur_face_ptr->inc_edge = edges[0];
         faces.push_back(cur_face_ptr);
         return cur_face_ptr;
      }

      Vertex<Scalar> find_point(point_2t<Scalar> const & p) {
         for (auto v : vertexes) {
            if (v->to_point() == p) {
               return v;
            }
         }
         return nullptr;
      }
      void add_constraint(point_2t<Scalar> pa, point_2t<Scalar> pb) {
         add_constraint_without_fixing(pa, pb);
         for (auto f : faces) {
            auto cur_edge = f->inc_edge;
            for (int i = 0; i < 3; i++) {
               if (cur_edge->start->to_point() == pa && cur_edge->next_edge->start->to_point() == pb) {
                  constraints.push_back(cur_edge);
                  std::cout << "Сonstraint is " << cur_edge->start->to_point() << " " << cur_edge->next_edge->start->to_point() << std::endl;
                  std::cout << "Fixes edges of constraint" << std::endl;

//                  fix_edge(cur_edge->next_edge->twin_edge);
//                  fix_edge(cur_edge->next_edge->next_edge->twin_edge);
//                  fix_edge(cur_edge->twin_edge->next_edge->twin_edge);
//                  fix_edge(cur_edge->twin_edge->next_edge->next_edge->twin_edge);
                  return;

               }
               cur_edge = cur_edge->next_edge;
            }
         }


      }

      void add_constraint_without_fixing(point_2t<Scalar> pa, point_2t<Scalar> pb, bool is_first = true) {
         segment_2t<Scalar> constraint_segment(pa, pb);
         auto face_a = faces[find_face_that_intersects_segment(find_face(pa, true).first, constraint_segment)];
         auto cur_face = face_a;

         std::vector<Edge<Scalar>> intersect_edges;
         while (true) {
            bool found = false;
            auto cur_edge = cur_face->inc_edge;
            for (int i = 0; i < 3; i++) {
               auto cur_seg = cur_edge->to_segment();
               if (common_points_number(cur_seg, constraint_segment) != 1 && has_intersection(cur_seg, constraint_segment)
                   && (intersect_edges.empty() || intersect_edges.back()->twin_edge != cur_edge)) {

                  if (!contains(constraint_segment, cur_seg[0]) || !contains(constraint_segment, cur_seg[1]))  {
                     intersect_edges.push_back(cur_edge);
                     cur_face = cur_edge->twin_edge->inc_face;
                     found = true;
                  } else {
                     found = false;
                  }

                  break;
               }
               cur_edge = cur_edge->next_edge;
            }
            if (!found) break;
         }

         for (auto e : intersect_edges) {
            flip(e);
         }

         if (is_first) add_constraint_without_fixing(pb, pa, false);

         if (intersect_edges.size() != 0) {
            add_constraint_without_fixing(pa, pb, true);
         }
      }

      int find_face_that_intersects_segment(std::vector<int> num_faces, segment_2t<Scalar> & seg) {
         for (int cur : num_faces) {
            auto cur_edge = faces[cur]->inc_edge;
            for (int i = 0; i < 3; i++) {
               auto cur_seg = cur_edge->to_segment();
               if (common_points_number(cur_seg, seg) != 1 && !cur_edge->start->is_inf_point &&
                   !cur_edge->next_edge->start->is_inf_point && has_intersection(cur_seg, seg)) {
                  return cur;
               }
               cur_edge = cur_edge->next_edge;
            }
         }
         return -1;
      }

      int common_points_number(segment_2t<Scalar> & seg1, segment_2t<Scalar> & seg2) {
         int count = 0;
         for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
               if (seg1[j] == seg2[k]) count++;
         return count;
      }


      std::vector<triangle_2t<Scalar> > get_triangulation() {
         std::vector<triangle_2t<Scalar> > res;
         std::cout << "Faces :" << std::endl;
         for (int i = 0; i < faces.size(); i++) {
            Face<Scalar> cur_face = faces[i];

            // DEBUG OUTPUT STARTED
            auto cur_edge = cur_face->inc_edge;
            for (int j = 0; j < 3; j++) {
               if (cur_edge->start->is_inf_point) {
                  std::cout << "(inf) ";
               } else {
                  std::cout << cur_edge->start->to_point();
               }
               cur_edge = cur_edge->next_edge;
            }
            std::cout << std::endl;
            // DEBUG OUTPUT FINISHED

            if (cur_face->inc_edge->start->is_inf_point || cur_face->inc_edge->next_edge->start->is_inf_point ||
                cur_face->inc_edge->next_edge->next_edge->start->is_inf_point) continue;

            res.push_back(triangle_2t<Scalar>(cur_face->inc_edge->start->to_point(),
                                              cur_face->inc_edge->next_edge->start->to_point(), cur_face->inc_edge->next_edge->next_edge->start->to_point()));
         }
         return res;
      }

   private:
      std::vector<Face<Scalar>> faces;
      std::vector<Vertex<Scalar>> vertexes;
      std::vector<Edge<Scalar>> constraints;
   };

   template <class Scalar>
   struct  delaunay_triangulation
   {

      delaunay_triangulation() {}

      void add_point(point_2t<Scalar> p)
      {
         if (!tr_cell.find_point(p)) tr_cell.add_vertex(p);
      }

      void add_constraint(point_2t<Scalar> a, point_2t<Scalar> b)
      {
         tr_cell.add_constraint(a, b);
      }

      std::vector<triangle_2t<Scalar> > get_delaunay_triangulation() {
         return tr_cell.get_triangulation();
      }

      // need for debugging (if we want to have an access to this method in case we have the instance of delaunay_triangulation)
      bool is_inside(point_2t<Scalar> a, point_2t<Scalar> b, point_2t<Scalar> c, point_2t<Scalar> d) {
         return tr_cell.is_inside(a, b, c, d);
      }

   private:
      std::vector<point_2t<Scalar> > points;
      std::vector<triangle_2t<Scalar> > res;
      cell<Scalar> tr_cell;
   };

}
