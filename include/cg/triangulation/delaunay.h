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
#include <cg/triangulation/is_inside.h>

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
         set_next_edge(_next_edge);
         inc_face = _inc_face;
      }

      segment_2t<Scalar> to_segment() {
         return segment_2t<Scalar>(start->to_point(), next_edge->start->to_point());
      }

      Edge<Scalar> prev_edge() {
         return next_edge->next_edge;
      }

      void set_next_edge(Edge<Scalar> _next_edge) {
         next_edge = _next_edge;
         end = _next_edge->start;
      }

      //members
      Vertex<Scalar> start, end;
      Edge<Scalar> twin_edge, next_edge;
      Face<Scalar> inc_face;
   };

   template <class Scalar>
   bool less(Edge<Scalar> e1, Edge<Scalar> e2) {
      auto e1_start = e1->start->to_point(), e1_end = e1->next_edge->start->to_point();
      auto e2_start = e2->start->to_point(), e2_end = e2->next_edge->start->to_point();
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
      Edge<Scalar> inc_edges;
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
      std::pair<bool, std::pair<std::vector<int>, std::pair<Edge<Scalar>, Edge<Scalar>>>> find_face(point_2t<Scalar> const & p) {
         std::vector<int> found_faces;
         std::pair<Edge<Scalar>, Edge<Scalar>> min_max_edge;
         bool find_containts = false;


         for (int i = 0; i < faces.size(); i++) {
            auto f = faces[i];
            f->is_inf_face = false;
            auto cur = f->inc_edge;
            bool ok = true;
            for (int j = 0; j < 3; j++) {
               if (cur->start->is_inf_point || cur->next_edge->start->is_inf_point) {
                  f->is_inf_face = true;
               }
               cur = cur->next_edge;
            }
            for (int j = 0; j < 3; j++) {
               if (cur->start->is_inf_point || cur->next_edge->start->is_inf_point) {
                  cur = cur->next_edge;
                  continue;
               }

               if (cg::contains(cur->to_segment(), p)) {
                  if (!find_containts) {
                     found_faces.clear();
                     find_containts = true;
                     found_faces.push_back(i);
                     break;
                  } else {
                     found_faces.push_back(i);
                     std::pair<std::vector<int>, std::pair<Edge<Scalar>, Edge<Scalar>>> second_ans(found_faces, min_max_edge);
                     return std::pair<bool, std::pair<std::vector<int>, std::pair<Edge<Scalar>, Edge<Scalar>>>>(true, second_ans);
                  }
               }

               if (cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) != CG_LEFT) {
                  ok = false;
               }
               cur = cur->next_edge;
            }

            if (f->is_inf_face) {
               cur = f->inc_edge;
               ok = false;
               for (int j = 0; j < 3; j++) {
                  if (!cur->start->is_inf_point && !cur->next_edge->start->is_inf_point &&
                      (cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) != CG_RIGHT)) {
                     if ((cg::orientation(cur->start->to_point(), cur->next_edge->start->to_point(), p) == CG_COLLINEAR) &&
                        (cur->next_edge->start->to_point() - cur->start->to_point()) * (p - cur->start->to_point()) < 0) continue;
                     ok = true;
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


            if (ok && !find_containts) {
               found_faces.push_back(i);
            }

         }
         std::pair<std::vector<int>, std::pair<Edge<Scalar>, Edge<Scalar>>> second_ans(found_faces, min_max_edge);
         return std::pair<bool, std::pair<std::vector<int>, std::pair<Edge<Scalar>, Edge<Scalar>>>>(false, second_ans);
      }

      Edge<Scalar> find_non_inf_edge(Face<Scalar> face) {
         auto cur = face->inc_edge;
         for (int i = 0; i < 3; i++) {
            if (cur->start->is_inf_point || cur->next_edge->start->is_inf_point) {
               cur = cur->next_edge;
            } else {
               return cur;
            }
         }
      }

      void add_vertex(point_2t<Scalar> const & p) {
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
         auto index = find_face_res.second.first;
         bool on_edge = find_face_res.first;

         assert(!index.empty());

         // min and max edges
         auto min_max_edge = find_face_res.second.second;

         // common as well (first edge of out chain and number of vertexes in chain
         Edge<Scalar> first_edge;
         int total_vertexes;

         if (faces[index[0]]->is_inf_face && !on_edge) {
            Edge<Scalar> max_collin_edge = nullptr;
            auto cur_edge = min_max_edge.first;
            std::cout << "Edges" <<std::endl;
            for (int i = 0; i < index.size(); i++) {
               std::cout << cur_edge->to_segment() << std::endl;
               if (cg::orientation(*(cur_edge->start), *(cur_edge->next_edge->start), p) == CG_COLLINEAR) {
                  max_collin_edge = cur_edge;
               }
               cur_edge = cur_edge->next_edge->twin_edge->next_edge;
            }
            cur_edge = min_max_edge.first;
            if (index.size() != 1) {               
               min_max_edge.second->next_edge->next_edge = min_max_edge.first->next_edge->next_edge;
               for (int i = 0; i < index.size() - 1; i++) {
                  cur_edge->next_edge = cur_edge->next_edge->twin_edge->next_edge;
                  cur_edge = cur_edge->next_edge;
               }
            }
            if (max_collin_edge != nullptr) {
                  // on the edge
                  Edge<Scalar> common_edge = max_collin_edge->next_edge;

                  // made fake next
                  common_edge->next_edge->next_edge->next_edge = common_edge->twin_edge->next_edge;
                  common_edge->twin_edge->next_edge->next_edge->next_edge = common_edge->next_edge;
                  total_vertexes = 4;
                  first_edge = max_collin_edge->next_edge ;
                  faces.erase(std::find(faces.begin(), faces.end(), max_collin_edge->inc_face));
                  faces.erase(std::find(faces.begin(), faces.end(), max_collin_edge->twin_edge->inc_face));
                  std::cout << max_collin_edge->to_segment() << std::endl;
//               total_vertexes = 3;
//               first_edge = max_collin_edge->next_edge ;
//               faces.erase(std::find(faces.begin(), faces.end(), max_collin_edge->inc_face));
//               std::cout << max_collin_edge->to_segment() << std::endl;
            } else {
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
            }
         } else {
            total_vertexes = 3;
            first_edge = faces[index[0]]->inc_edge;
            faces.erase(faces.begin() + index[0]);
         }

         std::vector<Edge<Scalar>> new_edges_ptr(total_vertexes);
         std::vector<Face<Scalar>> new_faces(total_vertexes);
         auto cur_edge = first_edge;

         // completing faces; creating edges + twins
         for (int i = 0; i < total_vertexes; i++) {
            edge<Scalar> * new_edge = new edge<Scalar>(a_ptr);
            edge<Scalar> * twin = new edge<Scalar>(cur_edge->start);
            Edge<Scalar> new_edge_ptr(new_edge), twin_ptr(twin);
            a->inc_edges = new_edge_ptr;
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
         std::vector<Edge<Scalar>> to_fix;
         for (int i = 0; i < total_vertexes; i++) {
            to_fix.push_back(cur_edge);
            cur_edge = cur_edge->next_edge->twin_edge->next_edge;
         }
         for (auto e : new_edges_ptr) {
            to_fix.push_back(e);
         }

         for (auto e : to_fix) fix_edge(e);
      }

      void add_point_on_edge(Edge<Scalar> e, Vertex<Scalar> v) {

      }

      void delete_vertex(Vertex<Scalar> v) {
         if (vertexes.size() <= 3) {
            vertexes.erase(std::find(vertexes.begin(), vertexes.end(), v));
            faces.clear();
            return;
         }

         std::vector<Edge<Scalar>> edges_to_flip, edges_to_fix, inc_edges;
         inc_edges.push_back(v->inc_edges);
         auto cur = v->inc_edges->twin_edge->next_edge;

         while (cur->to_segment() != v->inc_edges->to_segment()) {
            inc_edges.push_back(cur);
            cur = cur->twin_edge->next_edge;
         }

         for (int i = 3; i < inc_edges.size(); i++) {
            edges_to_flip.push_back(flip(inc_edges[i], true));
         }

         std::reverse(inc_edges.begin(), inc_edges.begin() + 3);
         inc_edges[0]->inc_face->inc_edge = inc_edges[0]->next_edge;

         for (int i = 0; i < 3; i++) {
            auto cur_face = std::find(faces.begin(), faces.end(), inc_edges[i]->inc_face);
            edges_to_fix.push_back(inc_edges[i]->next_edge);
            inc_edges[i]->next_edge->next_edge = inc_edges[(i + 1) % 3]->next_edge;
            inc_edges[i]->next_edge->inc_face = inc_edges[0]->inc_face;
            inc_edges[i]->next_edge->start->inc_edges = inc_edges[i]->next_edge;
            if (i != 0) {
               faces.erase(cur_face);
            }
         }

         std::reverse(edges_to_flip.begin(), edges_to_flip.end());
         for (auto e : edges_to_flip) {
            bool have = false;
            for (auto help : edges_to_fix) {
               if (e->to_segment() == help->to_segment()) have = true;
            }
            if (!have) {
               edges_to_fix.push_back(flip(e, true));
            }
         }

         vertexes.erase(std::find(vertexes.begin(), vertexes.end(), v));
         for (auto e : edges_to_fix) fix_edge(e);
      }

      void fix_edge(Edge<Scalar> e) {
         std::cout << "Fix edge" << e->to_segment() << std::endl;
         for (auto c : constraints) {
            if (e->to_segment() == c->to_segment()) {
               return;
            }
         }         

         if (is_edge_bad(e)) {
            Edge<Scalar> edges_to_fix[4];
            edges_to_fix[0] = e->twin_edge->next_edge;
            edges_to_fix[1] = e->twin_edge->next_edge->next_edge;
            edges_to_fix[2] = e->next_edge;
            edges_to_fix[3] = e->next_edge->next_edge;

            flip(e, true);

            //start new fixes
            for (int i = 0; i < 4; i++) fix_edge(edges_to_fix[i]);
         }
      }

      bool more_than_pi(point_2t<Scalar> const & a, point_2t<Scalar> const & b, point_2t<Scalar> const & c) {
         return orientation(a, b, c) != CG_RIGHT;
      }

      Edge<Scalar> flip(Edge<Scalar> e, bool do_anyway = false) {
         std::cout << "Flip" << e->to_segment() << std::endl;
         if (!do_anyway) {
            // check for flip correctness
            for (int i = 0; i < 2; i++) {
               if (more_than_pi(e->next_edge->next_edge->start->to_point(), e->next_edge->start->to_point(), e->twin_edge->next_edge->next_edge->start->to_point())) return nullptr;
               e = e->twin_edge;
            }
         }
         // vertex
         e->start->inc_edges = e->twin_edge->next_edge;
         e->twin_edge->start->inc_edges = e->next_edge;
         e->start = e->twin_edge->prev_edge()->start;
         e->twin_edge->start = e->prev_edge()->start;
         e->start->inc_edges = e;
         e->twin_edge->start->inc_edges = e->twin_edge;

         //faces
         Face<Scalar> first = e->inc_face, second = e->twin_edge->inc_face;
         first->inc_edge = e;
         second->inc_edge = e->twin_edge;
         e->next_edge->inc_face = second;
         e->twin_edge->next_edge->inc_face = first;

         //nexts
         Edge<Scalar> e_next = e->prev_edge(), e_twin_next = e->twin_edge->prev_edge();

         e->prev_edge()->set_next_edge(e->twin_edge->next_edge);
         e->next_edge->set_next_edge(e->twin_edge);
         e->twin_edge->prev_edge()->set_next_edge(e->next_edge);
         e->twin_edge->next_edge->set_next_edge(e);
         e->set_next_edge(e_next);
         e->twin_edge->set_next_edge(e_twin_next);

         return e;
      }

      bool is_edge_bad(Edge<Scalar> e, bool is_twin = false) {
         if (is_inside(e->start, e->next_edge->start, e->prev_edge()->start, e->twin_edge->prev_edge()->start) ||
             is_inside(e->twin_edge->start, e->twin_edge->next_edge->start, e->twin_edge->prev_edge()->start, e->prev_edge()->start)) {
            return true;
         }

         if (!is_twin) {
            return is_edge_bad(e->twin_edge, true);
         }

         return false;
      }

      bool is_inside(Vertex<Scalar> va, Vertex<Scalar> vb, Vertex<Scalar> vc, Vertex<Scalar> vd) {
         if (vd->is_inf_point) {
            return cg::orientation(*va, *vb, *vc) == CG_COLLINEAR;
         }
         Vertex<Scalar> pts[3] = {va, vb, vc};
         for (int i = 0; i < 3; i++)
            if (pts[i]->is_inf_point) return cg::orientation(pts[(i + 1) % 3]->to_point(), pts[(i + 2) % 3]->to_point(), vd->to_point()) == CG_LEFT;

         return cg::is_inside(va->to_point(), vb->to_point(), vc->to_point(), vd->to_point());
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
            vertexes[i]->inc_edges = edges[i];
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
            if (*v == p) {
               return v;
            }
         }
         return nullptr;
      }

      void delete_constraint(point_2t<Scalar> pa, point_2t<Scalar> pb) {
         segment_2t<Scalar> constraint_segment(pa, pb);
         for (int i = 0; i < constraints.size(); i++) {
            if (constraints[i]->to_segment() == constraint_segment) {
               auto constraint = constraints[i];
               constraints.erase(constraints.begin() + i);
               fix_edge(constraint);
               return;
            }
         }

      }

      void add_constraint(point_2t<Scalar> pa, point_2t<Scalar> pb) {
         add_constraint_without_fixing(pa, pb);
         segment_2t<Scalar> constraint_segment(pa, pb);
         for (auto e : get_edges()) {
            if (constraint_segment == e->to_segment()) {
               constraints.push_back(e);
               for (auto cur_e : get_edges()) fix_edge(cur_e);
               return;
            }
         }

      }
      std::vector<Edge<Scalar>> to_fix;
      void add_constraint_without_fixing(point_2t<Scalar> pa, point_2t<Scalar> pb, bool is_first = true) {
         segment_2t<Scalar> constraint_segment(pa, pb);
         auto face_a = faces[find_face_that_intersects_segment(find_face(pa).second.first, constraint_segment)];
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
            to_fix.push_back(flip(e));
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
         std::cout << "Faces" << std::endl;
         for (int i = 0; i < faces.size(); i++) {
            Face<Scalar> cur_face = faces[i];
            std::cout << cur_face->inc_edge->to_segment() << cur_face->inc_edge->next_edge->to_segment() << cur_face->inc_edge->prev_edge()->to_segment() << std::endl;
            if (cur_face->inc_edge->start->is_inf_point || cur_face->inc_edge->next_edge->start->is_inf_point ||
                cur_face->inc_edge->next_edge->next_edge->start->is_inf_point) continue;

            res.push_back(triangle_2t<Scalar>(cur_face->inc_edge->start->to_point(),
                                              cur_face->inc_edge->next_edge->start->to_point(), cur_face->inc_edge->next_edge->next_edge->start->to_point()));
         }
         return res;
      }

      std::vector<Edge<Scalar>> get_edges() {
         std::vector<Edge<Scalar> > res;
         for (auto f : faces) {
            auto cur_edge = f->inc_edge;
            for (int i = 0; i < 3; i++) {
               res.push_back(cur_edge);
               cur_edge = cur_edge->next_edge;
            }
         }
         return res;
      }

      std::vector<point_2t<Scalar> > get_points() {
         std::vector<point_2t<Scalar> > res;
         for (auto v : vertexes) {
            if (v->is_inf_point) continue;
            res.push_back(v->to_point());
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

      void delete_point(point_2t<Scalar> p)
      {
         auto v = tr_cell.find_point(p);
         if (v != nullptr) tr_cell.delete_vertex(v);
      }

      void add_constraint(point_2t<Scalar> a, point_2t<Scalar> b)
      {
         tr_cell.add_constraint(a, b);
      }

      void delete_constraint(point_2t<Scalar> a, point_2t<Scalar> b)
      {
         tr_cell.delete_constraint(a, b);
      }

      std::vector<triangle_2t<Scalar> > get_delaunay_triangulation() {
         return tr_cell.get_triangulation();
      }

      std::vector<point_2t<Scalar> > get_points() {
         return tr_cell.get_points();
      }

   private:
      std::vector<point_2t<Scalar> > points;
      std::vector<triangle_2t<Scalar> > res;
      cell<Scalar> tr_cell;
   };

}
