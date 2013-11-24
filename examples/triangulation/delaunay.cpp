#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>

#include <cg/triangulation/delaunay.h>
#include <cg/io/triangle.h>


using cg::point_2f;
using cg::point_2;
using cg::triangle_2;
using cg::segment_2;

struct delaunay_viewer : cg::visualization::viewer_adapter
{
   delaunay_viewer() : is_correct_triangulation(true) {
//      tr.add_point(point_2(0, 1));
//      tr.add_point(point_2(30, 60));
//      tr.add_point(point_2(60, 0));
//      tr.add_point(point_2(90, 60));
//      tr.add_point(point_2(120, 40));
      make_triangulation();
   }

   void draw(cg::visualization::drawer_type & drawer) const
   {

      index_of_current_point = -1;
      for (int i = 0; i < pts.size(); i++) {
         point_2 p = pts[i];
         bool same = (abs(p.x - cur.x) < eps && abs(p.y - cur.y) < eps);\
         if (same) {
            index_of_current_point = i;
         }
         if (std::find(selected_points.begin(), selected_points.end(), p) != selected_points.end()) continue;

         drawer.set_color(!same ? Qt::white : Qt::yellow);
         drawer.draw_point(p, !same ? 6 : 12);
      }
      drawer.set_color(Qt::blue);
      for (triangle_2 t : res) {
         if (std::find(red_res.begin(), red_res.end(), t) != red_res.end()) continue;
         drawer.draw_line(t[0], t[1]);
         drawer.draw_line(t[0], t[2]);
         drawer.draw_line(t[2], t[1]);
      }
      drawer.set_color(Qt::red);
      for (triangle_2 t : red_res) {
         drawer.draw_line(t[0], t[1]);
         drawer.draw_line(t[0], t[2]);
         drawer.draw_line(t[2], t[1]);
      }


      drawer.set_color(Qt::red);
      for (point_2 p : selected_points) drawer.draw_point(p, 12);

      drawer.set_color(Qt::white);
      for (auto s : constraints) drawer.draw_line(s[0], s[1]);

   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "press mouse rbutton on free space to add point" << cg::visualization::endl;
      p.corner_stream() << "press mouse rbutton on point to complete the constraint edge" << cg::visualization::endl;
      p.corner_stream() << "Is it what we are looking for? " << (is_correct_triangulation ? "YES!!!" : "No :(") << cg::visualization::endl;

   }

   bool check_triangulation() {
      bool ans = true;
      for (triangle_2 t : res) {
         bool cur = true;
         for (triangle_2 help_t : res) {
            for (int i = 0; i < 3; i++) {
               if (cur && t[0] != help_t[i] && t[1] != help_t[i] && t[2] != help_t[i] && tr.is_inside(t[0], t[1], t[2], help_t[i])) {
                  red_res.push_back(t);
                  ans = cur = false;
               }
            }
         }
      }
      return ans;
   }

   void make_triangulation() {
      res = tr.get_delaunay_triangulation();
   }

   bool on_release(const point_2f & p)
   {
      if (index_of_current_point != -1) {
         selected_points.push_back(pts[index_of_current_point]);
         if (!check_constaint_for_ready()) {
             return true;
         }

      } else {
         pts.push_back(p);
         tr.add_point(p);
      }

      make_triangulation();
      red_res.clear();
      is_correct_triangulation = check_triangulation();
      return true;
   }

   bool check_constaint_for_ready() {
      if (selected_points.size() == 2) {
         if (selected_points[0] == selected_points[1]) {
            std::cout << "Deleting point " << selected_points[0] << std::endl;

            if (in_constraint[selected_points[0]] == 0) {
               tr.delete_point(selected_points[0]);
               pts.erase(std::find(pts.begin(), pts.end(), selected_points[0]));
            }
            selected_points.clear();
            return true;
         } else {
            if (in_constraint[selected_points[0]] != 0 && in_constraint[selected_points[1]] != 0) {
               tr.delete_constraint(selected_points[0], selected_points[1]);
               in_constraint[selected_points[0]]--;
               in_constraint[selected_points[1]]--;
               constraints.erase(std::find(constraints.begin(), constraints.end(), segment_2(selected_points[0], selected_points[1])));
            } else {
               tr.add_constraint(selected_points[0], selected_points[1]);
               in_constraint[selected_points[0]]++;
               in_constraint[selected_points[1]]++;
               constraints.push_back(segment_2(selected_points[0], selected_points[1]));
            }
            selected_points.clear();
            return true;
         }
      }
      return true;
   }

   bool on_move(const point_2f & p)
   {
      cur = p;
      return true;
   }


private:
   cg::delaunay_triangulation<double> tr;
   std::vector<triangle_2> res;
   std::vector<triangle_2> red_res;
   std::vector<point_2> pts;
   bool is_correct_triangulation;
   point_2 cur;
   mutable int index_of_current_point;
   std::vector<point_2> selected_points;
   std::map<point_2, int> in_constraint;
   std::vector<segment_2> constraints;
   const double eps = 8;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   delaunay_viewer viewer;
   cg::visualization::run_viewer(&viewer, "delaunay_viewer");
}
