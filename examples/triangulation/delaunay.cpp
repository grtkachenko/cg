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
   delaunay_viewer() : is_correct_triangulation(true), draw_circle_mode(false) {
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

      if (draw_circle_mode && index_of_current_point != -1) {
         for (auto t : res) {
            for (int i = 0; i < 3; i++) {
               if (t[i] == pts[index_of_current_point]) {
                  draw_circle(drawer, t);
                  break;
               }
            }
         }
      }
   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "press mouse rbutton on free space to add point" << cg::visualization::endl;
      p.corner_stream() << "press mouse rbutton on point TWICE to delete point" << cg::visualization::endl;
      p.corner_stream() << "press mouse rbutton on point to complete the constraint edge" << cg::visualization::endl;
      p.corner_stream() << "press mouse rbutton on the ends of white edge(constraint) to delete it" << cg::visualization::endl;
      p.corner_stream() << "Is it a correct triangulation? " << (is_correct_triangulation ? "YES!!!" : "No :(") << cg::visualization::endl;
      p.corner_stream() << "PRESS 'C' to change mode to 'draw circle mode' (now it is" << (draw_circle_mode ? "ON)" : "OFF)") << cg::visualization::endl;

   }

   bool check_triangulation() {
      bool ans = true;
      for (triangle_2 t : res) {
         bool cur = true;
         for (triangle_2 help_t : res) {
            for (int i = 0; i < 3; i++) {
               if (cur && t[0] != help_t[i] && t[1] != help_t[i] && t[2] != help_t[i] && cg::is_inside(t[0], t[1], t[2], help_t[i])) {
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

   double area(triangle_2 const & tr) const {
          double res = 0;
          for (int i = 0; i < 3; i++) {
              double dx = tr[i].x - tr[(i+1)%3].x;
              double dy = tr[i].y + tr[(i+1)%3].y;
              res += dx*dy;
          }
          return fabs(res) / 2.;
      }

   double distance(point_2 const & p1, point_2 const & p2) const {
       double dx = p1.x - p2.x;
       double dy = p1.y - p2.y;
       return sqrt(dx*dx+dy*dy);
   }

   point_2 intersect(segment_2 const & s1, segment_2 const & s2) const {
       double a1 = s1[0].y - s1[1].y;
       double b1 = -s1[0].x + s1[1].x;
       double c1 = -(a1 * s1[0].x + b1 * s1[0].y);
       double a2 = s2[0].y - s2[1].y;
       double b2 = -s2[0].x + s2[1].x;
       double c2 = -(a2 * s2[0].x + b2 * s2[0].y);
       double zn = a1*b2-a2*b1;
       double x = (c2*b1-c1*b2) / zn;
       double y = (c1*a2-c2*a1) / zn;
       return point_2(x, y);
   }

   void draw_circle2(cg::visualization::drawer_type & drawer, point_2 const & c, double r) const {
       double PI = asin(1) * 2;
       const int MAX_ITER = 360;
       for (int it = 0; it< MAX_ITER; it++) {
           double angle1 = PI *2 *it / MAX_ITER;
           double angle2 = PI *2 * (it+1) / MAX_ITER;
           double x1 = c.x + r * cos(angle1);
           double x2 = c.x + r * cos(angle2);
           double y1 = c.y + r * sin(angle1);
           double y2 = c.y + r * sin(angle2);
           drawer.draw_line(point_2(x1, y1), point_2(x2, y2));
       }
   }

   void draw_circle(cg::visualization::drawer_type & drawer, triangle_2 const & tr) const {
       double s = area(tr);
       segment_2 s1, s2;
       {
           point_2 ab((tr[0].x+tr[1].x)/2, (tr[0].y+tr[1].y)/2);
           double dx = ab.x - tr[0].x;
           double dy = ab.y - tr[0].y;
           point_2 p2(ab.x + dy, ab.y - dx);
           s1 = segment_2(ab, p2);
       }
       {
           point_2 ac((tr[0].x+tr[2].x)/2, (tr[0].y+tr[2].y)/2);
           double dx = ac.x - tr[0].x;
           double dy = ac.y - tr[0].y;
           point_2 p2(ac.x + dy, ac.y - dx);
           s2 = segment_2(ac, p2);
       }
       point_2 center = intersect(s1, s2);
       double a = distance(tr[0], tr[1]);
       double b = distance(tr[1], tr[2]);
       double c = distance(tr[2], tr[0]);
       double r = a *b * c / 4 / s;
       draw_circle2(drawer, center, r);
   }

   bool on_move(const point_2f & p)
   {
      cur = p;
      return true;
   }

   bool on_key(int key)
   {
      if (key == Qt::Key_C) {
         draw_circle_mode = !draw_circle_mode;
         return true;
      }
      return true;
   }


private:
   cg::delaunay_triangulation<double> tr;
   std::vector<triangle_2> res, red_res;
   std::vector<point_2> pts, selected_points;
   bool is_correct_triangulation, draw_circle_mode;
   point_2 cur;
   mutable int index_of_current_point;
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
