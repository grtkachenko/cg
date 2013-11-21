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


struct delaunay_viewer : cg::visualization::viewer_adapter
{
   delaunay_viewer()
   {
//      tr.add_point(point_2(-50, 50));
//      tr.add_point(point_2(-50, -50));
//      tr.add_point(point_2(50, -50));
//      tr.add_point(point_2(50, 50));
//      tr.add_point(point_2(0, 84));
//      tr.add_point(point_2(-10, -86));

//      tr.add_point(point_2(-30, -10));
//      tr.add_point(point_2(0, 1));
//      tr.add_point(point_2(30, 0));
//      tr.add_point(point_2(60, -10));

      tr.add_point(point_2(-30, 0));
      tr.add_point(point_2(30, 0));
      tr.add_point(point_2(0, 50));
      tr.add_point(point_2(-10, 80));
      tr.add_point(point_2(12, 76));


      make_triangulation();

//      tr.add_point(point_2(0, -10));
   }

   void draw(cg::visualization::drawer_type & drawer) const
   {
      drawer.set_color(Qt::white);

      drawer.set_color(Qt::green);
      for (triangle_2 t : res) {
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



   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "press mouse rbutton to add point" << cg::visualization::endl;
      p.corner_stream() << "Is it what we are looking for? " << (check_triangulation() ? "YES!!!" : "No :(") << cg::visualization::endl;

   }

   bool check_triangulation() const {
      bool ans = true;
      for (triangle_2 t : res) {
         bool cur = true;
         for (triangle_2 help_t : res) {
            for (int i = 0; i < 3; i++) {
               if (cur && t[0] != help_t[i] && t[1] != help_t[i] && t[2] != help_t[i] && is_inside(t[0], t[1], t[2], help_t[i])) {
                  ans = cur = false;
               }
            }
         }
      }
      return ans;
   }

   bool check_triangulation1() {
      bool ans = true;
      for (triangle_2 t : res) {
         bool cur = true;
         for (triangle_2 help_t : res) {
            for (int i = 0; i < 3; i++) {
               if (cur && t[0] != help_t[i] && t[1] != help_t[i] && t[2] != help_t[i] && is_inside(t[0], t[1], t[2], help_t[i])) {
                  red_res.push_back(t);
                  ans = cur = false;
               }
            }
         }
      }
      return ans;
   }

   bool is_inside(point_2 a, point_2 b, point_2 c, point_2 d) const {
      double a11 = a.x - d.x, a12 = a.y - d.y, a13 = (a.x * a.x - d.x * d.x) + (a.y * a.y - d.y * d.y);
      double a21 = b.x - d.x, a22 = b.y - d.y, a23 = (b.x * b.x - d.x * d.x) + (b.y * b.y - d.y * d.y);
      double a31 = c.x - d.x, a32 = c.y - d.y, a33 = (c.x * c.x - d.x * d.x) + (c.y * c.y - d.y * d.y);
      return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31) > 0;
   }



   void make_triangulation() {
      res = tr.get_delaunay_triangulation();
//      for (auto t : res) std::cout << t << std::endl;
   }

   bool on_release(const point_2f & p)
   {
      tr.add_point(p);

      make_triangulation();
      red_res.clear();
      check_triangulation1();

      return true;
   }

private:
   cg::delaunay_triangulation<double> tr;
   std::vector<triangle_2> res;
   std::vector<triangle_2> red_res;
   std::vector<point_2> pts;

};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   delaunay_viewer viewer;
   cg::visualization::run_viewer(&viewer, "delaunay_viewer");
}
