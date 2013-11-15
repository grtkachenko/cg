#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>
#include <math.h>
#include <vector>
#include <iostream>

struct Newtone_method_viewer : cg::visualization::viewer_adapter
{

   Newtone_method_viewer() : point_colors(MAX_POINT_X * 2), last_pos(MAX_POINT_X + 1, MAX_POINT_Y + 1)
   {
      for (int i = -MAX_POINT_X; i < MAX_POINT_X; i++) {
         for (int j = -MAX_POINT_Y; j < MAX_POINT_Y; j++) {
            point_colors[i + MAX_POINT_X].push_back(get_point_color(cg::point_2(i * SCALE, j * SCALE)));
         }
      }
   }

   void fix_point(cg::point_2 & p) const {
      const double square = p.x * p.x + p.y * p.y;
      double new_x = 2.0 / 3 * p.x + (p.x * p.x - p.y * p.y) / 3.0 / square / square;
      double new_y = 2.0 / 3 * p.y - 2.0 * p.x * p.y / 3 / square / square;
      p.x = new_x;
      p.y = new_y;
   }

   QColor get_point_color(cg::point_2 const & p) const {
      cg::point_2 copy_p(p);
      for (int i = 0; i < ITERATION_NUMBER; i++) fix_point(copy_p);

      for (int i = 0; i < 3; i++) {
         if (abs(copy_p.x - solutions[i].x) < EPS && abs(copy_p.y - solutions[i].y) < EPS) return colors[i];
      }

      return Qt::green;
   }

   void draw(cg::visualization::drawer_type & drawer) const
   {
      drawer.set_color(Qt::red);
      for (int i = -MAX_POINT_X; i < MAX_POINT_X; i++) {
         for (int j = -MAX_POINT_Y; j < MAX_POINT_Y; j++) {
            drawer.set_color(point_colors[i + MAX_POINT_X][j + MAX_POINT_Y]);
            drawer.draw_point(cg::point_2(i, j));
         }
      }

      if (abs(last_pos.x) <= MAX_POINT_X && abs(last_pos.y) <= MAX_POINT_Y) {
         drawer.set_color(Qt::black);
         cg::point_2 prev(last_pos.x * SCALE, last_pos.y * SCALE), cur(last_pos.x * SCALE, last_pos.y * SCALE);

         for (int i = 0; i < ITERATION_NUMBER; i++) {
            fix_point(cur);
            drawer.draw_line(cg::point_2(prev.x / SCALE, prev.y / SCALE), cg::point_2(cur.x / SCALE, cur.y / SCALE), 2);
            prev = cg::point_2(cur.x, cur.y);
         }
      }
   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "Newtone_method: " << cg::visualization::endl;
   }

   bool on_release(const cg::point_2f & p)
   {
      if (abs(p.x) <= MAX_POINT_X && abs(p.y) <= MAX_POINT_Y) {
         last_pos = p;
         return true;
      }
      return false;
   }

private:
   const double SCALE = 0.01;
   const static int MAX_POINT_X = 700;
   const static int MAX_POINT_Y = 700;
   const int ITERATION_NUMBER = 50;
   const cg::point_2 solutions[3] = {cg::point_2(1, 0), cg::point_2(-1 / 2, sqrt(3) / 2), cg::point_2(-1 / 2, -sqrt(3) / 2)};
   const QColor colors[3] = {Qt::red, Qt::blue, Qt::yellow};
   std::vector<std::vector<QColor> > point_colors;
   const double EPS = 0.1;
   cg::point_2 last_pos;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   Newtone_method_viewer viewer;
   cg::visualization::run_viewer(&viewer, "Newtone_method_viewer");
}
