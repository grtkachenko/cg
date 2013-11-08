#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>

#include <cg/triangulation/delaunay.h>


using cg::point_2f;
using cg::point_2;
using cg::triangle_2;


struct delaunay_viewer : cg::visualization::viewer_adapter
{
   delaunay_viewer()
   {}

   void draw(cg::visualization::drawer_type & drawer) const
   {
      drawer.set_color(Qt::white);
      for (point_2 const & p : pts)
         drawer.draw_point(p);

      drawer.set_color(Qt::green);
      for (triangle_2 t : res) {
         drawer.draw_line(t[0], t[1]);
         drawer.draw_line(t[0], t[2]);
         drawer.draw_line(t[2], t[1]);

      }
   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "press mouse rbutton to add point" << cg::visualization::endl;
   }

   void make_triangulation() {
      res = cg::get_delaunay_triangulation(pts);
   }

   bool on_release(const point_2f & p)
   {
      pts.push_back(p);
      make_triangulation();
      return true;
   }

private:
   std::vector<point_2> pts;
   std::vector<triangle_2> res;

};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   delaunay_viewer viewer;
   cg::visualization::run_viewer(&viewer, "delaunay_viewer");
}
