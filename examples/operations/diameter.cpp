#include <QColor>
#include <QApplication>

#include "cg/visualization/viewer_adapter.h"
#include "cg/visualization/draw_util.h"

#include "cg/io/point.h"

#include <cg/primitives/triangle.h>
#include <cg/primitives/point.h>
#include <cg/operations/diameter.h>

#include <vector>

using cg::point_2f;
using cg::point_2;


struct diameter_viewer : cg::visualization::viewer_adapter
{
   diameter_viewer() {}

	void draw(cg::visualization::drawer_type & drawer) const override
   {
      drawer.set_color(Qt::white);
      for (point_2 p : pts) {
			drawer.draw_point(p);
      }
		auto d = cg::get_diameter(pts.begin(), pts.end());
		if (pts.size() != 0) {
			drawer.draw_line(*(d.first), *(d.second));
		}
   }

   void print(cg::visualization::printer_type & p) const override
   {
      p.corner_stream() << "Press mouse lbutton to make a new point"
								<< cg::visualization::endl;
   }

	bool on_press(const point_2f & p) override
   {
      pts.push_back(p);
		return true;
   }

private:
	mutable std::vector<point_2> pts;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
	diameter_viewer viewer;
   cg::visualization::run_viewer(&viewer, "Diameter of set of points");
}
