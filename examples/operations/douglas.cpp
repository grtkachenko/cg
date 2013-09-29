#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>

#include "cg/visualization/viewer_adapter.h"
#include "cg/visualization/draw_util.h"

#include "cg/io/point.h"

#include <cg/primitives/triangle.h>
#include <cg/primitives/point.h>

#include <cg/operations/contains/contour_point.h>
#include <cg/operations/douglas.h>
#include <vector>
#include <algorithm>
#include <iterator>
using cg::point_2f;
using cg::point_2;


struct douglas : cg::visualization::viewer_adapter
{
	douglas(std::vector<point_2> pts) : pts(pts)

   {}

   void draw(cg::visualization::drawer_type& drawer) const override
   {
		drawer.set_color(Qt::white);
		draw_chain(pts, drawer, Qt::white, Qt::white);

		std::vector<point_2> ans;
		simplify(pts.begin(), pts.end(), current_eps, back_inserter(ans));
		for (point_2 p : ans) {
			std::cout << p.x << " " << p.y << std::endl;
		}
		draw_chain(ans, drawer, Qt::green, Qt::green);
	}

	void draw_chain(std::vector<point_2> pts, cg::visualization::drawer_type& drawer, QColor color_l, QColor color_p) const {
		point_2 last;
		bool first = true;
		for (point_2 p : pts)
		{
			if (!first) {
				drawer.set_color(color_l);
				drawer.draw_line(last, p);
			}
			drawer.set_color(color_p);
			drawer.draw_point(p);
			last = p;
			first = false;
		}
	}

   void print(cg::visualization::printer_type& p) const override
   {
		p.corner_stream() << "press mouse lbutton to put the vertex"
                        << cg::visualization::endl
                        << "current eps : "
                        << current_eps
                        << cg::visualization::endl;
   }

   bool on_key(int key)
   {
      switch (key)
      {
		case Qt::Key_Up :
			current_eps += 5;
         break;

		case Qt::Key_Down :
			if (current_eps != 0) current_eps -=5;
			break;

      default :
         return false;
      }

      return true;
   }

	bool on_press(const point_2f & p)
	{
		pts.push_back(p);
		return true;
	}

private:
	double current_eps = 10;
	std::vector<point_2> pts, ans;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
	std::vector<cg::point_2> pts;


	douglas viewer(pts);
   cg::visualization::run_viewer(&viewer, "Douglas algo check");
}
