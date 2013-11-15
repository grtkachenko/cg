#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>
#include <iostream>

struct si_method_viewer : cg::visualization::viewer_adapter
{
   si_method_viewer()
   {}

   double phi(double x, double r) const
   {
      return r * x * (1 - x);
   }

   void recalc()
   {
       pts_.clear();
       pts_.resize(10 * scale);
       for(size_t r = 0; r < pts_.size(); ++r)
       {
           size_t count_iterations = 100;
           size_t count_for_draw = 100;
           double y = y0;
           for(size_t i = 0; i < count_iterations; ++i)
               y = phi(y, static_cast<double>(r) / scale);
           for(size_t i = 0; i < count_for_draw; ++i)
           {
              y = phi(y, static_cast<double>(r) / scale);
              pts_[r].push_back(y);
           }
       }
   }

   void draw(cg::visualization::drawer_type & drawer) const
   {
       drawer.set_color(Qt::red);
       for(size_t r = 0; r < pts_.size(); ++r)
         for(size_t i = 0; i < pts_[r].size(); ++i)
         {
             drawer.draw_point(cg::point_2(static_cast<double>(r) / scale * 100, pts_[r][i] * 100));
             if(r == 5000)
                 std::cout << pts_[r][i] << std::endl;
         }
   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "si_method: " << std::to_string(y0) << " " << cg::visualization::endl
                        << pts_.size() << " " << (pts_.size() > 0 ? pts_[0].size() : 0);
   }

   bool on_key(int key)
   {
      switch (key)
      {
      case Qt::Key_Up : y0 += 0.5; break;
      case Qt::Key_Down : y0 -= 0.5; break;
      default : return false;
      }
      recalc();
      return true;
   }

private:
   std::vector < std::vector < double > > pts_;
   double y0 = 0.0;
   const int scale = 1000;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   si_method_viewer viewer;
   cg::visualization::run_viewer(&viewer, "si_method_viewer");
}
