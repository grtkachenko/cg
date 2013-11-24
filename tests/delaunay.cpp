#include <gtest/gtest.h>
#include <cg/triangulation/delaunay.h>
#include "random_utils.h"

#include <iostream>
#include <cg/io/triangle.h>

using namespace std;
using namespace cg;

bool is_delaunay_triang(const vector<triangle_2> & faces, const vector<point_2> & pts)
{
   delaunay_triangulation<double> tr;
   for(triangle_2 t : faces)
      for(point_2 s : pts)
         if(t[0] != s && t[1] != s && t[2] != s && tr.is_inside(t[0], t[1], t[2], s))
         {
            cout << t << " contains " << s << endl;
            return false;
         }
   return true;
}

TEST(delaunay, uniform)
{
   for(size_t i = 0; i < 10; ++i)
   {
      vector<point_2> pts = uniform_points(100);
      delaunay_triangulation<double> tr;
      for(auto pt : pts)
      {
         tr.add_point(pt);
         vector<triangle_2> faces = tr.get_delaunay_triangulation();
         vector<point_2> points = tr.get_points();
         EXPECT_TRUE(is_delaunay_triang(faces, points));
      }
      for(auto pt : pts)
      {
         tr.delete_point(pt);
         vector<triangle_2> faces = tr.get_delaunay_triangulation();
         vector<point_2> points = tr.get_points();
         EXPECT_TRUE(is_delaunay_triang(faces, points));
      }
   }
}
