#include <gtest/gtest.h>
#include <cg/triangulation/delaunay.h>
#include "random_utils.h"

#include <iostream>
#include <cg/io/triangle.h>
#include <cg/triangulation/is_inside.h>

using namespace std;
using namespace cg;

bool is_delaunay_triang(const vector<triangle_2> & faces, const vector<point_2> & pts)
{
   for(triangle_2 t : faces)
      for(point_2 s : pts)
         if(t[0] != s && t[1] != s && t[2] != s && is_inside(t[0], t[1], t[2], s))
         {
            cout << t << " contains " << s << endl;
            return false;
         }
   return true;
}

TEST(delaunay, uniform)
{
   vector<point_2> pts = uniform_points(200);
   delaunay_triangulation<double> tr;
   for(auto pt : pts)
   {
      tr.add_point(pt);
   }
   vector<triangle_2> faces = tr.get_delaunay_triangulation();
   vector<point_2> points = tr.get_points();
   EXPECT_TRUE(is_delaunay_triang(faces, points));
   for(auto pt : pts)
   {
      tr.delete_point(pt);
   }
   faces = tr.get_delaunay_triangulation();
   points = tr.get_points();
   EXPECT_TRUE(is_delaunay_triang(faces, points));

}


TEST(delaunay, circle)
{
   const double PI = acos(-1);
   const size_t n = 60;
   const double r = 100;

   delaunay_triangulation<double> tr;

   for(size_t i = 0; i != n; ++i)
   {
      double angle = PI * 2 * i / n;
      double x = cos(angle) * r;
      double y = sin(angle) * r;
      tr.add_point({x, y});
   }
   vector<triangle_2> faces = tr.get_delaunay_triangulation();
   vector<point_2> points = tr.get_points();
   EXPECT_TRUE(is_delaunay_triang(faces, points));

   srand(time(0));

   points = tr.get_points();
   std::random_shuffle(points.begin(), points.end());

   for(size_t i = 0; i != n; ++i)
   {
      tr.delete_point(points[i]);
   }
   faces = tr.get_delaunay_triangulation();
   points = tr.get_points();
   EXPECT_TRUE(is_delaunay_triang(faces, points));
}

TEST(delaunay, random_vertical_line)
{
   const size_t n = 100;
   delaunay_triangulation<double> tr;

   srand(time(0));

   tr.add_point({0, 0});
   for(size_t i = 0; i < n; ++i)
   {
      point_2 p = {rand(), 0};
      tr.add_point(p);
   }
   vector<triangle_2> faces = tr.get_delaunay_triangulation();
   vector<point_2> points = tr.get_points();
   EXPECT_TRUE(is_delaunay_triang(faces, points));

//   vector<point_2> pts = tr.get_points();

//   for(auto pt : pts)
//   {
//      tr.del_vertex(pt);
//      vector<triangle_2> faces = tr.get_triangulation();
//      vector<point_2> points = tr.get_points();
//      EXPECT_TRUE(is_delaunay_triang(faces, points));
//   }
}

TEST(delaunay, random_horizontal_line)
{
   const size_t n = 100;
   delaunay_triangulation<double> tr;

   srand(time(0));
   tr.add_point({0, 0});
   for(size_t i = 0; i < n; ++i)
   {
      tr.add_point({0, rand()});
   }
   vector<triangle_2> faces = tr.get_delaunay_triangulation();
   vector<point_2> points = tr.get_points();
   EXPECT_TRUE(is_delaunay_triang(faces, points));

//   vector<point_2> pts = tr.get_points();

//   for(auto pt : pts)
//   {
//      tr.del_vertex(pt);
//      vector<triangle_2> faces = tr.get_triangulation();
//      vector<point_2> points = tr.get_points();
//      EXPECT_TRUE(is_delaunay_triang(faces, points));
//   }
}

TEST(delaunay, random_vertical_and_horizontal_line)
{
   const size_t n = 100;
   delaunay_triangulation<double> tr;

   srand(time(0));
   tr.add_point({0, 0});
   for(size_t i = 0; i < n; ++i)
   {
      if (rand() % 2) {
         tr.add_point({0, rand()});
      } else {
         tr.add_point({rand(), 0});
      }

   }
   vector<triangle_2> faces = tr.get_delaunay_triangulation();
   vector<point_2> points = tr.get_points();
   for (auto p : tr.get_points()) std::cout << p << std::endl;
   EXPECT_TRUE(is_delaunay_triang(faces, points));

//   vector<point_2> pts = tr.get_points();

//   for(auto pt : pts)
//   {
//      tr.del_vertex(pt);
//      vector<triangle_2> faces = tr.get_triangulation();
//      vector<point_2> points = tr.get_points();
//      EXPECT_TRUE(is_delaunay_triang(faces, points));
//   }
}
