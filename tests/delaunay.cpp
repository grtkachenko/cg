#include <gtest/gtest.h>
#include <cg/triangulation/delaunay.h>
#include "random_utils.h"

#include <iostream>
#include <cg/io/triangle.h>
#include <cg/triangulation/is_inside.h>

using namespace std;
using namespace cg;

bool check_for_triangulation(delaunay_triangulation<double> & tr) {
   vector<triangle_2> faces = tr.get_delaunay_triangulation();
   vector<point_2> pts = tr.get_points();
   for(triangle_2 t : faces)
      for(point_2 s : pts)
         if(t[0] != s && t[1] != s && t[2] != s && is_inside(t[0], t[1], t[2], s)) {
            cout << t << " contains " << s << endl;
            return false;
         }
   return true;
}

TEST(delaunay, uniform)
{
   vector<point_2> pts = uniform_points(200);
   delaunay_triangulation<double> tr;
   for(auto pt : pts) {
      tr.add_point(pt);
   }
   EXPECT_TRUE(check_for_triangulation(tr));
   for(auto pt : pts) {
      tr.delete_point(pt);
   }
   EXPECT_TRUE(check_for_triangulation(tr));
}


TEST(delaunay, circle)
{
   const double PI = acos(-1);
   const int n = 120;
   const double r = 100;
   delaunay_triangulation<double> tr;

   for(int i = 0; i < n; i++) {
      double angle = PI * 2 * i / n;
      double x = cos(angle) * r;
      double y = sin(angle) * r;
      tr.add_point({x, y});
   }
   vector<triangle_2> faces = tr.get_delaunay_triangulation();
   vector<point_2> points = tr.get_points();
   EXPECT_TRUE(check_for_triangulation(tr));
   srand(time(0));
   points = tr.get_points();
   std::random_shuffle(points.begin(), points.end());

   for (int i = 0; i < n; i++) {
      tr.delete_point(points[i]);
   }
   EXPECT_TRUE(check_for_triangulation(tr));
}

TEST(delaunay, random_vertical_line) {
   const int n = 100;
   delaunay_triangulation<double> tr;
   srand(time(0));

   tr.add_point({0, 0});
   for (int i = 0; i < n; i++) {
      tr.add_point({rand(), 0});
   }
   EXPECT_TRUE(check_for_triangulation(tr));

   vector<point_2> pts = tr.get_points();

   for(auto pt : pts) {
      tr.delete_point(pt);
   }
   EXPECT_TRUE(check_for_triangulation(tr));
}


TEST(delaunay, random_horizontal_line) {
   const int n = 100;
   delaunay_triangulation<double> tr;
   srand(time(0));

   tr.add_point({0, 0});
   for (int i = 0; i < n; i++) {
      tr.add_point({0, rand()});
   }
   EXPECT_TRUE(check_for_triangulation(tr));

   vector<point_2> pts = tr.get_points();

   for(auto pt : pts) {
      tr.delete_point(pt);
   }
   EXPECT_TRUE(check_for_triangulation(tr));
}


TEST(delaunay, random_vertical_and_horizontal_line)
{
   const size_t n = 600;
   delaunay_triangulation<double> tr;

   srand(time(0));
   tr.add_point({0, 0});
   for (int i = 0; i < n; i++) {
      if (rand() % 2) {
         tr.add_point({0, rand()});
      } else {
         tr.add_point({rand(), 0});
      }
   }
   EXPECT_TRUE(check_for_triangulation(tr));
   vector<point_2> points = tr.get_points();
   std::random_shuffle(points.begin(), points.end());

   for (int i = 0; i < n; i++) {
      tr.delete_point(points[i]);
   }
   EXPECT_TRUE(check_for_triangulation(tr));
}
