#include <gtest/gtest.h>
#include <cg/operations/douglas.h>
#include <cg/primitives/point.h>
#include <iterator>
#include <vector>
using namespace std;
TEST(douglas, simple_test)
{
	vector<cg::point_2> pts, ans;
	pts.push_back(cg::point_2(0, 0));
	pts.push_back(cg::point_2(4, 2));
	pts.push_back(cg::point_2(50, 1));
	pts.push_back(cg::point_2(6, 0));
	simplify(pts.begin(), pts.end(), 200, back_inserter(ans));
	for (cg::point_2 p : ans) {
		cout << p.x << " " << p.y << endl;
	}
	EXPECT_FALSE(false);
}

