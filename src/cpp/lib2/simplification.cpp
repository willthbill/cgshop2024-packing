#include<bits/stdc++.h>
#include <CGAL/Polyline_simplification_2/simplify.h>

#include "lib/util/cgal.h"
#include "lib/util/geometry_utils.h"

namespace PS = CGAL::Polyline_simplification_2;
typedef PS::Stop_above_cost_threshold        Stop;
typedef PS::Squared_distance_cost            Cost;

using namespace std;

int get_number_of_vertices(Polygon_set pset) {
    int res = 0;
    foe(pwh, to_polygon_vector(pset)) {
        res += pwh.outer_boundary().size();
        foe(hole, pwh.holes()) {
            res += hole.size();
        }
    }
    return res;
}

bool is_completely_inside(Polygon_set a, Polygon_set b) {
    Polygon_set t; t.difference(b,a);
    return t.is_empty();
}

class SimplifyExpand {
public:
    static Polygon_set run(Polygon_set& pset) {
    }
    static Polygon run(Polygon& pol, bool inside=false) {
        // Built unit square
        Polygon square;
        {
            square.push_back(Point(0,0));
            square.push_back(Point(1,0));
            square.push_back(Point(1,1));
            square.push_back(Point(0,1));
        }
        // Get configuration space for the unit square
        Polygon_with_holes annulus;
        if(inside) {
            auto t = get_complement(Polygon_set(pol));
            pol.reverse_orientation();
            auto sum = Polygon_set(CGAL::minkowski_sum_2(pol, square));
            if(sum.number_of_holes() == 0) {
                return Polygon();
           }
        } else {
            auto sum = Polygon_set(CGAL::minkowski_sum_2(pol, square));
            sum.difference(pol);
            assert(sum.number_of_polygons_with_holes() == 1);
            annulus = to_polygon_vector(sum)[0];
        }
        assert(annulus.number_of_holes() == 1);
        /*auto config_space = ConfigurationSpace(
            Polygon_set(pwh),
            square,
            get_centroid(square)
        ).space;*/
        // Simplify configuration space
        Polygon_with_holes polygon = PS::simplify(polygon, Cost(), Stop(0.25));
        // Make it integral
        polygon = get_complement(
            SnapToGrid(get_complement(polygon)).space
        );
        return polygon;
    }
    static Polygon_with_holes run(Polygon_with_holes& pwh) {
        cout << "[c++] Simplication input number of vertices: " << get_number_of_vertices(Polygon_set(pwh)) << endl;
        Polygon square;
        {
            square.push_back(Point(0,0));
            square.push_back(Point(1,0));
            square.push_back(Point(1,1));
            square.push_back(Point(0,1));
        }
        auto sum = CGAL::minkowski_sum_2(pwh, square);
        Polygon_with_holes polygon = PS::simplify(sum, Cost(), Stop(0.25 * 0.25));
        assert(is_completely_inside(Polygon_set(polygon), Polygon_set(pwh)));
        cout << "[c++] Simplication output number of vertices: " << get_number_of_vertices(Polygon_set(polygon)) << endl;
        return polygon;
    }
};