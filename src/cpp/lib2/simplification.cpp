#include<bits/stdc++.h>
#include <CGAL/Polyline_simplification_2/simplify.h>

#include "lib/util/cgal.h"
#include "lib/util/geometry_utils.h"

namespace PS = CGAL::Polyline_simplification_2;
typedef PS::Stop_below_count_ratio_threshold Stop;
typedef PS::Squared_distance_cost            Cost;

using namespace std;

class SimplifyExpand {
public:
    static Polygon_set run(Polygon_set& pset) {
    }
    static Polygon run(Polygon& pol) {
        // Built unit square
        Polygon square;
        {
            square.push_back(Point(0,0));
            square.push_back(Point(1,0));
            square.push_back(Point(1,1));
            square.push_back(Point(0,1));
        }
        // Get configuration space for the unit square
        auto sum = CGAL::minkowski_sum_2(pol, square);
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
    }
};