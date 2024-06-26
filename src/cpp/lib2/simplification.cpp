#include<bits/stdc++.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/minkowski_sum_2.h>

namespace PS = CGAL::Polyline_simplification_2;
typedef PS::Stop_above_cost_threshold        Stop;
typedef PS::Squared_distance_cost            Cost;

#include "lib2/simplification.h"
#include "lib2/util.h"
#include "lib/util/cgal.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

Polygon_set SimplifyExpand::run(Polygon_set& pset, FT scale) {
    Polygon_set res;
    foe(pwh, to_polygon_vector(pset)) {
        res.join(run(pwh, scale));
    }
    return res;
}

Polygon SimplifyExpand::run(Polygon& pol, FT scale) {
    Polygon_with_holes t (pol);
    auto res = run(t, scale);
    assert(res.number_of_holes() == 0);
    return res.outer_boundary();
}

Polygon_with_holes SimplifyExpand::run(Polygon_with_holes& pwh, FT scale) {
    // assert(pwh.outer_boundary().is_simple());
    // cout << "[c++] Simplication input number of vertices: " << get_number_of_vertices(Polygon_set(pwh)) << endl;
    /*debug("input");
    foe(p, pwh.outer_boundary()) {
        debug(p);
    }*/
    Polygon square;
    FT factor = 1;
    {
        square.push_back(Point(-factor * scale,-scale * factor));
        square.push_back(Point(factor * scale,-scale * factor));
        square.push_back(Point(factor * scale,scale * factor));
        square.push_back(Point(-factor * scale,scale * factor));
    }
    auto sum = CGAL::minkowski_sum_2(pwh, square);
    /*debug("sum");
    foe(p, sum.outer_boundary()) {
        debug(p);
    }*/
    Polygon_with_holes polygon = PS::simplify(sum, Cost(), Stop((scale * 2).to_double()));
    /*debug("res");
    foe(p, polygon.outer_boundary()) {
        debug(p);
    }*/
    // cout << "[c++] Simplication output number of vertices: " << get_number_of_vertices(Polygon_set(polygon)) << endl;
    assert(is_completely_inside(Polygon_set(polygon), Polygon_set(pwh)));
    //assert(polygon.outer_boundary().is_simple());
    return polygon;
}