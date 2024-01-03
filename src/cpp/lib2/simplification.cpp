#include<bits/stdc++.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/minkowski_sum_2.h>

namespace PS = CGAL::Polyline_simplification_2;
typedef PS::Stop_above_cost_threshold        Stop;
typedef PS::Squared_distance_cost            Cost;

#include "lib2/simplification.h"
#include "lib/util/cgal.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

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
    cout << "[c++] Simplication input number of vertices: " << get_number_of_vertices(Polygon_set(pwh)) << endl;
    /*debug("input");
    foe(p, pwh.outer_boundary()) {
        debug(p);
    }*/
    Polygon square;
    {
        square.push_back(Point(-scale,-scale));
        square.push_back(Point(scale,-scale));
        square.push_back(Point(scale,scale));
        square.push_back(Point(-scale,scale));
    }
    auto sum = CGAL::minkowski_sum_2(pwh, square);
    /*debug("sum");
    foe(p, sum.outer_boundary()) {
        debug(p);
    }*/
    Polygon_with_holes polygon = PS::simplify(sum, Cost(), Stop((scale * 1.99).to_double()));
    /*debug("res");
    foe(p, polygon.outer_boundary()) {
        debug(p);
    }*/
    cout << "[c++] Simplication output number of vertices: " << get_number_of_vertices(Polygon_set(polygon)) << endl;
    assert(is_completely_inside(Polygon_set(polygon), Polygon_set(pwh)));
    return polygon;
}