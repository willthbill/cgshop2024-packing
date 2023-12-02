#include<bits/stdc++.h>
#include <CGAL/Gmpq.h>

#include "lib2/snap.h"
#include "lib/geometry/intersection_predicates.h"
#include "lib2/util.h"
#include "lib/util/cgal.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/common.h"

using namespace std;

// TODO: somehow specify the grid size
// TODO: support polygon set
SnapToGrid::SnapToGrid(Polygon_set pset) {
    foe(pwh, to_polygon_vector(pset)) {
        assert(pwh.number_of_holes() == 0);
        space.join(snap(pwh.outer_boundary()));
    }
}
Polygon SnapToGrid::get_single_polygon() {
    assert(space.number_of_polygons_with_holes() == 1);
    foe(pwh, to_polygon_vector(space)) {
        assert(pwh.number_of_holes() == 0);
        return pwh.outer_boundary();
    }
    assert(false);
}
bool is_completely_inside(Polygon a, Polygon b) {
    Polygon_set intersection; intersection.intersection(
        to_polygon_set(a),
        to_polygon_set(b)
    );
    auto arr = to_polygon_vector(intersection);
    FT res = 0;
    foe(p, arr) res += p.outer_boundary().area();
    return res == b.area();
}
Polygon SnapToGrid::snap(Polygon pol) {
    Polygon_set pset;
    pset.insert(pol);
    foe(_p, pol) {
        Point p = _p;
        if(p.x() == floor_exact(p.x())) {
            p = Point(p.x() + 0.5, p.y());
        }
        if(p.y() == floor_exact(p.y())) {
            p = Point(p.x(), p.y() + 0.5);
        }
        foab(i, -1, 1) {
            foab(j, -1, 1) {
                Polygon box;
                box.push_back(Point(
                    floor_exact(p.x()) + i,
                    floor_exact(p.y()) + j
                ));
                box.push_back(Point(
                    ceil_exact(p.x()) + i,
                    floor_exact(p.y()) + j
                ));
                box.push_back(Point(
                    ceil_exact(p.x()) + i,
                    ceil_exact(p.y()) + j
                ));
                box.push_back(Point(
                    floor_exact(p.x()) + i,
                    ceil_exact(p.y()) + j
                ));
                pset.join(box);
            }
        }
    }
    assert(pset.number_of_polygons_with_holes() == 1);
    Polygon ep;
    foe(pwh, to_polygon_vector(pset)) {
        // assert(pwh.number_of_holes() == 0);
        ep = pwh.outer_boundary();
    }
    Polygon res;
    foe(p, ep) {
        bool is_int = is_integer(p.x()) && is_integer(p.y());
        if(is_int) {
            res.push_back(p);
        }
    }
    // TODO: remove redundant points
    //IntersectionPredicates pred (res);
    //assert(pred.is_completely_inside_slow(pol));
    assert(is_completely_inside(res,pol));
    return res;
}
