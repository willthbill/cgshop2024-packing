#include <bits/stdc++.h>
#include <CGAL/minkowski_sum_2.h>

#include "lib2/configuration_space.h"
#include "lib2/util.h"

#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

ConfigurationSpace::ConfigurationSpace(Polygon_set& s, Polygon& _pol, Point ref) {
    Polygon pol = scale_polygon(translate_point_to_origin(_pol, ref), -1);
    /*debug(ref);
    int tt = 0;
    foe(p, pol) {
        debug(p);
    }
    int t = 0;*/
    foe(pwh, to_polygon_vector(s)) {

        //debug(pwh.number_of_holes());
        //foe(p, pwh.outer_boundary()) debug(p);
        //int ttt = 0;

        auto sum = CGAL::minkowski_sum_2(pwh, pol);

        /*foe(p, sum.outer_boundary()) {
            debug(p);
        }*/
        /*debug(sum.number_of_holes());
        foe(h, sum.holes()) debug(h.area());*/

        space.join(sum);
    }
    space.complement();
}

Polygon ConfigurationSpace::get_single_polygon() {
    assert(space.number_of_polygons_with_holes() == 1);
    foe(pwh, to_polygon_vector(space)) {
        assert(pwh.number_of_holes() == 0);
        return pwh.outer_boundary();
    }
    assert(false);
}