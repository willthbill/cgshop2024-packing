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
    foe(_pwh, to_polygon_vector(s)) {

        auto pwh = _pwh;

        /*auto t = Polygon_set(pwh);
        bool is_inf_outerboundary = area(t) < FT(0); // meaning it has an infinite outer boundary
        FT inf = 1e30;
        Polygon square; square.pb(Point(-inf,-inf));square.pb(Point(inf,-inf));square.pb(Point(inf,inf));square.pb(Point(-inf,inf));
        if(is_inf_outerboundary) { // meaning it has an infinite outer boundary
            Polygon_set p (pwh);
            p.intersection(square);
            assert(p.number_of_polygons_with_holes() == 1);
            pwh = to_polygon_vector(p)[0];
            debug("here");
        }
        debug(area(t));
        t = Polygon_set(pwh);
        debug(area(t));*/

        auto _sum = CGAL::minkowski_sum_2(pwh, pol);
        Polygon_set sum (_sum);
        /*if(is_inf_outerboundary) {
            sum.join(get_complement(Polygon_set(square))); // remove the extra outer part coming from the square
        }*/

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