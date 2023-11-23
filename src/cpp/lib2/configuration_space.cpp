#include <bits/stdc++.h>
#include <CGAL/minkowski_sum_2.h>

#include "lib2/configuration_space.h"

#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

Polygon translate_point_to_origin(Polygon& pol, Point ref) {
    if (pol.is_empty()) {
        return pol;
    }
    std::vector<Point> translated_points;
    for (const Point& point : pol.vertices()) {
        translated_points.emplace_back(point.x() - ref.x(), point.y() - ref.y());
    }
    return Polygon(translated_points.begin(), translated_points.end());
}

// TODO: add to lib
Polygon translate_centroid_to_origin(Polygon& pol) {
    if (pol.is_empty()) {
        return pol;
    }
    Point centroid = get_centroid(pol);
    return translate_point_to_origin(pol, centroid);
}

// TODO: allow to use reference pol
Polygon scale_polygon(Polygon pol, int s) {
    Polygon scaled_poly;
    for (const Point& vertex : pol.vertices()) {
        FT x = vertex.x() * s;
        FT y = vertex.y() * s;
        scaled_poly.push_back(Point(x,y));
    }
    return scaled_poly;
}

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