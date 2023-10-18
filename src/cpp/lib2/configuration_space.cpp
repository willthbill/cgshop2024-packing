#include <bits/stdc++.h>
#include <CGAL/minkowski_sum_2.h>

#include "configuration_space.h"

#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

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
    foe(pwh, to_polygon_vector(s)) {
        auto sum = CGAL::minkowski_sum_2(pwh, pol);
        space.insert(sum);
    }
    space.complement();
}
