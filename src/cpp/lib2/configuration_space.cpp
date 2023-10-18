#include <bits/stdc++.h>

#include <CGAL/minkowski_sum_2.h>

#include "lib/util/geometry_utils.h"
#include "lib/util/com.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

using namespace std;

// TODO: add to lib
Polygon translate_centroid_to_origin(Polygon& pol) {
    if (pol.is_empty()) {
        return pol;
    }
    Point centroid = get_centroid(pol);
    std::vector<Point> translated_points;
    for (const Point& point : pol.vertices()) {
        translated_points.emplace_back(point.x() - centroid.x(), point.y() - centroid.y());
    }
    return Polygon(translated_points.begin(), translated_points.end());
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

class ConfigurationSpace {
public:
    Polygon_set space;
    // space where centroid of pol can be located without pol intersecting with s
    ConfigurationSpace(Polygon_set& s, Polygon& _pol) {
        Polygon pol = scale_polygon(translate_centroid_to_origin(_pol), -1);
        foe(pwh, to_polygon_vector(s)) {
            auto sum = CGAL::minkowski_sum_2(pwh, pol);
            space.insert(sum);
        }
        space.complement();
    }
};
