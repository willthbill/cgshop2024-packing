#include <bits/stdc++.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

bool is_polygon_inside_polygon(const Polygon& p1, const Polygon& p2) {
    auto a = to_polygon_set(p1);
    auto b = to_polygon_set(p2);
    return is_inside_polygon_set(a,b);
}

// deterministic (also lowest x)
Point get_lowest_point(const Polygon& polygon) {
    if (polygon.is_empty()) {
        ASSERT(false, "Polygon is empty");
    }
    Point lowest_point = polygon[0];
    for (Polygon::Vertex_iterator it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
        if (it->y() < lowest_point.y() || (it->y() == lowest_point.y() && it->x() < lowest_point.x())) {
            lowest_point = *it;
        }
    }
    return lowest_point;
}

