#include <bits/stdc++.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Gmpq.h>

#include "lib2/util.h"

#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

CGAL::Gmpq floor_exact(const CGAL::Gmpq &q) {
    // If the denominator is 1 or if 'q' is an integer, 'q' is already floored.
    if (q.denominator() == 1 || q == q.numerator()) {
        return q;
    } else if (q >= 0) {
        // For non-negative 'q', simply return the numerator divided by the denominator.
        return CGAL::Gmpq(q.numerator() / q.denominator(), 1);
    } else {
        // For negative 'q', subtract 1 after division if there's a remainder.
        CGAL::Gmpz floor_result = q.numerator() / q.denominator();
        if (q.numerator() % q.denominator() != 0) {
            floor_result -= 1;
        }
        return CGAL::Gmpq(floor_result, 1);
    }
}

CGAL::Gmpq ceil_exact(const CGAL::Gmpq &q) {
    // If the denominator is 1 or if 'q' is an integer, 'q' is already ceiled.
    if (q.denominator() == 1 || q == q.numerator()) {
        return CGAL::Gmpq(q.numerator(), 1);
    } else if (q >= 0) {
        // For non-negative 'q', add 1 after division if there's a remainder.
        CGAL::Gmpz ceil_result = q.numerator() / q.denominator();
        if (q.numerator() % q.denominator() != 0) {
            ceil_result += 1;
        }
        return CGAL::Gmpq(ceil_result, 1);
    } else {
        // For negative 'q', simply return the numerator divided by the denominator.
        return CGAL::Gmpq(q.numerator() / q.denominator(), 1);
    }
}

bool is_integer(const CGAL::Gmpq v) {
    return v == floor_exact(v);
}

bool is_polygon_inside_polygon(const Polygon& p1, const Polygon& p2) {
    auto a = to_polygon_set(p1);
    auto b = to_polygon_set(p2);
    return is_inside_polygon_set(a,b);
}

Polygon_set get_complement(const Polygon_set& pset) {
    Polygon_set complement_set;
    complement_set.complement(pset);
    return complement_set;
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


