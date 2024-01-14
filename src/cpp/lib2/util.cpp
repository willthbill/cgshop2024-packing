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

bool is_point_strictly_inside(Polygon poly, Point p) {
    switch (CGAL::bounded_side_2(poly.vertices_begin(), poly.vertices_end(), p)) {
        case CGAL::ON_BOUNDED_SIDE:
            return true;
        case CGAL::ON_BOUNDARY:
            return false;
        case CGAL::ON_UNBOUNDED_SIDE:
            return false;
    }
    assert(false);
}

Polygon get_single_polygon(Polygon_set& pset) {
    assert(pset.number_of_polygons_with_holes() == 1);
    auto vec = to_polygon_vector(pset);
    assert(vec[0].number_of_holes() == 0);
    return vec[0].outer_boundary();
}

/*bool is_completely_inside(Polygon a, Polygon b) {
    Polygon_set intersection; intersection.intersection(
        to_polygon_set(a),
        to_polygon_set(b)
    );
    auto arr = to_polygon_vector(intersection);
    FT res = 0;
    foe(p, arr) res += p.outer_boundary().area();
    return res == b.area();
}*/

bool is_completely_inside(Polygon_set a, Polygon_set b) {
    Polygon_set t; t.difference(b,a);
    return t.is_empty();
}
bool is_completely_outside(Polygon_set a, Polygon_set b) {
    return is_completely_inside(get_complement(a), b);
}
bool is_completely_inside(Polygon a, Polygon b) {
    return is_completely_inside(Polygon_set(a), Polygon_set(b));
}
bool is_completely_inside(Polygon_with_holes a, Polygon_with_holes b) {
    return is_completely_inside(Polygon_set(a), Polygon_set(b));
}

Polygon scale_polygon(Polygon pol, FT scale) {
    foe(e, pol) {
        e = {e.x() * scale, e.y() * scale};
    }
    return pol;
}

ItemsContainer scale_items(ItemsContainer items, FT scale) {
    foe(item, items) {
        item.pol = scale_polygon(item.pol, scale);
    }
    return items;
}


void assert_is_integer_polygon(Polygon& pol) {
    foe(p, pol) {
        assert(is_integer(p.x()));
        assert(is_integer(p.y()));
    }
}

Polygon get_bounding_box(vector<Point> arr) {
    FT mnx = 1e18, mxx = -1e18, mny = 1e18, mxy = -1e18;
    foe(p, arr) mnx = min(mnx, p.x());
    foe(p, arr) mxx = max(mxx, p.x());
    foe(p, arr) mny = min(mny, p.y());
    foe(p, arr) mxy = max(mxy, p.y());
    mnx = floor_exact(mnx);
    mxx = ceil_exact(mxx);
    mny = floor_exact(mny);
    mxy = ceil_exact(mxy);
    Polygon res;
    res.push_back(Point(mnx, mny));
    res.push_back(Point(mxx, mny));
    res.push_back(Point(mxx, mxy));
    res.push_back(Point(mnx, mxy));
    return res;
}

vector<Point> get_vertices_pset(Polygon_set& pset) {
    vector<Point> res;
    foe(pwh, to_polygon_vector(pset)) {
        foe(p, pwh.outer_boundary()) res.push_back(p);
        foe(hole, pwh.holes()) {
            foe(p, hole) res.push_back(p);
        }
    }
    return res;
}

vector<Point> get_vertices_pol(Polygon& pol) {
    auto pset = Polygon_set(pol);
    return get_vertices_pset(pset);
}

Polygon get_bounding_box(Polygon& pol) {
    return get_bounding_box(vector<Point>(pol.vertices_begin(), pol.vertices_end()));
}

Polygon get_bounding_box(Polygon_set& pset) {
    return get_bounding_box(get_vertices_pset(pset));
}

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
vector<Polygon_with_holes> to_polygon_vector_ref(Polygon_set pset) {
    vector<Polygon_with_holes> pols;
    pset.polygons_with_holes (back_inserter(pols));
    return pols;
}

template<typename T>
T permute(T vec, const std::vector<int>& indices) {
    assert(sz(vec) == sz(indices));
    T permuted = vec;
    for(size_t i = 0; i < indices.size(); ++i) {
        permuted[i] = vec[indices[i]];
    }
    return permuted;
}