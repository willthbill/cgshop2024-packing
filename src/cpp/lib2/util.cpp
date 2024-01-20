#include <bits/stdc++.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Gmpq.h>

#include "lib2/util.h"

#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

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

Polygon get_int_bounding_box(vector<Point> arr) {
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

Polygon get_int_bounding_box(Polygon& pol) {
    // return get_int_bounding_box(vector<Point>(pol.vertices_begin(), pol.vertices_end()));
    return get_int_bounding_box(get_vertices_pol(pol));
}

Polygon get_int_bounding_box(Polygon_set& pset) {
    return get_int_bounding_box(get_vertices_pset(pset));
}

vector<Polygon_with_holes> to_polygon_vector_ref(Polygon_set pset) {
    vector<Polygon_with_holes> pols;
    pset.polygons_with_holes (back_inserter(pols));
    return pols;
}

FT get_width(Polygon_set& pol) {
    auto bbox = get_int_bounding_box(pol);
    return bbox[2].x() - bbox[0].x();
}

FT get_height(Polygon_set& pol) {
    auto bbox = get_int_bounding_box(pol);
    return bbox[2].y() - bbox[0].y();
}

FT area(Polygon_set& pset) {
    FT res = 0;
    foe(pwh, to_polygon_vector(pset)) {
        res += pwh.outer_boundary().area();
        foe(hole, pwh.holes()) {
            ASSERT(hole.area() < 0,"not negative hole area");
            res += hole.area(); // hole area is negative
        }
    }
    return res;
}

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

void assert_is_integer_polygon(Polygon& pol) {
    foe(p, pol) {
        assert(is_integer(p.x()));
        assert(is_integer(p.y()));
    }
}

vector<Polygon> fix_repeated_points(Polygon pol) {
    vector<Point> points;
    foe(p, pol) points.push_back(p);
    map<Point, int> mp;
    int idx = -1;
    fon(i, sz(points)) {
        auto& p = points[i];
        if(mp.count(p)) {
            idx = i;
            break;
        }
        mp[p] = i;
    }
    if(idx == -1) return {pol};
    Polygon a, b;
    for(int i = 0; i < mp[points[idx]]; i++) a.push_back(points[i]);
    for(int i = mp[points[idx]]; i < idx; i++) b.push_back(points[i]);
    for(int i = idx; i < sz(points); i++) a.push_back(points[i]);
    vector<Polygon> res;
    foe(p, fix_repeated_points(a)) res.push_back(p);
    foe(p, fix_repeated_points(b)) res.push_back(p);
    return res;
}

Polygon_set clean(Polygon_set pset) {
    Polygon_set cleaned;
    foe(pwh, to_polygon_vector(pset)) {
        if(!pwh.is_unbounded()) {
            if(sz(pwh.outer_boundary()) < 3 || pwh.outer_boundary().area() == FT(0)) continue;
        }
        Polygon_with_holes new_pwh (pwh.outer_boundary());
        foe(hole, pwh.holes()) {
            if(sz(hole) < 3 || hole.area() == 0) continue;
            new_pwh.add_hole(hole);
        }
        cleaned.join(new_pwh);
    }
    return cleaned;
}