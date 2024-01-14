#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/util.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

typedef CGAL::Aff_transformation_2<K> Transformation;

using namespace std;

K::FT compute_area(const Polygon_with_holes& poly_with_holes) {
    K::FT area = poly_with_holes.outer_boundary().area();
    for (auto it = poly_with_holes.holes_begin(); it != poly_with_holes.holes_end(); ++it) {
        area -= it->area();
    }
    return area;
}

/*
bool is_polygon_inside_another(const Polygon& A, const Polygon& B) {
    std::list<Polygon_with_holes> difference_result;
    CGAL::difference(A, B, std::back_inserter(difference_result));
    //cout << compute_area(difference_result.front()) + B.area() << " " << A.area() << endl;
    if (difference_result.size() == 1 && compute_area(difference_result.front()) + B.area() == A.area()) {
        return true;
    }
    return false;
}
*/

/*bool is_polygon_inside_another(const Polygon& p1, const Polygon& p2) {
    if(p1 == p2) return true;
    std::list<Polygon_with_holes> res;
    CGAL::difference(p2, p1, std::back_inserter(res));
    //if (res.empty()) return false;
    if (res.size() == 1) {
        const Polygon_with_holes& diff = res.front();
        if (!diff.has_holes() && diff.outer_boundary() == p2) {
            return true;
        }
    }
    return false;
}*/

Polygon_set to_polygon_set(const Polygon& p) {
    Polygon_set res;
    res.insert(p);
    return res;
}

bool is_inside_polygon(const Polygon& p1, const Polygon& p2) {
    auto a = to_polygon_set(p1);
    auto b = to_polygon_set(p2);
    return is_inside_polygon_set(a,b);
}


vector<pair<int,Polygon>> dummy_algorithm(
    Polygon container,
    vector<tuple<int,int,Polygon>> items
) {
    auto lpc = get_lowest_point(container);
    fon(i, sz(items)) {
        auto [value, quantity, pol] = items[i];
        auto lp = get_lowest_point(pol);
        Transformation translate(
            CGAL::TRANSLATION,
            Vector(lpc.x() - lp.x(), lpc.y() - lp.y())
        );
        Polygon tpol = transform(translate, pol);
        if(is_inside_polygon(tpol, container)) {
            return {{i, tpol}};
        }
    }
    return {};
}

