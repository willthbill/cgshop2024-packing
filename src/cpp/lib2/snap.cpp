#include<bits/stdc++.h>
#include <CGAL/Gmpq.h>
#include <CGAL/intersections.h>

#include "lib2/snap.h"
#include "lib/geometry/intersection_predicates.h"
#include "lib2/util.h"
#include "lib/util/cgal.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

// TODO: somehow specify the grid size
// TODO: support polygon set
SnapToGrid::SnapToGrid(Polygon_set pset) {
    foe(pwh, to_polygon_vector(pset)) {
        space.join(snap(pwh));
    }
}
Polygon SnapToGrid::get_single_polygon() {
    assert(space.number_of_polygons_with_holes() == 1);
    foe(pwh, to_polygon_vector(space)) {
        assert(pwh.number_of_holes() == 0);
        return pwh.outer_boundary();
    }
    assert(false);
}

// this does not work exactly right. we check if the segment intersects in two district points.
bool does_segment_cut(Polygon& p, Segment s) {
    Point touch (-1e9, -1e9);
    bool seen_touch = false;
    for(auto e : p.edges()) {
        auto v = intersection(e, s);
        if (v) {
            if (const Point *p = boost::get<Point>(&*v)) {
                if(seen_touch && *p != touch) {
                    return true;
                } else {
                    touch = *p;
                    seen_touch = true;
                }
            } else {
                continue;
            }
        }
    }
    return false;
    /*Polygon pol; pol.push_back(s.source()); pol.push_back(s.target());
    Polygon_set t (p);
    t.difference(pol);
    return t.is_empty();*/
}

bool is_completely_inside(Polygon_set a, Polygon_set b) {
    Polygon_set t; t.difference(b,a);
    return t.is_empty();
}
bool is_completely_outside(Polygon_set a, Polygon_set b) {
    return is_completely_inside(get_complement(a), b);
}
bool is_completely_inside(Polygon a, Polygon b) {
    /*Polygon_set intersection; intersection.intersection(
        to_polygon_set(a),
        to_polygon_set(b)
    );
    auto arr = to_polygon_vector(intersection);
    FT res = 0;
    foe(p, arr) res += p.outer_boundary().area();
    return res == b.area();*/
    return is_completely_inside(Polygon_set(a), Polygon_set(b));
}
bool is_completely_inside(Polygon_with_holes a, Polygon_with_holes b) {
    return is_completely_inside(Polygon_set(a), Polygon_set(b));
}
Polygon_with_holes SnapToGrid::snap(Polygon_with_holes pol) {
    {
        bool flag = false;
        auto fix_polygon = [&flag](Polygon pol, int condition) {
            debug(condition);
            debug(CGAL::ON_BOUNDED_SIDE);
            debug(CGAL::ON_UNBOUNDED_SIDE);
            Polygon res;
            foe(p, pol) {
                auto t = Point(floor_exact(p.x()), floor_exact(p.y()));
                if(pol.bounded_side(t) != condition) {
                    res.push_back(t);
                    continue;
                }
                t = Point(floor_exact(p.x()), ceil_exact(p.y()));
                if(pol.bounded_side(t) != condition) {
                    res.push_back(t);
                    continue;
                }
                t = Point(ceil_exact(p.x()), floor_exact(p.y()));
                if(pol.bounded_side(t) != condition) {
                    res.push_back(t);
                    continue;
                }
                t = Point(ceil_exact(p.x()), ceil_exact(p.y()));
                if(pol.bounded_side(t) != condition) {
                    res.push_back(t);
                    continue;
                }
                debug("yo1");
                flag = true;
                return Polygon();
            }
            if(!res.is_simple() || res.orientation() == CGAL::CLOCKWISE) {
                debug("yo2");
                flag = true;
                return Polygon();
            }
            return res;
        };
        auto boundary = fix_polygon(pol.outer_boundary(), CGAL::ON_BOUNDED_SIDE);
        debug("old boundary");
        foe(p,pol.outer_boundary()) {
            debug(p);
        }

        debug("new boundary");
        foe(p,boundary) {
            debug(p);
        }
        assert(is_completely_inside(boundary,pol.outer_boundary()));
        Polygon_with_holes res (boundary);
        foe(hole, pol.holes()) {
            if(flag) break;
            auto new_hole = fix_polygon(hole, CGAL::ON_UNBOUNDED_SIDE);
            assert(is_completely_inside(boundary,new_hole));
            res.add_hole(new_hole);
        }
        if(!flag) {
            cout << "[c++, snapper]: used fast snapping" << endl;
            assert(is_completely_inside(res,pol));
            return res;
        }
    }

    // Snapping
    Polygon_set pset;
    pset.insert(pol);
    auto add_integer_points = [&pset](Polygon pol) {
        foe(_p, pol) {
            Point p = _p;

            bool is_int = is_integer(p.x()) && is_integer(p.y());
            if(is_int) continue;

            if(p.x() == floor_exact(p.x())) {
                p = Point(p.x() + FT(0.5), p.y());
            }
            if(p.y() == floor_exact(p.y())) {
                p = Point(p.x(), p.y() + FT(0.5));
            }
            Polygon box;
            box.push_back(Point(
                floor_exact(p.x()) -2,
                floor_exact(p.y()) -2
            ));
            box.push_back(Point(
                ceil_exact(p.x())  +2,
                floor_exact(p.y()) -2
            ));
            box.push_back(Point(
                ceil_exact(p.x())  +2,
                ceil_exact(p.y())  +2
            ));
            box.push_back(Point(
                floor_exact(p.x()) -2,
                ceil_exact(p.y())  +2
            ));
            pset.join(box);
            /*foab(i, -1, 1) {
                foab(j, -1, 1) {
                    Polygon box;
                    box.push_back(Point(
                        floor_exact(p.x()) + i,
                        floor_exact(p.y()) + j
                    ));
                    box.push_back(Point(
                        ceil_exact(p.x()) + i,
                        floor_exact(p.y()) + j
                    ));
                    box.push_back(Point(
                        ceil_exact(p.x()) + i,
                        ceil_exact(p.y()) + j
                    ));
                    box.push_back(Point(
                        floor_exact(p.x()) + i,
                        ceil_exact(p.y()) + j
                    ));
                    pset.join(box);
                }
            }*/
        }
    };
    add_integer_points(pol.outer_boundary());
    foe(hole, pol.holes()) {
        add_integer_points(hole);
    }
    assert(pset.number_of_polygons_with_holes() == 1);

    Polygon_with_holes ep;
    foe(pwh, to_polygon_vector(pset)) ep = pwh;

    auto get_integer_polygon = [](Polygon pol) {
        Polygon res;
        foe(p, pol) {
            bool is_int = is_integer(p.x()) && is_integer(p.y());
            if(is_int) {
                res.push_back(p);
            }
        }
        if(!res.is_simple()) {
            cout << "WARNING: integer-only polygon is not simple" << endl;
            return get_convex_hull_of_polygons({res}); // this is bad!
        }
        return res;
    };
    Polygon_with_holes res (get_integer_polygon(ep.outer_boundary()));
    foe(hole, ep.holes()) {
        auto t = hole; t.reverse_orientation();
        if(!is_completely_inside(Polygon_set(pol.outer_boundary()), Polygon_set(t))) continue;
        res.add_hole(get_integer_polygon(hole));
    }

    assert(is_completely_inside(res,pol));

    // Reduction
    auto reduce = [&](Polygon res, Polygon_set original) -> Polygon {
        int original_orientation = res.orientation();
        if(original_orientation == CGAL::CLOCKWISE) {
            res.reverse_orientation();
        }
        assert(
                original_orientation == CGAL::CLOCKWISE ? // a hole
                is_completely_outside(original, Polygon_set(res)) :
                is_completely_inside(Polygon_set(res), original)
        );
        // assert(original.area() > 0 && sz(original) > 2); // just checking
        // assert(is_completely_inside(Polygon_set(res), original));
        int before = res.size();
        int tried = 0;
        while(true) {
            bool found = false;
            for(int i = 0; i < sz(res); i++) {
                Polygon newres;
                int idx = -1;
                for(auto p : res) {
                    idx++;
                    if(idx == i) continue;
                    newres.push_back(p);
                }
                if(sz(newres) <= 2) continue;
                tried++;
                if(!newres.is_simple() || newres.orientation() == CGAL::CLOCKWISE) continue; // not simple or it changed the orientation
                FT dist = 1e18;
                for(int j = 0; j < sz(res); j++) {
                    if(i == j) continue;
                    dist = min(dist, CGAL::squared_distance(res[i], res[j]));
                }
                if(
                    original_orientation == CGAL::CLOCKWISE ?
                    (newres.area() < res.area()) : (newres.area() > res.area())
                    && dist >= 3
                ) continue;
                if(
                    original_orientation == CGAL::CLOCKWISE ? // a hole
                    is_completely_outside(original, Polygon_set(newres)) :
                    // is_completely_inside(Polygon_set(newres), original)
                    /*do_intersect(
                        Segment(res[(i - 1 + sz(res)) % sz(res)], res[(i+1)%sz(res)]),
                        to_polygon_vector(original)[0].outer_boundary()
                    )*/
                    // is_completely_outside(original, Polygon_set(Segment(res[(i - 1 + sz(res)) % sz(res)], res[(i+1)%sz(res)])))
                    !does_segment_cut(
                        to_polygon_vector(original)[0].outer_boundary(),
                        Segment(res[(i - 1 + sz(res)) % sz(res)], res[(i+1)%sz(res)])
                    )
                ) {
                    found = true;
                    res = newres;
                }
            }
            if(!found) break;
        }
        int after = res.size();
        cout << "[c++, snapper] tried " << tried << " times" << endl;
        cout << "[c++, snapper] optimization: " << before << " -> " << after << " (diff = " << before - after << ")" << endl;
        if(original_orientation == CGAL::CLOCKWISE) res.reverse_orientation();
        return res;
    };
    Polygon_with_holes reduced_res (reduce(
        res.outer_boundary(),
        Polygon_set(pol.outer_boundary())
    ));
    foe(hole, res.holes()) {
        auto newhole = reduce(hole, Polygon_set(pol));
        newhole.reverse_orientation();
        if(!is_completely_inside(Polygon_set(reduced_res.outer_boundary()), Polygon_set(newhole))) continue;
        newhole.reverse_orientation();
        reduced_res.add_hole(newhole);
    }
    {
        int total_before = sz(pol.outer_boundary()), total_after = sz(reduced_res.outer_boundary());
        foe(hole, pol.holes()) total_before += sz(hole);
        foe(hole, reduced_res.holes()) total_after += sz(hole);
        cout << "[c++, snapper] overall optimization: " << total_before << " -> " << total_after << endl;
    }

    //IntersectionPredicates pred (res);
    //assert(pred.is_completely_inside_slow(pol));
    assert(is_completely_inside(reduced_res,pol));
    return reduced_res;
}
