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

void assert_is_integer_polygon(Polygon& pol) {
    foe(p, pol) {
        assert(is_integer(p.x()));
        assert(is_integer(p.y()));
    }
}

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
    // Rounding method
    {
        bool flag = false;
        auto fix_polygon = [&flag](Polygon pol, int condition) {
            assert(sz(pol) >= 3);
            auto original_orientation = pol.orientation();
            if(original_orientation == CGAL::CLOCKWISE) {
                pol.reverse_orientation();
            }
            Polygon res;
            vector<Point> points;
            foe(p, pol) points.push_back(p);
            fon(i, sz(points)) {
                auto& pre = points[(i - 1 + sz(points)) % sz(points)];
                auto cur = points[i];
                auto& nxt = points[(i + 1) % sz(points)];
                auto o = CGAL::orientation(pre, cur, nxt);
                if(o == CGAL::COLLINEAR) continue;
                Vector dir = 
                    (o == CGAL::RIGHT_TURN ? -1 : 1) * (Vector(cur.x(), cur.y()) - Vector(pre.x(), pre.y())) +
                    (o == CGAL::LEFT_TURN ? -1 : 1) * (Vector(nxt.x(), nxt.y()) - Vector(cur.x(), cur.y()));
                assert(dir != Vector(0,0));
                if(dir.x() > 0) cur = Point(ceil_exact(cur.x()), cur.y());
                else cur = Point(floor_exact(cur.x()), cur.y());
                if(dir.y() > 0) cur = Point(cur.x(), ceil_exact(cur.y()));
                else cur = Point(cur.x(), floor_exact(cur.y()));
                res.push_back(cur);
            }
            if(original_orientation == CGAL::CLOCKWISE) {
                res.reverse_orientation();
            }
            if(
                !res.is_simple() || res.orientation() == CGAL::CLOCKWISE ||
                (condition == CGAL::ON_BOUNDED_SIDE ? !is_completely_inside(pol, res) : !is_completely_inside(res, pol))
            ) {
                flag = true;
                return res;
            }
            return res;
        };
        auto boundary = fix_polygon(pol.outer_boundary(), CGAL::ON_UNBOUNDED_SIDE);
        Polygon_with_holes res;
        if(!flag) {
            res = Polygon_with_holes(boundary);
            foe(hole, pol.holes()) {
                if(flag) break;
                auto new_hole = fix_polygon(hole, CGAL::ON_BOUNDED_SIDE);
                res.add_hole(new_hole);
            }
        }
        if(!flag) {
            cout << "[c++, snapper]: used rounding-based snapping" << endl;
            assert(is_completely_inside(res,pol));
            return res;
        } else {
            /*debug("Old boundary");
            foe(p, pol.outer_boundary()) {
                debug(p);
            }
            debug("New boundary");
            foe(p, boundary) {
                debug(p);
            }*/
        }
    }

    debug("yolo1");

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
    debug("yolo2");
    add_integer_points(pol.outer_boundary());
    debug("yolo3");
    foe(hole, pol.holes()) {
        add_integer_points(hole);
    }
    debug("yolo4");
    assert(pset.number_of_polygons_with_holes() == 1);

    Polygon_with_holes ep;
    foe(pwh, to_polygon_vector(pset)) ep = pwh;
    debug("yolo5");

    auto get_integer_polygon = [](Polygon pol, bool safe_option) {
        assert(pol.orientation() == CGAL::COUNTERCLOCKWISE);
        debug("hello4");
        Polygon res;
        foe(p, pol) {
            bool is_int = is_integer(p.x()) && is_integer(p.y());
            if(is_int) {
                res.push_back(p);
            }
        }
        debug("hello1");
        if(!res.is_simple() || res.orientation() != CGAL::COUNTERCLOCKWISE) {
            cout << "WARNING: integer-only polygon is not simple" << endl;
            if(safe_option) res = Polygon();
            else res = get_convex_hull_of_polygons({res}); // this is bad!
        }
        debug("hello2");
        if(sz(res) < 3 || res.area() == FT(0)) {
            cout << "WARNING: integer-only polygon has less than 3 points" << endl;
            return Polygon();
        }
        debug("hello3");
        assert(res.is_simple());
        debug("hello4");
        assert(res.orientation() == CGAL::COUNTERCLOCKWISE);
        return res;
    };
    debug("yolo6");
    Polygon_with_holes res (get_integer_polygon(ep.outer_boundary(), false));
    debug("heyothere");
    foe(hole, ep.holes()) {
        auto t = hole; t.reverse_orientation();
        auto int_hole = get_integer_polygon(t, true);
        debug("hello5");
        int_hole.reverse_orientation();
        debug("hello6");
        debug(sz(int_hole));
        // Catch that hole touches the outer boundary
        try {
            if(sz(int_hole) && is_completely_inside(Polygon_set(res), Polygon_set(int_hole))) {
                debug("hello7");
                res.add_hole(int_hole);
            }
        } catch(const std::exception& e) {
            continue;
        }
        
    }
    debug("letsgooo");

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
        //cout << "[c++, snapper] tried " << tried << " times" << endl;
        //cout << "[c++, snapper] optimization: " << before << " -> " << after << " (diff = " << before - after << ")" << endl;
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
        //cout << "[c++, snapper] overall optimization: " << total_before << " -> " << total_after << endl;
    }

    //IntersectionPredicates pred (res);
    //assert(pred.is_completely_inside_slow(pol));
    assert(is_completely_inside(reduced_res,pol));
    assert_is_integer_polygon(reduced_res.outer_boundary());
    assert(reduced_res.outer_boundary().is_simple());
    foe(hole, reduced_res.holes()) {
        assert_is_integer_polygon(hole);
        assert(hole.is_simple());
    }
    cout << "[c++, snapper]: used generator-based snapping" << endl;
    return reduced_res;
}
