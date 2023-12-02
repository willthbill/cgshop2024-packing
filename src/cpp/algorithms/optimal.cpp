#include <bits/stdc++.h>
// #include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/snap.h"
#include "lib2/configuration_space.h"
#include "lib2/util.h"
#include "lib2/mip/gurobi.h"
#include "io.h"
#include "lib/geometry/partition_constructor.h"
#include "lib/util/geometry_utils.h"

#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

const ll scale=1;
const ll inf = 5000;
const ll biginf = 1e7;

// TODO: speed up by only duplicating polygons as many times as the total area is less than area of container
// TODO: maybe see if there are common factors (or something like that) on each side so we can make numbers smaller
// TODO: optimize by drawing triangle around instead of square
// TODO: actually calculate bounds (inf, biginf)


bool is_completely_inside(Polygon a, Polygon b) {
    Polygon_set intersection; intersection.intersection(
        to_polygon_set(a),
        to_polygon_set(b)
    );
    auto arr = to_polygon_vector(intersection);
    FT res = 0;
    foe(p, arr) res += p.outer_boundary().area();
    return res == b.area();
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

pair<string,string> get_ref_coord_variable_names(int idx) {
    string x = "polygon_ref_" + to_string(idx) +  "_x";
    string y = "polygon_ref_" + to_string(idx) +  "_y";
    return {x,y};
}

void add_iteminitem_constraints(int i, int j, vector<string>& in_use_binaries, ItemsContainer& items, Gurobi_MIP& problem) {
    vector<Polygon> partition;
    {
        // Compute configuration space
        Polygon_set disallowed (items[j].pol);
        ConfigurationSpace cp (disallowed, items[i].pol, items[i].get_reference_point());
        // Snap it to a grid
        SnapToGrid snapper (get_complement(cp.space)); // TODO: taking complement twice
        // Computer intersection of square and completement of configuration space
        Polygon square; square.pb(Point(-inf,-inf));square.pb(Point(inf,-inf));square.pb(Point(inf,inf));square.pb(Point(-inf,inf));
        Polygon_set ps (square);
        ps.intersection(get_complement(snapper.space));
        // Get the single polygon with one hole (the complement of conf space)
        auto polygons = to_polygon_vector(ps);
        assert(sz(polygons) == 1);
        auto polygon = polygons[0];
        assert(polygon.number_of_holes() == 1);
        // Compute convex cover (triangulation, partiton or something)
        PartitionConstructor pc (polygon);
        foe(hole, polygon.holes()) {
            foe(p, hole) {
                ASSERT(is_integer(p.x()), "configuration space coord should be an integer");
                ASSERT(is_integer(p.y()), "configuration space coord should be an integer");
            }
        }
        partition = pc.get_constrained_delaunay_triangulation();
        foe(tri, partition) {
            foe(p, tri) {
                ASSERT(is_integer(p.x()), "configuration space partition coord should be an integer");
                ASSERT(is_integer(p.y()), "configuration space partition coord should be an integer");
            }
        }
        // partition = pc.get_approx_convex_partition();
    }
    Point ref = items[j].get_reference_point(); // (x1,y1)
    // Point centroid = get_centroid(get<1>(items[i])); // (x2,y2)
    auto [x1, y1] = get_ref_coord_variable_names(j);
    auto [x2, y2] = get_ref_coord_variable_names(i);
    vector<string> binvars;
    int idx = 0;
    foe(tri, partition) {
        ASSERT(tri.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");
        string b = "binary_" + to_string(i) + "_" + to_string(j) + "_" + to_string(idx);
        idx += 1;
        problem.add_binary_variable(b);
        binvars.push_back(b);
        fon(k, sz(tri)) {
            Vector c1 = tri[k] - ref;
            Vector c2 = tri[(k+1) % sz(tri)] - ref;
            Vector v1 (c2.x() - c1.x(), c2.y() - c1.y());
            // v2 = (x2 - x1 - c1.x(), y2 - y1 - c1.y());
            // Cross product
                // cross(v1,v2) = v1.x() * (y2 - y1 - c1.y()) - v1.y() * (x2 - x1 - c1.x())
                // = v1.x() * y2 - v1.x() * y1 - v1.x() * c1.y() - v1.y() * x2 + v1.y() * x1 + v1.y() * c1.x() <= 0   TODO: <= is correct right?
                // <=>
                // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 <= v1.x() * c1.y() - v1.y() * c1.x()
            // cout << "v1.x() * c1.y() - v1.y() * c1.x()" << " " << (v1.x() * c1.y() - v1.y() * c1.x()).to_double() << endl;
            problem.add_geq_constraint(
                {{y2, v1.x()},{y1,-v1.x()},{x2,-v1.y()},{x1,v1.y()},{b, -biginf}, {in_use_binaries[i],-biginf},{in_use_binaries[j],-biginf}},
                v1.x() * c1.y() - v1.y() * c1.x() - ((long long)3) * biginf // TODO: safe in terms of casting??
            );
        }
    }
    // at least one convex region must be satisfied
    {
        vector<pair<string,FT>> terms;
        foe(b, binvars) terms.push_back({b,1});
        terms.push_back({in_use_binaries[i],-10000});
        terms.push_back({in_use_binaries[j],-10000});
        problem.add_geq_constraint(terms, 1ll - 20000); // TODO: safe in terms of precision / automatic casting??
    }
}

PackingOutput optimal_algorithm(PackingInput input123) {
    cout << "[c++] RUNNING OPTIMAL ALGORITHM" << endl;
    ItemsContainer items_original123 = input123.items.expand();
    ItemsContainer _items = items_original123;

    ItemsContainer items;
    items.add_item(_items[0]);
    items.add_item(_items[4]);
    items.add_item(_items[5]);
    items.add_item(_items[6]);
    items.add_item(_items[7]);
    items_original123 = items;

    auto container = input123.container;
    foe(e, container) {
        e = {e.x() * scale, e.y() * scale};
    }
    foe(item, items) {
        foe(p, item.pol) {
            p = {p.x() * scale, p.y() * scale};
        }
    }
    cout << "[c++] Number of expanded items: " << sz(items) << endl;
    int binaries = 0;
    Gurobi_MIP problem;
    vector<string> in_use_binaries;
    vector<pair<string,FT>> obj_terms;
    fon(i, sz(items)) {
        in_use_binaries.push_back("is_polygon_" + to_string(i) + "_in_use");
        problem.add_binary_variable(in_use_binaries[i]);
        obj_terms.push_back({in_use_binaries[i], items[i].value});
    }
    cout << "hey2" << endl;
    problem.set_max_objective(obj_terms);
    fon(i, sz(items)) {
        auto [x, y] = get_ref_coord_variable_names(i);
        problem.add_integer_variable(x);
        problem.add_integer_variable(y);
    }
    cout << "hey3" << endl;
    auto add_constraints_inside_convex_polygon = [&](Polygon& pol, string x, string y, string binary, Vector offset) {
        ASSERT(pol.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");
        fon(k, sz(pol)) {
            Point c1 = pol[k];
            Point c2 = pol[(k+1) % sz(pol)];
            auto o = offset;
            Vector v1 (c2.x() - c1.x(), c2.y() - c1.y());
            // v2 = (x - c1.x(), y - c1.y())
            // v1 cross v2 = v1.x() * (y - c1.y()) - v1.y() * (x - c1.x()) <= 0
            // v1 cross v2 = v1.x() * y - v1.x() * c1.y() - v1.y() * x + v1.y() * c1.x() <= 0
            // v1 cross v2 = v1.x() * y - v1.y() * x <= v1.x() * c1.y() - v1.y() * c1.x()
            
            // v2 = (x - c1.x(), y - c1.y())
            // v2 cross v1 = (x - c1.x()) * v1.y() - (y - c1.y()) * v1.x() <= 0
            // v2 cross v1 = x * v1.y() - c1.x() * v1.y() - y * v1.x() + c1.y() * v1.x() <= 0
            // v2 cross v1 = x * v1.y() - y * v1.x() <= c1.x() * v1.y() - c1.y() * v1.x()

            // v2 = (x + o.x() - c1.x(), y + o.y() - c1.y())
            // v1 cross v2 = v1.x() * (y + o.y() - c1.y()) - v1.y() * (x + o.x() - c1.x()) <= 0
            // v1 cross v2 = v1.x() * y + v1.x() * o.y() - v1.x() * c1.y() - v1.y() * x - v1.y() * o.x() + v1.y() * c1.x() <= 0
            // v1 cross v2 = v1.x() * y - v1.y() * x <= -v1.x() * o.y() + v1.x() * c1.y() + v1.y() * o.x() - v1.y() * c1.x()
            
            problem.add_geq_constraint(
                {{y,v1.x()},{x,-v1.y()}, {binary,-biginf}},
                -v1.x() * o.y() + v1.x() * c1.y() + v1.y() * o.x() - v1.y() * c1.x() - biginf // TODO: safe in terms of casting??
            );
        }
    };
    cout << "hey4" << endl;
    fon(i, sz(items)) {
        fon(j, sz(items)) {
            if(i >= j) continue;
            // if(i >= j) continue;
            // TODO: we dont need i,j and j,i right?
            add_iteminitem_constraints(i, j, in_use_binaries, items, problem);
        }
    }
    fon(i, sz(items)) {
        auto [x, y] = get_ref_coord_variable_names(i);
        auto ref = items[i].get_reference_point();
        foe(p, items[i].pol) {
            add_constraints_inside_convex_polygon(container, x, y, in_use_binaries[i], p - ref);
        }
    }
    auto solution = problem.solve();
    foe(p, solution) debug(p);
    PackingOutput output (input123);
    fon(i, sz(items)) {
        if(solution[in_use_binaries[i]] > 0.5) {
            debug("in", solution[in_use_binaries[i]]);
            auto [xkey, ykey] = get_ref_coord_variable_names(i);
            // debug(solution[xkey].to_double(), solution[ykey].to_double());
            FT x = solution[xkey] / scale;
            FT y = solution[ykey] / scale;
            cout << "[c++] Item coordinates: " << x << " " << y << endl;
            // debug(x.to_double(),y.to_double());
            // debug(items_original123[i].get_reference_point().x().to_double(), items_original123[i].get_reference_point().y().to_double());
            // debug(items[i].get_reference_point().x().to_double(), items[i].get_reference_point().y().to_double());
            Item new_item = items_original123[i].move_ref_point(Point(x,y));
            output.add_item(new_item);
        } else {
            debug("out", solution[in_use_binaries[i]]);
        }
    }
    fon(i, sz(output)) {
        fon(j, sz(output)) {
            if(i >= j) continue;
            vector<Polygon> partition;
            cout << "i = " << i << " j = " << j << endl;
            {
                Polygon_set disallowed (output.items[j].pol);
                ConfigurationSpace cp (disallowed, output.items[i].pol, output.items[i].get_reference_point());
                {
                    auto comp = get_complement(cp.space);
                    auto pol = to_polygon_vector(comp)[0].outer_boundary();
                    cout << (!is_point_strictly_inside(pol, output.items[i].get_reference_point())) << endl;
                }
                SnapToGrid snapper (get_complement(cp.space)); // TODO: taking complement twice
                Polygon square; square.pb(Point(-inf,-inf));square.pb(Point(inf,-inf));square.pb(Point(inf,inf));square.pb(Point(-inf,inf));
                Polygon_set ps (square); ps.intersection(get_complement(snapper.space));
                auto polygons = to_polygon_vector(ps);
                assert(sz(polygons) == 1);
                auto polygon = polygons[0];
                assert(polygon.number_of_holes() == 1);
                // assert(!is_point_inside(polygon.outer_boundary(), output.items[i].get_reference_point()));
                cout << (!is_point_strictly_inside(polygon.holes()[0], output.items[i].get_reference_point())) << endl;
                PartitionConstructor pc (polygon);
                partition = pc.get_constrained_delaunay_triangulation();
            }

            Point ref = output.items[j].get_reference_point(); // (x1,y1)
            auto x1 = output.items[j].get_reference_point().x();
            auto y1 = output.items[j].get_reference_point().y();
            auto x2 = output.items[i].get_reference_point().x();
            auto y2 = output.items[i].get_reference_point().y();
            int mxcnt = 0;
            foe(tri, partition) {
                ASSERT(tri.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");
                int cnt = 0;
                fon(k, sz(tri)) {
                    Vector c1 = tri[k] - ref;
                    Vector c2 = tri[(k+1) % sz(tri)] - ref;
                    Vector v1 (c2.x() - c1.x(), c2.y() - c1.y());
                    // v2 = (x2 - x1 - c1.x(), y2 - y1 - c1.y());
                    // Cross product
                        // cross(v1,v2) = v1.x() * (y2 - y1 - c1.y()) - v1.y() * (x2 - x1 - c1.x())
                        // = v1.x() * y2 - v1.x() * y1 - v1.x() * c1.y() - v1.y() * x2 + v1.y() * x1 + v1.y() * c1.x() <= 0   TODO: <= is correct right?
                        // <=>
                        // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 <= v1.x() * c1.y() - v1.y() * c1.x()
                    // cout << "v1.x() * c1.y() - v1.y() * c1.x()" << " " << (v1.x() * c1.y() - v1.y() * c1.x()).to_double() << endl;
                    if(y2 * v1.x() + y1 * -v1.x() + x2 * -v1.y()+ x1 * v1.y() >= v1.x() * c1.y() - v1.y() * c1.x()) {
                        cnt++;
                    }
                }
                mxcnt = max(mxcnt, cnt);
            }
            cout << "mxcnt: " << mxcnt << endl;
        }
    }
    return output;
}

