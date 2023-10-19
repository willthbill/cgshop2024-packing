#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/configuration_space.h"
#include "lib2/mip.h"
#include "io.h"
#include "lib/geometry/partition_constructor.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

const double scale=0.05;
const double inf = 500 * scale;
const double biginf = 1e4 * scale;

// TODO: speed up by only duplicating polygons as many times as the total area is less than area of container
// TODO: maybe see if there are common factors (or something like that) on each side so we can make numbers smaller
// TODO: optimize by drawing triangle around instead of square
// TODO: actually calculate bounds (inf, biginf)

pair<string,string> get_ref_coord_variable_names(int idx) {
    string x = "polygon_ref_" + to_string(idx) +  "_x";
    string y = "polygon_ref_" + to_string(idx) +  "_y";
    return {x,y};
}

PackingOutput optimal_algorithm(PackingInput input123) {
    ItemsContainer items_original123 = input123.items.expand();
    ItemsContainer items = items_original123;
    auto container = input123.container;
    foe(e, container) {
        e = {e.x() * scale, e.y() * scale};
    }
    foe(item, items) {
        foe(p, item.pol) {
            p = {p.x() * scale, p.y() * scale};
        }
    }
    cout << "Number of expanded items: " << sz(items) << endl;
    int binaries = 0;
    MIP problem;
    vector<string> in_use_binaries;
    vector<pair<string,FT>> obj_terms;
    fon(i, sz(items)) {
        in_use_binaries.push_back("is_polygon_" + to_string(i) + "_in_use");
        problem.add_binary_variable(in_use_binaries[i]);
        obj_terms.push_back({in_use_binaries[i], items[i].value});
    }
    problem.set_max_objective(obj_terms);
    fon(i, sz(items)) {
        auto [x, y] = get_ref_coord_variable_names(i);
        problem.add_continuous_variable(x);
        problem.add_continuous_variable(y);
    }
    auto add_constraints_inside_convex_polygon = [&](Polygon& pol, string x, string y, string binary, Vector offset) {
        ASSERT(pol.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");
        fon(k, sz(pol)) {
            Point c1 = pol[k];
            Point c2 = pol[(k+1) % sz(pol)];
            cerr << c1.x() << " " << c1.y() << endl;
            cerr << c2.x() << " " << c2.y() << endl;
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
    fon(i, sz(items)) {
        fon(j, sz(items)) {
            if(i >= j) continue;
            // TODO: we dont need i,j and j,i right?
            debug("constraints for ",i,j);
            Polygon_set disallowed (items[j].pol);
            ConfigurationSpace cp (disallowed, items[i].pol, items[i].get_reference_point());
            vector<Polygon> partition;
            {
                Polygon square; square.pb(Point(-inf,-inf));square.pb(Point(inf,-inf));square.pb(Point(inf,inf));square.pb(Point(-inf,inf));
                Polygon_set ps (square);
                ps.intersection(cp.space);
                auto polygons = to_polygon_vector(ps);
                assert(sz(polygons) == 1);
                auto polygon = polygons[0];
                debug(polygon.number_of_holes());
                assert(polygon.number_of_holes() == 1);
                PartitionConstructor pc (polygon);
                partition = pc.get_constrained_delaunay_triangulation();
            }
            Point ref = items[j].get_reference_point(); // (x1,y1)
            // Point centroid = get_centroid(get<1>(items[i])); // (x2,y2)
            auto [x1, y1] = get_ref_coord_variable_names(j);
            auto [x2, y2] = get_ref_coord_variable_names(i);
            vector<string> binvars;
            foe(tri, partition) {
                ASSERT(tri.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");
                string b = "binary_" + to_string(binaries++);
                problem.add_binary_variable(b);
                binvars.push_back(b);
                fon(k, sz(tri)) {
                    Vector c1 = tri[k] - ref;
                    Vector c2 = tri[(k+1) % sz(tri)] - ref;
                    Vector v1 (c2.x() - c1.x(), c2.y() - c1.y());
                    // v2 = (x2 - x1 - c2.x(), y2 - y1 - c2.y());
                    // Cross product
                        // cross(v1,v2) = v1.x() * (y2 - y1 - c2.y()) - v1.y() * (x2 - x1 - c2.x())
                        // = v1.x() * y2 - v1.x() * y1 - v1.x() * c2.y() - v1.y() * x2 + v1.y() * x1 + v1.y() * c2.x() <= 0   TODO: <= is correct right?
                        // <=>
                        // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 <= v1.x() * c2.y() - v1.y() * c2.x()
                    // Now introduce binary
                        // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 <= v1.x() * c2.y() - v1.y() * c2.x() + (1 - b) * biginf
                        // <=>
                        // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 - (1 - b) * biginf <= v1.x() * c2.y() - v1.y() * c2.x()
                        // <=>
                        // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 + b * biginf <= v1.x() * c2.y() - v1.y() * c2.x() + biginf
                    
                    // v2 = (x2 - x1 - c1.x(), y2 - y1 - c1.y());
                    // Cross product
                        // cross(v1,v2) = v1.x() * (y2 - y1 - c1.y()) - v1.y() * (x2 - x1 - c1.x())
                        // = v1.x() * y2 - v1.x() * y1 - v1.x() * c1.y() - v1.y() * x2 + v1.y() * x1 + v1.y() * c1.x() <= 0   TODO: <= is correct right?
                        // <=>
                        // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 <= v1.x() * c1.y() - v1.y() * c1.x()
                    problem.add_geq_constraint(
                        {{y2, v1.x()},{y1,-v1.x()},{x2,-v1.y()},{x1,v1.y()},{b, -biginf}, {in_use_binaries[i],-biginf},{in_use_binaries[j],-biginf}},
                        v1.x() * c1.y() - v1.y() * c1.x() - 3.0 * biginf // TODO: safe in terms of casting??
                    );
                }
            }
            // at least one convex region must be satisfied
            {
                vector<pair<string,FT>> terms;
                foe(b, binvars) terms.push_back({b,1});
                terms.push_back({in_use_binaries[i],-100});
                terms.push_back({in_use_binaries[j],-100});
                problem.add_geq_constraint(terms, 1ll - 200); // TODO: safe in terms of precision / automatic casting??
            }
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
    PackingOutput output (input123);
    fon(i, sz(items)) {
        if(solution[in_use_binaries[i]] > 0.5) {
            auto [xkey, ykey] = get_ref_coord_variable_names(i);
            debug(solution[xkey].to_double(), solution[ykey].to_double());
            FT x = solution[xkey] / scale;
            FT y = solution[ykey] / scale;
            debug(x.to_double(),y.to_double());
            debug(items_original123[i].get_reference_point().x().to_double(), items_original123[i].get_reference_point().y().to_double());
            debug(items[i].get_reference_point().x().to_double(), items[i].get_reference_point().y().to_double());
            Item new_item = items_original123[i].move_ref_point(Point(x,y));
            output.add_item(new_item);
        }
    }
    return output;
}

