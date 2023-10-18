#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/configuration_space.cpp"
#include "lib2/mip.cpp"
#include "util.cpp"
#include "lib/geometry/partition_constructor.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

using namespace std;

const ll inf = 1e9;
const ll biginf = 1e18;

// TODO: speed up by only duplicating polygons as many times as the total area is less than area of container
PackingOutput optimal_algorithm(PackingInput input) {
    ItemsContainer items = input.items.expand();
    int binaries = 0;
    MIP problem;
    vector<string> in_use_binaries;
    vector<pair<string,FT>> obj_terms;
    fon(i, sz(items)) {
        in_use_binaries[i] = "is_polygon_" + to_string(i) + "_in_use";
        problem.add_binary_variable(in_use_binaries[i]);
        obj_terms.push_back({in_use_binaries[i], items[i].value});
    }
    fon(i, sz(items)) {
        fon(j, sz(items)) {
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
                assert(polygon.number_of_holes() == 1);
                PartitionConstructor pc (polygon);
                partition = pc.get_constrained_delaunay_triangulation();
            }
            Point ref = items[j].get_reference_point(); // (x1,y1)
            // Point centroid = get_centroid(get<1>(items[i])); // (x2,y2)
            string x1 = "polygon_ref0_" + to_string(j) +  "_x";
            string y1 = "polygon_ref0_" + to_string(j) +  "_y";
            string x2 = "polygon_ref0_" + to_string(i) +  "_x";
            string y2 = "polygon_ref0_" + to_string(i) +  "_y";
            problem.add_continuous_variable(x1);
            problem.add_continuous_variable(y1);
            problem.add_continuous_variable(x2);
            problem.add_continuous_variable(y2);
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
                    problem.add_leq_constraint(
                        {{y2, v1.x()},{y1,v1.x()},{x2,v1.y()},{x1,v1.y()},{b, biginf},{in_use_binaries[i],biginf},{in_use_binaries[j],biginf}},
                        v1.x() * c2.y() - v1.y() * c2.x() + 3 * biginf // TODO: safe in terms of casting??
                    );
                }
            }
            // at least one convex region must be satisfied
            {
                vector<pair<string,FT>> terms;
                foe(b, binvars) terms.push_back({b,1});
                terms.push_back({in_use_binaries[i],biginf});
                terms.push_back({in_use_binaries[j],biginf});
                problem.add_geq_constraint(terms, 1 + 2 * biginf); // TODO: safe in terms of precision / automatic casting??
            }
        }
    }
    auto solution = problem.solve();
    PackingOutput output (input);
    fon(i, sz(items)) {
        string xkey = "polygon_ref0_" + to_string(i) +  "_x";
        string ykey = "polygon_ref0_" + to_string(i) +  "_y";
        FT x = solution[xkey];
        FT y = solution[ykey];
        Item new_item = items[i].move_first_point(Point(x,y));
        output.add_item(new_item);
    }
    return output;
}

