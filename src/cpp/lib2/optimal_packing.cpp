#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/configuration_space.h"
#include "lib2/mip/mip.h"
#include "lib2/util.h"
#include "lib/geometry/partition_constructor.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

using namespace std;

# TODO: move these into MIP lib
typedef pair<string,string> MIPVariable;
typedef tuple<vector<pair<MIPVariable, FT>>, string, FT> MIPConstraint;

const ll inf = 1e5;
const ll biginf = 1e9;
const double scale = 1;
const ll max_partition_size 100000;

Polygon scale_polygon(Polygon pol, double scale) {
    foe(e, pol) {
        e = {e.x() * scale, e.y() * scale};
    }
    return pol;
}

ItemsContainer scale_items(ItemsContainer items, double scale) {
    foe(item, items) {
        item.pol = scale_polygon(item.pol);
    }
    return items;
}

void assert_is_integer_polygon(Polygon& pol) {
    foe(p, pol) {
        ASSERT(is_integer(p.x()), "polygon has non-integer coordinate");
        ASSERT(is_integer(p.y()), "polygon has non-integer coordinate");
    }
}

class OptimalAlgorithm {

    MIPVariable get_ref_coord_variable_x(int idx) {
        return {"int", "polygon_ref_" + to_string(idx) +  "_x"};
    }

    MIPVariable get_ref_coord_variable_y(int idx) {
        return {"int", "polygon_ref_" + to_string(idx) +  "_y"};
    }

    void add_variable(Gurobi_MIP& problem, MIPVariable& constraint) {
        string type = constraint.first;
        if(type == "con") {
            problem.add_continuous_variable(constraint.second);
        } else if(type == "bin") {
            problem.add_binary_variable(constraint.second);
        } else if (type == "int") {
            problem.add_integer_variable(constraint.second);
        } else {
            assert(false);
        }
    }

    void add_constraint(Gurobi_MIP& problem, MIPConstraint& constraint) {
        if(get<1>(constraint) == "eq") {
            problem.add_eq_constraint({get<0>(constraint), get<2>(constraint)});
        else if(get<1>(constraint) == "geq") {
            problem.add_geq_constraint({get<0>(constraint), get<2>(constraint)});
        } else if(get<1>(constraint) == "leq") {
            problem.add_leq_constraint({get<0>(constraint), get<2>(constraint)});
        } else {
            assert(false);
        }
    }

    pair<vector<MIPVariable>, vector<MIPConstraint>> get_iteminitem_constraints(
        pair<MIPVariable, MIPVariable> p1,
        pair<MIPVariable, MIPVariable> p2,
        Item item1,
        Item item2 // item2 should not be in item1
        string binary_prefix,
        vector<MIPVariable> enabled
    ) {
        // Compute partition
        vector<Polygon> partition;
        {
            // Compute configuration space
            auto config_space = ConfigurationSpace(
                Polygon_set(item1);
                item2,
                item2.get_reference_point()
            ).space;

            // Snap it to a grid
            auto config_space_int = get_complement(
                integerSnapToGrid(get_complement(cp.space)).space
            ); // TODO: taking complement twice

            // Computer intersection of large square and completement of configuration space
            Polygon square; square.pb(Point(-inf,-inf));square.pb(Point(inf,-inf));square.pb(Point(inf,inf));square.pb(Point(-inf,inf));
            Polygon_set ps (square);
            ps.intersection(config_space_int);

            // Get the single polygon with one hole (the complement of conf space)
            auto polygons = to_polygon_vector(ps);
            assert(sz(polygons) == 1);
            auto polygon = polygons[0];
            assert(polygon.number_of_holes() == 1);
            assert_is_integer_polygon(polygon.outer_boundary());
            foe(hole, polygon.holes()) assert_is_integer_polygon(hole);

            // Compute convex cover (triangulation, partiton or something)
            PartitionConstructor pc (polygon);
            partition = pc.get_constrained_delaunay_triangulation();
            // partition = pc.get_approx_convex_partition();
            foe(pol, partition) assert_is_integer_polygon(pol);
        }

        Point ref = item1.get_reference_point();
        auto [x1, y1] = get_ref_coord_variable_names(j);
        auto [x2, y2] = get_ref_coord_variable_names(i);
        int idx = 0;
        vector<MIPVariable> variables;
        vector<MIPConstraint> constraints;

        // Add constraints
        foe(pol, partition) {

            ASSERT(pol.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");

            string b = binary_prefix + to_string(idx);
            idx += 1;
            variables.push_back({"bin", b});

            fon(k, sz(pol)) {
                Vector c1 = tri[k] - ref;
                Vector c2 = tri[(k+1) % sz(tri)] - ref;
                Vector v1 (c2.x() - c1.x(), c2.y() - c1.y());
                // v2 = (x2 - x1 - c1.x(), y2 - y1 - c1.y());
                // Cross product
                    // cross(v1,v2) = v1.x() * (y2 - y1 - c1.y()) - v1.y() * (x2 - x1 - c1.x())
                    // = v1.x() * y2 - v1.x() * y1 - v1.x() * c1.y() - v1.y() * x2 + v1.y() * x1 + v1.y() * c1.x() <= 0   TODO: <= is correct right?
                    // <=>
                    // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 <= v1.x() * c1.y() - v1.y() * c1.x()
                MIPConstraint constraint = {
                    {
                        {y2, v1.x()},{y1,-v1.x()},{x2,-v1.y()},{x1,v1.y()},
                        {b, -biginf},
                    },
                    "geq",
                    v1.x() * c1.y() - v1.y() * c1.x() - ((long long) sz(enabled) + 1) * biginf // TODO: safe in terms of casting??
                };
                foe(v, enabled) get<0>(constraint).push_back({v, -biginf});
                constraints.push_back(constraint);
            }
        }

        // At least one convex region must be satisfied
        {
            vector<pair<string,FT>> terms;
            foe(b, variables) terms.push_back({b.se,1});
            foe(v, enabled) get<0>(constraint).push_back({v, -(max_partition_size + 100ll)});
            constraints.push_back(
                terms,
                "geq",
                1ll - (max_partition_size + 100ll) * sz(enabled)
            ); // TODO: safe in terms of precision / automatic casting??
        }

        return {variables, constraints};
    }

    vector<MIPConstraint> get_constraints_point_inside_convex_polygon(
        Polygon& pol,
        string var_x,
        string var_y,
        string binary, // whether the constraints should be enabled
        Vector offset // the point (var_x, var_y) + offset
    ) {
        ASSERT(pol.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");
        vector<MIPConstraint> res;
        fon(k, sz(pol)) {
            Point c1 = pol[k];
            Point c2 = pol[(k+1) % sz(pol)];
            auto o = offset;
            Vector v1 (c2.x() - c1.x(), c2.y() - c1.y());
            res.push_back({
                {{y,v1.x()},{x,-v1.y()}, {binary,-biginf}},
                "geq",
                -v1.x() * o.y() + v1.x() * c1.y() + v1.y() * o.x() - v1.y() * c1.x() - biginf // TODO: safe in terms of casting??
            });
        }
        return res;
    };

    PackingOutput run_inner(PackingInput input) {
        cout << "[c++] Number of expanded items: " << sz(items) << endl;
        Gurobi_MIP problem;
        vector<string> in_use_binaries;
        cout << "[c++] Adding 'in-use' binaries" << endl;
        {
            fon(i, sz(items)) {
                problem.add_binary_variable(in_use_binaries[i]);
            }
        }
        cout << "[c++] Setting objective" << endl;
        {
            vector<pair<string,FT>> obj_terms;
            fon(i, sz(items)) {
                in_use_binaries.push_back("is_polygon_" + to_string(i) + "_in_use");
                obj_terms.push_back({in_use_binaries[i], items[i].value});
            }
            problem.set_max_objective(obj_terms);
        }
        cout << "[c++] Adding reference x and y variables" << endl;
        {
            fon(i, sz(items)) {
                add_variable(problem, get_ref_coord_variable_x(i));
                add_variable(problem, get_ref_coord_variable_y(i));
            }
        }
        cout << "[c++] Adding items inside container constraints" << endl;
        fon(i, sz(items)) {
            auto x = get_ref_coord_variable_x(i).second;
            auto y = get_ref_coord_variable_y(i).second;
            auto ref = items[i].get_reference_point();
            foe(p, items[i].pol) {
                foe(constraint, get_constraints_point_inside_convex_polygon(
                    container, x, y, in_use_binaries[i], p - ref)
                ) {
                    add_constraint(problem, constraint);
                }
            }
        }
        cout << "[c++] Adding items no overlap constraints" << endl;
        fon(i, sz(items)) {
            fon(j, i) {
                auto [variables, constraints] = get_iteminitem_constraints(
                    {get_ref_coord_variable_x(j), get_ref_coord_variable_y(j)},
                    {get_ref_coord_variable_x(i), get_ref_coord_variable_y(i)},
                    items[j],
                    items[i], // item2 should not be in item1
                    "binary_" + to_string(i) + "_" + to_string(j) + "_",
                    {in_use_binaries[j], in_use_binaries[i]}
                );
                foe(v, varibles) add_variable(problem, v);
                foe(c, constraints) add_variable(problem, c);
            }
        }

        cout << "[c++] Computing solution using MIP" << endl;
        auto solution = problem.solve();

        cout << "[c++] Moving items to found reference point solution coordinates" << endl;
        PackingOutput output (input);
        fon(i, sz(items)) {
            if(solution[in_use_binaries[i]] > 0.5) {
                auto xkey = get_ref_coord_variable_x(i).second;
                auto ykey = get_ref_coord_variable_y(i).second;
                FT x = solution[xkey];
                FT y = solution[ykey];
                Item new_item = items[i].move_ref_point(Point(x,y));
                output.add_item(new_item);
            }
        }

        return output;
    }

    PackingOutput run(PackingInput input) {
        cout << "[c++] RUNNING OPTIMAL ALGORITHM" << endl;
        PackingInput modified_input {
            scale_polygon(original_input.container, scale),
            scale_items(original_input.items.expand(), scale),
        };
        PackingOutput output = run_inner(modified_input);
        output.items = scale_items(output.items, 1.0/scale);
        return output;
    }

};
