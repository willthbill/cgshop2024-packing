#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/optimal_packing.h"

#include "lib2/mip/gurobi.h"
#include "lib2/configuration_space.h"
#include "lib2/snap.h"
#include "lib2/mip/mip.h"
#include "lib2/util.h"
#include "lib/geometry/partition_constructor.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

// TODO: speed up by only duplicating polygons as many times as the total area is less than area of container
// TODO: maybe see if there are common factors (or something like that) on each side so we can make numbers smaller
// TODO: optimize by drawing triangle around instead of square???
// TODO: actually calculate bounds (inf, biginf, ...)

const ll inf = 5000;
const ll biginf = 1e7;
const double scale = 1;
const ll max_partition_size = 100000;

Polygon scale_polygon(Polygon pol, double scale) {
    foe(e, pol) {
        e = {e.x() * scale, e.y() * scale};
    }
    return pol;
}

ItemsContainer scale_items(ItemsContainer items, double scale) {
    foe(item, items) {
        item.pol = scale_polygon(item.pol, scale);
    }
    return items;
}

void assert_is_integer_polygon(Polygon& pol) {
    foe(p, pol) {
        ASSERT(is_integer(p.x()), "polygon has non-integer coordinate");
        ASSERT(is_integer(p.y()), "polygon has non-integer coordinate");
    }
}


MIPVariable OptimalPacking::get_ref_coord_variable_x(int idx) {
    return {"int", "polygon_ref_" + to_string(idx) +  "_x"};
}

MIPVariable OptimalPacking::get_ref_coord_variable_y(int idx) {
    return {"int", "polygon_ref_" + to_string(idx) +  "_y"};
}

// TODO: design is not very good, this functionality is already implemented in MIP lib
void OptimalPacking::add_variable(Gurobi_MIP& problem, MIPVariable variable) {
    string type = variable.first;
    if(type == "con") {
        problem.add_continuous_variable(variable.second);
    } else if(type == "bin") {
        problem.add_binary_variable(variable.second);
    } else if (type == "int") {
        problem.add_integer_variable(variable.second);
    } else {
        assert(false);
    }
}

// TODO: design is not very good, this functionality is already implemented in MIP lib
void OptimalPacking::add_constraint(Gurobi_MIP& problem, MIPConstraint constraint) {
    if(get<1>(constraint) == "eq") {
        problem.add_eq_constraint(get<0>(constraint), get<2>(constraint));
    } else if(get<1>(constraint) == "geq") {
        problem.add_geq_constraint(get<0>(constraint), get<2>(constraint));
    } else if(get<1>(constraint) == "leq") {
        problem.add_leq_constraint(get<0>(constraint), get<2>(constraint));
    } else {
        assert(false);
    }
}

pair<vector<MIPVariable>, vector<MIPConstraint>> OptimalPacking::get_iteminitem_constraints(
    pair<MIPVariable, MIPVariable> p1,
    pair<MIPVariable, MIPVariable> p2,
    Item item1,
    Item item2, // item2 should not be in item1
    string binary_prefix,
    vector<MIPVariable> enabled
) {
    // Compute partition
    vector<Polygon> partition;
    {
        // Compute configuration space
        Polygon_set disallowed (item1.pol);
        auto config_space = ConfigurationSpace(
            disallowed,
            item2.pol,
            item2.get_reference_point()
        ).space;

        // Snap it to a grid
        auto config_space_int = get_complement(
            SnapToGrid(get_complement(config_space)).space
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
    auto x1 = p1.fi.second;
    auto y1 = p1.se.second;
    auto x2 = p2.fi.second;
    auto y2 = p2.se.second;
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
            Vector c1 = pol[k] - ref;
            Vector c2 = pol[(k+1) % sz(pol)] - ref;
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
            foe(v, enabled) get<0>(constraint).push_back(make_pair(v.se, -biginf));
            constraints.push_back(constraint);
        }
    }

    // At least one convex region must be satisfied
    {
        vector<pair<string,FT>> terms;
        foe(b, variables) terms.push_back({b.se,1});
        foe(v, enabled) terms.push_back(make_pair(v.se, -(max_partition_size + 100ll)));
        constraints.push_back({
            terms,
            "geq",
            1ll - (max_partition_size + 100ll) * sz(enabled)
        }); // TODO: safe in terms of precision / automatic casting??
    }

    return {variables, constraints};
}

vector<MIPConstraint> OptimalPacking::get_constraints_point_inside_convex_polygon(
    Polygon& pol,
    string var_x,
    string var_y,
    MIPVariable binary, // whether the constraints should be enabled
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
            {{var_y,v1.x()},{var_x,-v1.y()}, {binary.se,-biginf}},
            "geq",
            -v1.x() * o.y() + v1.x() * c1.y() + v1.y() * o.x() - v1.y() * c1.x() - biginf // TODO: safe in terms of casting??
        });
    }
    return res;
}

PackingOutput OptimalPacking::run_inner(PackingInput input) {
    auto items = input.items.expand();

    cout << "[c++] Number of expanded items: " << sz(items) << endl;

    Gurobi_MIP problem;
    vector<MIPVariable> in_use_binaries;

    cout << "[c++] Adding 'in-use' binaries" << endl;
    {
        fon(i, sz(items)) {
            in_use_binaries.push_back(
                make_pair(
                    string("bin"),
                    string("is_polygon_" + to_string(i) + "_in_use")
                )
            );
            add_variable(problem, in_use_binaries[i]);
        }
    }

    cout << "[c++] Setting objective" << endl;
    {
        vector<pair<string,FT>> obj_terms;
        fon(i, sz(items)) {
            obj_terms.push_back({in_use_binaries[i].se, items[i].value});
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
                input.container, x, y, in_use_binaries[i], p - ref
            )) {
                add_constraint(problem, constraint);
            }
        }
    }

    cout << "[c++] Adding items no overlap constraints" << endl;
    fon(i, sz(items)) {
        fon(j, i) {
            auto [variables, constraints] = get_iteminitem_constraints(
                make_pair(get_ref_coord_variable_x(j), get_ref_coord_variable_y(j)),
                make_pair(get_ref_coord_variable_x(i), get_ref_coord_variable_y(i)),
                items[j],
                items[i],
                "binary_" + to_string(i) + "_" + to_string(j) + "_",
                {in_use_binaries[j], in_use_binaries[i]}
            );
            foe(v, variables) add_variable(problem, v);
            foe(c, constraints) add_constraint(problem, c);
        }
    }

    cout << "[c++] Computing solution using MIP" << endl;
    auto solution = problem.solve();

    cout << "[c++] Moving items to found reference point solution coordinates" << endl;
    PackingOutput output (input);
    fon(i, sz(items)) {
        if(solution[in_use_binaries[i].se] > 0.5) {
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

PackingOutput OptimalPacking::run(PackingInput input) {
    cout << "[c++] Scaling input" << endl;
    PackingInput modified_input {
        scale_polygon(input.container, scale),
        scale_items(input.items, scale),
    };
    PackingOutput output = run_inner(modified_input);
    cout << "[c++] Unscaling input" << endl;
    output.items = scale_items(output.items, 1.0/scale);
    cout << "[c++] Result found" << endl;
    return output;
}
