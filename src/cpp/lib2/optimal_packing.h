#pragma once

#include <bits/stdc++.h>

#include "lib/util/cgal.h"
#include "lib2/mip/gurobi.h"
#include "io.h"

// TODO: move these into MIP lib
typedef std::pair<std::string,std::string> MIPVariable;
//                                            string should be MIPVariable
typedef std::tuple<std::vector<std::pair<std::string, FT>>, std::string, FT> MIPConstraint;

class OptimalPacking {

public:

    MIPVariable get_ref_coord_variable_x(int idx); 

    MIPVariable get_ref_coord_variable_y(int idx); 

    void add_variable(Gurobi_MIP& problem, MIPVariable constraint); 

    void add_constraint(Gurobi_MIP& problem, MIPConstraint constraint); 

    std::pair<std::vector<MIPVariable>, std::vector<MIPConstraint>> get_iteminitem_constraints(
        std::pair<MIPVariable, MIPVariable> p1,
        std::pair<MIPVariable, MIPVariable> p2,
        Item item1,
        Item item2, // item2 should not be in item1
        std::string binary_prefix,
        std::vector<MIPVariable> enabled
    ); 

    std::vector<MIPConstraint> get_constraints_point_inside_convex_polygon(
        Polygon& pol,
        std::string var_x,
        std::string var_y,
        MIPVariable binary, // whether the constraints should be enabled
        Vector offset // the point (var_x, var_y) + offset
    );

    PackingOutput run_inner(PackingInput input); 

    PackingOutput run(PackingInput input); 

};