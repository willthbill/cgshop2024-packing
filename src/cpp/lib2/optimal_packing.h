#pragma once

#include <bits/stdc++.h>

#include "lib/util/cgal.h"
#include "lib2/mip/gurobi.h"
#include "io.h"

// TODO: move these into MIP lib
typedef std::pair<std::string,std::string> MIPVariable;
//                                            string should be MIPVariable
typedef std::tuple<std::vector<std::pair<std::string, FT>>, std::string, FT> MIPConstraint;

class OptimalPacking : public GurobiCallback {

private:

    PackingInput input;
    ItemsContainer items;
    Gurobi_MIP problem;
    std::vector<MIPVariable> in_use_binaries;
    std::set<std::pair<int,int>> itempair_state;

public:

    OptimalPacking(PackingInput input);

    MIPVariable get_ref_coord_variable_x(int idx); 

    MIPVariable get_ref_coord_variable_y(int idx); 

    void add_variable(MIPVariable constraint); 

    void add_constraint(MIPConstraint constraint, bool lazy); 

    void callback() override;

    std::pair<std::vector<MIPVariable>, std::vector<MIPConstraint>> get_iteminitem_constraints(
        std::pair<MIPVariable, MIPVariable> p1,
        std::pair<MIPVariable, MIPVariable> p2,
        Item item1,
        Item item2, // item2 should not be in item1
        std::string binary_prefix,
        std::vector<MIPVariable> enabled,
        bool use_bounding_boxes
    ); 

    std::vector<MIPConstraint> get_constraints_point_inside_convex_polygon(
        Polygon& pol,
        std::string var_x,
        std::string var_y,
        MIPVariable binary, // whether the constraints should be enabled
        Vector offset // the point (var_x, var_y) + offset
    );

    PackingOutput run_inner(bool lazy = false); 

    PackingOutput run(); 

};
