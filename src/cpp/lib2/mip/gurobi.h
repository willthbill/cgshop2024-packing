#pragma once

#include <bits/stdc++.h>
#include "gurobi_c++.h"

#include "lib2/mip/mip.h"
#include "lib/util/cgal.h"

class Gurobi_MIP: public MIP {

public:

    GRBModel solver;
    GRBEnv env;
    std::map<std::string,GRBVar> vars;
    std::vector<std::string> vars_order;

    Gurobi_MIP();

    void _add_constraint(std::vector<std::pair<std::string,FT>> a, FT b, std::string type) override;

    void _add_variable(std::string name, std::string type) override; 

    void _set_objective(std::string type, std::vector<std::pair<std::string,FT>> c) override; 

    std::map<std::string,FT> solve() override; 

    ~Gurobi_MIP() override;

};
