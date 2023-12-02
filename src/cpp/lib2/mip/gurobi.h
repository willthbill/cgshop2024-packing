#pragma once

#include <bits/stdc++.h>
#include "gurobi_c++.h"

#include "lib2/mip/mip.h"
#include "lib/util/cgal.h"

GRBEnv& get_env();

class Gurobi_MIP: public MIP {

public:

    GRBModel solver;
    std::map<std::string,GRBVar> vars;
    std::map<std::string,
        std::vector<
            std::pair<
                std::vector<
                    std::pair<
                        std::string,
                        FT
                    >
                >,
                FT
            >
        >
    > constraints;

    Gurobi_MIP();

    void _add_constraint(std::vector<std::pair<std::string,FT>> a, FT b, std::string type) override;

    void _add_variable(std::string name, std::string type) override; 

    void _set_objective(std::string type, std::vector<std::pair<std::string,FT>> c) override; 

    std::map<std::string,FT> solve() override; 

    ~Gurobi_MIP() override;

};
