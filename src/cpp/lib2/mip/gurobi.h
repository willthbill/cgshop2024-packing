#pragma once

#include <bits/stdc++.h>
#include "gurobi_c++.h"

#include "lib2/mip/mip.h"
#include "lib/util/common.h"
#include "lib/util/cgal.h"

class Gurobi_MIP;

GRBEnv& get_env();

class GurobiCallback: public GRBCallback {

private:

    Gurobi_MIP* cb_problem;

public:

    virtual void callback() = 0;

    void cb_set_problem(Gurobi_MIP* _problem); 

    void cb_add_lazy_constraint(
        std::vector<std::pair<std::string,FT>> a,
        FT b,
        std::string type
    ); 

};

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
    GRBCallback* callbackobj;

    Gurobi_MIP();

    void set_callback(
        GurobiCallback* _callbackobj
    );

    void _add_constraint(
        std::vector<std::pair<std::string,FT>> a,
        FT b,
        std::string type
    ) override;

    void _add_variable(std::string name, std::string type) override; 

    std::map<std::string, FT> get_values();

    void _set_objective(std::string type, std::vector<std::pair<std::string,FT>> c) override; 

    std::map<std::string,FT> solve() override; 

    void status();

    void set_warm_start(std::map<std::string,FT>& sol);

    void fix_variable(std::string var, FT value);

    ~Gurobi_MIP() override;

};
