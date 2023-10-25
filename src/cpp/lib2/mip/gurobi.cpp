#include <bits/stdc++.h>

#include "lib2/mip/gurobi.h"

#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

Gurobi_MIP::~Gurobi_MIP(){}

Gurobi_MIP::Gurobi_MIP() : env(true), solver(GRBModel(env)){
    cout << "HEYO" << endl;
    env.set("LogFile", "/tmp/mip.log");
    env.start();
}

void Gurobi_MIP::_add_constraint(vector<pair<string,FT>> a, FT b, string type) {
    foe(p, a) {
        ASSERT(vars.count(p.fi) == 1, "variable " << p.fi << " was not created");
    }
    cout << "[c++] Adding constraint" << endl;
    fon(i, sz(a)) {
        cout << "   " << a[i].fi << " * " << a[i].se.to_double();
        if(i < sz(a) - 1) cout << " +";
        cout << endl;
    }
    cout << "   " << type << " " << b.to_double() << endl;
    cout << "[c++] End of constraint" << endl;
    GRBLinExpr expr = 0;
    foe(p, a) {
        expr += vars[p.fi] * p.se.to_double();
    }
    if(type == "eq") {
        solver.addConstr(expr == b.to_double(), get_constraint_name());
    } else if(type == "leq") {
        solver.addConstr(expr <= b.to_double(), get_constraint_name());
    } else if(type == "geq") {
        solver.addConstr(expr >= b.to_double(), get_constraint_name());
    } else {
        ASSERT(false, "unknown constraint type " << type);
    }
}

void Gurobi_MIP::_add_variable(string name, string type) {
    ASSERT(vars.count(name) == 0, "variable " << name << " already exists");
    cout << "[c++] Adding variable " << name << " of type " << type << endl;
    GRBVar x;
    if(type == "con") {
        x = solver.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
    } else if(type == "bin") {
        x = solver.addVar(0.0, 1.0, 0.0, GRB_BINARY, name);
    } else if (type == "int") {
        x = solver.addVar(0.0, 1.0, 0.0, GRB_INTEGER, name);
    } else {
        ASSERT(false, "unknown variable type " << type);
    }
    vars[name] = x;
    vars_order.push_back(name);
}

void Gurobi_MIP::_set_objective(string type, vector<pair<string,FT>> c) {
    foe(p, c) {
        ASSERT(vars.count(p.fi) == 1, "variable " << p.fi << " was not created");
    }
    GRBLinExpr expr = 0;
    foe(p, c) {
        expr += vars[p.fi] * p.se.to_double();
    }
    if(type == "max") {
        solver.setObjective(expr, GRB_MAXIMIZE);
    } else if(type == "min") {
        solver.setObjective(expr, GRB_MINIMIZE);
    } else {
        ASSERT(false, "unknown objective type " << type);
    }
}

// TODO: we can iterate over multiple solutions
map<string,FT> Gurobi_MIP::solve() {
    // std::cout << "[c++] Solving MIP" << std::endl;
    // std::cout << "[c++] Number of variables: " << solver.get(GRB_IntAttr_NumVars) << std::endl;
    // std::cout << "[c++] Number of binary variables: " << solver.get(GRB_IntAttr_NumBinVars) << std::endl;
    // std::cout << "[c++] Number of integer variables: " << solver.get(GRB_IntAttr_NumIntVars) - solver.get(GRB_IntAttr_NumBinVars) << std::endl;
    // std::cout << "[c++] Number of continuous variables: " << solver.get(GRB_IntAttr_NumVars) - solver.get(GRB_IntAttr_NumIntVars) << std::endl;
    // std::cout << "[c++] Number of constraints: " << solver.get(GRB_IntAttr_NumConstrs) << std::endl;

    solver.optimize();

    int optimstatus = solver.get(GRB_IntAttr_Status);
    double objval = 0;
    if (optimstatus == GRB_OPTIMAL) {
      objval = solver.get(GRB_DoubleAttr_ObjVal); // TODO: Get int for int model
      cout << "Optimal objective: " << objval << endl;
    } else if (optimstatus == GRB_INF_OR_UNBD) {
      ASSERT(false, "Model is infeasible or unbounded");
    } else if (optimstatus == GRB_INFEASIBLE) {
      ASSERT(false, "Model is infeasible");
    } else if (optimstatus == GRB_UNBOUNDED) {
      ASSERT(false, "Model is unbounded");
    } else {
      ASSERT(false, "Optimization was stopped with status = " << optimstatus);
    }

    map<string,FT> res;
    foe(p, vars) {
        assert(p.fi == p.se.get(GRB_StringAttr_VarName));
        res[p.fi] = p.se.get(GRB_DoubleAttr_X); // TODO: get int for integer vars and bools
    }
    return res;
}

