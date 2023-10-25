#include <bits/stdc++.h>

#include "lib2/mip/gurobi.h"

#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

bool is_gurobi_initialized = false;
GRBEnv* gurobi_env;

GRBEnv& get_env() {
    if(is_gurobi_initialized) return *gurobi_env;
    gurobi_env = new GRBEnv(true);
    gurobi_env->set("LogFile", "/tmp/gurobi_mip.log");
    gurobi_env->start();
    is_gurobi_initialized = true;
    return *gurobi_env;
}

Gurobi_MIP::~Gurobi_MIP(){}

Gurobi_MIP::Gurobi_MIP() : solver(get_env()) {
    cout << "CONSTRUCTOR CALLED" << endl;
}

void get_status(GRBModel& solver) {
    std::cout << "[c++] Solving MIP" << std::endl;
    cout << solver.get(GRB_IntAttr_NumVars) << endl;
    std::cout << "[c++] Number of variables: " << solver.get(GRB_IntAttr_NumVars) << std::endl;
    std::cout << "[c++] Number of binary variables: " << solver.get(GRB_IntAttr_NumBinVars) << std::endl;
    std::cout << "[c++] Number of integer variables: " << solver.get(GRB_IntAttr_NumIntVars) - solver.get(GRB_IntAttr_NumBinVars) << std::endl;
    std::cout << "[c++] Number of continuous variables: " << solver.get(GRB_IntAttr_NumVars) - solver.get(GRB_IntAttr_NumIntVars) << std::endl;
    std::cout << "[c++] Number of constraints: " << solver.get(GRB_IntAttr_NumConstrs) << std::endl;
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
    get_status(solver);
}

void Gurobi_MIP::_add_variable(string name, string type) {
    ASSERT(vars.count(name) == 0, "variable " << name << " already exists");
    cout << "[c++] Adding variable " << name << " of type " << type << endl;
    if(type == "con") {
        vars[name] = solver.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
    } else if(type == "bin") {
        vars[name] = solver.addVar(0, 1, 0, GRB_BINARY, name);
    } else if (type == "int") {
        vars[name] = solver.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_INTEGER, name);
    } else {
        ASSERT(false, "unknown variable type " << type);
    }
    get_status(solver);
}

void Gurobi_MIP::_set_objective(string type, vector<pair<string,FT>> c) {
    foe(p, c) {
        ASSERT(vars.count(p.fi) == 1, "variable " << p.fi << " was not created");
    }
    GRBLinExpr expr = 0;
    foe(p, c) {
        expr += vars[p.fi] * p.se.to_double();
        debug(p.se.to_double());
    }
    if(type == "max") {
        cout << "setting objective to max" << endl;
        solver.setObjective(expr, GRB_MAXIMIZE);
    } else if(type == "min") {
        solver.setObjective(expr, GRB_MINIMIZE);
    } else {
        ASSERT(false, "unknown objective type " << type);
    }
    get_status(solver);
}

// TODO: we can iterate over multiple solutions
map<string,FT> Gurobi_MIP::solve() {

    // Get linear and quadratic objective components
    /*GRBLinExpr linObj = solver.getObjective().getLinExpr();
    std::string linObjStr = linObj.toString();
    std::cout << "Objective Function: " << linObjStr << std::endl;*/
    /*GRBVar* vars = solver.getVars();
    int numVars = solver.get(GRB_IntAttr_NumVars);
    debug(numVars);

    std::cout << "Objective Function: ";
    for (int i = 0; i < numVars; ++i) {
        double coeff = vars[i].get(GRB_DoubleAttr_Obj);
        if (coeff != 0) {
            std::cout << coeff << " * " << vars[i].get(GRB_StringAttr_VarName);
            if (i < numVars - 1) {
                std::cout << " + ";
            }
        }
    }
    std::cout << std::endl;*/

    get_status(solver);

    debug(solver.getObjective().size());
    auto expr = solver.getObjective().getLinExpr();
    int terms = expr.size();
    debug(terms);
    for (int i = 0; i < terms; ++i) {
        GRBVar var = expr.getVar(i);  // get variable associated with this term
        double coeff = expr.getCoeff(i);  // get coefficient of this term
        std::string varName = var.get(GRB_StringAttr_VarName);  // get variable name
        std::cout << coeff << "*" << varName;

        // if not the last term, print " + "
        if (i < terms - 1) {
            std::cout << " + ";
        }
    }
    std::cout << std::endl;

    get_status(solver);

    solver.set(GRB_DoubleParam_Cutoff, 100);
    solver.set(GRB_DoubleParam_MIPGap, 0.1);
    solver.set(GRB_IntParam_MIPFocus, 1);
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
    debug(sz(vars));
    foe(p, vars) {
        assert(p.fi == p.se.get(GRB_StringAttr_VarName));
        res[p.fi] = p.se.get(GRB_DoubleAttr_X); // TODO: get int for integer vars and bools
    }
    return res;
}

