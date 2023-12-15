#include <bits/stdc++.h>

#include "lib2/mip/gurobi.h"

#include "lib/util/cgal.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

bool is_gurobi_initialized = false;
GRBEnv* gurobi_env;

GRBEnv& get_env() {
    if(is_gurobi_initialized) return *gurobi_env;
    gurobi_env = new GRBEnv(true);
    gurobi_env->set("LogFile", "/tmp/gurobi_mip.log");
    cout << "set env" << endl;
    gurobi_env->start();
    cout << "started env" << endl;
    is_gurobi_initialized = true;
    return *gurobi_env;
}

Gurobi_MIP::~Gurobi_MIP(){}

Gurobi_MIP::Gurobi_MIP() : solver(get_env()) {
}

void Gurobi_MIP::status() {
    std::cout << "[c++] Number of variables: " << solver.get(GRB_IntAttr_NumVars) << std::endl;
    std::cout << "[c++] Number of binary variables: " << solver.get(GRB_IntAttr_NumBinVars) << std::endl;
    std::cout << "[c++] Number of integer variables: " << solver.get(GRB_IntAttr_NumIntVars) - solver.get(GRB_IntAttr_NumBinVars) << std::endl;
    std::cout << "[c++] Number of continuous variables: " << solver.get(GRB_IntAttr_NumVars) - solver.get(GRB_IntAttr_NumIntVars) << std::endl;
    std::cout << "[c++] Number of constraints: " << solver.get(GRB_IntAttr_NumConstrs) << std::endl;
}

ll get_ll(FT v) {
    string fraction = decompose_fraction(v);
    std::istringstream iss(fraction);
    std::string part;
    ASSERT(std::getline(iss, part, '/'), "v is not a fraction");
    ll numerator = std::stoll(part);
    ASSERT(std::getline(iss, part), "denominator problem");
    ll denominator = std::stoll(part);
    ASSERT(denominator == 1, "denominator must be 1");
    return numerator;
}

void GurobiCallback::cb_set_problem(Gurobi_MIP* _problem) {
    cb_problem = _problem;
}

void GurobiCallback::cb_add_lazy_constraint(
    std::vector<std::pair<std::string,FT>> a,
    FT b,
    std::string type
) {
    foe(p, a) {
        ASSERT(cb_problem->vars.count(p.fi) == 1, "variable " << p.fi << " was not created");
    }
    cb_problem->constraints[type].pb({a,b});
    GRBLinExpr expr = 0;
    foe(p, a) {
        expr += cb_problem->vars[p.fi] * get_ll(p.se);
    }
    if(type == "eq") {
        addLazy(expr, GRB_EQUAL, get_ll(b));
    } else if(type == "leq") {
        addLazy(expr, GRB_LESS_EQUAL, get_ll(b));
    } else if(type == "geq") {
        addLazy(expr, GRB_GREATER_EQUAL, get_ll(b));
    } else {
        ASSERT(false, "unknown constraint type " << type);
    }
}

void Gurobi_MIP::set_callback(
    GurobiCallback* _callbackobj
) {
    callbackobj = _callbackobj;
    solver.setCallback(callbackobj);
}

void Gurobi_MIP::_add_constraint(vector<pair<string,FT>> a, FT b, string type) {
    foe(p, a) {
        ASSERT(vars.count(p.fi) == 1, "variable " << p.fi << " was not created");
    }
    constraints[type].pb({a,b});
    /*cout << "[c++] Adding constraint" << endl;
    fon(i, sz(a)) {
        cout << "   " << a[i].fi << " * " << get_ll(a[i].se);
        if(i < sz(a) - 1) cout << " +";
        cout << endl;
    }
    cout << "   " << type << " " << get_ll(b) << endl;
    cout << "[c++] End of constraint" << endl;*/
    GRBLinExpr expr = 0;
    foe(p, a) {
        expr += vars[p.fi] * get_ll(p.se);
    }
    if(type == "eq") {
        solver.addConstr(expr == get_ll(b), get_constraint_name());
    } else if(type == "leq") {
        solver.addConstr(expr <= get_ll(b), get_constraint_name());
    } else if(type == "geq") {
        solver.addConstr(expr >= get_ll(b), get_constraint_name());
    } else {
        ASSERT(false, "unknown constraint type " << type);
    }
    // get_status(solver);
}

void Gurobi_MIP::_add_variable(string name, string type) {
    ASSERT(vars.count(name) == 0, "variable " << name << " already exists");
    // cout << "[c++] Adding variable " << name << " of type " << type << endl;
    if(type == "con") {
        ASSERT(false, "cannot use continous vairable right now");
        vars[name] = solver.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
    } else if(type == "bin") {
        vars[name] = solver.addVar(0, 1, 0, GRB_BINARY, name);
    } else if (type == "int") {
        vars[name] = solver.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_INTEGER, name);
    } else {
        ASSERT(false, "unknown variable type " << type);
    }
    // get_status(solver);
}

void Gurobi_MIP::_set_objective(string type, vector<pair<string,FT>> c) {
    foe(p, c) {
        ASSERT(vars.count(p.fi) == 1, "variable " << p.fi << " was not created");
    }
    GRBLinExpr expr = 0;
    foe(p, c) {
        expr += vars[p.fi] * get_ll(p.se);
    }
    // cout << "[c++] Objective: " << expr << endl;
    if(type == "max") {
        cout << "[c++] Setting objective to max" << endl;
        solver.setObjective(expr, GRB_MAXIMIZE);
    } else if(type == "min") {
        cout << "[c++] Setting objective to min" << endl;
        solver.setObjective(expr, GRB_MINIMIZE);
    } else {
        ASSERT(false, "unknown objective type " << type);
    }
    // get_status(solver);
}


map<string,FT> Gurobi_MIP::get_values() {
    map<string,FT> res;
    foe(p, vars) {
        assert(p.fi == p.se.get(GRB_StringAttr_VarName));
        res[p.fi] = p.se.get(GRB_DoubleAttr_X);
    }
    return res;
}

// TODO: we can iterate over multiple solutions
map<string,FT> Gurobi_MIP::solve() {

    status();

    // solver.set(GRB_DoubleParam_Cutoff, 100);
    solver.set(GRB_DoubleParam_MIPGap, 0.1);
    solver.set(GRB_IntParam_MIPFocus, 1);
    solver.set(GRB_IntParam_NumericFocus, 3);
    solver.set(GRB_IntParam_Presolve, 2);
    solver.set(GRB_DoubleParam_FeasibilityTol, 1e-9);
    solver.set(GRB_DoubleParam_IntFeasTol, 1e-9);
    solver.set(GRB_IntParam_LPWarmStart, 2);
    solver.set(GRB_DoubleParam_Heuristics, 0.5);
    // solver.set(GRB_IntParam_LazyConstraints, 1); // TODO: only set when using lazy constraints
    solver.optimize();

    auto expr = solver.getObjective();//.getLinExpr();
    cout << "[c++] " << "Objective function: " << expr << endl;

    int optimstatus = solver.get(GRB_IntAttr_Status);
    double objval = 0;
    if (optimstatus == GRB_OPTIMAL) {
      objval = solver.get(GRB_DoubleAttr_ObjVal); // TODO: Get int for int model
      cout << "[c++] Optimal objective: " << objval << endl;
    } else if (optimstatus == GRB_INF_OR_UNBD) {
      ASSERT(false, "Model is infeasible or unbounded");
    } else if (optimstatus == GRB_INFEASIBLE) {
      ASSERT(false, "Model is infeasible");
    } else if (optimstatus == GRB_UNBOUNDED) {
      ASSERT(false, "Model is unbounded");
    } else {
      ASSERT(false, "Optimization was stopped with status = " << optimstatus);
    }

    map<string,FT> res = get_values();

    double tol = 1e-3;
    foe(c, constraints["eq"]) {
        FT lhs = 0;
        foe(term, c.fi) lhs += res[term.fi] * term.se;
        FT rhs = c.se;
        FT diff = rhs - lhs;
        ASSERT(lhs == rhs, "equal constraint does not hold with diff=" << diff.to_double());
    }
    foe(c, constraints["geq"]) {
        FT lhs = 0;
        foe(term, c.fi) lhs += res[term.fi] * term.se;
        FT rhs = c.se;
        FT diff = rhs - lhs;
        ASSERT(lhs >= rhs - tol, "geq constraint does not hold with diff=" << diff.to_double());
    }
    foe(c, constraints["leq"]) {
        FT lhs = 0;
        foe(term, c.fi) lhs += res[term.fi] * term.se;
        FT rhs = c.se;
        FT diff = rhs - lhs;
        ASSERT(lhs <= rhs + tol, "geq constraint does not hold with diff=" << diff.to_double());
    }
    return res;
}


void Gurobi_MIP::set_warm_start(std::map<std::string,FT>& sol) {
    cout << "[c++] " << "Using warm start" << endl;
    foe(p, sol) {
        auto& var = vars[p.fi];
        var.set(GRB_DoubleAttr_Start, p.se.to_double());
    }
}
