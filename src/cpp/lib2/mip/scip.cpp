#include <bits/stdc++.h>

#include "lib2/mip/scip.h"

#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

SCIP_MIP::~SCIP_MIP(){}

void SCIP_MIP::_add_constraint(vector<pair<string,FT>> a, FT b, string type) {
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
    Linear_constraint* c;
    if(type == "eq") {
        c = solver.create_constraint(b.to_double(), b.to_double(), get_constraint_name());
    } else if(type == "leq") {
        c = solver.create_constraint(-Linear_constraint::infinity(), b.to_double(), get_constraint_name());
    } else if(type == "geq") {
        c = solver.create_constraint(b.to_double(), Linear_constraint::infinity(), get_constraint_name());
    } else {
        ASSERT(false, "unknown constraint type " << type);
    }
    foe(p, a) {
        c->add_coefficient(vars[p.fi], p.se.to_double());
    }
}

void SCIP_MIP::_add_variable(string name, string type) {
    ASSERT(vars.count(name) == 0, "variable " << name << " already exists");
    cout << "[c++] Adding variable " << name << " of type " << type << endl;
    Variable* x;
    if(type == "con") {
        x = solver.create_variable(Variable::CONTINUOUS, -Variable::infinity(), Variable::infinity(), name);
    } else if(type == "bin") {
        x = solver.create_variable(Variable::BINARY);
        x->set_name(name);
    } else if (type == "int") {
        x = solver.create_variable(Variable::INTEGER, -Variable::infinity(), Variable::infinity(), name);
    } else {
        ASSERT(false, "unknown variable type " << type);
    }
    vars[name] = x;
    vars_order.push_back(name);
}

void SCIP_MIP::_set_objective(string type, vector<pair<string,FT>> c) {
    foe(p, c) {
        ASSERT(vars.count(p.fi) == 1, "variable " << p.fi << " was not created");
    }
    Linear_objective * obj;
    if(type == "max") {
        obj = solver.create_objective(Linear_objective::MAXIMIZE);
    } else if(type == "min") {
        obj = solver.create_objective(Linear_objective::MINIMIZE);
    } else {
        ASSERT(false, "unknown objective type " << type);
    }
    foe(p, c) {
        obj->add_coefficient(vars[p.fi], p.se.to_double());
    }
}

map<string,FT> SCIP_MIP::solve() {
    cout << "[c++] Solving MIP" << endl;
    cout << "[c++] Number of variables: " << solver.number_of_variables() << endl;
    cout << "[c++] Number of binary variables: " << solver.number_of_binary_variables() << endl;
    cout << "[c++] Number of integer variables: " << solver.number_of_integer_variables() << endl;
    cout << "[c++] Number of continuous variables: " << solver.number_of_continuous_variables() << endl;
    cout << "[c++] Number of constraints: " << solver.number_of_constraints() << endl;
    if(!solver.solve()) {
        ASSERT(false,"solving MIP problem failed");
        return {};
    }
    const vector<double>& results = solver.solution();
    // TODO: are these actually exact results or are they internally converted to doubles?
    // TODO: probably more efficient with doubles though
    ASSERT(sz(results) == sz(vars_order),"");
    map<string,FT> res;
    fon(i, sz(results)) {
        res[vars_order[i]] = results[i];
    }
    return res;
}

