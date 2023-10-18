#include <bits/stdc++.h>
#include <CGAL/SCIP_mixed_integer_program_traits.h>

#include "lib/util/com.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

using namespace std;

typedef CGAL::SCIP_mixed_integer_program_traits<double>      MIP_Solver;
// typedef CGAL::SCIP_mixed_integer_program_traits<FT>          MIP_Solver;
typedef typename MIP_Solver::Variable                          Variable;
typedef typename MIP_Solver::Linear_objective          Linear_objective;
typedef typename MIP_Solver::Linear_constraint        Linear_constraint;

class MIP {

private:
    int constraint_name = 0;
    MIP_Solver solver;
    map<string,Variable*> vars;
    vector<string> vars_order;

    string get_constraint_name() {
        return "c_" + to_string(constraint_name++);
    }

    void _add_constraint(vector<pair<string,FT>> a, FT b, string type) {
        foe(p, a) {
            ASSERT(vars.count(p.fi) == 1, "variable " << p.fi << " was not created");
        }
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

    void _add_variable(string name, string type) {
        ASSERT(vars.count(name) == 0, "variable " << name << " already exists");
        Variable* x;
        if(type == "con") {
            x = solver.create_variable(Variable::CONTINUOUS, Variable::infinity(), Variable::infinity(), name);
        } else if(type == "bin") {
            x = solver.create_variable(Variable::BINARY);
            x->set_name(name);
        } else {
            ASSERT(false, "unknown variable type " << type);
        }
        vars[name] = x;
        vars_order.push_back(name);
    }

    void _set_objective(string type, vector<pair<string,FT>> c) {
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

public:


    void add_eq_constraint(vector<pair<string,FT>> a, FT b) {
        _add_constraint(a,b,"eq");
    }

    void add_leq_constraint(vector<pair<string,FT>> a, FT b) {
        _add_constraint(a,b,"leq");
    }

    void add_geq_constraint(vector<pair<string,FT>> a, FT b) {
        _add_constraint(a,b,"geq");
    }

    void add_binary_variable(string name) {
        _add_variable(name, "bin");
    }

    void add_continuous_variable(string name) {
        _add_variable(name, "con");
    }

    void set_max_objective(vector<pair<string,FT>> c) {
        _set_objective("max", c);
    }

    void set_min_objective(vector<pair<string,FT>> c) {
        _set_objective("min", c);
    }

    map<string,FT> solve() {
        const vector<double>& results = solver.solution();
        // TODO: are these actually exact results or are they internally converted to doubles?
        // TODO: probably more efficient with doubles though
        assert(sz(results) == sz(vars_order));
        map<string,FT> res;
        fon(i, sz(results)) {
            res[vars_order[i]] = results[i];
        }
        return res;
    }
};
