#include <bits/stdc++.h>
#include <CGAL/SCIP_mixed_integer_program_traits.h>

#include "lib/util/cgal.h"

class MIP_Solver;

class MIP {

private:
    int constraint_name = 0;
    MIP_Solver solver;
    map<string,Variable*> vars;
    vector<string> vars_order;

    string get_constraint_name();

    void _add_constraint(vector<pair<string,FT>> a, FT b, string type);

    void _add_variable(string name, string type); 

    void _set_objective(string type, vector<pair<string,FT>> c); 

public:


    void add_eq_constraint(vector<pair<string,FT>> a, FT b); 

    void add_leq_constraint(vector<pair<string,FT>> a, FT b); 

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
