#include <bits/stdc++.h>

#include "lib2/mip/mip.h"

using namespace std;

string MIP::get_constraint_name() {
    return "c_" + to_string(constraint_name++);
}

void MIP::add_eq_constraint(vector<pair<string,FT>> a, FT b) {
    _add_constraint(a,b,"eq");
}

void MIP::add_leq_constraint(vector<pair<string,FT>> a, FT b) {
    _add_constraint(a,b,"leq");
}

void MIP::add_geq_constraint(vector<pair<string,FT>> a, FT b) {
    _add_constraint(a,b,"geq");
}

void MIP::add_binary_variable(string name) {
    _add_variable(name, "bin");
}

void MIP::add_integer_variable(string name) {
    _add_variable(name, "int");
}

void MIP::add_continuous_variable(string name) {
    _add_variable(name, "con");
}

void MIP::set_max_objective(vector<pair<string,FT>> c) {
    _set_objective("max", c);
}

void MIP::set_min_objective(vector<pair<string,FT>> c) {
    _set_objective("min", c);
}

