#pragma once

#include <bits/stdc++.h>
#include "lib/util/cgal.h"

class MIP {

public:

    int constraint_name = 0;

    virtual void _add_constraint(std::vector<std::pair<std::string,FT>> a, FT b, std::string type) = 0; // TODO: why do we need to equal 0?

    virtual void _add_variable(std::string name, std::string type) = 0;

    virtual void _set_objective(std::string type, std::vector<std::pair<std::string,FT>> c) = 0;

    std::string get_constraint_name();

    virtual ~MIP() = default;

    void add_eq_constraint(std::vector<std::pair<std::string,FT>> a, FT b); 

    void add_leq_constraint(std::vector<std::pair<std::string,FT>> a, FT b); 

    void add_geq_constraint(std::vector<std::pair<std::string,FT>> a, FT b); 

    void add_binary_variable(std::string name); 

    void add_integer_variable(std::string name); 

    void add_continuous_variable(std::string name); 

    void set_max_objective(std::vector<std::pair<std::string,FT>> c); 

    void set_min_objective(std::vector<std::pair<std::string,FT>> c); 

    virtual std::map<std::string,FT> solve() = 0; 
};

