#pragma once

#include <bits/stdc++.h>
#include <CGAL/SCIP_mixed_integer_program_traits.h>

#include "lib/util/cgal.h"

typedef CGAL::SCIP_mixed_integer_program_traits<double>      MIP_Solver;
// typedef CGAL::SCIP_mixed_integer_program_traits<FT>          MIP_Solver;
typedef typename MIP_Solver::Variable                          Variable;
typedef typename MIP_Solver::Linear_objective          Linear_objective;
typedef typename MIP_Solver::Linear_constraint        Linear_constraint;

class MIP {

private:
    int constraint_name = 0;
    MIP_Solver solver;
    std::map<std::string,Variable*> vars;
    std::vector<std::string> vars_order;

    std::string get_constraint_name();

    void _add_constraint(std::vector<std::pair<std::string,FT>> a, FT b, std::string type);

    void _add_variable(std::string name, std::string type); 

    void _set_objective(std::string type, std::vector<std::pair<std::string,FT>> c); 

public:

    void add_eq_constraint(std::vector<std::pair<std::string,FT>> a, FT b); 

    void add_leq_constraint(std::vector<std::pair<std::string,FT>> a, FT b); 

    void add_geq_constraint(std::vector<std::pair<std::string,FT>> a, FT b); 

    void add_binary_variable(std::string name); 

    void add_continuous_variable(std::string name); 

    void set_max_objective(std::vector<std::pair<std::string,FT>> c); 

    void set_min_objective(std::vector<std::pair<std::string,FT>> c); 

    std::map<std::string,FT> solve(); 
};
