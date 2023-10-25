#pragma once

#include <bits/stdc++.h>
#include <CGAL/SCIP_mixed_integer_program_traits.h>

#include "lib2/mip/mip.h"
#include "lib/util/cgal.h"

typedef CGAL::SCIP_mixed_integer_program_traits<double>      MIP_Solver;
// typedef CGAL::SCIP_mixed_integer_program_traits<FT>          MIP_Solver;
typedef typename MIP_Solver::Variable                          Variable;
typedef typename MIP_Solver::Linear_objective          Linear_objective;
typedef typename MIP_Solver::Linear_constraint        Linear_constraint;

class SCIP_MIP : public MIP {

public:

    MIP_Solver solver;
    std::map<std::string,Variable*> vars;
    std::vector<std::string> vars_order;

    void _add_constraint(std::vector<std::pair<std::string,FT>> a, FT b, std::string type) override;

    void _add_variable(std::string name, std::string type) override; 

    void _set_objective(std::string type, std::vector<std::pair<std::string,FT>> c) override; 

    ~SCIP_MIP() override;

    std::map<std::string,FT> solve() override; 
};
