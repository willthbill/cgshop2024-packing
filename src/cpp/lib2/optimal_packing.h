#pragma once

#include <bits/stdc++.h>

#include "lib/util/cgal.h"
#include "lib2/mip/gurobi.h"
#include "io.h"

// TODO: move these into MIP lib
typedef std::pair<std::string,std::string> MIPVariable;
//                                            string should be MIPVariable
typedef std::tuple<std::vector<std::pair<std::string, FT>>, std::string, FT> MIPConstraint;

class OptimalPackingSlow {

private:

public:

    PackingOutput run(PackingInput); 

};

class OptimalPackingCallback : public GurobiCallback {

private:

    PackingInput cb_input;
    Gurobi_MIP cb_problem;
    std::vector<MIPVariable> cb_in_use_binaries;
    std::vector<std::pair<MIPVariable,MIPVariable>> cb_xys;
    std::set<std::pair<int,int>> cb_itempair_state;

public:

    void callback() override;

    PackingOutput run(PackingInput); 

};

class OptimalPackingFast {

private:

public:

    PackingOutput run(PackingInput); 

};