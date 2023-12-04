#include <bits/stdc++.h>
// #include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/optimal_packing.h"
#include "io.h"

using namespace std;

PackingOutput optimal_algorithm(PackingInput input) {
    cout << "[c++] RUNNING OPTIMAL ALGORITHM" << endl;
    return OptimalPacking(input).run();
}

