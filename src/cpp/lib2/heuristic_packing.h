#pragma once

#include <bits/stdc++.h>

#include "lib/util/cgal.h"
#include "lib2/mip/gurobi.h"
#include "io.h"

// CHECK OLD COMMIT FOR THIS IMPLEMENTATION hash: 2b34aec48ebac679f796c25bcf04b49a3cf3231e
/*class HeuristicPackingMIP {

private:

public:

    PackingOutput run(PackingInput); 

};*/

class HeuristicPackingNOMIP {

private:

public:

    PackingOutput run(PackingInput _input, bool print = true, int sort_type = 0);

};

class HeuristicPackingGrid {

private:

public:

    PackingOutput run(PackingInput); 

};

#include<ext/pb_ds/tree_policy.hpp>
#include<ext/pb_ds/assoc_container.hpp>
using namespace __gnu_pbds;
template<typename T>
using ordered_set_rev = tree<T, null_type, std::greater<T>, rb_tree_tag, tree_order_statistics_node_update>;
template<typename T1, typename T2>
using ordered_map_rev = tree<T1, T2, std::greater<T1>, rb_tree_tag, tree_order_statistics_node_update>;
/// Example
/*
    ordered_set<int> oset;
    oset.insert(1);
    oset.find_by_order(0); // 1
    oset.order_of_key(1); // 0
*/

class AdvancedItemsContainer {
public:
    FT avg_area;
    ordered_set_rev<std::pair<FT,int>> available_items;
    ItemsContainer items;
    std::vector<FT> item_areas;
    FT sorting_metric(int idx);
    AdvancedItemsContainer(ItemsContainer& items); 
    int size(); 
    void add_item(int idx); 
    void erase_item(int idx); 
    std::pair<ItemsContainer,std::vector<int>> extract_items_random_area(int k, FT area_up, FT area_lb); 
    std::pair<ItemsContainer,std::vector<int>> extract_items_random(int k); 
};

class HeuristicPackingRecursive {

private:

public:

    PackingOutput run(PackingInput); 

    void solve(
        Polygon_set& container,
        AdvancedItemsContainer& items,
        PackingOutput& output,
        Polygon_set& packed,
        int depth
    );
};