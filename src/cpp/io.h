#pragma once

#include <bits/stdc++.h>

#include "lib/util/cgal.h"

struct Item {

    FT value;
    long long quantity;
    Polygon pol;
    int idx;
    Vector ref_scaling_translation; // TODO: move this somewhere else

    Item move_ref_point(Point p);

    Point get_reference_point();

};

class ItemsContainer {
private:
    std::vector<Item> items;
public:
    ItemsContainer();
    ItemsContainer(std::vector<std::tuple<int,int,Polygon>> _items);
    void add_item(Item item); 
    void add_item(FT v, long long q, Polygon p, int idx, Vector ref_scaling_translation); 
    ItemsContainer expand(); 
    int size(); 
    Item& operator[](std::size_t i); 

    auto begin() { return items.begin(); }
    auto end() { return items.end(); }
    auto begin() const { return items.begin(); }
    auto end() const { return items.end(); }

    void pop_item();

    std::vector<int> sort_by_value_over_area(); 
    std::vector<int> sort_by_area();

    FT get_average_area(); 
};

ItemsContainer scale_items(ItemsContainer items, FT scale);

struct PackingInput {
    Polygon_set container;
    ItemsContainer items;
};

class PackingOutput {

public:

    PackingInput input;
    ItemsContainer items;
    std::map<int,int> item_count;
    FT score;

    void validate_item(Item item); 

    PackingOutput(PackingInput _input); 

    void add_item(Item item); 

    // also validates item.pol
    Vector get_translation(Item item); 

    /*vector<Vector> get_translations() {
        vector<Vector> translations;
        foe(item, items) {
            translations.push_back(get_translation(item));
        }
        return translations;
    }*/

    void validate_result(); 

    FT get_score(); 

    int size(); 

    auto begin() { return items.begin(); }
    auto end() { return items.end(); }
    auto begin() const { return items.begin(); }
    auto end() const { return items.end(); }

    std::pair<PackingInput,std::map<int,int>> get_equiv_input();

};
