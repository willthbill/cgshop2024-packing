#pragma once

#include <bits/stdc++.h>

#include "lib/util/cgal.h"
#include "lib/util/common.h"

struct Item {

    FT value;
    ll quantity;
    Polygon pol;
    int idx;
    Vector ref_scaling_translation; // how do undo scaling

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
    void add_item(FT v, ll q, Polygon p, int idx, Vector ref_scaling_translation); 
    ItemsContainer expand(); 
    int size(); 
    Item& operator[](std::size_t i); 

    auto begin() { return items.begin(); }
    auto end() { return items.end(); }
    auto begin() const { return items.begin(); }
    auto end() const { return items.end(); }
};

struct PackingInput {
    Polygon container;
    ItemsContainer items;
};

class PackingOutput {

public:

    PackingInput input;
    ItemsContainer items;
    std::map<int,int> item_count;

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

};
