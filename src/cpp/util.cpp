#include <bits/stdc++.h>

#include "lib2/util.cpp"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

using namespace std;

typedef CGAL::Aff_transformation_2<K> Transformation;

struct Item {

    ll value;
    ll quantity;
    Polygon pol;
    int idx;

    Item move_first_point(Point p) {
        Transformation translate(
            CGAL::TRANSLATION,
            Vector(p.x() - pol[0].x(), p.y() - pol[0].y())
        );
        Polygon tpol = transform(translate, pol);
        return Item {value, quantity, tpol, idx};
    }

    Point get_reference_point() {
        return get_lowest_point(pol);
    }

};

class ItemsContainer {
private:
    vector<Item> items;
public:
    ItemsContainer(){}
    ItemsContainer(vector<tuple<int,int,Polygon>> _items) {
        int idx = 0;
        foe(e, _items) {
            items.push_back(Item{
                get<0>(e),
                get<1>(e),
                get<2>(e),
                idx++
            });
        }
    }
    void add_item(Item item) {
        assert(item.idx == sz(items));
        items.push_back(item);
    }
    void add_item(ll v, ll q, Polygon p, int idx) {
        add_item(Item{v,q,p,idx});
    }
    ItemsContainer expand() {
        ItemsContainer res;
        foe(item, items) {
            rep(item.quantity) {
                res.add_item(item.value, 1ll, item.pol, item.idx);
            }
        }
        return res;
    }
    int size() {
        return sz(items);
    }
    Item& operator[](std::size_t i) {
        return items[i];
    }

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
private:

    PackingInput input;
    ItemsContainer items;
    map<int,int> item_count;

    void validate_item(Item item) {
        ASSERT(item_count[item.idx] < input.items[item.idx].quantity,"");
        ASSERT(item.quantity == 1ll, "in a result all quanities must be 1");
        ASSERT(input.items[item.idx].value == item.value,"");
        ASSERT(input.items[item.idx].idx == item.idx,"");
        ASSERT(is_polygon_inside_polygon(item.pol, input.container), "translated item " << item.idx << " is not inside container");
        get_translation(item);
    }

public:

    PackingOutput(PackingInput _input) {
        input = _input;
    }

    void add_item(Item item) {
        validate_item(item);
        item_count[item.idx]++;
        items.add_item(item);
    }

    // also validates item.pol
    Vector get_translation(Item item) {
        auto& original_pol = input.items[item.idx].pol;
        ASSERT(sz(item.pol) == sz(original_pol), "translated item " << item.idx << " has different size from original item");
        Vector translation = item.pol[0] - original_pol[0];
        fon(j, sz(item.pol)) {
            ASSERT(original_pol[j] + translation == item.pol[j], "translated points (" << j << "th) for polygon " << item.idx << " does not match");
        }
        return translation;
    }

    /*vector<Vector> get_translations() {
        vector<Vector> translations;
        foe(item, items) {
            translations.push_back(get_translation(item));
        }
        return translations;
    }*/

    void validate_result() {
        // TODO: check no overlap
    }

    ll get_score() {
        ll sum = 0ll;
        foe(item, items) {
            sum += item.value;
        }
        return sum;
    }

    int size() {
        return sz(items);
    }

    auto begin() { return items.begin(); }
    auto end() { return items.end(); }
    auto begin() const { return items.begin(); }
    auto end() const { return items.end(); }

};
