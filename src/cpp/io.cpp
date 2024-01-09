#include <bits/stdc++.h>
#include <CGAL/Simple_cartesian.h>


#include "lib/geometry/intersection_predicates.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

#include "lib2/util.h"
#include "lib/util/debug.h"

#include "io.h"

using namespace std;

typedef CGAL::Aff_transformation_2<K> Transformation;

// TODO: duplicate implement in optimal_packing.cpp
void assert_is_integer_polygon(Polygon& pol) {
    foe(p, pol) {
        assert(is_integer(p.x()));
        assert(is_integer(p.y()));
    }
}

Item Item::move_ref_point(Point p) {
    auto ref = get_reference_point();
    Transformation translate(
        CGAL::TRANSLATION,
        Vector(p.x() - ref.x(), p.y() - ref.y())
    );
    Polygon tpol = transform(translate, pol);
    // foe(e, tpol) cout << e.x() << " " << e.y() << endl;
    return Item {value, quantity, tpol, idx};
}

Point Item::get_reference_point() {
    return get_lowest_point(pol);
}

ItemsContainer::ItemsContainer(){}

ItemsContainer::ItemsContainer(vector<tuple<int,int,Polygon>> _items) {
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
void ItemsContainer::add_item(Item item) {
    items.push_back(item);
}
void ItemsContainer::add_item(FT v, ll q, Polygon p, int idx, Vector ref_scaling_translation) {
    add_item(Item{v,q,p,idx,ref_scaling_translation});
}
ItemsContainer ItemsContainer::expand() {
    ItemsContainer res;
    foe(item, items) {
        rep(item.quantity) {
            res.add_item(item.value, 1, item.pol, item.idx, item.ref_scaling_translation);
        }
    }
    return res;
}
int ItemsContainer::size() {
    return sz(items);
}
Item& ItemsContainer::operator[](std::size_t i) {
    return items[i];
}

void PackingOutput::validate_item(Item item) {
    assert_is_integer_polygon(item.pol);
    ASSERT(item_count[item.idx] < input.items[item.idx].quantity, to_string(item_count[item.idx]) + " " + to_string(input.items[item.idx].quantity));
    ASSERT(item.quantity == 1ll, "in a result all quanities must be 1");
    assert(input.items[item.idx].value == item.value);
    ASSERT(input.items[item.idx].idx == item.idx,"");
    // ASSERT(is_polygon_inside_polygon(item.pol, input.container), "translated item " << item.idx << " is not inside container");
    get_translation(item);
}

PackingOutput::PackingOutput(PackingInput _input) {
    input = _input;
}

void PackingOutput::add_item(Item item) {
    validate_item(item);
    item_count[item.idx]++;
    items.add_item(item);
}

// also validates item.pol
Vector PackingOutput::get_translation(Item item) {
    auto& original_pol = input.items[item.idx].pol;
    ASSERT(sz(item.pol) == sz(original_pol), "translated item " << item.idx << " has different size from original item");
    Vector translation = item.pol[0] - original_pol[0];
    // TODO: fix with less precision
    /*fon(j, sz(item.pol)) {
        ASSERT(original_pol[j] + translation == item.pol[j], "translated points (" << j << "th) for polygon " << item.idx << " does not match");
    }*/
    return translation;
}

bool is_completely_outside(Polygon a, Polygon b) {
    Polygon_set intersection; intersection.intersection(
        to_polygon_set(a),
        to_polygon_set(b)
    );
    auto arr = to_polygon_vector(intersection);
    FT res = 0;
    foe(p, arr) res += p.outer_boundary().area();
    return res == 0;
}

bool is_completely_inside(Polygon a, Polygon b) {
    Polygon_set intersection; intersection.intersection(
        to_polygon_set(a),
        to_polygon_set(b)
    );
    auto arr = to_polygon_vector(intersection);
    FT res = 0;
    foe(p, arr) res += p.outer_boundary().area();
    return res == b.area();
}

bool is_point_strictly_inside(Polygon poly, Point p) {
    switch (CGAL::bounded_side_2(poly.vertices_begin(), poly.vertices_end(), p)) {
        case CGAL::ON_BOUNDED_SIDE:
            return true;
        case CGAL::ON_BOUNDARY:
            return false;
        case CGAL::ON_UNBOUNDED_SIDE:
            return false;
    }
    assert(false);
}

void PackingOutput::validate_result() {
    /*fon(i, sz(items)) {
        debug("newpol");
        foe(p, items[i].pol) {
            debug(p);
        }
    }*/
    fon(i, min(sz(items), 50)) {
        auto& i1 = items[i];
        fon(j, i) {
            if(i == j) continue;
            auto& i2 = items[j];
            if(!is_completely_outside(i1.pol, i2.pol)) {
                cout << "!!!!! OVERLAP BETWEEN ITEMS IN SOLUTION !!!!!" << endl;
            }
            if(is_point_strictly_inside(i1.pol, i2.get_reference_point())) {
                cout << "!!!!! ITEM REFERENCE POINT IS INSIDE OTHER ITEM !!!!!" << endl;
            }
        }
        if(!is_completely_inside(input.container, i1.pol)) {
            cout << "!!!!! ITEM NOT IN CONTAINER !!!!!" << endl;
        }
    }
}

FT PackingOutput::get_score() {
    FT sum = 0;
    foe(item, items) {
        sum += item.value;
    }
    return sum;
}

int PackingOutput::size() {
    return sz(items);
}
