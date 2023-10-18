#include <bits/stdc++.h>

#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

using namespace std;

struct Item {
    int value;
    int quantity;
    Polygon pol;
};

class ItemsContainer {
private:
    vector<Item> items;
public:
    ItemsContainer(){}
    ItemsContainer(vector<tuple<int,int,Polygon>> _items) {
        foe(e, _items) {
            items.push_back(Item{
                get<0>(e),
                get<1>(e),
                get<2>(e)
            });
        }
    }
    void add_item(int v, int q, Polygon p) {
        items.push_back(Item{v,q,p});
    }
    ItemsContainer expand() {
        ItemsContainer res;
        foe(item, items) {
            rep(item.quantity) {
                res.add_item(item.value, 1, item.pol);
            }
        }
        return res;
    }
    int n() {
        return sz(items);
    }
};

