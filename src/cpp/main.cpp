#include <bits/stdc++.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "lib/util/com.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

namespace py = pybind11;

using namespace std;

pair<Polygon,vector<tuple<int,int,Polygon>>> input2cgal(
    vector<pii>& _container,
    vector<tuple<int,int,vector<pii>>>& _items
) {
    auto container = points2polygon(_container);
    vector<tuple<int,int,Polygon>> items;
    foe(item, _items) {
        auto& [v, q, ps] = item;
        items.push_back({v, q, points2polygon(ps)});
    }
    return {container, items};
}

vector<pair<int,vector<pair<string,string>>>> cgal2output(
    vector<pair<int,Polygon>>& tmp
) {
    vector<pair<int,vector<pair<string,string>>>> res;
    for(int i = 0; i < tmp.size(); i++) {
        res.eb(i, polygon2points(tmp[i].se));
    }
    return res;
}

#define OUT_TYPE vector<pair<int,Polygon>>
#define IN_TYPE Polygon,vector<tuple<int,int,Polygon>>

OUT_TYPE dummy_algorithm(IN_TYPE);

vector<pair<int,vector<pair<string,string>>>> main_algorithm(
    vector<pii>& _container,
    vector<tuple<int,int,vector<pii>>>& _items
) {
    auto [container, items] = input2cgal(_container, _items);
    vector<pair<int,Polygon>> res = dummy_algorithm(container, items);
    return cgal2output(res);
}

PYBIND11_MODULE(main, m) {
    m.doc() = "main_algorithm";
    m.def("main_algorithm", &main_algorithm);
}

