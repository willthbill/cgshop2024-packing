#include <bits/stdc++.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "util.cpp"
#include "lib/util/com.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"

namespace py = pybind11;

using namespace std;

PackingInput input2cgal(
    vector<pii>& _container,
    vector<tuple<int,int,vector<pii>>>& _items
) {
    auto container = points2polygon(_container);
    ItemsContainer items;
    int idx = 0;
    foe(item, _items) {
        auto& [v, q, ps] = item;
        items.add_item(v, q, points2polygon(ps), idx++);
    }
    return {container, items};
}

vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> cgal2output(
    PackingOutput output
) {
    vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> res;
    foe(item, output) {
        res.eb(item.idx, vector2pstr(output.get_translation(item)), polygon2points(item.pol));
    }
    return res;
}

#define OUT_TYPE PackingOutput
#define IN_TYPE PackingInput

OUT_TYPE dummy_algorithm(IN_TYPE);
OUT_TYPE optimal_algorithm(IN_TYPE);

vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> main_algorithm(
    vector<pii>& _container,
    vector<tuple<int,int,vector<pii>>>& _items
) {
    PackingInput input = input2cgal(_container, _items);
    PackingOutput res = optimal_algorithm(input);
    { // extra validation
        PackingOutput val (input);
        foe(item, res) val.add_item(item);
        val.validate_result();
        auto score = val.get_score();
        ASSERT(score == res.get_score(),"scores do not match");
        cout << "[c++] score: " << score << endl;
    }
    return cgal2output(res);
}

PYBIND11_MODULE(main, m) {
    m.doc() = "main_algorithm";
    m.def("main_algorithm", &main_algorithm);
}

