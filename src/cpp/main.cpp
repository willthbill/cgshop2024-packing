#include <bits/stdc++.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "lib2/utils.cpp"

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

vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> cgal2output(
    vector<pair<int,Polygon>>& tmp,
    vector<Vector>& translations
) {
    vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> res;
    for(int i = 0; i < tmp.size(); i++) {
        res.eb(i, vector2pstr(translations[i]), polygon2points(tmp[i].se));
    }
    return res;
}

#define OUT_TYPE vector<pair<int,Polygon>>
#define IN_TYPE Polygon,vector<tuple<int,int,Polygon>>

OUT_TYPE dummy_algorithm(IN_TYPE);
OUT_TYPE optimal_algorithm(IN_TYPE);

vector<Vector> validate_solution(
    const Polygon& container,
    const vector<tuple<int,int,Polygon>>& items,
    OUT_TYPE solution
) {
    vector<Vector> translations;
    foe(e, solution) {
        int i = e.fi;
        auto& pol = e.se;
        ASSERT(is_polygon_inside_polygon(pol, container), "translated item " << i << " is not inside container");
        auto& original_pol = get<2>(items[i]);
        ASSERT(sz(pol) == sz(original_pol), "translated item " << i << " has different size from original item");
        Vector translation = pol[0] - original_pol[0];
        fon(j, sz(pol)) {
            ASSERT(original_pol[j] + translation == pol[j], "translated points (" << j << "th) for polygon " << i << " does not match");
        }
        translations.push_back(translation);
    }
    return translations;
}

vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> main_algorithm(
    vector<pii>& _container,
    vector<tuple<int,int,vector<pii>>>& _items
) {
    auto [container, items] = input2cgal(_container, _items);
    vector<pair<int,Polygon>> res = dummy_algorithm(container, items);
    auto translations = validate_solution(container, items, res);
    return cgal2output(res, translations);
}

PYBIND11_MODULE(main, m) {
    m.doc() = "main_algorithm";
    m.def("main_algorithm", &main_algorithm);
}

