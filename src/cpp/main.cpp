#include <bits/stdc++.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "io.h"
#include "lib2/util.h"
#include "lib/util/com.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

//#include "lib2/optimal_packing.h"
#include "lib2/heuristic_packing.h"
//#include "lib2/repacking.h"

namespace py = pybind11;

using namespace std;

PackingInput input2cgal(
    vector<pii>& _container,
    vector<tuple<int,int,vector<pii>>>& _items
) {
    auto container = Polygon_set(points2polygon(_container));
    ItemsContainer items;
    int idx = 0;
    foe(item, _items) {
        auto& [v, q, ps] = item;
        items.add_item(v, q, points2polygon(ps), idx++, Vector(0,0));
    }
    return {container, items};
}

PackingOutput output2cgal(
    PackingInput& input,
    vector<pair<int,pii>>& pyoutput
) {
    PackingOutput output (input);
    foe(e, pyoutput) {
        int idx = e.first;
        pii _offset = e.second;
        auto ref = input.items[idx].get_reference_point();
        Point offset (ref.x() + FT(_offset.first), ref.y() + FT(_offset.second));
        auto new_item = input.items[idx].move_ref_point(offset);
        output.add_item(Item {new_item.value, 1ll, new_item.pol, idx, Vector(0,0)});
    }
    return output;
}

vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> cgal2output(
    PackingOutput output
) {
    vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> res;
    foe(item, output) {
        res.eb(
            item.idx,
            vector2pstr(output.get_translation(item)),
            polygon2points(item.pol)
        );
    }
    return res;
}

#define OUT_TYPE PackingOutput
#define IN_TYPE PackingInput

// OUT_TYPE dummy_algorithm(IN_TYPE);

OUT_TYPE optimal_algorithm(IN_TYPE input) {
    cout << "[c++] RUNNING OPTIMAL ALGORITHM" << endl;
    //return OptimalPackingFast().run(input);
}

OUT_TYPE heuristic_algorithm(IN_TYPE input) {
    cout << "[c++] RUNNING HEURISTIC ALGORITHM" << endl;
    // return HeuristicPackingFast().run(input);
    // return HeuristicPackingNOMIP().run(input);
    // return HeuristicPackingGrid().run(input);
    // return HeuristicPackingRecursive().run(input);
    return HeuristicPackingMultiple().run(input,100,0);
}

OUT_TYPE heuristic_repacking_algorithm(IN_TYPE input, OUT_TYPE output) {
    cout << "[c++] RUNNING HEURISTIC REPACKING ALGORITHM" << endl;
    //return HeuristicRepacking().run(input, output);
}

vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> main_algorithm(
    vector<pii> _container,
    vector<tuple<int,int,vector<pii>>> _items
) {
    srand(42);

    cout << std::fixed;
    cerr << std::fixed;

    PackingInput input = input2cgal(_container, _items);
    cout << "[c++] input information: " << endl;
    cout << "      number of items: " << sz(input.items) << endl;
    cout << "      number of vertices on container: " << get_number_of_vertices(input.container) << endl;
    PackingOutput res = heuristic_algorithm(input);
    { // extra validation
        cout << "[c++] Validating result" << endl;
        PackingOutput val (input);
        foe(item, res) val.add_item(item);
        val.validate_result();
        auto score = val.get_score();
        ASSERT(score == res.get_score(),"scores do not match");
        assert(is_integer(score));
        cout << "[c++] Score: " << score << " ~= " << score.to_double() << endl;
        cout << "[c++] Number of items in solutions: " << sz(val) << endl;
    }
    return cgal2output(res);
}

vector<tuple<int,pair<string,string>,vector<pair<string,string>>>> repacking_algorithm(
    vector<pii> _container,
    vector<tuple<int,int,vector<pii>>> _items,
    vector<pair<int,pii>>& _pyoutput
) {
    srand(42);

    cout << std::fixed;
    cerr << std::fixed;

    PackingInput input = input2cgal(_container, _items);
    PackingOutput initial = output2cgal(input, _pyoutput);
    cout << "[c++] input information: " << endl;
    cout << "      number of items: " << sz(input.items) << endl;
    cout << "      number of vertices on container: " << get_number_of_vertices(input.container) << endl;
    PackingOutput res = heuristic_repacking_algorithm(input, initial);
    { // extra validation
        cout << "[c++] Validating result" << endl;
        PackingOutput val (input);
        foe(item, res) val.add_item(item);
        val.validate_result();
        auto score = val.get_score();
        ASSERT(score == res.get_score(),"scores do not match");
        assert(is_integer(score));
        cout << "[c++] Score: " << score << " ~= " << score.to_double() << endl;
        cout << "[c++] Number of items in solutions: " << sz(val) << endl;
    }
    return cgal2output(res);
}

PYBIND11_MODULE(main, m) {
    m.doc() = "main_algorithm and repacking_algorithm";
    m.def("main_algorithm", &main_algorithm);
    m.def("repacking_algorithm", &repacking_algorithm);
}

