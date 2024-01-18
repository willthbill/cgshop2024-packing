#include <bits/stdc++.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/repacking.h"
#include "lib2/simplification.h"
#include "lib2/convex_cover.cpp"
#include "lib2/optimal_packing.h"
#include "lib2/heuristic_packing.h"
#include "lib2/mip/gurobi.h"
#include "lib2/configuration_space.h"
#include "lib2/snap.h"
#include "lib2/mip/mip.h"
#include "lib2/util.h"
#include "lib/geometry/tree.h"
#include "lib/geometry/partition_constructor.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

class DynamicQueryUtil {
    const int MAX_DELETED = 10000;
    const int BUFFER_SIZE = 1000;
    const int QUERY_UTILS_SIZE = 25;

    set<Point> buffer;
    vector<SegmentQueryUtil> query_utils;
    int deleted = 0;

    map<Point,int> all_points; // the only truth
    void rebuild_query_utils() {
        delete_datastructures();
        vector<Point> points;
        foe(p, all_points) points.push_back(p.fi);
        SegmentQueryUtil ut; ut.init_pointset(points);
        query_utils = {ut};
    }
    void rebuild_buffer() {
        vector<Point> points (buffer.begin(), buffer.end());
        buffer = {};
        SegmentQueryUtil ut; ut.init_pointset(points);
        query_utils.push_back(ut);
    }
    void rebuild() {
        if(sz(buffer) > BUFFER_SIZE) {
            rebuild_buffer();
        }
        if(deleted > MAX_DELETED || sz(query_utils) > QUERY_UTILS_SIZE) {
            rebuild_query_utils();
        }
    }
public:
    DynamicQueryUtil(vector<Point> points) {
        foe(p, points) {
            all_points[p]++;
        }
    }
    vector<Point> query(Point bottom_left, Point top_right) {
        vector<Point> res;
        foe(p, buffer) {
            if(all_points.count(p) && all_points[p] > 0) {
                res.push_back(p);
            }
        }
        foe(util, query_utils) {
            auto query_res = util.query_pointset(bottom_left, top_right);
            foe(p, query_res) {
                if(all_points.count(p) && all_points[p] > 0) {
                    res.push_back(p);
                }
            }
        }
        return res;
    }
    void remove_point(Point p) {
        if(!all_points.count(p)) return;
        if(all_points[p] <= 0) {
            all_points.erase(p);
            return;
        }
        all_points[p]--;
        if(all_points[p] <= 0) {
            all_points.erase(p);
        }
        deleted++;
        rebuild();
    }
    void add_point(Point p) {
        buffer.insert(p);
        all_points[p]++;
        rebuild();
    }
    void delete_datastructures() {
        foe(util, query_utils) {
            util.delete_datastructures();
        }
    }
};

const int MAX_ITEMS_IN_PACKING = 50;

PackingOutput HeuristicRepacking::run(PackingInput _input, PackingOutput _initial) {
    // Expand input
    auto input = _input;
    input.items = input.items.expand();
    vector<int> item_indices;
    map<int, vector<int>> idxs;
    fon(i, sz(input.items)) {
        idxs[input.items[i].idx].push_back(i);
        item_indices.push_back(input.items[i].idx);
    }
    fon(i, sz(input.items)) input.items[i].idx = i;

    // Expand output
    auto initial = _initial;
    foe(item, initial.items) {
        item.idx = idxs[item.idx].back();
        idxs[item.idx].pop_back();
    }
    initial.item_count = {};
    foe(item, initial.items) {
        initial.item_count[item.idx]++;
    }

    // Repack
    auto toutput = _run(input, initial);

    // Construct output
    PackingOutput output (_input);
    foe(item, toutput.items) {
        Item new_item {item.value, 1, item.pol, item_indices[item.idx], Vector(0,0)};
        output.add_item(new_item);
    }
    return output;
}

template<typename T1, typename T2>
struct IgnoreSecondComparator {
    bool operator()(pair<T1,T2>& a, pair<T1,T2>& b) {
        return a.first < b.first;
    }
};

PackingOutput HeuristicRepacking::_run(PackingInput input, PackingOutput initial) {
    FT average_item_area = input.items.get_average_area();

    // Polygon_set non_repacked = input.container;
    // AdvancedItemsContainer available_items; // items not yet packed
    // TODO: initialize items

    // Setup initial solution
    map<int, Polygon> solution;
    foe(item, initial.items) {
        solution[item.idx] = item.pol;
    }

    // Create query datastructures
    vector<Point> points;
    map<Point, set<int>> point_to_items;
    foe(item, initial.items) {
        foe(p, item.pol) {
            point_to_items[p].insert(item.idx);
            points.push_back(p);
        }
    }
    DynamicQueryUtil query_util (points);

    priority_queue<
        pair<FT,Polygon_set>,
        vector<pair<FT,Polygon_set>>,
        IgnoreSecondComparator<FT,Polygon_set>
    > repacking_spaces;
    rep(10) {
    // while(true) {

        // Generate repacking spaces
        if(sz(repacking_spaces) == 0) {
            cout << "[c++] Generating spaces to repack" << endl;
            FT square_size = sqrt((FT(MAX_ITEMS_IN_PACKING) / FT(2.1) * average_item_area).to_double()) - 2;
            auto squares = HeuristicPackingHelpers().overlay_grid(
                input.container,
                square_size,
                false,
                true
            );
            foe(square, squares) {
                repacking_spaces.push(make_pair((int)1, square)); // TODO: something smarter than 1
            }
        }

        // Extract highest priority space to repack
        auto [priority, space] = repacking_spaces.top(); repacking_spaces.pop();

        // Find items to repack (that are completely inside the space)
        vector<int> items_to_repack;
        {
            auto bbox = get_int_bounding_box(space);
            vector<Point> points = query_util.query(bbox[0], bbox[2]);
            auto comp_space = get_complement(space);
            set<int> checked;
            Polygon_set packed_space;
            foe(p, points) {
                foe(idx, point_to_items[p]) {
                    if(checked.count(idx)) {
                        continue;
                    }
                    auto& pol = solution[idx];
                    if(comp_space.oriented_side(pol) != CGAL::ON_POSITIVE_SIDE) {
                        items_to_repack.push_back(idx);
                    } else {
                        packed_space.join(pol);
                    }
                    checked.insert(idx);
                }
            }
            space.intersection(get_complement(packed_space));
        }

        // Remove items to repack from the query util and solution
        foe(idx, items_to_repack) {
            foe(p, solution[idx]) {
                point_to_items[p].erase(idx);
                query_util.remove_point(p);
            }
            solution.erase(idx);
        }

        // Construct input for packing algorithm
        ItemsContainer items;
        fon(i, sz(items_to_repack)) {
            int idx = items_to_repack[i];
            auto& item = input.items[idx];
            Item new_item {item.value, 1, item.pol, i, Vector(0,0)};
            items.add_item(new_item);
        }
        PackingInput tinput {space, items};

        // Pack using NOMIP sorted by area
        PackingOutput toutput = HeuristicPackingNOMIP().run(tinput, false, 1);

        // Add items to solution
        foe(item, toutput.items) {
            int idx = items_to_repack[item.idx];
            solution[idx] = item.pol;
            foe(p, item.pol) {
                point_to_items[p].insert(idx);
                query_util.add_point(p);
            }
        }
    }

    // Construct output
    PackingOutput output (input);
    foe(p, solution) {
        auto& idx = p.fi;
        auto& pol = p.se;
        Item new_item {input.items[idx].value, 1, pol, idx, Vector(0,0)};
        output.add_item(new_item);
    }
    return output;
}