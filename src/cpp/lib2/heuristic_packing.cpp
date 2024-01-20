#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/minkowski_sum_2.h>

#include "lib2/simplification.h"
#include "lib2/convex_cover.cpp"
#include "lib2/optimal_packing.h"
#include "lib2/heuristic_packing.h"
#include "lib2/mip/gurobi.h"
#include "lib2/configuration_space.h"
#include "lib2/snap.h"
#include "lib2/mip/mip.h"
#include "lib2/util.h"
#include "lib/geometry/partition_constructor.h"
#include "lib/util/geometry_utils.h"
#include "lib/util/cgal.h"
#include "lib/util/common.h"
#include "lib/util/debug.h"

using namespace std;

PackingOutput HeuristicPackingNOMIP::run(PackingInput _input, bool print, int sort_type) {
    auto input = _input;

    if(print) cout << "[c++] Expanding items" << endl;
    input.items = input.items.expand();

    vector<int> sorted_idxs;
    if(sort_type == 0) {
        if(print) cout << "[c++] Sorting items by value/area" << endl;
        sorted_idxs = input.items.sort_by_value_over_area();
    } else if(sort_type == 1) {
        if(print) cout << "[c++] Sorting items by area" << endl;
        sorted_idxs = input.items.sort_by_area(); // TODO: does this even sort non-increasingly?
    }
    input.items = permute(input.items, sorted_idxs);
    /*foe(item, input.items) {
        debug(item.pol.area().to_double());
    }*/

    PackingOutput output (_input);
    auto add_item = [&](Item& item, Point p) {
        assert(is_integer(p.x()));
        assert(is_integer(p.y()));
        Item new_item = item.move_ref_point(p);
        // assert(is_completely_inside(input.container, Polygon_set(new_item.pol)));
        output.add_item(new_item);
    };
    Polygon_set existing;
    int number_of_included_items = 0;
    Polygon_set complement_of_container = get_complement(input.container);
    fon(i, sz(input.items)) {
        if(print) cout << "[c++] Iteration " << (i + 1) << " / " << sz(input.items) << endl;
        auto& item = input.items[i];

        if(print) cout << "[c++] Computing configuration space" << endl;
        Polygon_set disallowed_space = complement_of_container;
        disallowed_space.join(existing);
        auto config_space = ConfigurationSpace(
            disallowed_space,
            item.pol,
            item.get_reference_point()
        ).space;

        if(print) cout << "[c++] Finding lowest integral point" << endl;
        vector<Point> vertices;
        foe(pwh, to_polygon_vector(config_space)) {
            foe(p, pwh.outer_boundary()) vertices.push_back(p);
            foe(hole, pwh.holes()) foe(p, hole) vertices.push_back(p);
            // TODO: holes needed?
        }
        sort(vertices.begin(), vertices.end(), [](Point& a, Point& b) {
            if(a.y() == b.y()) return a.x() < b.x();
            return a.y() < b.y();
        });
        if(sz(vertices)) {
            foe(v, vertices) {
                foab(dy, -10, 5) foab(dx, -10, 5) { // TODO: probably don't need to check negative
                    Point p (floor_exact(v.x() + dx), floor_exact(v.y() + dy));
                    if(config_space.oriented_side(p) != CGAL::ON_NEGATIVE_SIDE) {
                        if(print) cout << "[c++] Found lowest integral point: " << p << endl;
                        add_item(item, p);
                        existing.join(item.move_ref_point(p).pol); // TODO: just insert instead of join?
                        // turn to integer polygon
                        // existing = SnapToGrid(existing).space;
                        number_of_included_items++;
                        goto next_item;
                    }
                }
            }
            assert(false);
        }
next_item:
        if(print && i % 15 == 0) cout << "[c++] Number of included items: " << number_of_included_items << " / " << (i + 1) << endl;
    }

    return output;
}

PackingOutput HeuristicPackingGrid::run(PackingInput _input) {
    auto input = _input;
    input.items = input.items.expand();
    vector<int> sorted_idxs = input.items.sort_by_value_over_area();
    input.items = permute(input.items, sorted_idxs);

    auto container = get_single_polygon(input.container);

    // Compute parameters
    auto get_average_area = [](ItemsContainer& items) -> FT {
        FT sum = 0;
        foe(item, items) sum += item.pol.area();
        return sum / FT(sz(items));
    };
    FT max_number_of_items_in_square = 750; // this time 2
    assert(max_number_of_items_in_square >= 5);
    // square_size * square_size / get_average_area(input.items) = max_number_of_items_in_square
    // =>
    // square_size = sqrt(max_number_of_items_in_square * get_average_area(input.items))
    FT square_size = sqrt((max_number_of_items_in_square * get_average_area(input.items)).to_double()) - 2;
    assert(square_size >= 10);

    // Get grid dimensions
    auto bbox = get_int_bounding_box(container);
    Point lowest = bbox[0];
    Point rightmost = bbox[2];
    Point highest= bbox[2];
    Point leftmost = bbox[0];
    Point start (leftmost.x() - FT(2), lowest.y() - FT(2)); // -2 should not be necessary
    FT width = rightmost.x() - leftmost.x() + FT(4);
    FT height = highest.y() - lowest.y() + FT(4);
    FT number_of_steps_x = ceil_exact(width / square_size);
    FT number_of_steps_y = ceil_exact(height / square_size);
    // TODO: potentially recalculate square size
    assert(width >= 10);
    assert(height >= 10);
    assert(number_of_steps_x >= 1);
    assert(number_of_steps_y >= 1);

    cout << "[c++] Max number of items in a square: " << max_number_of_items_in_square.to_double() << endl;
    cout << "[c++] Square size: " << square_size.to_double() << endl;
    cout << "[c++] Width of container: " << width.to_double() << endl;
    cout << "[c++] Height of container: " << height.to_double() << endl;
    cout << "[c++] Number of steps in x direction: " << number_of_steps_x.to_double() << endl;
    cout << "[c++] Number of steps in y direction: " << number_of_steps_y.to_double() << endl;

    // Generate containers
    vector<Polygon_set> containers;
    fon(_i, number_of_steps_x) {
        fon(_j, number_of_steps_y) {
            FT i = _i;
            FT j = _j;
            Polygon square;
            {
                square.push_back(Point(start.x() + i * square_size, start.y() + j * square_size));
                square.push_back(Point(start.x() + (i + FT(1)) * square_size, start.y() + j * square_size));
                square.push_back(Point(start.x() + (i + FT(1)) * square_size, start.y() + (j + FT(1)) * square_size));
                square.push_back(Point(start.x() + i * square_size, start.y() + (j + FT(1)) * square_size));
            }
            Polygon_set allowed_space (container);
            allowed_space.intersection(Polygon_set(square));
            if(allowed_space.is_empty()) continue;
            get_single_polygon(allowed_space); // for assertions
            containers.push_back(allowed_space);
        }
    }
    int number_of_containers = sz(containers);
    assert(number_of_containers <= number_of_steps_x * number_of_steps_y);
    cout << "[c++] Number of boxes: " << number_of_containers << endl;

    /*// Divide items into containers
    vector<ItemsContainer> items_containers (number_of_containers);
    vector<vector<int>> items_indices (number_of_containers);
    fon(i, sz(input.items)) {
        auto& item = input.items[i];
        assert(item.quantity == 1);
        auto& item_container = items_containers[i % number_of_containers];
        Item new_item {item.value, 1, item.pol, sz(item_container), Vector(0,0)};
        item_container.add_item(new_item);
        items_indices[i % number_of_containers].push_back(i);
    }
    
    // Solve each container
    PackingOutput output (_input);
    fon(i, number_of_containers) {
        while(sz(items_containers[i]) > max_number_of_items_in_square * 2) items_containers[i].pop_item();
        assert(sz(items_containers[i]) <= max_number_of_items_in_square * 2);
        PackingInput container_input {containers[i], items_containers[i]};
        PackingOutput toutput = HeuristicPackingNOMIP().run(container_input);
        foe(item, toutput.items) {
            assert(item.quantity == 1);
            Item new_item {item.value, 1, item.pol, input.items[items_indices[i][item.idx]].idx, Vector(0,0)};
            output.add_item(new_item);
        }
    }*/

    // Solve each container
    PackingOutput output (_input);
    ordered_set_rev<pair<FT, int>> available_items;
    fon(i, sz(input.items)) {
        available_items.insert({input.items[i].value / input.items[i].pol.area(), i});
    }
    fon(i, number_of_containers) {
        int start_number_of_items = sz(available_items);
        ItemsContainer items;
        set<pair<FT,int>> all_taken;
        vector<int> item_indices;
        {
            int left = sz(available_items);
            int skip = number_of_containers - i;
            int idx = 0;
            set<pair<FT,int>> taken;
            while(sz(items) < max_number_of_items_in_square * 2 && sz(available_items) > 0) {
                auto p = *available_items.find_by_order(idx);
                int item_idx = p.second;
                auto& item = input.items[item_idx];
                Item new_item {item.value, 1, item.pol, sz(items), Vector(0,0)};
                item_indices.push_back(item_idx);
                items.add_item(new_item);
                taken.insert(p);
                all_taken.insert(p);
                idx += skip;
                if(idx >= sz(available_items)) {
                    idx = 0;
                    foe(e, taken) available_items.erase(e);
                    taken = {};
                }
            }
            foe(e, taken) available_items.erase(e);
        }
        assert(sz(items) <= max_number_of_items_in_square * 2);
        debug(sz(items));
        PackingInput container_input {containers[i], items};
        PackingOutput toutput = HeuristicPackingNOMIP().run(container_input);
        set<pair<FT,int>> real_taken;
        foe(item, toutput.items) {
            assert(item.quantity == 1);
            Item new_item {item.value, 1, item.pol, input.items[item_indices[item.idx]].idx, Vector(0,0)};
            output.add_item(new_item);
            real_taken.insert({item.value / item.pol.area(), item_indices[item.idx]});
        }
        foe(e, all_taken) {
            if(real_taken.count(e)) continue;
            available_items.insert(e);
        }
        int end_number_of_items = sz(available_items);
        assert(start_number_of_items - end_number_of_items == sz(toutput.items));
    }

    return output;
}


///////////// Heuristic Packing helpers ///////////////////
vector<Polygon_set> HeuristicPackingHelpers::overlay_grid(Polygon_set& container, FT square_size, bool random_offset, bool overlap) {
    FT overlap_factor = FT(0.2);
    if(overlap) square_size *= FT(1) - overlap_factor;
    Point start;
    {
        auto bbox = get_int_bounding_box(container);
        start = Point(bbox[0].x() - FT(2), bbox[0].y() - FT(2)); // -2 should not be necessary
        if(random_offset) {
            int md = ((int)(square_size * 0.8).to_double());
            start = Point(
                start.x() - rand() % md,
                start.y() - rand() % md
            );
        }
    }
    FT width = get_width(container) + FT(4);
    FT height = get_height(container) + FT(4);
    FT number_of_steps_x = ceil_exact(width / square_size) + 2;
    FT number_of_steps_y = ceil_exact(height / square_size) + 2;
    vector<Polygon_set> containers;
    fon(_i, number_of_steps_x) {
        fon(_j, number_of_steps_y) {
            FT i = _i;
            FT j = _j;
            FT sub = overlap ? overlap_factor * square_size : 0;
            Polygon square;
            {
                square.push_back(Point(start.x() + i * square_size - sub, start.y() + j * square_size - sub));
                square.push_back(Point(start.x() + (i + FT(1)) * square_size, start.y() + j * square_size - sub));
                square.push_back(Point(start.x() + (i + FT(1)) * square_size, start.y() + (j + FT(1)) * square_size));
                square.push_back(Point(start.x() + i * square_size - sub, start.y() + (j + FT(1)) * square_size));
            }
            Polygon_set allowed_space (container);
            allowed_space.intersection(square);
            if(allowed_space.is_empty()) continue;
            //assert(to_polygon_vector(allowed_space).size() == 1);
            //assert(to_polygon_vector(allowed_space)[0].number_of_holes() == 0);
            //containers.push_back(to_polygon_vector(allowed_space)[0].outer_boundary());
            containers.push_back(allowed_space);
        }
    }
    return containers;
}

const string SAMPLING_METHOD = "median_0.2";
/*
best_mcc
best_total
median_0.2
*/

////// ADVANCED ITEMS CONTAINER ///////
FT AdvancedItemsContainer::sorting_metric(int idx) {
    return items[idx].value / item_areas[idx];
}
AdvancedItemsContainer::AdvancedItemsContainer(ItemsContainer& _items) {
    items = _items;
    foe(item, items) assert(item.quantity == 1);
    fon(i, sz(items)) assert(items[i].idx == i);
    foe(item, items) item_areas.push_back(item.pol.area());
    smallest_item_area = *min_element(item_areas.begin(), item_areas.end());
    largest_item_area = *max_element(item_areas.begin(), item_areas.end());
    avg_area = 0;
    {
        vector<FT> item_areas_copy = item_areas;
        sort(item_areas_copy.begin(), item_areas_copy.end());
        median_item_area = item_areas_copy[(int)(sz(item_areas_copy) * 0.2)];
    }
    if(SAMPLING_METHOD == "median_0.2") {
        buckets.push_back({{0,median_item_area}, ordered_set_rev<pair<FT,int>>()});
        buckets.push_back({{median_item_area, FT(1e100)}, ordered_set_rev<pair<FT,int>>()});
        fon(i, sz(items)) {
            if(item_areas[i] > median_item_area) {
                buckets[1].se.insert({sorting_metric(i), i});
            } else {
                buckets[0].se.insert({sorting_metric(i), i});
            }
        }
    }

    auto tbuckets = buckets;
    if(SAMPLING_METHOD == "best_mcc" || SAMPLING_METHOD == "best_total") {
        {
            double f = 1.2; // TODO: test different
            double a = f;
            double pa = 0;
            while(pa <= 1e30) {
                buckets.push_back({{FT(pa - 0.01), FT(a + 0.01)}, ordered_set_rev<pair<FT,int>>()});
                pa = a;
                a *= f;
            }
        }
    }

    total_area_packed = accumulate(item_areas.begin(), item_areas.end(), FT(0));
    fon(i, sz(items)) {
        add_item(i);
    }

    ASSERT(total_area_packed == FT(0),"");

    if(SAMPLING_METHOD == "best_mcc" || SAMPLING_METHOD == "best_total") {
        // Just to remove unused buckets
        foe(bucket, buckets) {
            if(sz(bucket.se)) {
                tbuckets.push_back(bucket);
            }
        }
        buckets = tbuckets;
    }
}
int AdvancedItemsContainer::size() {
    return sz(available_items);
}
/*int AdvancedItemsContainer::bucket_size(FT area) {
    return sz(buckets[get_bucket_for_area_container(area)].se);
}*/
int AdvancedItemsContainer::get_bucket_for_area_container(FT area) {
    int bidx = 0;
    fon(i, sz(buckets)) {
        auto& a = buckets[i].fi.fi;
        auto& b = buckets[i].fi.se;
        /*if(a <= area && area <= b) {
            bidx = i;
            break;
        } else */if(b <= area) {
            bidx = i;
        }
    }
    return bidx;
}
vector<int> AdvancedItemsContainer::get_buckets_for_area_item(FT item_area) {
    if(SAMPLING_METHOD == "median_0.2") {
        if(item_area > median_item_area) return {1};
        else return {0};
    }
    if(SAMPLING_METHOD == "best_mcc" || SAMPLING_METHOD == "best_total") {
        FT min_area_container = item_area * FT(1);
        FT max_area_container = FT(1e100);
        /*if(item_area / largest_item_area > 0.5) {
            max_area_container = FT(1e100);
        } else {
            max_area_container = max(
                item_area / smallest_item_area * largest_item_area,
                item_area * 20
            );
        }*/
        int min_bidx = get_bucket_for_area_container(min_area_container);
        int max_bidx = get_bucket_for_area_container(max_area_container);
        vector<int> res;
        for(int i = min_bidx; i <= max_bidx; i++) {
            res.push_back(i);
        }
        return res;
    }
}
void AdvancedItemsContainer::add_item(int idx) {
    avg_area = (avg_area * FT(size()) + item_areas[idx]) / FT(size() + 1);
    pair<FT,int> e = {sorting_metric(idx), idx};
    if(available_items.find(e) != available_items.end()) {
        cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ITEM ALREADY IN AVAILABLE ITEMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    }
    available_items.insert(e);

    total_area_packed -= item_areas[idx];
    foe(bidx, get_buckets_for_area_item(item_areas[idx])) {
        auto& bucket = buckets[bidx].se;
        bucket.insert(e);
    }
}
void AdvancedItemsContainer::erase_item(int idx) {
    if(size() == 1) avg_area = 0;
    else avg_area = (avg_area * FT(size()) - item_areas[idx]) / FT(size() - 1);
    pair<FT,int> e = {sorting_metric(idx), idx};
    available_items.erase(e);

    total_area_packed += item_areas[idx];
    foe(bidx, get_buckets_for_area_item(item_areas[idx])) {
        auto& bucket = buckets[bidx].se;
        if(bucket.find(e) != bucket.end()) {
            bucket.erase(e);
        }
    }
}
pair<ItemsContainer,vector<int>> AdvancedItemsContainer::extract_items_median_sampling(
    int k,
    int depth,
    FT original_container_area,
    FT area
) {
    vector<int> bidxs;
    if(depth == 0) {
        bidxs = {1};
    } else {
        bidxs = {0,1};
    }
    int orig_k = k;
    FT total_area_left = original_container_area - total_area_packed;
    ItemsContainer res;
    vector<int> indices;
    int skip = max(1, (int)((total_area_left / area).to_double() / 1.5)); // TODO: should take into account that bucket does not have all items, but it is probably fine
    foe(bidx, bidxs) {
        vector<int> taken;
        int idx = 0;
        while(k && sz(buckets[bidx].se)) {
            auto [value_over_area, item_idx] = *buckets[bidx].se.find_by_order(idx);
            taken.push_back(item_idx);

            auto& item = items[item_idx];
            indices.push_back(item_idx);
            assert(item.quantity == 1);
            Item new_item {item.value, item.quantity, item.pol, sz(res), Vector(0,0)};
            res.add_item(new_item);
            k--;

            idx += skip;
            if(idx >= sz(buckets[bidx].se)) {
                idx = 0;
                foe(e, taken) erase_item(e);
                taken = {};
            }
            if(sz(buckets[bidx].se)) {
                idx %= sz(buckets[bidx].se);
            }
        }
        foe(e, taken) erase_item(e);
    }
    // debug("Sample ratio: ", (double)sz(res) / (double)orig_k);
    return {res, indices};
}

pair<ItemsContainer,vector<int>> AdvancedItemsContainer::extract_items_bucket_sampling(
    int k,
    FT area,
    FT original_container_area
) {
    // TODO: put in buckets relative to the size of the largest item and the smallest item
        // save largest area item. then the relation beetween the current item and the largest is the dynamic factor, F
        // so F * item_area == largest_area
        // the smallest 20% should be available when container area is less than 10 * largest_area
        // item_area * max_factor == 10 * largest_area
        // max_factor == 10 * largest_area / item_area = 10 * F
        // min_factor = 1.5

        // Smallest item should be available when container area is less than 10 * largest_area
        // smallest_area * max_factor == 10 * largest_area
        // => max_factor = 10 * largest_area / smallest_area = SOME_CONSTANT
        // make sure to take into account that these constants may have wried values. use min/max trick
        // Probably better to take average area of the smallest 10% of items or something
            // but what about super duper small items then? maybe placed in repacking?
    int orig_k = k;
    FT total_area_left = original_container_area - total_area_packed;
    ItemsContainer res;
    vector<int> indices;
    int skip = max(1, (int)((total_area_left / area).to_double() / 1.5)); // TODO: should take into account that bucket does not have all items, but it is probably fine
    int bidx = get_bucket_for_area_container(area);
    while(k && bidx >= 0) {
        vector<int> taken;
        int idx = 0;
        while(k && sz(buckets[bidx].se)) {
            auto [value_over_area, item_idx] = *buckets[bidx].se.find_by_order(idx);
            taken.push_back(item_idx);

            auto& item = items[item_idx];
            indices.push_back(item_idx);
            assert(item.quantity == 1);
            Item new_item {item.value, item.quantity, item.pol, sz(res), Vector(0,0)};
            res.add_item(new_item);
            k--;

            idx += skip;
            if(idx >= sz(buckets[bidx].se)) {
                idx = 0;
                foe(e, taken) erase_item(e);
                taken = {};
            }
            if(sz(buckets[bidx].se)) idx %= sz(buckets[bidx].se);
        }
        foe(e, taken) erase_item(e);
        bidx--;
    }
    // debug("Sample ratio: ", (double)sz(res) / (double)orig_k);
    return {res, indices};
}
pair<ItemsContainer,vector<int>> AdvancedItemsContainer::extract_items_random_area(int k, FT area_up, FT area_lb) {
    ItemsContainer res;
    vector<int> indices;
    int idx = 0;
    rep(5 * k) {
        if(sz(res) >= k) break;
        idx++;
        idx %= sz(available_items);
        //int idx = rand() % sz(available_items);
        auto [value_over_area, item_idx] = *available_items.find_by_order(idx);
        if(item_areas[item_idx] > area_up || item_areas[item_idx] < area_lb) continue;
        erase_item(item_idx);
        auto& item = items[item_idx];
        indices.push_back(item_idx);
        assert(item.quantity == 1);
        Item new_item {item.value, item.quantity, item.pol, sz(res), Vector(0,0)};
        res.add_item(new_item);
    }
    // debug("Sample ratio: ", (double)sz(res) / (double)k);
    return {res, indices};
}
pair<ItemsContainer,vector<int>> AdvancedItemsContainer::extract_items_random(int k) { // TODO: do better than random
    ItemsContainer res;
    vector<int> indices;
    // TODO: possibly try to sample with area below, but stop after a certain number of iterations if k is not reached
    rep(k) {
        int idx = rand() % sz(available_items);
        auto [value_over_area, item_idx] = *available_items.find_by_order(idx);
        erase_item(item_idx);
        auto& item = items[item_idx];
        indices.push_back(item_idx);
        assert(item.quantity == 1);
        Item new_item {item.value, item.quantity, item.pol, sz(res), Vector(0,0)};
        res.add_item(new_item);
    }
    return {res, indices};
}
///////////////////////

////// PARAMS ///////
const int MAX_ITEMS_IN_PACKING = 10000;
///////////////////////


////// HEURISTIC PACKING RECURSIVE ///////
PackingOutput HeuristicPackingRecursive::run(PackingInput _input) {
    auto input = _input;
    input.items = input.items.expand();
    vector<int> item_indices;
    fon(i, sz(input.items)) {
        item_indices.push_back(input.items[i].idx);
    }
    fon(i, sz(input.items)) input.items[i].idx = i;
    AdvancedItemsContainer items (input.items);
    PackingOutput toutput (input);
    Polygon_set packed = get_complement(input.container);
    cout << "[c++] Recursive algorithm starting" << endl;
    solve(input.container, items, toutput, packed, area(input.container), 0);
    cout << "[c++] Recursive algorithm finished" << endl;
    PackingOutput output (_input);
    foe(item, toutput.items) {
        Item new_item {item.value, 1, item.pol, item_indices[item.idx], Vector(0,0)};
        output.add_item(new_item);
    }
    return output;
}


Polygon_with_holes expand(Polygon& pol, FT scale) {
    Polygon square;
    FT factor = 0.5;
    {
        square.push_back(Point(-factor * scale,-scale * factor));
        square.push_back(Point(factor * scale,-scale * factor));
        square.push_back(Point(factor * scale,scale * factor));
        square.push_back(Point(-factor * scale,scale * factor));
    }
    auto sum = CGAL::minkowski_sum_2(pol, square);
    return sum;
}

FT get_area_of_largest_convex_cover_piece(Polygon_set& container) {
    auto pwhs = to_polygon_vector(container);
    FT res = 0;
    foe(pwh, pwhs) {
        auto cover = ConvexCover::get_convex_cover(pwh);
        foe(pol, cover) {
            res = max(res, pol.area());
        }
    }
    return res;
}

void HeuristicPackingRecursive::solve(
    Polygon_set& container, // guaranteed that it is free
    AdvancedItemsContainer& items, // the items that are not yet packed
    PackingOutput& output, // the current packing
    Polygon_set& packed, // the current packed area
    FT original_container_area,
    int depth
) {
    // TODO: we could set a cap on the depth of the recursion (ex. 10)
    // TODO: there is a lot of copying of polygons happening (ex. items)

    // If not items left, return. This should probably never happen
    if(sz(items) == 0) {
        // cout << "[c++] No items left" << endl;
        return;
    }
    cout << "[c++] Recursive solving at depth " << depth << endl;
    //debug(area(container).to_double());
    //debug(items.avg_area.to_double());
    // TODO: area might be small because of psets, but completely high (very high)
    // TODO: only consider items that are small enough to fit in the container (use buckets)
    if(sz(items) > MAX_ITEMS_IN_PACKING && area(container) / items.avg_area > FT(MAX_ITEMS_IN_PACKING) / FT(1.9)) { // SPACE FOR TOO MANY ITEMS

        // Split container into squares
        FT square_size = sqrt((FT(MAX_ITEMS_IN_PACKING) / FT(2.1) * items.avg_area).to_double()) - 2;
        assert(square_size >= 10);
        auto squares = HeuristicPackingHelpers().overlay_grid(
            container,
            square_size,
            depth != 0, // choose random offset when depth is not 0. should happen rarely
            true
        );
        cout << "[c++] Splitting container at depth " << depth << " into " << sz(squares) << " squares" << endl;

        // Solve each square recursively
        int sz_before = sz(items);
        /*foe(square, squares) {
            debug(area(square).to_double() / area(container).to_double());
        }*/
        foe(square, squares) {
            auto sub_packed = packed; sub_packed.intersection(square);
            auto allowed = sub_packed;
            allowed = get_complement(allowed);
            allowed.intersection(square);
            solve(allowed, items, output, sub_packed, original_container_area, depth); // not +1 on depth
            packed.join(sub_packed);
        }
        int sz_after = sz(items);
        ASSERT(sz_after <= sz_before,"size not smaller");
        if(sz_before == sz_after) {
            return;
        }
    } else {
        //cout << "[c++] Packing container directly" << endl;
        // TODO: we could samples just 2 * container.area() / items.avg_area items
            // YES do that
        // TODO: only consider items that fit when computing avg_area i guess, when sampling is updated to only include items with small enough area

        // Pack container directly
        FT fits = area(container) / items.avg_area;
        int to_sample = max(
            min(
                min(sz(items), MAX_ITEMS_IN_PACKING),
                depth == 0 ? ((int)(1e9)) : ((int)((2 * fits).to_double()) + 2) // TODO: big optimization but also a compromise on quality
            ),
            20
        );
        ItemsContainer sampled_items; vector<int> indices;
        if(SAMPLING_METHOD == "best_mcc") {
            auto p = items.extract_items_bucket_sampling(
                to_sample,
                get_area_of_largest_convex_cover_piece(container),
                original_container_area
            );
            sampled_items = p.fi;
            indices = p.se;
        }
        if(SAMPLING_METHOD == "best_total") {
            auto p = items.extract_items_bucket_sampling(
                to_sample,
                area(container),
                original_container_area
            );
            sampled_items = p.fi;
            indices = p.se;
        }
        if(SAMPLING_METHOD == "median_0.2") {
            auto p = items.extract_items_median_sampling(
                to_sample,
                depth,
                original_container_area,
                area(container)
            );
            sampled_items = p.fi;
            indices = p.se;
        }
        /*if(sz(sampled_items)) {
            debug(sz(sampled_items), sampled_items.get_average_area().to_double(), depth);
        } else {
            debug("no items", depth);
        }*/
        /*foe(item, sampled_items) {
            ASSERT(item.pol.area() <= area(container),"");
        }*/
        /*auto [sampled_items, indices] = items.extract_items_random_area(
            to_sample,
            area(container) * 0.9,
            depth >= 1 ? 0 : min(items.avg_area / 5, fits / MAX_ITEMS_IN_PACKING * items.avg_area / 5)
        ); // , container_area*/
        PackingInput tinput {container, sampled_items};
        PackingOutput toutput = HeuristicPackingNOMIP().run(tinput, false);

        // Deal with the result
        set<int> unused_indices;
        foe(idx, indices) {
            unused_indices.insert(idx);
        }
        if(sz(toutput.items)) {
            FT prev_score = output.get_score();

            // Remove items that were packed
            Polygon_set subpacked;
            foe(item, toutput.items) {
                subpacked.join(expand(item.pol, 3)); // TODO: add 1.5 square around item.pol
                Item new_item {item.value, 1, item.pol, indices[item.idx], Vector(0,0)};
                output.add_item(new_item);
                unused_indices.erase(indices[item.idx]);
            }
            packed.join(subpacked);

            FT new_score = output.get_score();
            if(prev_score > 0) cout << "[c++] Score improvement: " << new_score.to_double() / prev_score.to_double() * 100 - 100 << "%" << " at depth " << depth << endl;
            else cout << "[c++] First score " << new_score.to_double() << " at depth " << depth << endl;
        }

        // Add back the items that were not packed
        foe(idx, unused_indices) {
            items.add_item(idx);
        }

        // Nothing was packed so we assume it is impossible to pack anything
        if(sz(toutput.items) == 0) {
            return;
        }
    }

    // Get empty space within the container
    vector<Polygon_set> sub_containers;
    Polygon_set empty_space = container;
    empty_space.intersection(packed);
    empty_space = get_complement(empty_space);
    empty_space.intersection(container);
    /*Polygon_set empty_space = container;
    empty_space.intersection(get_complement(packed));*/

    // Get subcontainers
    // sub_containers.push_back(empty_space); // TODO: this is probably better in some cases
        // TODO: If it is not going to be split into squares then this is better probably. but also slower.
    foe(pwh, to_polygon_vector(empty_space)) {
        sub_containers.push_back(Polygon_set(pwh));
    }

    // Solve subcontainers recursively
    cout << "[c++] Number of sub-containers " << sz(sub_containers) << " at depth " << depth << endl;
    foe(sub_container, sub_containers) {
        /*if(false && depth < 3) {
            auto sub_packed = packed; sub_packed.intersection(sub_container);
            solve(sub_container, items, output, sub_packed, depth + 1);
            packed.join(sub_packed);
        } else {*/
        solve(sub_container, items, output, packed, original_container_area, depth + 1);
        //}
    }
}
///////////////////////

/*PLAN
1. Reuse items usin OST
2. Recursive algorithm
- recursive algorithm
- splitting is done using grid. randomly shifted (fixed seed)
    - or the grid could be moved predictively in each recursive call, but with the guarantee that it is the same shift as the parent
    - thus splitting is based on area
- after solving a hole we do (if it is not too big), we resolve it but sorted in order of area (but keep the solution in case it is worsened with sorting by area). this creates holes. we repeat a certain number of times
- right now we just give a hole access to all items and they pick a fair distribution of them less than their area
    - no first just keep them sorted by value/area
3. repacking
    - simply repack a space. just remove the items in that space and run the recursive algorithm with all items (the removed and the additional not placed)
        - or potentially repack and then fill holes with items that are not in the space (extra items)
4. Improvements to discuss
    - I think (by intutiion) that MIP on a pset (more polygons) is slower than on a pwh
    - Is it a good idea to pack into a polygon set (set of holes) at once?
    - Different constant than 2 for max param?
    - Keep items sorted by area and pick a fair distribution less than.
    - Ensure no items can be placed.
    - Consider better splitting (ex. random lines, split across the longest dimension)
        - Only split when complexity is too high, not just area
    - Solve by moving towards a common point (to make holes go in the same direction)
    - Consider more advanced forms of repacking (ex. moving all holes to a common place)
    - Using MIP (ex. for repacking)
        - MIP improvements
            - Warmstart on existing packing
            - Dynamic parameters
                - higher mip gap in the beginning
            = bounding box trick
            - use separator lines to simplify iteminitem constraints. Supportvectormachine?
            - gurobi parameters?
            - try to use the fast integer approximation algorithm where we also add 2,3 or something like that in the correct direction
            - allow some pieces to move around and others to be part of the big piece. for example most recent pieces can move
            - there are some unnecessary variables/constraints:
                - in iteminitem and inside container constraints we don't need to condition it on in_use_binaries, since we have the extra constraint that the sum of the helper binaries should be at least one.??
                - in inside pset basically the same thing???
    - Sample more items for smaller holes since there can never be a lot of items placed inside
    - Value / convex hull?
    - Use long vertical rectangles to move items down in the packing?
    - Using density
    - Reduce number of holes by running a unit square (or bigger/smaller) around items using minkowski sum
    - Fractional knapsack as estimate for something?
    - Make the holes overlap (ex. square grid overlap)
    - Make repacking spaces overlap
    - Somehow remove already packed items and replace with not packed items
    - Repack around holes
    - Try this?: https://github.com/tamasmeszaros/libnest2d
    - Use different parameters for different instances
    - Try running with the median method as well
    - Try median method with other than 0.2

    Repacking
        - Move to (0,0)
        - multiple objectives gurobi
        - gurobi heuristics (see chatgpt)
            - try disable NoRed
        - gurobi parameters
        - dynamisk sætte tidsgrænser for gurobi og andet
        - try just minimizing sum of y or x (with fixed x/y) for faster
        - kør fase 1 og 2 igen og igen
        - what if before MIP we are not able to place all items. crash?
        - use density
        - Repack around holes
        - use try statement so that a small error does not ruin all progress

    - Value / convex hull?
    - Improve the small instances a lot more
    - Use different parameters for different instances
    - Try median method with other than 0.2
    - Try this?: https://github.com/tamasmeszaros/libnest2d
*/

// TODO: speed up by only duplicating polygons as many times as the total area is less than area of container
// TODO: maybe see if there are common factors (or something like that) on each side so we can make numbers smaller
// TODO: optimize by drawing triangle around instead of square???
// TODO: actually calculate bounds (inf, biginf, ...)

/*
MEETING
- sampling strategies
- overlapping squares (problem with holes in grid)
- optimization (problem is minkowski sum)
*/