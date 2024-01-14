#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

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
            foe(pol, pwh.outer_boundary()) vertices.push_back(pol);
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
                        existing.join(item.move_ref_point(p).pol);
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

class HeuristicPackingHelpers {
public:

    vector<Polygon_set> overlay_grid(Polygon_set& container, FT square_size, bool random_offset=true) {
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
                Polygon square;
                {
                    square.push_back(Point(start.x() + i * square_size, start.y() + j * square_size));
                    square.push_back(Point(start.x() + (i + FT(1)) * square_size, start.y() + j * square_size));
                    square.push_back(Point(start.x() + (i + FT(1)) * square_size, start.y() + (j + FT(1)) * square_size));
                    square.push_back(Point(start.x() + i * square_size, start.y() + (j + FT(1)) * square_size));
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
};

////// ADVANCED ITEMS CONTAINER ///////
FT AdvancedItemsContainer::sorting_metric(int idx) {
    return items[idx].value / items[idx].pol.area();
}
AdvancedItemsContainer::AdvancedItemsContainer(ItemsContainer& _items) {
    items = _items;
    foe(item, items) assert(item.quantity == 1);
    fon(i, sz(items)) assert(items[i].idx == i);
    avg_area = 0;
    fon(i, sz(items)) {
        add_item(i);
    }
}
int AdvancedItemsContainer::size() {
    return sz(available_items);
}
void AdvancedItemsContainer::add_item(int idx) {
    avg_area = (avg_area * FT(size()) + items[idx].pol.area()) / FT(size() + 1);
    pair<FT,int> e = {sorting_metric(idx), idx};
    if(available_items.find(e) != available_items.end()) {
        cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ITEM ALREADY IN AVAILABLE ITEMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    }
    available_items.insert(e);
}
void AdvancedItemsContainer::erase_item(int idx) {
    if(size() == 1) avg_area = 0;
    else avg_area = (avg_area * FT(size()) - items[idx].pol.area()) / FT(size() - 1);
    available_items.erase({sorting_metric(idx), idx});
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
const int MAX_ITEMS_IN_PACKING = 200;
///////////////////////


////// HEURISTIC PACKING RECURSIVE ///////
PackingOutput HeuristicPackingRecursive::run(PackingInput _input) {
    auto input = _input;
    input.items = input.items.expand();
    fon(i, sz(input.items)) input.items[i].idx = i;
    vector<int> item_indices;
    fon(i, sz(input.items)) {
        item_indices.push_back(input.items[i].idx);
    }
    AdvancedItemsContainer items (input.items);
    PackingOutput toutput (input);
    cout << "[c++] Recursive algorithm starting" << endl;
    solve(input.container, items, toutput, 0);
    cout << "[c++] Recursive algorithm finished" << endl;
    PackingOutput output (_input);
    foe(item, toutput.items) {
        Item new_item {item.value, 1, item.pol, item_indices[item.idx], Vector(0,0)};
        output.add_item(new_item);
    }
    return output;
}

void HeuristicPackingRecursive::solve(
    Polygon_set& container,
    AdvancedItemsContainer& items,
    PackingOutput& output,
    int depth
) {
    // cout << "[c++] Recursive solving at depth " << depth << endl;
    if(sz(items) == 0) {
        // cout << "[c++] No items left" << endl;
        return;
    }
    vector<Polygon_set> sub_containers;
    // TODO: area might be small because of psets, but completely high (very high)
    if(sz(items) > MAX_ITEMS_IN_PACKING && area(container) / items.avg_area > FT(MAX_ITEMS_IN_PACKING) / FT(1.9)) { // SPACE FOR TOO MANY ITEMS
        cout << "[c++] Splitting container at depth " << depth << endl;
        FT square_size = sqrt((FT(MAX_ITEMS_IN_PACKING) / FT(2.1) * items.avg_area).to_double()) - 2;
        assert(square_size >= 10);
        sub_containers = HeuristicPackingHelpers().overlay_grid(container, square_size, depth != 0); // choose random offset when depth is not 0
    } else {
        //cout << "[c++] Packing container directly" << endl;
        // TODO: we could samples just 2 * container.area() / items.avg_area items
            // YES do that
        // TODO: only consider items that fit when computing avg_area i guess, when sampling is updated to only include items with small enough area
        auto [sampled_items, indices] = items.extract_items_random(min(sz(items), MAX_ITEMS_IN_PACKING)); // , container.area()
        PackingInput tinput {container, sampled_items};
        debug("yo1");
        PackingOutput toutput = HeuristicPackingNOMIP().run(tinput, false);
        debug("yo2");
        set<int> unused_indices;
        if(sz(toutput.items)) {
            FT prev_score = output.get_score();
            foe(idx, indices) unused_indices.insert(idx);
            Polygon_set packed = get_complement(container);
            foe(item, toutput.items) {
                packed.join(item.pol); // TODO: add 1.5 square around item.pol
                Item new_item {item.value, 1, item.pol, indices[item.idx], Vector(0,0)};
                output.add_item(new_item);
                unused_indices.erase(indices[item.idx]);
            }
            Polygon_set empty_space = get_complement(packed);
            // sub_containers.push_back(empty_space); // TODO: this is probably better in some cases
            foe(pwh, to_polygon_vector(empty_space)) {
                sub_containers.push_back(Polygon_set(pwh));
            }
            FT new_score = output.get_score();
            if(prev_score > 0) cout << "[c++] Score improvement: " << new_score.to_double() / prev_score.to_double() * 100 - 100 << "%" << " at depth " << depth << endl;
            else cout << "[c++] First score " << new_score.to_double() << " at depth " << depth << endl;
        } else {
            foe(idx, indices) {
                unused_indices.insert(idx);
            }
        }
        foe(idx, unused_indices) {
            items.add_item(idx);
        }
    }
    // cout << "[c++] Number of sub-containers: " << sz(sub_containers) << endl;
    foe(sub_container, sub_containers) {
        solve(sub_container, items, output, depth + 1);
    }
    // TODO: we are not filling holes after packing squares!!!!!
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
*/

// TODO: speed up by only duplicating polygons as many times as the total area is less than area of container
// TODO: maybe see if there are common factors (or something like that) on each side so we can make numbers smaller
// TODO: optimize by drawing triangle around instead of square???
// TODO: actually calculate bounds (inf, biginf, ...)