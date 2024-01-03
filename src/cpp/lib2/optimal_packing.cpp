#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/convex_cover.cpp"
#include "lib2/optimal_packing.h"
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

// TODO: speed up by only duplicating polygons as many times as the total area is less than area of container
// TODO: maybe see if there are common factors (or something like that) on each side so we can make numbers smaller
// TODO: optimize by drawing triangle around instead of square???
// TODO: actually calculate bounds (inf, biginf, ...)


vector<Polygon_with_holes> to_polygon_vector_ref(Polygon_set pset) {
    vector<Polygon_with_holes> pols;
    pset.polygons_with_holes (back_inserter(pols));
    return pols;
}

template<typename T>
T permute(T vec, const std::vector<int>& indices) {
    assert(sz(vec) == sz(indices));
    T permuted = vec;
    for(size_t i = 0; i < indices.size(); ++i) {
        permuted[i] = vec[indices[i]];
    }
    return permuted;
}

vector<Polygon> fix_repeated_points(Polygon pol) {
    vector<Point> points;
    foe(p, pol) points.push_back(p);
    map<Point, int> mp;
    int idx = -1;
    fon(i, sz(points)) {
        auto& p = points[i];
        if(mp.count(p)) {
            idx = i;
            break;
        }
        mp[p] = i;
    }
    if(idx == -1) return {pol};
    Polygon a, b;
    for(int i = 0; i < mp[points[idx]]; i++) a.push_back(points[i]);
    for(int i = mp[points[idx]]; i < idx; i++) b.push_back(points[i]);
    for(int i = idx; i < sz(points); i++) a.push_back(points[i]);
    vector<Polygon> res;
    foe(p, fix_repeated_points(a)) res.push_back(p);
    foe(p, fix_repeated_points(b)) res.push_back(p);
    return res;
}

bool is_completely_inside(Polygon_set a, Polygon_set b) {
    Polygon_set t; t.difference(b,a);
    return t.is_empty();
}

Polygon scale_polygon(Polygon pol, FT scale) {
    foe(e, pol) {
        e = {e.x() * scale, e.y() * scale};
    }
    return pol;
}

ItemsContainer scale_items(ItemsContainer items, FT scale) {
    foe(item, items) {
        item.pol = scale_polygon(item.pol, scale);
    }
    return items;
}


void assert_is_integer_polygon(Polygon& pol) {
    foe(p, pol) {
        assert(is_integer(p.x()));
        assert(is_integer(p.y()));
    }
}

Polygon get_bounding_box(Polygon& pol) {
    FT mnx = 1e18, mxx = -1e18, mny = 1e18, mxy = -1e18;
    foe(p, pol) mnx = min(mnx, p.x());
    foe(p, pol) mxx = max(mxx, p.x());
    foe(p, pol) mny = min(mny, p.y());
    foe(p, pol) mxy = max(mxy, p.y());
    mnx = floor_exact(mnx);
    mxx = ceil_exact(mxx);
    mny = floor_exact(mny);
    mxy = ceil_exact(mxy);
    Polygon res;
    res.push_back(Point(mnx, mny));
    res.push_back(Point(mxx, mny));
    res.push_back(Point(mxx, mxy));
    res.push_back(Point(mnx, mxy));
    return res;
}

FT find_min_distance(const Polygon& poly1, const Polygon& poly2) {
    FT minDist = CGAL::squared_distance(poly1[0], poly2[0]);
    for (auto v1 = poly1.vertices_begin(); v1 != poly1.vertices_end(); ++v1) {
        for (auto v2 = poly2.vertices_begin(); v2 != poly2.vertices_end(); ++v2) {
            Segment seg1(*v1, *(next(v1) == poly1.vertices_end() ? poly1.vertices_begin() : next(v1)));
            Segment seg2(*v2, *(next(v2) == poly2.vertices_end() ? poly2.vertices_begin() : next(v2)));
            FT dist = CGAL::squared_distance(seg1, seg2);
            if (dist < minDist) {
                minDist = dist;
            }
        }
    }
    return std::sqrt(CGAL::to_double(minDist));
}


class MIPPackingHelpers {
private:
    Gurobi_MIP* problem;
    GurobiCallback* callbackobj;
public:

    static FT scale; // = FT(1) / FT(15000); // IMPORTANT: it is important to set it like 1/int. the inverse must be integer
    static FT inf; // = 250000000 * scale.to_double(); // upper bound on coordinates
    static FT biginf; // = 3e8; // BIG M, must be greater than inf * scale * inf * scale
    static FT max_partition_size; // = 100000000;

    MIPPackingHelpers(Gurobi_MIP* _problem, GurobiCallback* _callbackobj) {
        problem = _problem;
        callbackobj = _callbackobj;
    }
    void set_global_packing_parameter(PackingInput& input) {
        FT mx = -1e9;
        foe(item, input.items) {
            foe(p, item.pol) {
                mx = max(mx, p.x() < 0 ? -p.x() : p.x());
                mx = max(mx, p.y() < 0 ? -p.y() : p.y());
            }
        }
        foe(p, input.container) {
            mx = max(mx, p.x() < 0 ? -p.x() : p.x());
            mx = max(mx, p.y() < 0 ? -p.y() : p.y());
        }
        cout << "[c++] Parameters:" << endl;
        cout << " - maximum coordinate value: " << mx << endl;

        max_partition_size = 100'000'000;
        assert(max_partition_size > 0);

        biginf = 10'000'000; // BIG M, must be greater than inf * scale * inf * scale
        assert(biginf > 0);

        // mx * mx * 1.1 * 1.1 * 1.3 * 1.3 + 10000 < biginf
        // -->
        // mx * mx < (biginf - 10000) / (1.1 * 1.1 * 1.3 * 1.3)
        // -->
        // mx < sqrt((biginf - 10000) / (1.1 * 1.1 * 1.3 * 1.3))
        // -->
        // mx * scale < sqrt((biginf - 10000) / (1.1 * 1.1 * 1.3 * 1.3))
        // -->
        // scale < sqrt((biginf - 10000) / (1.1 * 1.1 * 1.3 * 1.3)) / mx

        scale = ceil_exact(mx / (FT(sqrt(biginf.to_double()))) / 0.4 / 0.935);
        assert(scale >= -0.00001);
        if(scale <= 1) scale = 1;
        scale = FT(1)/FT(scale);

        assert(scale > 0.00000001);
        assert(scale <= 1);
        if(mx * mx < biginf * 0.1) assert(scale == 1);

        mx = ceil((mx * scale.to_double()).to_double());
        inf = ceil((100 + mx * 2.5).to_double()); // upper bound on coordinates
        assert(inf > 0);
        assert(mx > 0);
        assert(mx * 2.5 < inf);
        assert(inf * inf * 1.1 + 10000 < biginf);

        assert(is_integer(inf));
        assert(is_integer(mx));
        assert(is_integer(biginf));
        assert(is_integer(max_partition_size));

        cout << " - max_partition_size: " << max_partition_size << " ~=" << max_partition_size.to_double() << endl;
        cout << " - biginf: " << biginf << " ~= " << biginf.to_double() << endl;
        cout << " - inf: " << inf << " ~= " << inf.to_double() << endl;
        cout << " - scale: " << scale << " ~= " << scale.to_double() << endl;
        cout << " - mx / inf = " << (mx / inf).to_double() << ", mx * mx / biginf = " << (mx * mx / biginf).to_double() << ", inf * inf / biginf = " << (inf * inf / biginf).to_double() << endl;
        cout << endl;
    }
    Polygon get_big_square() {
        Polygon square; square.pb(Point(-inf,-inf));square.pb(Point(inf,-inf));square.pb(Point(inf,inf));square.pb(Point(-inf,inf));
        return square;
    }
    MIPVariable get_ref_coord_variable_x(int idx) {
        return {"int", "polygon_ref_" + to_string(idx) +  "_x"};
    }
    MIPVariable get_ref_coord_variable_y(int idx) {
        return {"int", "polygon_ref_" + to_string(idx) +  "_y"};
    }
    void add_variable(MIPVariable variable) {
        string type = variable.first;
        if(type == "con") {
            problem->add_continuous_variable(variable.second);
        } else if(type == "bin") {
            problem->add_binary_variable(variable.second);
        } else if (type == "int") {
            problem->add_integer_variable(variable.second);
        } else {
            assert(false);
        }
    }
    vector<pair<MIPVariable, MIPVariable>> get_and_add_xy_ref_variables(int number_of_items) {
        vector<pair<MIPVariable, MIPVariable>> xys;
        fon(i, number_of_items) {
            auto x = get_ref_coord_variable_x(i);
            auto y = get_ref_coord_variable_y(i);
            if(problem) add_variable(x);
            if(problem) add_variable(y);
            xys.push_back({x,y});
        }
        return xys;
    }
    void add_constraint(
        MIPConstraint constraint,
        bool lazy=false
    ) {
        if(lazy) {
            callbackobj->cb_add_lazy_constraint(get<0>(constraint), get<2>(constraint), get<1>(constraint));
        } else {
            if(get<1>(constraint) == "eq") {
                problem->add_eq_constraint(get<0>(constraint), get<2>(constraint));
            } else if(get<1>(constraint) == "geq") {
                problem->add_geq_constraint(get<0>(constraint), get<2>(constraint));
            } else if(get<1>(constraint) == "leq") {
                problem->add_leq_constraint(get<0>(constraint), get<2>(constraint));
            } else {
                assert(false);
            }
        }
    }
    void set_max_objective_with_in_use_binaries(ItemsContainer& items, vector<MIPVariable>& in_use_binaries) {
        assert(sz(items) == sz(in_use_binaries));
        vector<pair<string,FT>> obj_terms;
        fon(i, sz(items)) {
            obj_terms.push_back({in_use_binaries[i].se, items[i].value});
        }
        problem->set_max_objective(obj_terms);
    }
    vector<MIPVariable> get_and_add_in_use_binaries(int number_of_items) {
        vector<MIPVariable> in_use_binaries;
        fon(i, number_of_items) {
            in_use_binaries.push_back(
                make_pair(
                    string("bin"),
                    string("is_polygon_" + to_string(i) + "_in_use")
                )
            );
            if(problem) add_variable(in_use_binaries[i]);
        }
        return in_use_binaries;
    }
    vector<int> sort_by_value_over_area(ItemsContainer items) {
        vector<int> idxs (sz(items)); iota(idxs.begin(), idxs.end(), 0);
        sort(idxs.begin(), idxs.end(), [&](int a, int b) {
            return FT(items[a].value) / items[a].pol.area() > FT(items[b].value) / items[b].pol.area();
        });
        /*sort(items.begin(), items.end(), [](Item& a, Item& b) {
            return FT(a.value) / a.pol.area() > FT(b.value) / b.pol.area();
        });*/
        return idxs;
    }
    pair<vector<MIPVariable>, vector<MIPConstraint>> get_iteminitem_constraints(
        pair<MIPVariable, MIPVariable>& p1,
        pair<MIPVariable, MIPVariable>& p2,
        Item& item1,
        Item& item2, // item2 should not be in item1
        string binary_prefix,
        vector<MIPVariable> enabled,
        bool use_bounding_box=false
    ) {
        // Compute partition
        vector<Polygon> partition;
        {
            Polygon_set config_space_int;
            if(use_bounding_box) {
                // Compute configuration space
                Polygon_set disallowed (get_bounding_box(item1.pol));
                auto bbox2 = get_bounding_box(item2.pol);
                config_space_int = ConfigurationSpace(
                    disallowed,
                    bbox2,
                    item2.get_reference_point()
                ).space;
            } else {
                // Compute configuration space
                Polygon_set disallowed (item1.pol);
                assert(item1.pol.is_simple());
                assert(item2.pol.is_simple());
                auto config_space = ConfigurationSpace(
                    disallowed,
                    item2.pol,
                    item2.get_reference_point()
                ).space;
                // Snap it to a grid
                config_space_int = get_complement(
                    SnapToGrid(get_complement(config_space)).space
                ); // TODO: taking complement twice
            }

            // Computer intersection of large square and completement of configuration space
            Polygon square = get_big_square();
            Polygon_set ps (square);
            ps.intersection(config_space_int);

            // Get the single polygon with one hole (the complement of conf space)
            auto polygons = to_polygon_vector(ps);
            foe(polygon, polygons) {
                //assert(sz(polygons) == 1);
                //auto polygon = polygons[0];
                // assert(polygon.number_of_holes() == 1);
                assert_is_integer_polygon(polygon.outer_boundary());
                foe(hole, polygon.holes()) assert_is_integer_polygon(hole);

                // Compute convex cover (triangulation, partiton or something)
                /*PartitionConstructor pc (polygon);
                foe(tri, pc.get_constrained_delaunay_triangulation()) {
                    partition.push_back(tri);
                }*/
                auto cover = ConvexCover::get_convex_cover(polygon);
                foe(pol, cover) {
                    partition.push_back(pol);
                }

                foe(pol, partition) assert_is_integer_polygon(pol);
            }
        }

        Point ref = item1.get_reference_point();
        auto x1 = p1.fi.second;
        auto y1 = p1.se.second;
        auto x2 = p2.fi.second;
        auto y2 = p2.se.second;
        int idx = 0;
        vector<MIPVariable> variables;
        vector<MIPConstraint> constraints;

        // Add constraints
        foe(pol, partition) {

            ASSERT(pol.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");

            string b = binary_prefix + to_string(idx);
            idx += 1;
            variables.push_back({"bin", b});

            fon(k, sz(pol)) {
                Vector c1 = pol[k] - ref;
                Vector c2 = pol[(k+1) % sz(pol)] - ref;
                Vector v1 (c2.x() - c1.x(), c2.y() - c1.y());
                // v2 = (x2 - x1 - c1.x(), y2 - y1 - c1.y());
                // Cross product
                    // cross(v1,v2) = v1.x() * (y2 - y1 - c1.y()) - v1.y() * (x2 - x1 - c1.x())
                    // = v1.x() * y2 - v1.x() * y1 - v1.x() * c1.y() - v1.y() * x2 + v1.y() * x1 + v1.y() * c1.x() <= 0   TODO: <= is correct right?
                    // <=>
                    // v1.x() * y2 - v1.x() * y1 - v1.y() * x2 + v1.y() * x1 <= v1.x() * c1.y() - v1.y() * c1.x()
                MIPConstraint constraint = {
                    {
                        {y2, v1.x()},{y1,-v1.x()},{x2,-v1.y()},{x1,v1.y()},
                        {b, -biginf},
                    },
                    "geq",
                    v1.x() * c1.y() - v1.y() * c1.x() - ((long long) sz(enabled) + 1) * biginf // TODO: safe in terms of casting??
                };
                foe(v, enabled) get<0>(constraint).push_back(make_pair(v.se, -biginf));
                constraints.push_back(constraint);
            }
        }

        // At least one convex region must be satisfied
        {
            vector<pair<string,FT>> terms;
            foe(b, variables) terms.push_back({b.se,1});
            foe(v, enabled) terms.push_back(make_pair(v.se, -(max_partition_size + 100ll))); // TODO: do we need this? not really right?
            assert(sz(terms) < max_partition_size - 1000);
            constraints.push_back({
                terms,
                "geq",
                1ll - (max_partition_size + 100ll) * sz(enabled)
            }); // TODO: safe in terms of precision / automatic casting??
        }

        return {variables, constraints};
    }

    PackingInput scalesnap_input(PackingInput& input) {
        cout << "Original container area: " << input.container.area().to_double() << endl;
        Polygon scaled_container;
        {
            auto t = get_complement(Polygon_set(scale_polygon(input.container, FT(scale))));
            t.intersection(get_big_square());
            auto v = to_polygon_vector_ref(SnapToGrid(t).space);
            assert(sz(v) == 1);
            assert(v[0].number_of_holes() == 1);
            scaled_container = *v[0].holes_begin();
            scaled_container.reverse_orientation();
        }
        if(scale < 1) {
            // assert(is_completely_inside(Polygon_set(input.container), Polygon_set(scaled_container)));
        } else {
            //assert(is_completely_inside(Polygon_set(scaled_container), Polygon_set(input.container)));
        }
        cout << "Snapped container area: " << scaled_container.area().to_double() << endl;
        foe(p, scaled_container) {
            assert(is_integer(p.x()));
            assert(is_integer(p.y()));
            //assert(p.x() >= 0);
            //assert(p.y() >= 0);
        }
        PackingInput modified_input {
            scaled_container,
            scale_items(input.items, FT(scale))
        };
        foe(item, modified_input.items) {
            auto old = item.pol;
            auto old_ref = item.get_reference_point();
            foe(p, old) {
                assert(p.x() >= 0);
                assert(p.y() >= 0);
            }
            cout << "Original item area: " << item.pol.area().to_double() << endl;
            item.pol = SnapToGrid(Polygon_set(item.pol)).get_single_polygon();
            cout << "Snapped item area: " << item.pol.area().to_double() << endl;
            assert(is_completely_inside(Polygon_set(item.pol), Polygon_set(old)));
            Vector ref_translation = item.get_reference_point() - old_ref;
            assert(is_integer(item.get_reference_point().x()));
            assert(is_integer(item.get_reference_point().y()));
            item.ref_scaling_translation = ref_translation;
            foe(p, item.pol) {
                assert(is_integer(p.x()));
                assert(is_integer(p.y()));
            }
        }
        return modified_input;
    }

    vector<MIPConstraint> get_constraints_point_inside_convex_polygon(
        Polygon& pol,
        MIPVariable& var_x,
        MIPVariable& var_y,
        vector<MIPVariable>& binaries, // whether the constraints should be enabled
        Vector offset // the point (var_x, var_y) + offset
    ) {
        ASSERT(pol.orientation() == CGAL::COUNTERCLOCKWISE, "must have counterclockwise orientation");
        vector<MIPConstraint> res;
        fon(k, sz(pol)) {
            Point c1 = pol[k];
            Point c2 = pol[(k+1) % sz(pol)];
            auto o = offset;
            Vector v1 (c2.x() - c1.x(), c2.y() - c1.y());
            vector<pair<string, FT>> lhs = {{var_y.se,v1.x()},{var_x.se,-v1.y()}};
            foe(bin, binaries) {
                lhs.push_back({bin.se,-biginf});
            }
            res.push_back({
                lhs,
                "geq",
                -v1.x() * o.y() + v1.x() * c1.y() + v1.y() * o.x() - v1.y() * c1.x() - FT(sz(binaries)) * biginf // TODO: safe in terms of casting??
            });
        }
        return res;
    }
    void add_constraints_inside_polygon_set(
        Polygon_set& pset,
        ItemsContainer& items,
        vector<MIPVariable>& in_use_binaries,
        vector<pair<MIPVariable,MIPVariable>>& xy_ref_variables
    ) {
        assert(false); // TODO: the problem is that this says that the entire polygon should be inside one of the convex regions. but it should just say that the reference point is inside one region.
        auto cover = ConvexCover::get_convex_cover(pset);
        {
            FT area_pset = 0;
            FT area_cover = 0;
            foe(p, to_polygon_vector(pset)) {
                // area_pset += p.outer_boundary().area();
                assert(p.outer_boundary().is_simple());
                foe(pol, fix_repeated_points(p.outer_boundary())) {
                    area_pset += pol.area();
                }
                foe(h, p.holes()) {
                    assert(h.is_simple());
                    foe(pol, fix_repeated_points(h)) {
                        area_pset -= -pol.area();
                    }
                    // area_pset -= -h.area();
                }
            }
            foe(pol, cover) {
                assert(pol.area() > 0);
                area_cover += pol.area();
                assert(pol.is_simple());
                assert(pol.orientation() == CGAL::COUNTERCLOCKWISE);
                debug(pol.area());
            }
            debug(area_pset, area_cover);
            Polygon_set un;
            foe(pol, cover) un.join(pol);
            assert(is_completely_inside(un, pset));
            assert(is_completely_inside(pset, un));
            assert(area_pset <= area_cover);

            /*foe(pwh, to_polygon_vector(existing)) {
                foe(pol, fix_repeated_points(pwh.outer_boundary())) {
                    items.add_item(0, 1, pol, 0, Vector(0,0));
                }
            }*/
        }
        vector<vector<MIPVariable>> all_tmp_binaries (sz(items));
        fon(idx, sz(cover)) {
            assert_is_integer_polygon(cover[idx]);
            vector<vector<MIPVariable>> binaries;
            foe(b, in_use_binaries) binaries.push_back({b});
            fon(i, sz(items)) {
                MIPVariable var = {"int", "polygon_set_binary_" + to_string(i) + "_" + to_string(idx)}; // TODO: must be unique
                add_variable(var);
                binaries[i].push_back(var);
                all_tmp_binaries[i].push_back(var);
            }
            add_constraints_inside_convex_polygon(cover[idx], items, binaries, xy_ref_variables);
        }
        // At least one convex region used for every item
        fon(i, sz(items)) {
            vector<pair<string,FT>> terms;
            foe(b, all_tmp_binaries[i]) terms.push_back({b.se,1});
            terms.push_back(make_pair(in_use_binaries[i].se, -(max_partition_size + 100ll))); // TODO: do we need this? not really right?
            assert(sz(terms) < max_partition_size - 1000);
            add_constraint({
                terms,
                "geq",
                1ll - (max_partition_size + 100ll)
            });
        }
    }
    void add_constraints_inside_convex_polygon(
        Polygon& pol,
        ItemsContainer& items,
        vector<vector<MIPVariable>>& binaries,
        vector<pair<MIPVariable,MIPVariable>>& xy_ref_variables
    ) {
        fon(i, sz(items)) {
            auto& x = xy_ref_variables[i].fi;
            auto& y = xy_ref_variables[i].se;
            auto ref = items[i].get_reference_point();
            foe(p, items[i].pol) {
                foe(constraint, get_constraints_point_inside_convex_polygon(
                    pol, x, y, binaries[i], p - ref
                )) {
                    add_constraint(constraint);
                }
            }
        }
    }
    void add_constraints_inside_convex_polygon(
        Polygon& pol,
        ItemsContainer& items,
        vector<MIPVariable>& in_use_binaries,
        vector<pair<MIPVariable,MIPVariable>>& xy_ref_variables
    ) {
        vector<vector<MIPVariable>> binaries;
        foe(b, in_use_binaries) binaries.push_back({b});
        add_constraints_inside_convex_polygon(pol, items, binaries, xy_ref_variables);
    }
    void add_bbox_constraints(
        pair<MIPVariable,MIPVariable>& xy_1,
        pair<MIPVariable,MIPVariable>& xy_2,
        Item& item1,
        Item& item2,
        string uniqueness,
        MIPVariable& inuse1, MIPVariable& inuse2,
        bool add_vars=true, bool add_cons=true
    ) {
        auto [variables, constraints] = get_iteminitem_constraints(
            xy_1,
            xy_2,
            item1,
            item2,
            "binary_bbox_" + uniqueness + "_",
            vector<MIPVariable>{inuse1, inuse2},
            true
        );
        if(add_vars) foe(v, variables) add_variable(v);
        if(add_cons) foe(c, constraints) add_constraint(c);
    }

    void add_exact_constraints(
        pair<MIPVariable,MIPVariable>& xy_1,
        pair<MIPVariable,MIPVariable>& xy_2,
        Item& item1,
        Item& item2,
        string uniqueness,
        MIPVariable& inuse1, MIPVariable& inuse2,
        bool add_vars=true, bool add_cons=true
    ) {
        auto [variables, constraints] = get_iteminitem_constraints(
            xy_1,
            xy_2,
            item1,
            item2,
            "binary_exact_" + uniqueness + "_",
            vector<MIPVariable>{inuse1, inuse2}
        );
        if(add_vars) foe(v, variables) add_variable(v);
        if(add_cons) foe(c, constraints) add_constraint(c);
    };

    PackingOutput produce_output(
        PackingInput& input,
        ItemsContainer& items,
        map<string,FT>& solution,
        vector<MIPVariable>& in_use_binaries,
        vector<pair<MIPVariable,MIPVariable>>& xys
    ) {
        PackingOutput output (input);
        fon(i, sz(items)) {
            if(solution[in_use_binaries[i].se] > 0.5) {
                FT x = solution[xys[i].fi.se];
                FT y = solution[xys[i].se.se];
                assert(is_integer(x));
                assert(is_integer(y));
                Item new_item = items[i].move_ref_point(Point(x,y));
                // assert(is_completely_inside(Polygon_set(input.container), Polygon_set(new_item.pol)));
                output.add_item(new_item);
            }
        }
        return output;
    }

    map<string, FT> unscale_xy_coords(
        map<string, FT> solution,
        vector<pair<MIPVariable,MIPVariable>>& xys,
        ItemsContainer& items
    ) {
        FT factor = FT(1) / scale;
        assert(is_integer(factor));
        int idx = 0;
        foe(p, xys) {
            assert(is_integer(solution[p.fi.se]));
            assert(is_integer(solution[p.se.se]));
            auto ref_scaling_translation = items[idx].ref_scaling_translation;
            solution[p.fi.se] -= ref_scaling_translation.x();
            solution[p.se.se] -= ref_scaling_translation.y();
            solution[p.fi.se] *= factor;
            solution[p.se.se] *= factor;
            assert(is_integer(solution[p.fi.se]));
            assert(is_integer(solution[p.se.se]));
            idx++;
        }
        return solution;
    }

};

FT MIPPackingHelpers::scale = 1; 
FT MIPPackingHelpers::max_partition_size = 1; 
FT MIPPackingHelpers::biginf = 1; 
FT MIPPackingHelpers::inf = 1; 


void OptimalPackingCallback::callback() {
    try {
        if (where == GRB_CB_MIPSOL) {
            cout << "feasible solution" << endl;
            auto& items = cb_input.items;
            MIPPackingHelpers helper (&cb_problem, this);
            fon(i, sz(items)) {
                fon(j, i) {
                    if(cb_itempair_state.count({i,j})) continue;
                    FT dist = find_min_distance(items[i].pol, items[j].pol);
                    if(dist <= 5) {
                        cb_itempair_state.insert({i,j});
                        cb_itempair_state.insert({j,i});
                        helper.add_exact_constraints(
                            cb_xys[i], cb_xys[j], items[i], items[j],
                            to_string(i) + "_" + to_string(j),
                            cb_in_use_binaries[i], cb_in_use_binaries[j],
                            false, true
                        );
                    }
                }
            }
            // problem.status();
        }
    } catch (GRBException e) {
        std::cout << "[c++] Error number: " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "[c++] Error during callback" << std::endl;
    }
}

PackingOutput OptimalPackingCallback::run(PackingInput _input) {

    cout << "[c++] Optimal algorithm using gurobi callback optimization" << endl;
    cb_input = {_input.container, _input.items.expand()};

    cb_set_problem(&cb_problem);
    cb_problem.set_callback(this);
    MIPPackingHelpers helper (&cb_problem, this);

    cout << "[c++] Adding 'in-use' binaries" << endl;
    cb_in_use_binaries = helper.get_and_add_in_use_binaries(sz(cb_input.items));

    cout << "[c++] Setting objective" << endl;
    helper.set_max_objective_with_in_use_binaries(cb_input.items, cb_in_use_binaries);

    cout << "[c++] Adding reference x and y variables" << endl;
    cb_xys = helper.get_and_add_xy_ref_variables(sz(cb_input.items));

    cout << "[c++] Adding items inside container constraints" << endl;
    helper.add_constraints_inside_convex_polygon(cb_input.container, cb_input.items, cb_in_use_binaries, cb_xys);

    cout << "[c++] Adding items no overlap constraints using bounding boxes" << endl;
    fon(i, sz(cb_input.items)) fon(j, i) {
        helper.add_bbox_constraints(
            cb_xys[i], cb_xys[j], cb_input.items[i], cb_input.items[j],
            to_string(i) + "_" + to_string(j),
            cb_in_use_binaries[i], cb_in_use_binaries[j]
        );
    }

    cout << "[c++] Adding items no overlap variables" << endl;
    fon(i, sz(cb_input.items)) fon(j, i) {
        helper.add_exact_constraints(
            cb_xys[i], cb_xys[j], cb_input.items[i], cb_input.items[j],
            to_string(i) + "_" + to_string(j),
            cb_in_use_binaries[i], cb_in_use_binaries[j],
            true, false
        );
    }

    cout << "[c++] Computing solution using MIP" << endl;
    auto solution = cb_problem.solve();

    cout << "[c++] Moving items to found reference point solution coordinates" << endl;
    PackingOutput output = helper.produce_output(_input, cb_input.items, solution, cb_in_use_binaries, cb_xys);

    return output;
}


PackingOutput OptimalPackingSlow::run(PackingInput _input) {

    cout << "[c++] Optimal slow algorithm" << endl;

    cout << "[c++] Scaling input" << endl;
    auto input = MIPPackingHelpers(NULL, NULL).scalesnap_input(_input);
    assert(false); // USE REF TRANSLATIONS
    input.items = input.items.expand();

    Gurobi_MIP problem;
    MIPPackingHelpers helper (&problem, NULL);

    cout << "[c++] Adding 'in-use' binaries" << endl;
    auto in_use_binaries = helper.get_and_add_in_use_binaries(sz(input.items));

    cout << "[c++] Setting objective" << endl;
    helper.set_max_objective_with_in_use_binaries(input.items, in_use_binaries);

    cout << "[c++] Adding reference x and y variables" << endl;
    auto xys = helper.get_and_add_xy_ref_variables(sz(input.items));

    cout << "[c++] Adding items inside container constraints" << endl;
    helper.add_constraints_inside_convex_polygon(input.container, input.items, in_use_binaries, xys);

    cout << "[c++] Adding items no overlap variables and constraints" << endl;
    fon(i, sz(input.items)) fon(j, i) {
        helper.add_exact_constraints(
            xys[i], xys[j], input.items[i], input.items[j],
            to_string(i) + "_" + to_string(j),
            in_use_binaries[i], in_use_binaries[j]
        );
    }

    cout << "[c++] Computing solution using MIP" << endl;
    auto solution = problem.solve();

    cout << "[c++] Unscaling solution coords" << endl;
    solution = helper.unscale_xy_coords(solution, xys, input.items);

    cout << "[c++] Moving items to found reference point solution coordinates" << endl;
    auto _expanded = _input.items.expand();
    PackingOutput output = helper.produce_output(_input, _expanded, solution, in_use_binaries, xys);

    return output;
}

PackingOutput OptimalPackingFast::run(PackingInput _input) {

    cout << "[c++] Optimal fast algorithm" << endl;
    PackingInput input {
        _input.container,
        _input.items.expand()
    };

    sort(input.items.begin(), input.items.end(), [](Item& a, Item& b) {
        return a.value / a.pol.area() > b.value / b.pol.area();
    });

    cout << "[c++] Sorting:" << endl;
    foe(item, input.items) {
        cout << "    " << item.value << " " << item.pol.area() << " " << (item.value / item.pol.area()).to_double() << endl;
    }

    map<string,FT> solution;
    vector<MIPVariable> in_use_binaries;
    vector<pair<MIPVariable,MIPVariable>> xys;
    map<pair<int,int>,pair<vector<MIPVariable>, vector<MIPConstraint>>> iteminitem_constraints;
    fon(i, sz(input.items)) {
        auto& item = input.items[i];

        ItemsContainer subset;
        fon(j, i + 1) subset.add_item(input.items[j]);

        Gurobi_MIP problem;
        MIPPackingHelpers helper (&problem, NULL);

        cout << "[c++] Adding 'in-use' binaries" << endl;
        in_use_binaries = helper.get_and_add_in_use_binaries(sz(subset));

        cout << "[c++] Setting objective" << endl;
        helper.set_max_objective_with_in_use_binaries(subset, in_use_binaries);

        cout << "[c++] Adding reference x and y variables" << endl;
        xys = helper.get_and_add_xy_ref_variables(sz(subset));

        cout << "[c++] Adding items inside container constraints" << endl;
        helper.add_constraints_inside_convex_polygon(input.container, subset, in_use_binaries, xys);

        cout << "[c++] Adding items no overlap variables and constraints" << endl;
        fon(i, sz(subset)) fon(j, i) {
            if(!iteminitem_constraints.count({i,j})) {
                auto [variables, constraints] = helper.get_iteminitem_constraints(
                    xys[i],
                    xys[j],
                    input.items[i],
                    input.items[j],
                    "binary_exact_" + to_string(i) + "_" + to_string(j) + "_",
                    vector<MIPVariable>{in_use_binaries[i], in_use_binaries[j]}
                );
                iteminitem_constraints[{i,j}] = {variables, constraints};
                iteminitem_constraints[{j,i}] = {variables, constraints};
            }
            auto& vars = iteminitem_constraints[{i,j}].fi;
            auto& cons = iteminitem_constraints[{i,j}].se;
            foe(v, vars) helper.add_variable(v);
            foe(c, cons) helper.add_constraint(c);
        }

        problem.set_warm_start(solution);
        fon(i, sz(subset)) {
            if(!solution.count(in_use_binaries[i].se)) continue;
            if(solution[in_use_binaries[i].se] > 0.5) {
                cout << i << endl;
                problem.fix_variable(in_use_binaries[i].se, 1);
            } else {
                // problem.fix_variable(in_use_binaries[i].se, 0); // TODO: maybe delete
            }
        }

        cout << "[c++] Computing solution using MIP" << endl;
        solution = problem.solve_with_params({180.0 / (double)i + 1, 0.1});
    }

    cout << "[c++] Moving items to found reference point solution coordinates" << endl;
    MIPPackingHelpers helper (NULL, NULL);
    PackingOutput output = helper.produce_output(_input, input.items, solution, in_use_binaries, xys);

    return output;
}

PackingOutput HeuristicPackingFast::run(PackingInput _input) {

    cout << "[c++] Optimal fast algorithm" << endl;
    MIPPackingHelpers helper (NULL, NULL);
    helper.set_global_packing_parameter(_input);

    cout << "[c++] Scaling input" << endl;
    auto input = MIPPackingHelpers(NULL, NULL).scalesnap_input(_input);

    cout << "[c++] Expanding items" << endl;
    input.items = input.items.expand();

    cout << "[c++] Sorting items" << endl;
    vector<int> sorted_idxs = helper.sort_by_value_over_area(input.items);
    input.items = permute(input.items, sorted_idxs);

    auto original_in_use_binaries = helper.get_and_add_in_use_binaries(sz(input.items));
    auto original_xys = helper.get_and_add_xy_ref_variables(sz(input.items));

    map<string,FT> solution;
    Polygon_set existing;
    int number_of_included_items = 0;
    fon(i, sz(input.items)) {
        cout << "[c++] Iteration " << i << " / " << sz(input.items) << endl;
        auto& item = input.items[i];
        Gurobi_MIP problem;
        MIPPackingHelpers helper (&problem, NULL);

        // TODO: try to place in holes first in increasing order of area
        cout << "[c++] Building tmp items from already placed items" << endl;
        ItemsContainer items; items.add_item(item);
        /*foe(pwh, to_polygon_vector(existing)) {
            foe(pol, fix_repeated_points(pwh.outer_boundary())) {
                items.add_item(0, 1, pol, 0, Vector(0,0));
            }
        }*/

        cout << "[c++] Adding 'in-use' binaries" << endl;
        auto in_use_binaries = helper.get_and_add_in_use_binaries(sz(items));

        cout << "[c++] Adding reference x and y variables" << endl;
        auto xys = helper.get_and_add_xy_ref_variables(sz(items));

        cout << "[c++] Setting objective" << endl;
        // helper.set_max_objective_with_in_use_binaries(items, in_use_binaries);
        problem.set_min_objective({{xys[0].fi.se, 1}, {xys[0].se.se, MIPPackingHelpers::inf}, {in_use_binaries[0].se, -MIPPackingHelpers::biginf}});

        cout << "[c++] Compute allowed space" << endl;
        //Polygon_set allowed_space;
        //allowed_space.intersection(Polygon_set(input.container), get_complement(existing)); // TODO: dont compute this every time if not changed

        cout << "[c++] Adding items inside container constraints" << endl;
        //auto t = Polygon_set(to_polygon_vector(allowed_space)[0].outer_boundary());
        Polygon_set finalone;
        foe(pwh, to_polygon_vector(existing)) {
            foe(pol, fix_repeated_points(pwh.outer_boundary())) {
                finalone.join(pol);
            }
        }
        Polygon_set allowed_space;
        allowed_space.intersection(Polygon_set(input.container), get_complement(finalone)); // TODO: dont compute this every time if not changed
        helper.add_constraints_inside_polygon_set(
            allowed_space,
            items,
            in_use_binaries,
            xys
        );
        /*helper.add_constraints_inside_convex_polygon(
            input.container,
            items,
            in_use_binaries,
            xys
        );*/

        //cout << "[c++] Adding items no overlap variables and constraints" << endl;
        /*for(int j = 1; j < sz(items); j++) {
            helper.add_exact_constraints(
                xys[0], xys[j], items[0], items[j],
                to_string(0) + "_" + to_string(j),
                in_use_binaries[0], in_use_binaries[j]
            );
        }*/

        /*cout << "[c++] Setting known coordinates" << endl;
        for(int j = 1; j < sz(items); j++) {
            auto p = items[j].get_reference_point();
            problem.fix_variable(xys[j].fi.se, p.x());
            problem.fix_variable(xys[j].se.se, p.y());
            problem.fix_variable(in_use_binaries[j].se, 1);
        }*/

        cout << "[c++] Computing solution using MIP" << endl;
        auto tmp = problem.solve_with_params({.time_limit = 30, .mipgap = 0});

        if(tmp[in_use_binaries[0].se] > 0.5) {
            cout << "[c++] Item was included" << endl;
            solution[original_in_use_binaries[i].se] = 1;
            assert(tmp.count(xys[0].fi.se));
            assert(tmp.count(xys[0].se.se));
            FT x = tmp[xys[0].fi.se];
            FT y = tmp[xys[0].se.se];
            solution[original_xys[i].fi.se] = x;
            solution[original_xys[i].se.se] = y;
            auto pol = item.move_ref_point(Point(x,y)).pol;
            assert(is_completely_inside(Polygon_set(input.container), Polygon_set(pol)));
            assert(is_completely_inside(get_complement(existing), Polygon_set(pol)));
            existing.join(pol);
            number_of_included_items++;
        } else {
            cout << "[c++] Item was not included" << endl;
            solution[original_in_use_binaries[i].se] = 0;
        }
        cout << "[c++] Number of included items: " << number_of_included_items << " / " << (i + 1) << endl;
    }

    cout << "[c++] Unscaling solution coords" << endl;
    solution = helper.unscale_xy_coords(solution, original_xys, input.items);

    cout << "[c++] Moving items to found reference point solution coordinates" << endl;
    auto _expanded = permute(_input.items.expand(), sorted_idxs);
    {
        fon(i, sz(_expanded))  {
            assert(_expanded[i].idx == input.items[i].idx);
        }
    }
    PackingOutput output = helper.produce_output(
        _input,
        _expanded,
        solution,
        original_in_use_binaries,
        original_xys
    );

    return output;
}

PackingOutput HeuristicPackingNOMIP::run(PackingInput _input) {
    auto input = _input;

    cout << "[c++] Expanding items" << endl;
    input.items = input.items.expand();

    cout << "[c++] Sorting items" << endl;
    MIPPackingHelpers helper (NULL, NULL);
    vector<int> sorted_idxs = helper.sort_by_value_over_area(input.items); // TODO: does this even sort non-increasingly?
    input.items = permute(input.items, sorted_idxs);

    PackingOutput output (_input);
    auto add_item = [&](Item& item, Point p) {
        assert(is_integer(p.x()));
        assert(is_integer(p.y()));
        Item new_item = item.move_ref_point(p);
        // assert(is_completely_inside(Polygon_set(input.container), Polygon_set(new_item.pol)));
        output.add_item(new_item);
    };
    Polygon_set existing;
    int number_of_included_items = 0;
    fon(i, sz(input.items)) {
        cout << "[c++] Iteration " << (i + 1) << " / " << sz(input.items) << endl;
        auto& item = input.items[i];

        cout << "[c++] Computing configuration space" << endl;
        Polygon_set disallowed_space = get_complement(Polygon_set(input.container));
        disallowed_space.join(existing);
        auto config_space = ConfigurationSpace(
            disallowed_space,
            item.pol,
            item.get_reference_point()
        ).space;

        cout << "[c++] Finding lowest integral point" << endl;
        vector<Point> vertices;
        foe(pwh, to_polygon_vector(config_space)) {
            foe(pol, pwh.outer_boundary()) vertices.push_back(pol);
            // TODO: holes needed?
        }
        sort(vertices.begin(), vertices.end(), [](Point& a, Point& b) {
            if(a.y() == b.y()) return a.x() < b.x();
            return a.y() < b.y();
        });
        foe(v, vertices) {
            foab(dy, -10, 5) foab(dx, -10, 5) { // TODO: probably don't need to check negative
                Point p (floor_exact(v.x() + dx), floor_exact(v.y() + dy));
                if(config_space.oriented_side(p) != CGAL::ON_NEGATIVE_SIDE) {
                    cout << "[c++] Found lowest integral point: " << p << endl;
                    add_item(item, p);
                    existing.join(item.move_ref_point(p).pol);
                    number_of_included_items++;
                    goto next_item;
                }
            }
        }
next_item:
        cout << "[c++] Number of included items: " << number_of_included_items << " / " << (i + 1) << endl;
    }

    return output;
}
