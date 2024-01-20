#include <bits/stdc++.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "lib2/simplification.h"
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
#include "lib2/heuristic_packing.h"

using namespace std;

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

    // TODO: remove static
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
        foe(p, get_vertices_pset(input.container)) {
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

        scale = ceil_exact(mx / (FT(sqrt(biginf.to_double()))) / 0.4 / 0.8); //0.935);
        assert(scale >= -0.00001);
        if(scale <= 1) scale = 1;
        scale = FT(1)/FT(scale);

        assert(scale > 0.00000001);
        assert(scale <= 1);
        if(mx * mx < biginf * 0.1) assert(scale == 1);

        mx = ceil((mx * scale.to_double()).to_double());
        inf = ceil((100 + mx * 2.5).to_double()); // upper bound on coordinates
        debug(inf);
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
                Polygon_set disallowed (get_int_bounding_box(item1.pol));
                auto bbox2 = get_int_bounding_box(item2.pol);
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
                debug(config_space.is_valid());
                // Snap it to a grid
                config_space_int = get_complement(
                    SnapToGrid(get_complement(config_space)).space
                ); // TODO: taking complement twice
                debug(config_space_int.is_valid());
            }

            // Computer intersection of large square and completement of configuration space
            // TODO: why are we doing this??????
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
        Polygon_set scaled_container_all;
        foe(pwh, to_polygon_vector(input.container)) {
            auto t = Polygon_set(pwh);
            auto unfixed_container = get_single_polygon(t); // it cannot have holes
            foe(container, fix_repeated_points(unfixed_container)) {
                /*debug("container");
                foe(p, container) {
                    debug(p);
                }
                debug("container end");*/
                // cout << "Original container area: " << container.area().to_double() << endl;
                Polygon_set scaled_container;
                {
                    auto t = get_complement(Polygon_set(scale_polygon(container, FT(scale))));
                    t.intersection(get_big_square());
                    auto v = to_polygon_vector_ref(SnapToGrid(t).space);
                    assert(sz(v) == 1);
                    // there might be 0 holes
                    foe(hole, v[0].holes()) {
                        auto t = hole;
                        t.reverse_orientation();
                        scaled_container.join(t);
                    }
                }
                if(scale < 1) {
                    // assert(is_completely_inside(Polygon_set(container), Polygon_set(scaled_container)));
                } else {
                    //assert(is_completely_inside(Polygon_set(scaled_container), Polygon_set(container)));
                }
                //cout << "Snapped container area: " << scaled_container.area().to_double() << endl;
                /*foe(p, scaled_container) {
                    assert(is_integer(p.x()));
                    assert(is_integer(p.y()));
                    //assert(p.x() >= 0);
                    //assert(p.y() >= 0);
                }*/
                scaled_container_all.join(scaled_container);
            }
        }
        PackingInput modified_input {
            scaled_container_all,
            scale_items(input.items)
        };
        foe(item, modified_input.items) {
            auto old = item.pol;
            /*debug("begin item");
            foe(p, old) {
                debug(p);
            }
            debug("end item");*/
            auto old_ref = item.get_reference_point();
            foe(p, old) {
                assert(p.x() >= 0);
                assert(p.y() >= 0);
            }
            //cout << "Original item area: " << item.pol.area().to_double() << endl;
            item.pol = SnapToGrid(Polygon_set(item.pol)).get_single_polygon();
            //cout << "Snapped item area: " << item.pol.area().to_double() << endl;
            assert(is_completely_inside(Polygon_set(item.pol), Polygon_set(old)));
            Vector ref_translation = item.get_reference_point() - old_ref;
            assert(is_integer(item.get_reference_point().x()));
            assert(is_integer(item.get_reference_point().y()));
            item.ref_scaling_translation = ref_translation;
            foe(p, item.pol) {
                assert(is_integer(p.x()));
                assert(is_integer(p.y()));
            }
            // TODO: what happens if sz(item.pol) == 0??
        }
        return modified_input;
    }

    ItemsContainer scale_items(ItemsContainer items) {
        foe(item, items) {
            item.pol = scale_polygon(item.pol, scale);
        }
        return items;
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
        fon(i, sz(items)) {
            auto& item = items[i];
            vector<Polygon> partition;
            {
                // Compute configuration space
                Polygon_set disallowed = get_complement(pset);
                debug(area(pset));
                auto config_space = ConfigurationSpace(
                    disallowed,
                    item.pol,
                    item.get_reference_point()
                ).space;
                debug(config_space.is_valid());
                debug(area(config_space));

                // Snap it to a grid
                auto config_space_int = get_complement(
                    SnapToGrid(get_complement(config_space)).space
                ); // TODO: taking complement twice
                debug(config_space_int.is_valid());
                debug(area(config_space_int));

                partition = ConvexCover::get_convex_cover(config_space_int);
                debug(sz(partition));
            }
            vector<MIPVariable> all_tmp_binaries;
            fon(idx, sz(partition)) {
                assert_is_integer_polygon(partition[idx]);
                vector<MIPVariable> binaries = {in_use_binaries[i]};
                MIPVariable var = {"bin", "polygon_set_binary_" + to_string(i) + "_" + to_string(idx)}; // TODO: must be unique
                add_variable(var);
                binaries.push_back(var);

                all_tmp_binaries.push_back(var); // TODO

                // add_constraints_inside_convex_polygon(cover[idx], items, binaries, xy_ref_variables);
                auto& x = xy_ref_variables[i].fi;
                auto& y = xy_ref_variables[i].se;
                auto ref = item.get_reference_point();
                foe(constraint, get_constraints_point_inside_convex_polygon(
                    partition[idx], x, y, binaries, Vector(0,0)
                )) {
                    add_constraint(constraint);
                }
            }
            // At least one convex region
            vector<pair<string,FT>> terms;
            foe(b, all_tmp_binaries) terms.push_back({b.se,1});
            terms.push_back(make_pair(in_use_binaries[i].se, -(max_partition_size + 100ll))); // TODO: do we need this? not really right?
            assert(sz(terms) < max_partition_size - 1000);
            add_constraint({
                terms,
                "geq",
                1ll - (max_partition_size + 100ll)
            });
        }
    }
    // NOT WORKING
    void _add_constraints_inside_polygon_set(
        Polygon_set& pset,
        ItemsContainer& items,
        vector<MIPVariable>& in_use_binaries,
        vector<pair<MIPVariable,MIPVariable>>& xy_ref_variables
    ) {
        assert(false); // TODO: the problem is that this says that the entire polygon should be inside one of the convex regions. but it should just say that the reference point is inside one region.
        auto cover = ConvexCover::get_convex_cover(pset);
        /*{
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
            }
            Polygon_set un;
            foe(pol, cover) un.join(pol);
            assert(is_completely_inside(un, pset));
            assert(is_completely_inside(pset, un));
            assert(area_pset <= area_cover);

            foe(pwh, to_polygon_vector(existing)) {
                foe(pol, fix_repeated_points(pwh.outer_boundary())) {
                    items.add_item(0, 1, pol, 0, Vector(0,0));
                }
            }
        }*/
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

    get_single_polygon(_input.container); // just asserting that it is a single polygon

    cout << "[c++] Optimal algorithm using gurobi callback optimization" << endl;
    PackingInput cb_input = {_input.container, _input.items.expand()};

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
    {
        auto p = get_single_polygon(cb_input.container);
        helper.add_constraints_inside_convex_polygon(p, cb_input.items, cb_in_use_binaries, cb_xys);
    }

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

    get_single_polygon(_input.container); // just asserting that it is a single polygon

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
    {
        auto p = get_single_polygon(input.container);
        helper.add_constraints_inside_convex_polygon(p, input.items, in_use_binaries, xys);
    }

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

    get_single_polygon(_input.container); // just asserting that it is a single polygon

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
        {
            auto p = get_single_polygon(input.container);
            helper.add_constraints_inside_convex_polygon(p, subset, in_use_binaries, xys);
        }

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
                problem.fix_variable(in_use_binaries[i].se, 1, 0.01);
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


PackingOutput OptimalRearrangement::run(PackingInput _input, PackingOutput initial) {
    cout << "[c++] Optimal rearrangement algorithm" << endl;
    MIPPackingHelpers helper (NULL, NULL);
    helper.set_global_packing_parameter(_input);

    cout << "[c++] Scaling input" << endl;
    auto input = helper.scalesnap_input(_input);
    if(area(input.container) < 1) {
        cout << "[c++] WARNING: Container area is 0" << endl;
        return PackingInput(_input);
    }
    initial.items = helper.scale_items(initial.items); // TODO: check if any has area 0
    initial = HeuristicPackingNOMIP().run(input, false, 0); // TODO: 0 or 1?
    debug(initial.get_score());

    foe(item, input.items) {
        if(item.pol.area() < 1) {
            cout << "[c++] WARNING: Item area is 0" << endl;
        }
    }

    auto in_use_binaries = helper.get_and_add_in_use_binaries(sz(input.items));
    auto xys = helper.get_and_add_xy_ref_variables(sz(input.items));

    map<string,FT> solution;

    auto solve = [&](int stage) {
        Gurobi_MIP problem;
        MIPPackingHelpers helper (&problem, NULL);
        helper.get_and_add_in_use_binaries(sz(input.items));
        helper.get_and_add_xy_ref_variables(sz(input.items));

        cout << "[c++] Adding items inside container constraints" << endl;
        Polygon_set allowed_space;
        foe(pwh, to_polygon_vector(input.container)) {
            assert(pwh.number_of_holes() == 0);
            foe(pol, fix_repeated_points(pwh.outer_boundary())) {
                // TODO: could have extra binaries here for each polygon
                debug("POL");
                foe(p, pol) {
                    debug(p);
                }
                debug("END POL");
                allowed_space.join(pol); // thats stupid!
            }
        }
        debug("END BUILDING ALLOWED SPACE");
        foe(item, input.items) {
            debug("ITEM");
            foe(p, item.pol) {
                debug(p);
            }
            debug("END ITEM");
        }
        helper.add_constraints_inside_polygon_set(
            allowed_space,
            input.items,
            in_use_binaries,
            xys
        );

        // TODO: don't compute these twice
        cout << "[c++] Adding items no overlap variables and constraints" << endl;
        fon(i, sz(input.items)) fon(j, i) {
            helper.add_exact_constraints(
                xys[i], xys[j], input.items[i], input.items[j],
                to_string(i) + "_" + to_string(j),
                in_use_binaries[i], in_use_binaries[j]
            );
        }

        cout << "[c++] Settings objective" << endl;
        { // minimizing top y value
            MIPVariable topvar = {"int", "topvar"};
            helper.add_variable(topvar);
            fon(i, sz(input.items)) {
                vector<pair<string,FT>> terms;
                terms.push_back(make_pair(xys[i].second.second, 1));
                terms.push_back(make_pair(topvar.second, -1));
                auto tpol = Polygon_set(input.items[i].pol);
                helper.add_constraint({
                    terms,
                    "leq",
                    -get_height(tpol)
                });
            }
            if(stage <= 1) problem.set_min_objective({{topvar.second, 1}});
            else problem.fix_variable(topvar.second, solution[topvar.second], 0.01);
        }
        if(stage == 2) { // minimize sum of y values
            vector<pair<string,FT>> terms;
            fon(i, sz(input.items)) terms.push_back(make_pair(xys[i].second.second, 1)); // base on area
            problem.set_min_objective(terms);
        }
        if(stage == 3) { // minimize sum of x values
            fon(i, sz(input.items)) {
                problem.fix_variable(xys[i].second.second, solution[xys[i].second.second], 0.01);
            }
            vector<pair<string,FT>> terms;
            fon(i, sz(input.items)) terms.push_back(make_pair(xys[i].first.second, 1)); // base on area
            problem.set_min_objective(terms);
        }

        cout << "[c++] Fixing binaries" << endl;
        set<int> in_initial;
        foe(item, initial.items) in_initial.insert(item.idx);
        fon(i, sz(input.items)) {
            if(in_initial.count(i)) {
                problem.fix_variable(in_use_binaries[i].se, 1, stage == 0 ? 0.01 : 0.01);
                solution[in_use_binaries[i].se] = 1;
            } else {
                problem.fix_variable(in_use_binaries[i].se, 0, 0.01);
                solution[in_use_binaries[i].se] = 0;
            }
        }

        if(stage == 0) {
            cout << "[c++] Fixed start" << endl;
            foe(item, initial.items) {
                auto& idx = item.idx;
                Point ref = item.get_reference_point();
                //solution[xys[idx].fi.se] = ref.x();
                //solution[xys[idx].se.se] = ref.y();
                problem.fix_variable(xys[idx].fi.se, ref.x(), 0.01);
                problem.fix_variable(xys[idx].se.se, ref.y(), 0.01);
            }
        } else {
            cout << "[c++] Setting warm start" << endl;
            problem.set_warm_start(solution);
        }

        cout << "[c++] Computing solution using MIP" << endl;
        auto tmp = problem.solve_with_params({.time_limit = 300, .mipgap = 0});
        foe(e, tmp) {
            solution[e.fi] = e.se;
        }
        /*fon(i, sz(input.items)) {
            solution[xys[i].fi.se] = tmp[xys[i].fi.se];
            solution[xys[i].se.se] = tmp[xys[i].se.se];
        }*/
    };

    solve(0);
    solve(1);
    solve(2);
    solve(3);

    cout << "[c++] Unscaling solution coords" << endl;
    solution = helper.unscale_xy_coords(solution, xys, input.items);

    cout << "[c++] Moving items to found reference point solution coordinates" << endl;
    PackingOutput output = helper.produce_output(
        _input,
        _input.items,
        solution,
        in_use_binaries,
        xys
    );
    return output;
}