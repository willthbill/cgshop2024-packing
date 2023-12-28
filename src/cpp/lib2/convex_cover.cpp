#include<bits/stdc++.h>

#include "lib/util/cgal.h"
#include "lib/util/geometry_utils.h"
#include "lib/geometry/partition_constructor.h"
#include "lib/geometry/visgraph.h"
#include "lib/graph/cliquecover.h"
#include "lib/util/debug.h"
#include "lib/util/common.h"

using namespace std;

class ConvexCover{
public:
    static vector<Polygon> get_convex_cover(Polygon_set& pset) {
        vector<Polygon> convex_cover;
        foe(pwh, to_polygon_vector(pset)) {
            foe(pol, get_convex_cover(pwh)) {
                convex_cover.push_back(pol);
            }
        }
        return convex_cover;
    }
    static vector<Polygon> get_convex_cover(Polygon& input_polygon) {
        Polygon_with_holes pwh (input_polygon);
        return get_convex_cover(pwh);
    }
    static vector<Polygon> get_convex_cover(Polygon_with_holes& input_polygon) {
        PartitionConstructor partition (input_polygon);
        // partition.add_extension_segments(); results in non-integral coordinates
        auto extension_triangulation = partition.get_constrained_delaunay_triangulation();

        CHGraph chgraph_extension (input_polygon, extension_triangulation, "unset");
        chgraph_extension.add_fully_visible_bfs_edges_exact();

        CliqueCover clique_cover (chgraph_extension);
        auto polygons = clique_cover.get_cliques_smalladj_naive();
        chgraph_extension.delete_datastructures();
        
        return polygons;
    }
};