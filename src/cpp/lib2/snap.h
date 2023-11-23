#pragma once

#include<bits/stdc++.h>

#include "lib/util/cgal.h"

// TODO: somehow specify the grid size
// TODO: support polygon set
class SnapToGrid {
public:
    Polygon_set space;
    SnapToGrid(Polygon_set pset);
    Polygon get_single_polygon(); 
    Polygon snap(Polygon pol); 
};
