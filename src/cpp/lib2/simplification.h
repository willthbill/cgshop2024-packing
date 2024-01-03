#pragma once

#include<bits/stdc++.h>
#include "lib/util/cgal.h"

class SimplifyExpand {
public:

    static Polygon_set run(Polygon_set& pset, FT scale);

    static Polygon run(Polygon& pol, FT scale);

    static Polygon_with_holes run(Polygon_with_holes& pwh, FT scale);

};