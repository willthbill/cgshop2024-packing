#pragma once

#include <bits/stdc++.h>

#include "lib/util/cgal.h"

bool is_polygon_inside_polygon(const Polygon& p1, const Polygon& p2);

// deterministic (also lowest x)
Point get_lowest_point(const Polygon& polygon);

