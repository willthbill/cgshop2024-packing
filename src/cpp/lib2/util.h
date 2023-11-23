#pragma once

#include <bits/stdc++.h>
#include <CGAL/Gmpq.h>

#include "lib/util/cgal.h"

bool is_polygon_inside_polygon(const Polygon& p1, const Polygon& p2);

// deterministic (also lowest x)
Point get_lowest_point(const Polygon& polygon);

Polygon_set get_complement(const Polygon_set& polygon_set);

CGAL::Gmpq floor_exact(const CGAL::Gmpq &q);

CGAL::Gmpq ceil_exact(const CGAL::Gmpq &q); 

bool is_integer(const CGAL::Gmpz v);

// Usage
// CGAL::Gmpq q = /* your rational number */;
// CGAL::Gmpz floored_value = floor_exact(q);
// CGAL::Gmpz ceiled_value = ceil_exact(q);

