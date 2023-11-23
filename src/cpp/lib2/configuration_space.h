#pragma once

#include "lib/util/cgal.h"

class ConfigurationSpace {
public:
    Polygon_set space;
    // space where ref point of pol can be located without pol intersecting with s
    ConfigurationSpace(Polygon_set& s, Polygon& _pol, Point ref);
    Polygon get_single_polygon();
};

