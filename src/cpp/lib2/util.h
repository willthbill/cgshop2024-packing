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
// Usage
// CGAL::Gmpq q = /* your rational number */;
// CGAL::Gmpz floored_value = floor_exact(q);
// CGAL::Gmpz ceiled_value = ceil_exact(q);

bool is_integer(const CGAL::Gmpq v);

bool is_completely_inside(Polygon_set a, Polygon_set b); 
bool is_completely_outside(Polygon_set a, Polygon_set b); 
bool is_completely_inside(Polygon a, Polygon b); 
bool is_completely_inside(Polygon_with_holes a, Polygon_with_holes b); 

Polygon get_single_polygon(Polygon_set&);

Polygon scale_polygon(Polygon pol, FT scale); 

void assert_is_integer_polygon(Polygon& pol); 

Polygon get_int_bounding_box(std::vector<Point> arr); 

std::vector<Point> get_vertices_pset(Polygon_set& pset); 

std::vector<Point> get_vertices_pol(Polygon& pol); 

Polygon get_int_bounding_box(Polygon& pol); 

Polygon get_int_bounding_box(Polygon_set& pset); 

int get_number_of_vertices(Polygon_set pset); 

std::vector<Polygon_with_holes> to_polygon_vector_ref(Polygon_set pset); 

template<typename T>
T permute(T vec, const std::vector<int>& indices) {
    assert((int)vec.size() == (int)indices.size());
    T permuted = vec;
    for(size_t i = 0; i < indices.size(); ++i) {
        permuted[i] = vec[indices[i]];
    }
    return permuted;
}

FT get_width(Polygon_set& pol); 

FT get_height(Polygon_set& pol); 

FT area(Polygon_set& pset);

Polygon translate_point_to_origin(Polygon& pol, Point ref); 

// TODO: add to lib
Polygon translate_centroid_to_origin(Polygon& pol); 

// TODO: allow to use reference pol
Polygon scale_polygon(Polygon pol, int s); 

int get_number_of_vertices(Polygon_set pset);

void assert_is_integer_polygon(Polygon& pol); 

std::vector<Polygon> fix_repeated_points(Polygon pol); 
