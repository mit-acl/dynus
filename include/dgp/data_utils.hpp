/**
 * @file data_utils.hpp
 * @brief Provide a few widely used function for basic type
 */
#ifndef DATA_UTILS_H
#define DATA_UTILS_H

#include "dgp/data_type.hpp"

/// Template for transforming a vector
template <class T, class TF>
vec_E<T> transform_vec(const vec_E<T> &t, const TF &tf)
{
  vec_E<T> new_t;
  for (const auto &it : t)
    new_t.push_back(tf * it);
  return new_t;
}

/// Template for calculating distance
template <class T>
decimal_t total_distance(const vec_E<T> &vs)
{
  decimal_t dist = 0;
  for (unsigned int i = 1; i < vs.size(); i++)
    dist += (vs[i] - vs[i - 1]).norm();

  return dist;
}

/// Transform all entries in a vector using given TF
#define transform_vec3 transform_vec<Vec3f, Aff3f>
/// Sum up total distance for vec_Vec2f
#define total_distance2f total_distance<Vec2f>
/// Sum up total distance for vec_Vec3f
#define total_distance3f total_distance<Vec3f>
/// Sum up total distance for vec_Vec2i
#define total_distance2i total_distance<Vec2i>
/// Sum up total distance for vec_Vec3i
#define total_distance3i total_distance<Vec3i>
#endif
