/*
 * Utils.h
 *
 *  Created on: Jul 10, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#pragma once

namespace grid_map {
namespace signed_distance_field {

// Check existence of inf
static_assert(std::numeric_limits<float>::has_infinity, "float does not support infinity");

//! Distance value that is considered infinite.
constexpr float INF = std::numeric_limits<float>::infinity();

}  // namespace signed_distance_field
}  // namespace grid_map