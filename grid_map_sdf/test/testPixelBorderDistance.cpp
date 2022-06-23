/*
 * testPixelBorderDistance.cpp
 *
 *  Created on: Aug 7, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#include <gtest/gtest.h>

#include "grid_map_sdf/PixelBorderDistance.hpp"
#include "grid_map_sdf/Utils.hpp"

using namespace grid_map;
using namespace signed_distance_field;

TEST(testPixelBorderDistance, distanceFunction) {
  // Basic properties of the distance function
  ASSERT_TRUE(pixelBorderDistance(0, 0) == 0.0F);
  ASSERT_FLOAT_EQ(pixelBorderDistance(0, 1), 0.5);
  ASSERT_FLOAT_EQ(pixelBorderDistance(0, 2), 1.5);
  ASSERT_TRUE(pixelBorderDistance(0, 1) == pixelBorderDistance(1, 0));
  ASSERT_TRUE(pixelBorderDistance(-10, 42) == pixelBorderDistance(42, -10));
}

TEST(testPixelBorderDistance, equidistantPoint) {
  int pixelRange = 10;
  float offsetRange = 20.0;
  float offsetStep = 0.25;
  float tol = 1e-4;

  for (int p = -pixelRange; p < pixelRange; ++p) {
    for (float fp = -offsetRange; fp < offsetRange; fp += offsetStep) {
      for (int q = -pixelRange; q < pixelRange; ++q) {
        for (float fq = -offsetRange; fq < offsetRange; fq += offsetStep) {
          // Fix that offset is the same if pixels are the same
          if (p == q) {
            fp = fq;
          }
          // Check symmetry of the equidistant point computation
          float s0 = equidistancePoint(q, fq, p, fp);
          float s1 = equidistancePoint(p, fp, q, fq);
          ASSERT_LT(std::abs(s0 - s1), tol);

          // Check that the distance from s0 to p and q is indeed equal
          float dp = squarePixelBorderDistance(s0, p, fp);
          float dq = squarePixelBorderDistance(s0, q, fq);
          ASSERT_LT(std::abs(dp - dq), tol) << "p: " << p << ", q: " << q << ", fp: " << fp << ", fq: " << fq;
        }
      }
    }
  }
}

TEST(testPixelBorderDistance, equidistantPointInfCases) {
  const float pixelTestDistance{1e6};  // Pick a very high pixel index
  // With one of the cells at +INF, the intersection will always be +- INF on the side of that cell
  // Here the intersection is at the left = -INF
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, INF, pixelTestDistance, std::numeric_limits<float>::max()), -INF);
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, INF, pixelTestDistance, 0.0F), -INF);
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, INF, pixelTestDistance, std::numeric_limits<float>::lowest()), -INF);
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, INF, pixelTestDistance, -INF), -INF);
  // Here the intersection is at the right = +INF
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, std::numeric_limits<float>::max(), pixelTestDistance, INF), INF);
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, 0.0F, pixelTestDistance, INF), INF);
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, std::numeric_limits<float>::lowest(), pixelTestDistance, INF), INF);
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, -INF, pixelTestDistance, INF), INF);
  // Except when both are infinite, then the intersection is in the middle
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, INF, pixelTestDistance, INF), 0.0F);
  EXPECT_FLOAT_EQ(
      equidistancePoint(-pixelTestDistance, std::numeric_limits<float>::max(), pixelTestDistance, std::numeric_limits<float>::max()), 0.0F);
  EXPECT_FLOAT_EQ(
      equidistancePoint(-pixelTestDistance, std::numeric_limits<float>::lowest(), pixelTestDistance, std::numeric_limits<float>::lowest()),
      0.0F);
  EXPECT_FLOAT_EQ(equidistancePoint(-pixelTestDistance, -INF, pixelTestDistance, -INF), 0.0F);
}
