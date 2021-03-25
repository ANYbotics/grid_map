#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <grid_map_core/GridMap.hpp>
#include <filters/filter_base.h>

#include "grid_map_filters/MedianFillFilter.hpp"

using namespace grid_map;
using namespace ::testing;

using BASE = filters::FilterBase<grid_map::GridMap>;
using MedianFillFilterT = MedianFillFilter<GridMap>;

TEST(MedianFillFilter, ConstructFilterTest) {  // NOLINT
  MedianFillFilterT medianFillFilter{};
  SUCCEED();
}

TEST(MedianFillFilter, LoadParametersAndUpdateTest) {  // NOLINT
  MedianFillFilterT medianFillFilter{};

  // Set up the parameters
  XmlRpc::XmlRpcValue config;
  config["name"] = "median";
  config["type"] = "gridMapFilters/MedianFillFilter";

  XmlRpc::XmlRpcValue params;
  params["input_layer"] = "elevation";
  params["output_layer"] = "elevation_filtered";
  params["fill_hole_radius"] = 0.11;
  params["filter_existing_values"] = true;
  params["existing_value_radius"] = 0.02;

  config["params"] = params;

  medianFillFilter.BASE::configure(config);
  medianFillFilter.configure();

  // Set up some test data
  GridMap filterInput = GridMap({"elevation", "variance"});
  filterInput.setGeometry(Length(0.4, 0.4), 0.02, Position(1.0, 5.0));
  filterInput.setFrameId("map");

  Matrix& elevationLayer = filterInput["elevation"];
  Matrix& varianceLayer = filterInput["variance"];

  elevationLayer.setConstant(1);
  varianceLayer.setConstant(0.05);

  GridMap filterOutput;
  medianFillFilter.update(filterInput, filterOutput);

  ASSERT_THAT(filterOutput.getLayers(), ElementsAre(StrEq("elevation"), StrEq("variance"), StrEq("elevation_filtered")));

  ASSERT_TRUE(filterInput["elevation"].isApprox(filterOutput["elevation"]));
  ASSERT_TRUE(filterInput["variance"].isApprox(filterOutput["variance"]));

  // Now we add some NaNs and see if they are filtered out.

  GridMap noisyFilterInput{filterInput};

  // Add some holes
  noisyFilterInput["elevation"](0, 0) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](10, 5) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](2, 4) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](6, 3) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](11, 8) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](3, 15) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](10, 10) = std::numeric_limits<float>::quiet_NaN();

  // Check if the filter has removed the nans by comparing it with the original input.
  GridMap noisyFilterOutput;
  medianFillFilter.update(noisyFilterInput, noisyFilterOutput);
  ASSERT_TRUE(filterInput["elevation"].isApprox(noisyFilterOutput["elevation_filtered"]))
      << "Expected output: " << filterInput["elevation"] << "\nReceived: " << noisyFilterOutput["elevation"];
}
