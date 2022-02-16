#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/utils/testing.hpp>

#include "grid_map_filters/MedianFillFilter.hpp"

using namespace grid_map;
using namespace ::testing;

using BASE = filters::FilterBase<grid_map::GridMap>;

TEST(MedianFillFilter, ConstructFilterTest) {  // NOLINT
  MedianFillFilter medianFillFilter{};
  SUCCEED();
}

TEST(MedianFillFilter, LoadParametersAndUpdateTest) {  // NOLINT
  MedianFillFilter medianFillFilter{};

  // Set up the parameters
  XmlRpc::XmlRpcValue config;
  config["name"] = "median";
  config["type"] = "gridMapFilters/MedianFillFilter";

  XmlRpc::XmlRpcValue params;
  params["input_layer"] = "elevation";
  params["output_layer"] = "elevation_filtered";
  params["fill_hole_radius"] = 0.02;
  params["filter_existing_values"] = true;
  params["existing_value_radius"] = 0.02;
  params["fill_mask_layer"] = "fill_mask";
  params["debug"] = false;
  params["num_erode_dilation_iterations"] = 4;

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

  ASSERT_THAT(filterOutput.getLayers(),
              ElementsAre(StrEq("elevation"), StrEq("variance"), StrEq("elevation_filtered"), StrEq("fill_mask")));

  ASSERT_TRUE(filterInput["elevation"].isApprox(filterOutput["elevation"]))
      << "Expected output:\n " << filterInput["elevation"] << "\nReceived:\n " << filterOutput["elevation"];
  ASSERT_TRUE(filterInput["variance"].isApprox(filterOutput["variance"]))
      << "Expected output:\n " << filterInput["variance"] << "\nReceived:\n " << filterOutput["variance"];
  ASSERT_TRUE(filterOutput["fill_mask"].isApprox(Matrix::Ones(filterOutput.getSize().x(), filterOutput.getSize().y())))
      << "Expected output:\n " << Matrix::Ones(filterOutput.getSize().x(), filterOutput.getSize().y()) << "\nReceived:\n "
      << filterOutput["fill_mask"];

  // Now we add some NaNs and see if they are filtered out.

  // add a large unobserved area that should not be filled arbitrarily
  filterInput["elevation"].bottomRightCorner<5, 5>().setConstant(NAN);

  GridMap noisyFilterInput{filterInput};

  // Add some holes that should be filled.
  noisyFilterInput["elevation"](0, 0) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](10, 5) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](2, 4) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](6, 3) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](11, 8) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](3, 15) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](10, 10) = std::numeric_limits<float>::quiet_NaN();

  // Check if the filter has removed the sparse nans by comparing it with the original input.
  GridMap noisyFilterOutput;
  medianFillFilter.update(noisyFilterInput, noisyFilterOutput);
  ASSERT_MATRICES_EQ_WITH_NAN(filterInput["elevation"], noisyFilterOutput["elevation_filtered"]);
}
