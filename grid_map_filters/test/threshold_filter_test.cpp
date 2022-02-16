#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/utils/testing.hpp>

#include "grid_map_filters/ThresholdFilter.hpp"

#include <Eigen/Core>

using namespace grid_map;
using namespace ::testing;

using BASE = filters::FilterBase<grid_map::GridMap>;

TEST(ThresholdFilter, LoadParametersAndUpdateTest) {  // NOLINT
  ThresholdFilter thresholdFilter{};

  // Set up the parameters
  XmlRpc::XmlRpcValue config;
  config["name"] = "threshold_filter";
  config["type"] = "gridMapFilters/ThresholdFilter";

  XmlRpc::XmlRpcValue params;
  params["condition_layer"] = "standard_deviation";
  params["output_layer"] = "standard_deviation_filtered";
  params["upper_threshold"] = 0.05;
  params["set_to"] = NAN;

  config["params"] = params;

  thresholdFilter.BASE::configure(config);
  thresholdFilter.configure();

  // Set up some test data

  // General grid map geometry.
  GridMap filterInput = GridMap({"standard_deviation", "standard_deviation_filtered"});
  filterInput.setGeometry(Length(0.4, 0.4), 0.02, Position(1.0, 5.0));
  filterInput.setFrameId("map");

  // Set the condition layer to be generally within the threshold except two corners.
  Matrix& conditionLayer = filterInput["standard_deviation"];
  conditionLayer.setConstant(0.03);
  conditionLayer.bottomRightCorner<5, 5>().setConstant(0.06);
  conditionLayer.topLeftCorner<3, 3>().setConstant(NAN);

  // Initialize the output layer with ones.
  Matrix& outputLayer = filterInput["standard_deviation_filtered"];
  outputLayer.setConstant(1.0);

  // Setup the expected output, eg all values unchanged except the two corners set to nan.
  Matrix outputLayerExpected{outputLayer.rows(), outputLayer.cols()};
  outputLayerExpected.setConstant(1.0f);
  outputLayerExpected.topLeftCorner<3, 3>().setConstant(NAN);
  outputLayerExpected.bottomRightCorner<5, 5>().setConstant(NAN);

  // Run the filter.
  GridMap filterOutput;
  thresholdFilter.update(filterInput, filterOutput);

  // Check the layers existent in the output and the expected outputs of them.
  ASSERT_THAT(filterOutput.getLayers(), ElementsAre(StrEq("standard_deviation"), StrEq("standard_deviation_filtered")));
  ASSERT_MATRICES_EQ_WITH_NAN(conditionLayer, filterOutput["standard_deviation"]);
  ASSERT_MATRICES_EQ_WITH_NAN(filterOutput["standard_deviation_filtered"], outputLayerExpected);
}
