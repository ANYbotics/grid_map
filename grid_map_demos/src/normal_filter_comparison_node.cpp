/**
 * @authors     Peter Fankhauser, Simone Arreghini
 * @affiliation ANYbotics
 * @brief       Node for comparing different normal filters performances.
 */

#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <Eigen/Core>
#include <cmath>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Function headers
namespace grid_map
{
/*!
 * Function to calculate the normal vector computation error.Error is defined as sum over the map of absolute cosines
 * between analytical and computed normals. The cosines are calculated using the dot product between two unitary vectors.
 * The error sum is then averaged between the latest 20 iteration of the functions.
 *
 * @param map: grid map to which add the layers that contains the errors.
 * @param gridMapSize: Dimensions of grid map, passed as parameter to not being calculated every time.
 * @param directionalErrorAreaSum: Average of the summed error over the map for area method.
 * @param directionalErrorRasterSum: Average of the summed error over the map for raster method.
 */
void normalsErrorCalculation(
  grid_map::GridMap & map, const grid_map::Size & gridMapSize, double & directionalErrorAreaSum,
  double & directionalErrorRasterSum);

/*!
 * Function to add noise to the elevation map. Noise added is white noise from a uniform distribution [-1:1] multiplied by
 * the amount of noise wanted specified by noise_on_map.
 *
 * @param map: grid map to which add the layers that contains the errors.
 * @param gridMapSize: Dimensions of grid map, passed as parameter to not being calculated every time.
 * @param noise_on_map: Amount of noise wanted in meters, can be set as an argument in the roslaunch phase.
 */
void mapAddNoise(
  grid_map::GridMap & map, const grid_map::Size & gridMapSize,
  const double & noise_on_map);

/*!
 * Function to add outliers to the elevation map. Outliers are point where the elevation value is set to infinity.
 * It has to be performed after mapAddNoise.
 *
 * @param map: grid map to which add the layers that contains the errors.
 * @param gridMapSize: Dimensions of grid map, passed as parameter to not being calculated every time.
 * @param outlierPercentage: Amount of outliers wanted percentage, can be set as an argument in the roslaunch phase.
 */
void mapAddOutliers(
  grid_map::GridMap & map, const grid_map::Size & gridMapSize,
  const double outlierPercentage);

/*!
 * Function to add outliers to the elevation map. Outliers are point where the elevation value is set to infinity.
 * It has to be performed after mapAddNoise.
 *
 * @param map: grid map to which add the layers that contains the errors.
 * @param gridMapSize: Dimensions of grid map, passed as parameter to not being calculated every time.
 * @param filterRadius: Radius of the wanted shifting average filter.
 */
void mapAverageFiltering(
  grid_map::GridMap & map, const grid_map::Size & gridMapSize,
  const double filterRadius);
}  // namespace grid_map

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("normal_filter_comparison_demo");

  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());

  // Start time.
  double begin = node->now().seconds();

  // Load filter chain, defined in grid_map_demos/config/normal_filter_comparison.yaml.
  filters::FilterChain<grid_map::GridMap> filterChain_("grid_map::GridMap");

  node->declare_parameter("filter_chain_parameter_name", "filters");
  node->declare_parameter("noise_on_map", 0.015);
  node->declare_parameter("outliers_percentage", 0.0);

  std::string filterChainParametersName_;
  node->get_parameter("filter_chain_parameter_name", filterChainParametersName_);

  // Read noise amount, in meters, from parameters server.
  double noise_on_map;
  node->get_parameter("noise_on_map", noise_on_map);
  RCLCPP_INFO(node->get_logger(), "noise_on_map = %f", noise_on_map);

  double outliers_percentage;
  node->get_parameter("outliers_percentage", outliers_percentage);
  RCLCPP_INFO(node->get_logger(), "outliers_percentage = %f", outliers_percentage);

  // Configuration of chain filter.
  if (!filterChain_.configure(
      filterChainParametersName_, node->get_node_logging_interface(),
      node->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(node->get_logger(), "Could not configure the filter chain!");
  }

  // Parameters for grid map dimensions.
  const double mapLength = 2.5;
  const double mapWidth = 4.0;
  const double mapResolution = 0.02;

  // Grid map object creation.
  grid_map::GridMap map({"elevation", "normal_analytic_x", "normal_analytic_y",
      "normal_analytic_z"});
  map.setFrameId("map");
  map.setGeometry(
    grid_map::Length(mapLength, mapWidth), mapResolution,
    grid_map::Position(0.0, 0.0));
  const grid_map::Size gridMapSize = map.getSize();
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).\n"
    " The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(), map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(),
    map.getFrameId().c_str());

  // Initialize variables for normal quality comparison.
  double directionalErrorAreaSum = 0;
  double directionalErrorRasterSum = 0;

  // Surface shape variables.
  const double surfaceSpeed = 0.5;
  const double surfaceSlope = 0.2;
  const double surfaceBias = -0.04;
  const double wavePeriod = 5.0;

  // Work with grid map in a loop.
  // Grid map and analytic normals are continuously generated using exact functions.
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    rclcpp::Time time = node->now();

    // Calculate wave shaped elevation and analytic surface normal.
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) =
        surfaceBias + surfaceSlope * std::sin(
        surfaceSpeed * time.seconds() + wavePeriod * position.y()) * position.x();

      // Analytic normals computation.
      grid_map::Vector3 normalAnalytic(-surfaceSlope * std::sin(
          surfaceSpeed * time.seconds() + wavePeriod * position.y()),
        -position.x() * std::cos(surfaceSpeed * time.seconds() + wavePeriod * position.y()), 1.0);
      normalAnalytic.normalize();
      map.at("normal_analytic_x", *it) = normalAnalytic.x();
      map.at("normal_analytic_y", *it) = normalAnalytic.y();
      map.at("normal_analytic_z", *it) = normalAnalytic.z();
    }

    // elevation_filtered layer is used for visualization,
    // initialize it here and if there is noise and filtering it will be updated.
    map.add("elevation_filtered", map.get("elevation"));

    // Perturb and then filter map only if noise != 0.
    if (noise_on_map != 0.0) {
      grid_map::mapAddNoise(map, gridMapSize, noise_on_map);
    }
    if (outliers_percentage != 0.0) {
      grid_map::mapAddOutliers(map, gridMapSize, outliers_percentage);
    }
    if ((noise_on_map != 0.0) || (outliers_percentage != 0.0)) {
      grid_map::mapAverageFiltering(map, gridMapSize, 0.1);
    }

    // Computation of normals using filterChain_.update function.
    if (!filterChain_.update(map, map)) {
      RCLCPP_ERROR(node->get_logger(), "Could not update the grid map filter chain!");
    }

    // Normals error computation
    grid_map::normalsErrorCalculation(
      map, gridMapSize, directionalErrorAreaSum,
      directionalErrorRasterSum);
    rclcpp::Clock clock;
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), clock, 2000, "directionalErrorArea = %f", directionalErrorAreaSum);
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), clock, 2000, "directionalErrorRaster = %f", directionalErrorRasterSum);

    // Publish grid map.
    map.setTimestamp(time.nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map);
    publisher->publish(std::move(message));
    double end = node->now().seconds();

    // Limit simulation length to 1 minute.
    if ((end - begin) > 60) {
      break;
    }
  }

  return 0;
}

namespace grid_map
{
void normalsErrorCalculation(
  grid_map::GridMap & map, const grid_map::Size & gridMapSize, double & directionalErrorAreaSum,
  double & directionalErrorRasterSum)
{
  // If layers saved as matrices accessing values is faster.
  const grid_map::Matrix mapNormalAnalyticX = map["normal_analytic_x"];
  const grid_map::Matrix mapNormalAnalyticY = map["normal_analytic_y"];
  const grid_map::Matrix mapNormalAnalyticZ = map["normal_analytic_z"];
  const grid_map::Matrix mapNormalAreaX = map["normal_area_x"];
  const grid_map::Matrix mapNormalAreaY = map["normal_area_y"];
  const grid_map::Matrix mapNormalAreaZ = map["normal_area_z"];
  const grid_map::Matrix mapNormalRasterX = map["normal_raster_x"];
  const grid_map::Matrix mapNormalRasterY = map["normal_raster_y"];
  const grid_map::Matrix mapNormalRasterZ = map["normal_raster_z"];

  // Initialize vector to use eigen scalar product (Vector3 = Eigen::Vector3d).
  grid_map::Vector3 normalVectorAnalytic = Vector3::Zero();
  grid_map::Vector3 normalVectorArea = Vector3::Zero();
  grid_map::Vector3 normalVectorRaster = Vector3::Zero();

  // Add layer for possible future visualization in RViz to understand where error is.
  map.add("directionalErrorArea", grid_map::Matrix::Zero(gridMapSize(0), gridMapSize(1)));
  map.add("directionalErrorRaster", grid_map::Matrix::Zero(gridMapSize(0), gridMapSize(1)));

  // Raster normals not defined for outermost layer of cells.
  const grid_map::Index submapStartIndex(1, 1);
  const grid_map::Index submapBufferSize(gridMapSize(0) - 2, gridMapSize(1) - 2);
  for (grid_map::SubmapIterator iterator(map, submapStartIndex, submapBufferSize);
    !iterator.isPastEnd(); ++iterator)
  {
    const grid_map::Index index(*iterator);

    normalVectorAnalytic(0) = mapNormalAnalyticX(index(0), index(1));
    normalVectorAnalytic(1) = mapNormalAnalyticY(index(0), index(1));
    normalVectorAnalytic(2) = mapNormalAnalyticZ(index(0), index(1));

    normalVectorArea(0) = mapNormalAreaX(index(0), index(1));
    normalVectorArea(1) = mapNormalAreaY(index(0), index(1));
    normalVectorArea(2) = mapNormalAreaZ(index(0), index(1));

    normalVectorRaster(0) = mapNormalRasterX(index(0), index(1));
    normalVectorRaster(1) = mapNormalRasterY(index(0), index(1));
    normalVectorRaster(2) = mapNormalRasterZ(index(0), index(1));

    // Error(alpha) = 1.0 - abs(cos(alpha)),
    // where alpha = angle(normalVectorAnalytic, normalVectorArea)
    // Error of perfect normals will be 0.0.
    map.at(
      "directionalErrorArea",
      *iterator) = 1.0 - std::abs(normalVectorAnalytic.dot(normalVectorArea));
    map.at(
      "directionalErrorRaster",
      *iterator) = 1.0 - std::abs(normalVectorAnalytic.dot(normalVectorRaster));
  }

  // Directional error defined as sum of
  // absolute cosines of normal calculated with different methods
  const double directionalErrorArea = map["directionalErrorArea"].array().sum();
  const double directionalErrorRaster = map["directionalErrorRaster"].array().sum();

  // Calculate mean value of directional error of last 20 cycles.
  directionalErrorAreaSum = (directionalErrorAreaSum * 19.0 + directionalErrorArea) / 20.0;
  directionalErrorRasterSum = (directionalErrorRasterSum * 19.0 + directionalErrorRaster) / 20.0;
}

void mapAddNoise(
  grid_map::GridMap & map, const grid_map::Size & gridMapSize,
  const double & noise_on_map)
{
  // Add noise (using Eigen operators).
  map.add("noise", noise_on_map * Matrix::Random(gridMapSize(0), gridMapSize(1)));
  map.add("elevation_noisy", map.get("elevation") + map["noise"]);
}

void mapAddOutliers(
  grid_map::GridMap & map, const grid_map::Size & gridMapSize,
  const double outlierPercentage)
{
  // Adding outliers at infinite height (accessing cell by position).
  const double numberInfPoints = outlierPercentage * gridMapSize(0) * gridMapSize(1);

  for (int i = 0; i < static_cast<int>(numberInfPoints); ++i) {
    grid_map::Position randomPosition = grid_map::Position::Random();
    if (map.isInside(randomPosition)) {
      map.atPosition("elevation_noisy", randomPosition) = std::numeric_limits<float>::infinity();
    }
  }
}

void mapAverageFiltering(
  grid_map::GridMap & map, const grid_map::Size & gridMapSize,
  const double filterRadius)
{
  grid_map::Index startIndex(0, 0);
  grid_map::SubmapIterator it(map, startIndex, gridMapSize);
  // Iterate over whole map.
  for (; !it.isPastEnd(); ++it) {
    Position currentPosition;
    map.getPosition(*it, currentPosition);
    double mean = 0.0;
    double sumOfWeights = 0.0;

    // Compute weighted mean.
    for (grid_map::CircleIterator circleIt(map, currentPosition, filterRadius);
      !circleIt.isPastEnd(); ++circleIt)
    {
      if (!map.isValid(*circleIt, "elevation_noisy")) {
        continue;
      }
      grid_map::Position currentPositionInCircle;
      map.getPosition(*circleIt, currentPositionInCircle);

      // Computed weighted mean based on Euclidian distance.
      double distance = (currentPosition - currentPositionInCircle).norm();
      double weight = pow(filterRadius - distance, 2);
      mean += weight * map.at("elevation_noisy", *circleIt);
      sumOfWeights += weight;
    }
    if (sumOfWeights != 0) {
      map.at("elevation_filtered", *it) = mean / sumOfWeights;
    } else {
      map.at("elevation_filtered", *it) = std::numeric_limits<float>::infinity();
    }
  }
}
}  // namespace grid_map
