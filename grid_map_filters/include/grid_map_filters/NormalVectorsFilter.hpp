/*
 * NormalVectorsFilter.hpp
 *
 *  Created on: May 05, 2015
 *      Author: Peter Fankhauser, Martin Wermelinger
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>
#include <grid_map_core/grid_map_core.hpp>

#include <Eigen/Core>
#include <string>

namespace grid_map {

/*!
 * Compute the normal vectors of a layer in a map.
 */
template <typename T>
class NormalVectorsFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  NormalVectorsFilter();

  /*!
   * Destructor.
   */
  ~NormalVectorsFilter() override;

  /*!
   * Configures the filter from parameters on the Parameter Server.
   * This comprehend in order:
   *    1) algorithm, used for choosing between area and raster algorithm (default=area)
   *    2)If algorithm is not raster estimationRadius_ is read an a basic check on its sign is made.
   *      If something is wrong it switches to raster algorithm.
   *    3) parallelization_enabled, used for choosing between serial and parallel algorithm (default=false)
   *    4) thread_count, used to set the number of threads to be used in parallelization is enabled (default=auto)
   * The parallelization_enabled and algorithm parameters allow to choose between the 4 different methods { AreaSerial, AreaParallel,
   * RasterSerial, RasterParallel }
   *    4) normal_vector_positive_axis, used to define the upward positive direction for the normals
   *    5) input_layer, defines in which layer of the grid map lie the information needed (usually elevation or elevation_filtered)
   *    6) output_layers_prefix, defines the prefix for the new 3 layers (x,y,z) that will define the normal vectors
   * Those parameters have to be written in a .yaml file that will be processed as a sequence of filters.
   * An example can be seen in grid_map_demos/config/normal_filter_comparison.yaml.
   */
  bool configure() override;

  /*!
   * Compute the normal vectors of a layer in a map and
   * saves it as additional grid map layer.
   * @param mapIn grid map containing the layer for which the normal vectors are computed for.
   * @param mapOut grid map containing mapIn and the new layers for the normal vectors.
   */
  bool update(const T& mapIn, T& mapOut) override;

 private:
  /*!
   * Estimate the normal vector at each point of the input layer by using the areaSingleNormalComputation function.
   * This function makes use of the area method and is the serial version of such normal vector computation using a
   * simple for cycle to iterate over the whole map.
   *
   * @param map: grid map containing the layer for which the normal vectors are computed for.
   * @param inputLayer: Layer the normal vector should be computed for.
   * @param outputLayersPrefix: Output layer name prefix.
   */
  void computeWithAreaSerial(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);

  /*!
   * Estimate the normal vector at each point of the input layer by using the areaSingleNormalComputation function.
   * This function makes use of the area method and is the parallel version of such normal vector computation using
   * a parallel_for cycle to iterate over the whole map. The parallel_for construct is provided by Intel TBB library.
   *
   * @param map: grid map containing the layer for which the normal vectors are computed for.
   * @param inputLayer: Layer the normal vector should be computed for.
   * @param outputLayersPrefix: Output layer name prefix.
   */
  void computeWithAreaParallel(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);

  /*!
   * Estimate the normal vector at one point of the input layer, specified by the index parameter, by using points within
   * a circle of specified radius.
   *
   * The eigen decomposition of the covariance matrix (3x3) of all data points is used to establish the normal direction.
   * Four cases can be identified when the eigenvalues are ordered in ascending order:
   *    1) The data is in a cloud -> all eigenvalues are non-zero
   *    2) The data is on a plane -> The first eigenvalue is zero
   *    3) The data is on a line -> The first two eigenvalues are zero.
   *    4) The data is in one point -> All eigenvalues are zero
   *
   * Only case 1 & 2 provide enough information the establish a normal direction.
   * The degenerate cases (3 or 4) are identified by checking if the second eigenvalue is zero.
   *
   * The numerical threshold (1e-8) for the eigenvalue being zero is given by the accuracy of the decomposition, as reported by Eigen:
   * https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
   *
   * Finally, the sign normal vector is correct to be in the same direction as the user defined "normal vector positive axis"
   *
   * @param map: grid map containing the layer for which the normal vectors are computed for.
   * @param inputLayer: Layer the normal vector should be computed for.
   * @param outputLayersPrefix: Output layer name prefix.
   * @param index: Index of point in the grid map for which this function calculates the normal vector.
   */
  void areaSingleNormalComputation(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix,
                                          const grid_map::Index& index);
  /*!
   * Estimate the normal vector at each point of the input layer by using the rasterSingleNormalComputation function.
   * This function makes use of the raster method and is the serial version of such normal vector computation using a
   * simple for cycle to iterate over the whole map.
   *
   * @param map: grid map containing the layer for which the normal vectors are computed for.
   * @param inputLayer: Layer the normal vector should be computed for.
   * @param outputLayersPrefix: Output layer name prefix.
   */
  void computeWithRasterSerial(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);

  /*!
   * Estimate the normal vector at each point of the input layer by using the rasterSingleNormalComputation function.
   * This function makes use of the raster method and is the parallel version of such normal vector computation using
   * a parallel_for cycle to iterate over the whole map. The parallel_for construct is provided by Intel TBB library.
   *
   * @param map: grid map containing the layer for which the normal vectors are computed for.
   * @param inputLayer: Layer the normal vector should be computed for.
   * @param outputLayersPrefix: Output layer name prefix.
   */
  void computeWithRasterParallel(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);

  /*!
   * Estimate the normal vector at one point of the input layer, specified by the index parameter, by using neighboring points
   * around the point for which the normal vector is being calculated.
   *
   * The neighboring cells are used to linearly approximate the first derivative in both directions.
   *
   *   |T|   ^
   * |L|C|R| | x Axis
   *   |B|   |
   * <-------|
   * Y Axis
   *
   * Those values are then used to reconstruct the normal vector in the point of interest.
   * This algorithm doesn't make use of the central cell if all the neighboring point heights are present.
   * However, thanks to that, it can accommodate missing point heights, up to one in each direction, by using
   * exactly the central height value.
   * In this case, the resulting approximation, will suffer a loss of quality depending on the local surface shape.
   * This implementation skips the outermost values in order to avoid checks on the validity of height values
   * and therefore have a faster algorithm.
   *
   * Inspiration for algorithm: http://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
   *
   * Finally, the sign normal vector is correct to be in the same direction as the user defined "normal vector positive axis"
   *
   * @param map: grid map containing the layer for which the normal vectors are computed for.
   * @param inputLayer: Layer the normal vector should be computed for.
   * @param outputLayersPrefix: Output layer name prefix.
   * @param dataMap: Matrix containing the input layer of the grid map in question.
   * @param index: Index of point in the grid map for which this function calculates the normal vector.
   */
  void rasterSingleNormalComputation(GridMap& map, const std::string& outputLayersPrefix, const grid_map::Matrix& dataMap,
                                            const grid_map::Index& index);

  enum class Method { AreaSerial, AreaParallel, RasterSerial, RasterParallel };

  Method method_;

  //! Radius of submap for normal vector estimation.
  double estimationRadius_;

  //! Parameter that specifies whether to parallelize or not.
  bool parallelizationEnabled_;

  //! Parameter that specifies the number of thread used.
  int threadCount_;

  //! Normal vector positive axis.
  Eigen::Vector3d normalVectorPositiveAxis_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayersPrefix_;

  //! Grid Map Resolution.
  double gridMapResolution_;
};

}  // namespace grid_map
