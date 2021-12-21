/*
 * MedianFillFilter.hpp
 *
 *  Created on: September 7, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <Eigen/Core>
#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <opencv2/core.hpp>

namespace grid_map {

/*!
 * Uses std::nth_element to fill holes in the input layer by the median of the surrounding values. The result is put into the output_layer.
 * Note: Only values for which the fill_layer is true will be filled. The fill_layer is auto computed if not present in the input.
 */
class MedianFillFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  MedianFillFilter();

  /*!
   * Destructor.
   */
  ~MedianFillFilter() override;

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  bool configure() override;

  /*!
   * Adds a new output layer to the map.
   * Uses the Boost accumulator median in the input layer.
   * Saves the filter output in mapOut[output_layer].
   * @param mapIn grid map containing input layer
   * @param mapOut grid map containing mapIn and median filtered input layer.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 protected:
  /*!
   * Returns the median of the values in inputData in the neighbourhood around the centerIndex. The size of the quadratic neighbourhood is
   * specified by radiusInPixels. If the number of values is even the "lower center" value is taken, eg with four values the second lowest
   * is taken as median.
   * @param inputMap The data layer to compute a local median.
   * @param centerIndex The center cell of the neighbourhood.
   * @param radiusInPixels The maximum L_inf distance from index.
   * @param bufferSize The buffer size of the input
   * @return The median of finites in the specified neighbourhood.
   */
  static float getMedian(Eigen::Ref<const Matrix> inputMap, const Index& centerIndex, size_t radiusInPixels, Size bufferSize);

  /**
   * Computes a mask of which cells to fill-in based on the validity of input cells. I.e small holes between (and including) valid cells are
   * marked to be filled.
   *
   * @remark The returned fill_mask is also added as layer to the output to be reused in following iterations of this filter.
   *         If debug is enabled, an intermediate mask is added as layer to the output grid map.
   * @param inputMap The input layer, used to check which cells contain valid values.
   * @param mapOut The output GridMap will contain the additional fill_mask layer afterwards.
   * @return An eigen mask indicating which cells should be filled by the median filter.
   */
  Eigen::MatrixXf computeAndAddFillMask(const Eigen::MatrixXf& inputMap, GridMap& mapOut);

  /**
   * Remove sparse valid regions by morphological opening.
   * @remark Check https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html
   * for more information about the opening operation.
   * @param inputMask Initial mask possibly containing also sparse valid regions that will be removed.
   * @return An opencv mask of the same size as input mask with small sparse valid regions removed.
   */
  static cv::Mat_<bool> cleanedMask(const cv::Mat_<bool>& inputMask);

  /**
   * Performs morphological closing on a boolean cv matrix mask.
   * @param [in] isValidMask A 2d mask where holes up to a certain size will be filled.
   * @param [in] numDilationClosingIterations Algorithm specific parameter. Higher means that bigger holes will still be filled.
   * @return A mask of the same size as isValidMask but with small holes filled.
   */
  static cv::Mat_<bool> fillHoles(const cv::Mat_<bool>& isValidMask, size_t numDilationClosingIterations);

  /**
   * Adds a float cv matrix as layer to a given map.
   * @param [in, out] gridMap The map to add the layer.
   * @param [in] cvLayer The cv matrix to add.
   * @param [in] layerName The layer name
   */
  static void addCvMatAsLayer(GridMap& gridMap, const cv::Mat& cvLayer, const std::string& layerName);

 private:
  //! Median filtering radius of NaN values in the input.
  double fillHoleRadius_;

  //! Median filtering radius for existing values in the input.
  double existingValueRadius_;

  //! Flag indicating whether to also filter finite values.
  bool filterExistingValues_;

  //! Number of erode-dilate iterations to calculate mask. Higher means that bigger holes will still be filled.
  int numErodeDilationIterations_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;

  //! Layer containing indicating which areas to fill, will be computed if not present.
  std::string fillMaskLayer_ = "should_fill";

  //! Layer used to visualize the intermediate, sparse outlier removed fill mask.
  std::string debugInfillMaskLayer_ = "debug_infill_mask";

  //! If set, the filtered grid_map is augmented with additional debug layers.
  bool debug_;
};

}  // namespace grid_map
