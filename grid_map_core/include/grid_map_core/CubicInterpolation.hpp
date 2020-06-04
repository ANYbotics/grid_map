/*
 * CubicInterpolation.hpp
 *
 *  Created on: Jan 21, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */


#ifndef GRID_MAP_CORE__CUBICINTERPOLATION_HPP_
#define GRID_MAP_CORE__CUBICINTERPOLATION_HPP_

#include <Eigen/Core>
#include <vector>
#include <map>
#include <string>
#include "grid_map_core/TypeDefs.hpp"

/*
 * For difference between bicubic convolution interpolation (piecewise cubic)
 * and bicubic interpolation see:
 *
 * https://en.wikipedia.org/wiki/Bicubic_interpolation
 *
 * R. Keys (1981). "Cubic convolution interpolation for digital image processing".
 * IEEE Transactions on Acoustics, Speech, and Signal Processing. 29 (6): 1153–1160.
 *
*   https://web.archive.org/web/20051024202307/
*   http://www.geovista.psu.edu/sites/geocomp99/Gc99/082/gc_082.htm
 */

namespace grid_map
{

class GridMap;

/*
 * Data structure (matrix) that contains data
 * necessary for interpolation. These are either 16
 * function values in the case of bicubic convolution interpolation
 * or function values and their derivatives for the case
 * of standard bicubic inteprolation.
 */
using FunctionValueMatrix = Eigen::Matrix4d;

/*!
 * Takes the id requested, performs checks and returns
 * an id that it is within the specified bounds.
 * @param[in] idReq - input index .
 * @param[in] nElem - number of elements in the container
 * @return index that is within [0, nElem-1].
 */
unsigned int bindIndexToRange(int idReq, unsigned int nElem);


/*!
 * Extract the value of the specific layer at the
 * row and column requested. If row and column are out
 * of bounds they will be bound to range.
 * @param[in] layerMat - matrix of the layer from where
 *                       the data is extracted
 * @param[in] rowReq - row requested
 * @param[in] colReq - column requested
 * @return - value of the layer at rowReq and colReq
 */
double getLayerValue(const Matrix & layerMat, int rowReq, int colReq);

namespace bicubic_conv
{

/*!
 * Matrix for cubic interpolation via convolution. Taken from:
 * https://en.wikipedia.org/wiki/Bicubic_interpolation
 */
static const Eigen::Matrix4d cubicInterpolationConvolutionMatrix {
  (Eigen::Matrix4d() << 0.0, 2.0, 0.0, 0.0,
    -1.0, 0.0, 1.0, 0.0,
    2.0, -5.0, 4.0, -1.0,
    -1.0, 3.0, -3.0, 1.0).finished()};

/*
 * Index of the middle knot for bicubic inteprolation. This is
 * the function value with subscripts (0,0), i.e. f00 in
 * https://en.wikipedia.org/wiki/Bicubic_interpolation
 * In the grid map it corresponds to the grid map point closest to the
 * queried point (in terms of Euclidean distance). Queried point has
 * coordinates (x,y) for at which the interpolation is requested.
 * @param[in]  gridMap - grid map with the data
 * @param[in]  queriedPosition - position for which the interpolated data is requested
 * @param[out] index - indices of the middle knot for the interpolation
 * @return - true if success
 */
bool getIndicesOfMiddleKnot(
  const GridMap & gridMap, const Position & queriedPosition,
  Index * index);

/*
 * Coordinates used for interpolation need to be shifted and scaled,
 * since the original algorithm operates around the origin and with unit
 * resolution
 * @param[in]  gridMap - grid map with the data
 * @param[in]  queriedPosition - position for which the interpolation is requested
 * @param[out] position - normalized coordinates of the point for which the interpolation is requested
 * @return - true if success
 */
bool getNormalizedCoordinates(
  const GridMap & gridMap, const Position & queriedPosition,
  Position * position);

/*
 * Queries the grid map for function values at the coordiantes which are neccesary for
 * performing the interpolation. The right function values are then assembled
 * in a matrix.
 * @param[in]  gridMap - grid map with the data
 * @param[in]  layer - name of the layer that we are interpolating
 * @param[in]  queriedPosition - position for which the interpolation is requested
 * @param[out] data - 4x4 matrix with 16 function values used for interpolation, see
 *           R. Keys (1981). "Cubic convolution interpolation for digital image processing".
 *           IEEE Transactions on Acoustics, Speech, and Signal Processing. 29 (6): 1153–1160.
 *           for the details.
 * @return - true if success
 */
bool assembleFunctionValueMatrix(
  const GridMap & gridMap, const std::string & layer,
  const Position & queriedPosition, FunctionValueMatrix * data);

/*
 * Performs convolution in 1D. the function requires 4 function values
 * to compute the convolution. The result is interpolated data in 1D.
 * @param[in]  t - normalized coordinate (x or y)
 * @param[in]  functionValues - vector of 4 function values neccessary to perform
 *                            interpolation in 1 dimension.
 * @return - interpolated value at normalized coordinate t
 */
double convolve1D(double t, const Eigen::Vector4d & functionValues);

/*
 * Performs convolution in 1D. the function requires 4 function values
 * to compute the convolution. The result is interpolated data in 1D.
 * @param[in]  gridMap - grid map with discrete function values
 * @param[in]  layer - name of the layer for which we want to perform interpolation
 * @param[in]  queriedPosition - position for which the interpolation is requested
 * @param[out] interpolatedValue - interpolated value at queried point
 * @return - true if success
 */
bool evaluateBicubicConvolutionInterpolation(
  const GridMap & gridMap, const std::string & layer,
  const Position & queriedPosition,
  double * interpolatedValue);

}  // namespace bicubic_conv

namespace bicubic
{

/*
 * Enum for the derivatives direction
 * to perform interpolation one needs
 * derivatives w.r.t. to x and y dimension.
 */
enum class Dim2D: int
{
  X,
  Y
};

/*!
 * Matrix for cubic interpolation. Taken from:
 * https://en.wikipedia.org/wiki/Bicubic_interpolation
 */
static const Eigen::Matrix4d bicubicInterpolationMatrix {
  (Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    -3.0, 3.0, -2.0, -1.0,
    2.0, -2.0, 1.0, 1.0).finished()};

/*
 * Data matrix that can hold function values
 * these can be either function values at requested
 * positions or their derivatives.
 */
struct DataMatrix
{
  double topLeft_ = 0.0;
  double topRight_ = 0.0;
  double bottomLeft_ = 0.0;
  double bottomRight_ = 0.0;
};

/*
 * Interpolation is performed on a unit square.
 * Hence we need to compute 4 corners of that unit square,
 * and find their indices in the grid map. IndicesMatrix
 * is a container that stores those indices. Each index
 * contains two numbers (row number, column number) in the
 * grid map.
 */
struct IndicesMatrix
{
  Index topLeft_ {0, 0};
  Index topRight_ {0, 0};
  Index bottomLeft_ {0, 0};
  Index bottomRight_ {0, 0};
};

/*
 * Makes sure that all indices in side the
 * data structure IndicesMatrix are within the
 * range of the grid map.
 * @param[in] gridMap - input grid map with discrete function values
 * @param[in/out] indices - indices that are bound to range, i.e.
 *                          rows and columns are with ranges
 */
void bindIndicesToRange(const GridMap & gridMap, IndicesMatrix * indices);

/*
 * Performs bicubic interpolation at requested position.
 * @param[in]  gridMap - grid map with discrete function values
 * @param[in]  layer - name of the layer for which we want to perform interpolation
 * @param[in]  queriedPosition - position for which the interpolation is requested
 * @param[out] interpolatedValue - interpolated value at queried point
 * @return - true if success
 */
bool evaluateBicubicInterpolation(
  const GridMap & gridMap, const std::string & layer,
  const Position & queriedPosition, double * interpolatedValue);

/*
 * Deduces which points in the grid map close a unit square around the
 * queried point and returns their indices (row and column number)
 * @param[in]  gridMap - grid map with discrete function values
 * @param[in]  queriedPosition - position for which the interpolation is requested
 * @param[out] indicesMatrix - data structure with indices forming a unit square
 *                            around the queried point
 * @return - true if success
 */
bool getUnitSquareCornerIndices(
  const GridMap & gridMap, const Position & queriedPosition,
  IndicesMatrix * indicesMatrix);

/*
 * Get index (row and column number) of a point in grid map, which
 * is closest to the queried position.
 * @param[in]  gridMap - grid map with discrete function values
 * @param[in]  queriedPosition - position for which the interpolation is requested
 * @param[out] index - indices of the closest point in grid_map
 * @return - true if success
 */
bool getClosestPointIndices(
  const GridMap & gridMap, const Position & queriedPosition,
  Index * index);

/*
 * Retrieve function values from the grid map at requested indices.
 * @param[in]  layerData - layer of a grid map with function values
 * @param[in]  indices - indices (row and column numbers) for which function values are requested
 * @param[out] data - requested function values
 * @return - true if success
 */
bool getFunctionValues(const Matrix & layerData, const IndicesMatrix & indices, DataMatrix * data);

/*
 * Retrieve function derivative values from the grid map at requested indices. Function
 * derivatives are approximated using central difference.
 * @param[in]  layerData - layer of a grid map with function values
 * @param[in]  indices - indices (row and column numbers) for which function derivative
 *                       values are requested
 * @param[in]  dim - dimension along which we want to evaluate partial derivatives (X or Y)
 * @param[in]  resolution - resolution of the grid map
 * @param[out] derivatives - values of derivatives at requested indices
 * @return - true if success
 */
bool getFirstOrderDerivatives(
  const Matrix & layerData, const IndicesMatrix & indices, Dim2D dim,
  double resolution, DataMatrix * derivatives);

/*
 * Retrieve second order function derivative values from the grid map at requested indices.
 * Function derivatives are approximated using central difference. We compute partial derivative
 * w.r.t to one coordinate and then the other. Note that the order of differentiation
 * does not matter.
 * @param[in]  layerData - layer of a grid map with function values
 * @param[in]  indices - indices (row and column numbers) for which function derivative
 *                       values are requested
 * @param[in]  resolution - resolution of the grid map
 * @param[out] derivatives - values of second order mixed derivatives at requested indices
 * @return - true if success
 */
bool getMixedSecondOrderDerivatives(
  const Matrix & layerData, const IndicesMatrix & indices,
  double resolution, DataMatrix * derivatives);

/*
 * First order derivative for a specific point determined by index.
 * Approximated by central difference.
 * See https://www.mathematik.uni-dortmund.de/~kuzmin/cfdintro/lecture4.pdf
 * for details
 * @param[in]  layerData - layer of a grid map with function values
 * @param[in]  index - index (row and column number) for which function derivative
 *                       value is requested
 * @param[in]  dim - dimension along which we want to evaluate partial derivative (X or Y)
 * @param[in]  resolution - resolution of the grid map
 * @return - value of the derivative at requested index
 */
double firstOrderDerivativeAt(
  const Matrix & layerData, const Index & index, Dim2D dim,
  double resolution);

/*
 * Second order mixed derivative for a specific point determined by index.
 * See https://www.mathematik.uni-dortmund.de/~kuzmin/cfdintro/lecture4.pdf
 * for details
 * @param[in]  layerData - layer of a grid map with function values
 * @param[in]  index - index (row and column number) for which function derivative
 *                       value is requested
 * @param[in]  resolution - resolution of the grid map
 * @return - value of the second order mixed derivative at requested index
 */
double mixedSecondOrderDerivativeAt(
  const Matrix & layerData, const Index & index,
  double resolution);

/*
 * Evaluate polynomial at requested coordinates. the function will compute the polynomial
 * coefficients and then evaluate it. See
 * https://en.wikipedia.org/wiki/Bicubic_interpolation
 * for details.
 * @param[in]  functionValues - function values and derivatives required to
 *                              compute polynomial coefficients
 * @param[in]  tx - normalized x coordinate for which the interpolation should be computed
 * @param[in]  ty - normalized y coordinate for which the interpolation should be computed
 * @return - interpolated value at requested normalized coordinates.
 */
double evaluatePolynomial(const FunctionValueMatrix & functionValues, double tx, double ty);

/*
 * Assemble function value matrix from small submatrices containing function values
 * or derivative values at the corners of the unit square.
 * See https://en.wikipedia.org/wiki/Bicubic_interpolation for details.
 *
 * @param[in]  f - Function values at the corners of the unit square
 * @param[in]  dfx - Partial derivative w.r.t to x at the corners of the unit square
 * @param[in]  dfy - Partial derivative w.r.t to y at the corners of the unit square
 * @param[in]  ddfxy - Second order partial derivative w.r.t to x and y at the corners of the unit square
 * @param[out]  functionValues - function values and derivatives required to
 *                              compute polynomial coefficients
 */
void assembleFunctionValueMatrix(
  const DataMatrix & f, const DataMatrix & dfx, const DataMatrix & dfy,
  const DataMatrix & ddfxy, FunctionValueMatrix * functionValues);

/*
 * Coordinates used for interpolation need to be shifter and scaled,
 * since the original algorithm operates on a unit square around the origin.
 * @param[in]  gridMap - grid map with the data
 * @param[in]  originIndex - index of a bottom left corner if the unit square in the grid map
 *                            this corner is the origin for the normalized coordinates.
 * @param[in]  queriedPosition - position for which the interpolation is requested
 * @param[out] position - normalized coordinates of the point for which the interpolation is requested
 * @return - true if success
 */
bool computeNormalizedCoordinates(
  const GridMap & gridMap, const Index & originIndex,
  const Position & queriedPosition, Position * normalizedCoordinates);

}  // namespace bicubic

}  // namespace grid_map
#endif  // GRID_MAP_CORE__CUBICINTERPOLATION_HPP_
