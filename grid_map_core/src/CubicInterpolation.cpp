/*
 * CubicInterpolation.cpp
 *
 *  Created on: Jan 21, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <string>

#include "grid_map_core/CubicInterpolation.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/GridMapMath.hpp"

namespace grid_map
{

unsigned int bindIndexToRange(int idReq, unsigned int nElem)
{
  if (idReq < 0) {
    return 0;
  }
  if ((unsigned)idReq >= nElem) {
    return static_cast<unsigned int>(nElem - 1);
  }
  return static_cast<unsigned int>(idReq);
}

double getLayerValue(const Matrix & layerMat, int rowReq, int colReq)
{
  const auto numCol = layerMat.cols();
  const auto numRow = layerMat.rows();
  const unsigned int iBoundToRange = bindIndexToRange(rowReq, numRow);
  const unsigned int jBoundToRange = bindIndexToRange(colReq, numCol);
  return layerMat(iBoundToRange, jBoundToRange);
}


/**
 * BICUBIC CONVLUTION INTERPOLATION ALGORITHM
 * also known as piecewise bicubic interpolation,
 * it does not ensure continuity of the first derivatives.
 * see:
 * https://en.wikipedia.org/wiki/Bicubic_interpolation
 * https://web.archive.org/web/20051024202307/http://www.geovista.psu.edu/sites/geocomp99/Gc99/082/gc_082.htm
 */

namespace bicubic_conv
{

bool evaluateBicubicConvolutionInterpolation(
  const GridMap & gridMap, const std::string & layer,
  const Position & queriedPosition,
  double * interpolatedValue)
{
  FunctionValueMatrix functionValues;
  if (!assembleFunctionValueMatrix(gridMap, layer, queriedPosition, &functionValues)) {
    return false;
  }

  Position normalizedCoordinate;
  if (!getNormalizedCoordinates(gridMap, queriedPosition, &normalizedCoordinate)) {
    return false;
  }

  const double tx = normalizedCoordinate.x();
  const double ty = normalizedCoordinate.y();

  // bm1 stands for b minus one, i.e. index decreased by one
  // b2 stands for b plus 2, i.e. index increased by two
  const double bm1 = convolve1D(tx, functionValues.row(0));
  const double b0 = convolve1D(tx, functionValues.row(1));
  const double b1 = convolve1D(tx, functionValues.row(2));
  const double b2 = convolve1D(tx, functionValues.row(3));
  const Eigen::Vector4d vectorBs(bm1, b0, b1, b2);
  *interpolatedValue = convolve1D(ty, vectorBs);
  return true;
}

double convolve1D(double t, const Eigen::Vector4d & functionValues)
{
  const Eigen::Vector4d tVector(1.0, t, t * t, t * t * t);
  const Eigen::Vector4d temp = cubicInterpolationConvolutionMatrix *
    functionValues;
  const double retVal = 0.5 * tVector.transpose() * temp;
  return retVal;
}

bool assembleFunctionValueMatrix(
  const GridMap & gridMap, const std::string & layer,
  const Position & queriedPosition, FunctionValueMatrix * data)
{
  Index middleKnotIndex;
  if (!getIndicesOfMiddleKnot(gridMap, queriedPosition, &middleKnotIndex)) {
    return false;
  }

  const Matrix & layerMatrix = gridMap.get(layer);
  auto f = [&layerMatrix](int rowReq, int colReq) {
      double retVal = getLayerValue(layerMatrix, rowReq, colReq);
      return retVal;
    };

  const unsigned int i = middleKnotIndex.x();
  const unsigned int j = middleKnotIndex.y();
  /*
   * Notation taken from: https://en.wikipedia.org/wiki/Bicubic_interpolation
   * increasing f's indices is flipped w.r.t. to the above since in the article
   * they use a coordinate frame centered around (i,j). Therefore:
   * f(i+1,j-1) in their notation corresponds to f(i-1,j+1) in ours. This is
   * because our coordinate frame sits in the top left corner, see
   * https://github.com/ANYbotics/grid_map
   */
  *data << f(i + 1, j + 1), f(i, j + 1), f(i - 1, j + 1), f(i - 2, j + 1), f(i + 1, j), f(i, j), f(
    i - 1, j), f(i - 2, j), f(i + 1, j - 1), f(i, j - 1), f(i - 1, j - 1), f(i - 2, j - 1), f(
    i + 1, j - 2), f(i, j - 2), f(i - 1, j - 2), f(i - 2, j - 2);

  return true;
}

bool getNormalizedCoordinates(
  const GridMap & gridMap, const Position & queriedPosition,
  Position * position)
{
  Index index;
  if (!getIndicesOfMiddleKnot(gridMap, queriedPosition, &index)) {
    return false;
  }

  Position middleKnot;
  if (!gridMap.getPosition(index, middleKnot)) {
    return false;
  }

  position->x() = (queriedPosition.x() - middleKnot.x()) / gridMap.getResolution();
  position->y() = (queriedPosition.y() - middleKnot.y()) / gridMap.getResolution();

  return true;
}

bool getIndicesOfMiddleKnot(
  const GridMap & gridMap, const Position & queriedPosition,
  Index * index)
{
  if (!gridMap.getIndex(queriedPosition, *index)) {
    return false;
  }
  return true;
}

}  // namespace bicubic_conv

/**
 * BICUBIC INTERPOLATION ALGORITHM
 * it does ensure continuity of the first derivatives.
 * More expensive to compute than bicubic convolution interpolation
 * see:
 * https://en.wikipedia.org/wiki/Bicubic_interpolation
 * https://web.archive.org/web/20051024202307/http://www.geovista.psu.edu/sites/geocomp99/Gc99/082/gc_082.htm
 */

namespace bicubic
{

bool evaluateBicubicInterpolation(
  const GridMap & gridMap, const std::string & layer,
  const Position & queriedPosition, double * interpolatedValue)
{
  const Matrix & layerMat = gridMap.get(layer);
  const double resolution = gridMap.getResolution();

  // get indices of data points needed for interpolation
  IndicesMatrix unitSquareCornerIndices;
  if (!getUnitSquareCornerIndices(gridMap, queriedPosition, &unitSquareCornerIndices)) {
    return false;
  }

  // get function values
  DataMatrix f;
  if (!getFunctionValues(layerMat, unitSquareCornerIndices, &f)) {
    return false;
  }

  // get the first derivatives in x
  DataMatrix dfx;
  if (!getFirstOrderDerivatives(layerMat, unitSquareCornerIndices, Dim2D::X, resolution, &dfx)) {
    return false;
  }

  // the first derivatives in y
  DataMatrix dfy;
  if (!getFirstOrderDerivatives(layerMat, unitSquareCornerIndices, Dim2D::Y, resolution, &dfy)) {
    return false;
  }
  // mixed derivatives in x y
  DataMatrix ddfxy;
  if (!getMixedSecondOrderDerivatives(layerMat, unitSquareCornerIndices, resolution, &ddfxy)) {
    return false;
  }

  // assemble function value matrix matrix
  FunctionValueMatrix functionValues;
  assembleFunctionValueMatrix(f, dfx, dfy, ddfxy, &functionValues);

  // get normalized coordiantes
  Position normalizedCoordinates;
  if (!computeNormalizedCoordinates(
      gridMap, unitSquareCornerIndices.bottomLeft_, queriedPosition,
      &normalizedCoordinates))
  {
    return false;
  }

  // evaluate polynomial
  *interpolatedValue = evaluatePolynomial(
    functionValues, normalizedCoordinates.x(),
    normalizedCoordinates.y());

  return true;
}

bool getUnitSquareCornerIndices(
  const GridMap & gridMap, const Position & queriedPosition,
  IndicesMatrix * indicesMatrix)
{
  Index closestPointId;
  if (!getClosestPointIndices(gridMap, queriedPosition, &closestPointId)) {
    return false;
  }

  Position closestPoint;
  if (!gridMap.getPosition(closestPointId, closestPoint)) {
    return false;
  }

  const int idx0 = closestPointId.x();
  const int idy0 = closestPointId.y();
  const double x0 = closestPoint.x();
  const double y0 = closestPoint.y();
  const double x = queriedPosition.x();
  const double y = queriedPosition.y();

  if (x > x0) {  // first or fourth quadrant
    if (y > y0) {  // first quadrant
      indicesMatrix->topLeft_ = Index(idx0, idy0 - 1);
      indicesMatrix->topRight_ = Index(idx0 - 1, idy0 - 1);
      indicesMatrix->bottomLeft_ = Index(idx0, idy0);
      indicesMatrix->bottomRight_ = Index(idx0 - 1, idy0);
    } else {  // fourth quadrant
      indicesMatrix->topLeft_ = Index(idx0, idy0);
      indicesMatrix->topRight_ = Index(idx0 - 1, idy0);
      indicesMatrix->bottomLeft_ = Index(idx0, idy0 + 1);
      indicesMatrix->bottomRight_ = Index(idx0 - 1, idy0 + 1);
    }
  } else {  // second or third quadrant
    if (y > y0) {  // second quadrant
      indicesMatrix->topLeft_ = Index(idx0 + 1, idy0 - 1);
      indicesMatrix->topRight_ = Index(idx0, idy0 - 1);
      indicesMatrix->bottomLeft_ = Index(idx0 + 1, idy0);
      indicesMatrix->bottomRight_ = Index(idx0, idy0);
    } else {  // third quadrant
      indicesMatrix->topLeft_ = Index(idx0 + 1, idy0);
      indicesMatrix->topRight_ = Index(idx0, idy0);
      indicesMatrix->bottomLeft_ = Index(idx0 + 1, idy0 + 1);
      indicesMatrix->bottomRight_ = Index(idx0, idy0 + 1);
    }
  }

  bindIndicesToRange(gridMap, indicesMatrix);

  return true;
}

bool getClosestPointIndices(
  const GridMap & gridMap, const Position & queriedPosition,
  Index * index)
{
  if (!gridMap.getIndex(queriedPosition, *index)) {
    return false;
  }
  return true;
}

bool computeNormalizedCoordinates(
  const GridMap & gridMap, const Index & originIndex,
  const Position & queriedPosition, Position * normalizedCoordinates)
{
  Position origin;
  if (!gridMap.getPosition(originIndex, origin)) {
    return false;
  }

  normalizedCoordinates->x() = (queriedPosition.x() - origin.x()) / gridMap.getResolution();
  normalizedCoordinates->y() = (queriedPosition.y() - origin.y()) / gridMap.getResolution();

  return true;
}

bool getFunctionValues(const Matrix & layerData, const IndicesMatrix & indices, DataMatrix * data)
{
  data->topLeft_ = layerData(indices.topLeft_.x(), indices.topLeft_.y());
  data->topRight_ = layerData(indices.topRight_.x(), indices.topRight_.y());
  data->bottomLeft_ = layerData(indices.bottomLeft_.x(), indices.bottomLeft_.y());
  data->bottomRight_ = layerData(indices.bottomRight_.x(), indices.bottomRight_.y());
  return true;
}

void bindIndicesToRange(const GridMap & gridMap, IndicesMatrix * indices)
{
  const int numCol = gridMap.getSize().y();
  const int numRow = gridMap.getSize().x();

  // top left
  {
    const unsigned int iBoundToRange = bindIndexToRange(indices->topLeft_.x(), numRow);
    const unsigned int jBoundToRange = bindIndexToRange(indices->topLeft_.y(), numCol);
    indices->topLeft_ = Index(iBoundToRange, jBoundToRange);
  }

  // top right
  {
    const unsigned int iBoundToRange = bindIndexToRange(indices->topRight_.x(), numRow);
    const unsigned int jBoundToRange = bindIndexToRange(indices->topRight_.y(), numCol);
    indices->topRight_ = Index(iBoundToRange, jBoundToRange);
  }

  // bottom left
  {
    const unsigned int iBoundToRange = bindIndexToRange(indices->bottomLeft_.x(), numRow);
    const unsigned int jBoundToRange = bindIndexToRange(indices->bottomLeft_.y(), numCol);
    indices->bottomLeft_ = Index(iBoundToRange, jBoundToRange);
  }

  // bottom right
  {
    const unsigned int iBoundToRange = bindIndexToRange(indices->bottomRight_.x(), numRow);
    const unsigned int jBoundToRange = bindIndexToRange(indices->bottomRight_.y(), numCol);
    indices->bottomRight_ = Index(iBoundToRange, jBoundToRange);
  }
}

bool getFirstOrderDerivatives(
  const Matrix & layerData, const IndicesMatrix & indices, Dim2D dim,
  double resolution, DataMatrix * derivatives)
{
  derivatives->topLeft_ = firstOrderDerivativeAt(layerData, indices.topLeft_, dim, resolution);
  derivatives->topRight_ = firstOrderDerivativeAt(layerData, indices.topRight_, dim, resolution);
  derivatives->bottomLeft_ = firstOrderDerivativeAt(
    layerData, indices.bottomLeft_, dim,
    resolution);
  derivatives->bottomRight_ = firstOrderDerivativeAt(
    layerData, indices.bottomRight_, dim,
    resolution);
  return true;
}

double firstOrderDerivativeAt(
  const Matrix & layerData, const Index & index, Dim2D dim,
  double resolution)
{
  const int numCol = layerData.cols();
  const int numRow = layerData.rows();

  double left, right;
  switch (dim) {
    case Dim2D::X: {
        left = layerData(bindIndexToRange(index.x() + 1, numRow), index.y());
        right = layerData(bindIndexToRange(index.x() - 1, numRow), index.y());
        break;
      }
    case Dim2D::Y: {
        left = layerData(index.x(), bindIndexToRange(index.y() + 1, numCol));
        right = layerData(index.x(), bindIndexToRange(index.y() - 1, numCol));
        break;
      }
    default: {
        throw std::runtime_error("Unknown derivative direction");
      }
  }

  const double perturbation = resolution;
  // central difference approximation
  // we need to multiply with resolution since we are
  // operating in scaled coordinates
  return (right - left) / (2.0 * perturbation) * resolution;
}

double mixedSecondOrderDerivativeAt(
  const Matrix & layerData, const Index & index,
  double resolution)
{
  /*
   * no need for dimensions since the we have to differentiate w.r.t. x and y
   * the order doesn't matter. Derivative values are the same.
   * Taken from https://www.mathematik.uni-dortmund.de/~kuzmin/cfdintro/lecture4.pdf
   */

  const int numCol = layerData.cols();
  const int numRow = layerData.rows();

  const double f11 = layerData(
    bindIndexToRange(index.x() - 1, numRow),
    bindIndexToRange(index.y() - 1, numCol));
  const double f1m1 = layerData(
    bindIndexToRange(index.x() - 1, numRow),
    bindIndexToRange(index.y() + 1, numCol));
  const double fm11 = layerData(
    bindIndexToRange(index.x() + 1, numRow),
    bindIndexToRange(index.y() - 1, numCol));
  const double fm1m1 = layerData(
    bindIndexToRange(index.x() + 1, numRow),
    bindIndexToRange(index.y() + 1, numCol));

  const double perturbation = resolution;
  // central difference approximation
  // we need to multiply with resolution^2 since we are
  // operating in scaled coordinates. Second derivative scales
  // with the square of the resolution
  return (f11 - f1m1 - fm11 + fm1m1) / (4.0 * perturbation * perturbation) * resolution *
         resolution;
}

bool getMixedSecondOrderDerivatives(
  const Matrix & layerData, const IndicesMatrix & indices,
  double resolution, DataMatrix * derivatives)
{
  derivatives->topLeft_ = mixedSecondOrderDerivativeAt(layerData, indices.topLeft_, resolution);
  derivatives->topRight_ = mixedSecondOrderDerivativeAt(layerData, indices.topRight_, resolution);
  derivatives->bottomLeft_ = mixedSecondOrderDerivativeAt(
    layerData, indices.bottomLeft_,
    resolution);
  derivatives->bottomRight_ = mixedSecondOrderDerivativeAt(
    layerData, indices.bottomRight_,
    resolution);
  return true;
}

double evaluatePolynomial(const FunctionValueMatrix & functionValues, double tx, double ty)
{
  const Eigen::Vector4d xVector(1, tx, tx * tx, tx * tx * tx);
  const Eigen::Vector4d yVector(1, ty, ty * ty, ty * ty * ty);
  const Eigen::Matrix4d tempMat = functionValues *
    bicubicInterpolationMatrix.transpose();
  const Eigen::Matrix4d polynomialCoeffMatrix = bicubicInterpolationMatrix * tempMat;
  const Eigen::Vector4d tempVec = polynomialCoeffMatrix * yVector;
  return xVector.transpose() * tempVec;
}

void assembleFunctionValueMatrix(
  const DataMatrix & f, const DataMatrix & dfx, const DataMatrix & dfy,
  const DataMatrix & ddfxy, FunctionValueMatrix * functionValues)
{
  auto toEigenMatrix = [](const DataMatrix & d) -> Eigen::Matrix2d {
      Eigen::Matrix2d e;
      e(0, 0) = d.bottomLeft_;
      e(1, 0) = d.bottomRight_;
      e(0, 1) = d.topLeft_;
      e(1, 1) = d.topRight_;
      return e;
    };

  functionValues->block<2, 2>(0, 0) = toEigenMatrix(f);
  functionValues->block<2, 2>(2, 2) = toEigenMatrix(ddfxy);
  functionValues->block<2, 2>(0, 2) = toEigenMatrix(dfy);
  functionValues->block<2, 2>(2, 0) = toEigenMatrix(dfx);
}

}  // namespace bicubic

}  // namespace grid_map
