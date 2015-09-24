/*
 * EigenMatrixBaseAddonsTest.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/grid_map_core.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

#include <iostream>

using namespace std;
using namespace Eigen;

TEST(EigenMatrixBaseAddons, numberOfFinites)
{
  Eigen::Matrix3f matrix(Eigen::Matrix3f::Ones());
  matrix(0, 0) = NAN;
  matrix(1, 0) = NAN;
  EXPECT_EQ(7, matrix.numberOfFinites());

  Matrix<double, 13, 10> matrix2;
  matrix2.setOnes();
  EXPECT_EQ(matrix2.rows() * matrix2.cols(), matrix2.numberOfFinites());

  Matrix<double, 13, 10> matrix3;
  matrix3.setConstant(NAN);
  matrix3.col(3).setConstant(0.0);
  EXPECT_EQ(matrix3.rows(), matrix3.numberOfFinites());
}

TEST(EigenMatrixBaseAddons, sumOfFinites)
{
  Matrix<double, 7, 18> matrix;
  matrix.setRandom();
  EXPECT_NEAR(matrix.sum(), matrix.sumOfFinites(), 1e-10);
  double finiteSum = matrix.sum() - matrix(0, 0) - matrix(1, 2) - matrix(3, 6) - matrix(6, 12);
  matrix(0, 0) = NAN;
  matrix(1, 2) = NAN;
  matrix(3, 6) = NAN;
  matrix(6, 12) = NAN;
  EXPECT_NEAR(finiteSum, matrix.sumOfFinites(), 1e-10);
  matrix.setConstant(NAN);
  EXPECT_TRUE(std::isnan(matrix.sumOfFinites()));
  matrix(5, 7) = 1.0;
  EXPECT_NEAR(1.0, matrix.sumOfFinites(), 1e-10);
}

TEST(EigenMatrixBaseAddons, meanOfFinites)
{
  Eigen::Matrix3f matrix(Eigen::Matrix3f::Ones());
  matrix(0, 0) = NAN;
  matrix(1, 1) = NAN;
  EXPECT_DOUBLE_EQ(1.0, matrix.meanOfFinites());

  Matrix<double, 13, 10> matrix2;
  matrix2.setRandom();
  EXPECT_NEAR(matrix2.mean(), matrix2.meanOfFinites(), 1e-10);
}
