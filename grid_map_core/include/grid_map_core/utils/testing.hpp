#pragma once

#include <gtest/gtest.h>
#include <Eigen/Core>

#define ASSERT_MATRICES_EQ_WITH_NAN(first, second) assertMatrixesEqualWithNan((first), #first, (second), #second, __LINE__)
static void assertMatrixesEqualWithNan(Eigen::Ref<const Eigen::MatrixXf> first, std::string firstName,
                                       Eigen::Ref<const Eigen::MatrixXf> second, std::string secondName, int line) {
  ASSERT_EQ(first.rows(), second.rows());
  ASSERT_EQ(first.cols(), second.cols());

  bool matricesAreEqual = true;
  for (Eigen::Index row = 0; row < first.rows() && matricesAreEqual; ++row) {
    for (Eigen::Index col = 0; col < first.cols() && matricesAreEqual; ++col) {
      bool ifRealThenValid = first.block<1, 1>(row, col).isApprox(second.block<1, 1>(row, col));
      bool bothNaN = std::isnan(first(row, col)) && std::isnan(second(row, col));
      if (ifRealThenValid || bothNaN) {
        continue;
      } else {
        matricesAreEqual = false;
      }
    }
  }

  Eigen::IOFormat compactFormat(2, 0, ",", "\n", "[", "]");
  ASSERT_TRUE(matricesAreEqual)                                       // NO LINT
      << "L. " << std::to_string(line) << ": Matrices are not equal"  // NO LINT
      << "\n"                                                         // NO LINT
      << firstName << "\n"                                            // NO LINT
      << first.format(compactFormat) << "\n"                          // NO LINT
      << secondName << "\n"                                           // NO LINT
      << second.format(compactFormat) << "\n";                        // NO LINT
}
