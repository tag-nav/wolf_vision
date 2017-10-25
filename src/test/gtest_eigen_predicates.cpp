#include "utils_gtest.h"

#include "../eigen_predicates.h"

TEST(TestEigenPredicates, TestEigenDynPredZero)
{
  Eigen::MatrixXs A, B, C;

  A = Eigen::MatrixXs::Zero(4,4);
  B = Eigen::MatrixXs::Random(4,4);
  C = Eigen::MatrixXs::Ones(4,4) * (wolf::Constants::EPS/2.);

  EXPECT_TRUE(wolf::pred_zero(A,  wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_zero(B, wolf::Constants::EPS));
  EXPECT_TRUE(wolf::pred_zero(C,  wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenFixPredZero)
{
  Eigen::MatrixXs A, B, C;

  A = Eigen::Matrix4s::Zero();
  B = Eigen::Matrix4s::Random();
  C = Eigen::Matrix4s::Ones() * (wolf::Constants::EPS/2.);

  EXPECT_TRUE(wolf::pred_zero(A,  wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_zero(B, wolf::Constants::EPS));
  EXPECT_TRUE(wolf::pred_zero(C,  wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenDynPredDiffZero)
{
  Eigen::MatrixXs A, B, C;

  A = Eigen::MatrixXs::Random(4,4);
  B = Eigen::MatrixXs::Random(4,4);
  C = A;

  EXPECT_TRUE(wolf::pred_diff_zero(A, C, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_diff_zero(A, B, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenFixPredDiffZero)
{
  Eigen::MatrixXs A, B, C;

  A = Eigen::Matrix4s::Random();
  B = Eigen::Matrix4s::Random();
  C = A;

  EXPECT_TRUE(wolf::pred_diff_zero(A, C, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_diff_zero(A, B, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenDynPredDiffNorm)
{
  Eigen::MatrixXs A, B, C;

  A = Eigen::MatrixXs::Random(4,4);
  B = Eigen::MatrixXs::Random(4,4);
  C = A;

  EXPECT_TRUE(wolf::pred_diff_norm_zero(A, C, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_diff_norm_zero(A, B, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenFixPredDiffNorm)
{
  Eigen::MatrixXs A, B, C;

  A = Eigen::Matrix4s::Random();
  B = Eigen::Matrix4s::Random();
  C = A;

  EXPECT_TRUE(wolf::pred_diff_norm_zero(A, C, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_diff_norm_zero(A, B, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenDynPredIsApprox)
{
  Eigen::MatrixXs A, B, C;

  A = Eigen::MatrixXs::Random(4,4);
  B = Eigen::MatrixXs::Random(4,4);
  C = A;

  EXPECT_TRUE(wolf::pred_is_approx(A, C, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_is_approx(A, B, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenFixPredIsApprox)
{
  Eigen::MatrixXs A, B, C;

  A = Eigen::Matrix4s::Random();
  B = Eigen::Matrix4s::Random();
  C = A;

  EXPECT_TRUE(wolf::pred_is_approx(A, C, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_is_approx(A, B, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenPredQuatIsApprox)
{
  Eigen::Quaternions A, B, C;

  /// @todo which version of Eigen provides this ?
//  A = Eigen::Quaternions::UnitRandom();

  A.coeffs() = Eigen::Vector4s::Random().normalized();
  B.coeffs() = Eigen::Vector4s::Random().normalized();
  C = A;

  EXPECT_TRUE(wolf::pred_quat_is_approx(A, C, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_quat_is_approx(A, B, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenPredQuatIsIdentity)
{
  Eigen::Quaternions A, B;

  A = Eigen::Quaternions::Identity();
  B.coeffs() = Eigen::Vector4s::Random().normalized();

  EXPECT_TRUE(wolf::pred_quat_identity(A, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_quat_identity(B, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenPredAngleIsApprox)
{
  wolf::Scalar a = M_PI;
  wolf::Scalar b = -M_PI;
  wolf::Scalar c = 0;

  EXPECT_TRUE(wolf::pred_angle_is_approx(a, b, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_angle_is_approx(a, c, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

TEST(TestEigenPredicates, TestEigenPredAngleIsZero)
{
  wolf::Scalar a = 0;
  wolf::Scalar b = M_PI;
  wolf::Scalar c = 2.*M_PI;

  EXPECT_TRUE(wolf::pred_angle_zero(a, wolf::Constants::EPS));
  EXPECT_FALSE(wolf::pred_angle_zero(b, wolf::Constants::EPS));
  EXPECT_TRUE(wolf::pred_angle_zero(c, wolf::Constants::EPS));

  PRINT_TEST_FINISHED;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
