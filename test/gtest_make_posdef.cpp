#include "utils_gtest.h"
#include "base/wolf.h"

using namespace Eigen;
using namespace wolf;

TEST(MakePosDefTest, OkTest)
{
    MatrixXs M = MatrixXs::Identity(3,3);

    EXPECT_TRUE(isCovariance(M));
    EXPECT_FALSE(makePosDef(M));
}

TEST(MakePosDefTest, FixTest)
{
    MatrixXs M = MatrixXs::Zero(3,3);

    EXPECT_FALSE(isCovariance(M));
    EXPECT_TRUE(makePosDef(M));
    EXPECT_TRUE(isCovariance(M));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
