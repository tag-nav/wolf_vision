#include "utils_gtest.h"
#include "base/converter.h"

using namespace std;
using namespace wolf;

TEST(Converter, ParseToVector)
{
    string v1 = "[3,4,5,6,7,8,9,10,11]";
    vector<int> v = converter<vector<int>>::convert(v1);
    ASSERT_EQ(v.size(),9);
    ASSERT_EQ(v[0],3);
    ASSERT_EQ(v[1],4);
    ASSERT_EQ(v[2],5);
    ASSERT_EQ(v[3],6);
    ASSERT_EQ(v[4],7);
    ASSERT_EQ(v[5],8);
    ASSERT_EQ(v[6],9);
    ASSERT_EQ(v[7],10);
    ASSERT_EQ(v[8],11);
    vector<string> vs {"a","b","c"};
    ASSERT_EQ(converter<string>::convert(vs), "[a,b,c]");

}
TEST(Converter, ParseToEigenMatrix)
{
    string v1 = "[[3,3],3,4,5,6,7,8,9,10,11]";
    auto v = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, 3, 5, 5>();
    EXPECT_NO_THROW(([=, &v]{v = converter<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, 3, 5, 5>>::convert(v1);}()));
    EXPECT_EQ(v.size(),9);
    string v2 = "[[3,3],3,4,5,6,7,8,9,10,11,12]";
    EXPECT_THROW(([=, &v]{v = converter<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, 3, 5, 5>>::convert(v2);}()), std::runtime_error);
    string v3 = "[[3],3,4,5,6,7,8,9,10,11]";
    EXPECT_THROW(([=, &v]{v = converter<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, 3, 5, 5>>::convert(v3);}()), std::runtime_error);
}
TEST(Converter, ParseToMap)
{
    string str = "[{x:1},{y:[1,23,3]},{z:3}]";
    EXPECT_THROW(([=]{auto a = converter<std::map<std::string, double>>::convert(str);}()), std::invalid_argument);
    string str2 = "[{x:[1]},{y:[1,23,3]},{z:[3]}]";
    EXPECT_NO_THROW(([=]{auto a = converter<std::map<std::string, std::vector<int>>>::convert(str2);}()));
    map<string, vector<int>> m = {{"x",vector<int>{1,2}}, {"y",vector<int>{}}, {"z",vector<int>{3}}};
    ASSERT_EQ(converter<string>::convert(m), "[{x:[1,2]},{y:[]},{z:[3]}]");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
