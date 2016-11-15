/**
 * \file utils_gtest.h
 * \brief Some utils for gtest
 * \author Jeremie Deray
 *  Created on: 26/09/2016
 */

#ifndef WOLF_UTILS_GTEST_H
#define WOLF_UTILS_GTEST_H

#include <gtest/gtest.h>


// Macros for testing equalities and inequalities.
//
//    * {ASSERT|EXPECT}_EQ(expected, actual): Tests that expected == actual
//    * {ASSERT|EXPECT}_NE(v1, v2):           Tests that v1 != v2
//    * {ASSERT|EXPECT}_LT(v1, v2):           Tests that v1 < v2
//    * {ASSERT|EXPECT}_LE(v1, v2):           Tests that v1 <= v2
//    * {ASSERT|EXPECT}_GT(v1, v2):           Tests that v1 > v2
//    * {ASSERT|EXPECT}_GE(v1, v2):           Tests that v1 >= v2


// http://stackoverflow.com/a/29155677

namespace testing
{
namespace internal
{
enum GTestColor
{
  COLOR_DEFAULT,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_YELLOW
};

extern void ColoredPrintf(GTestColor color, const char* fmt, ...);

#define PRINTF(...) \
  do { testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN,\
  "[          ] "); \
  testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); } \
  while(0)

// C++ stream interface
class TestCout : public std::stringstream
{
public:
  ~TestCout()
  {
    PRINTF("%s\n", str().c_str());
  }
};

#define TEST_COUT testing::internal::TestCout()

} // namespace internal
} // namespace testing

/** Usage :

TEST(Test, Foo)
{
  // the following works but prints default stream
  EXPECT_TRUE(false) << "Testing Stream.";

  // or you can play with AINSI color code
  EXPECT_TRUE(false) << "\033[1;31m" << "Testing Stream.";

  // or use the above defined macros

  PRINTF("Hello world");

  // or

  TEST_COUT << "Hello world";
}

*/

#endif /* WOLF_UTILS_GTEST_H */
