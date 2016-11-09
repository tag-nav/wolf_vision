#include "utils_gtest.h"
#include "../src/time_stamp.h"

#include <thread>

TEST(WolfTestTimeStamp, TimeStampInitNow)
{
  wolf::TimeStamp start;

  // If we don't sleep, start == time_stamp sometimes.
  // And sometimes start <= time_stamp ...
  std::this_thread::sleep_for(std::chrono::microseconds(1));

  ASSERT_NE(start.get(), 0);

  wolf::TimeStamp time_stamp;

//  std::cout << std::fixed;
//  std::cout << std::setprecision(15);
//  std::cout << start.get() << " | " << time_stamp.get() << std::endl;

  ASSERT_NE(time_stamp.get(), start.get());

  ASSERT_LT(start.get(), time_stamp.get());

  PRINTF("All good at WolfTestTimeStamp::TimeStampInitNow !\n");
}

TEST(WolfTestTimeStamp, TimeStampInitScalar)
{
  wolf::Scalar val(101010);

  wolf::TimeStamp start(val);

  ASSERT_EQ(start.get(), val);
  ASSERT_EQ(start.getSeconds(), val);
  ASSERT_EQ(start.getNanoSeconds(), 0);

  std::stringstream ss;
  start.print(ss);

  ASSERT_STREQ("101010.0000000000", ss.str().c_str());

  PRINTF("All good at WolfTestTimeStamp::TimeStampInitScalar !\n");
}

TEST(WolfTestTimeStamp, TimeStampInitScalarSecNano)
{
  wolf::Scalar sec(101010);
  wolf::Scalar nano(202020);
  wolf::Scalar val(101010.000202020);

  wolf::TimeStamp start(sec, nano);

  // start.get -> 101010.000202020004508

  ASSERT_EQ(start.get(), val);
  ASSERT_EQ(start.getSeconds(), sec);
  ASSERT_EQ(start.getNanoSeconds(), nano);

  std::stringstream ss;
  start.print(ss);

  ASSERT_STREQ("101010.0002020200", ss.str().c_str());

  PRINTF("All good at WolfTestTimeStamp::TimeStampInitScalarSecNano !\n");
}

TEST(WolfTestTimeStamp, TimeStampEquality)
{
  wolf::TimeStamp start;

  wolf::TimeStamp time_stamp(start);

  // error: no match for ‘operator==’
  //ASSERT_EQ(time_stamp, start);

  ASSERT_EQ(time_stamp.get(), start.get());

  time_stamp.setToNow();

  ASSERT_NE(time_stamp.get(), start.get());

  time_stamp = start;

  ASSERT_EQ(time_stamp.get(), start.get());

  PRINTF("All good at WolfTestTimeStamp::TimeStampEquality !\n");
}

TEST(WolfTestTimeStamp, TimeStampInequality)
{
  wolf::TimeStamp start;

  std::this_thread::sleep_for(std::chrono::microseconds(1));

  wolf::TimeStamp time_stamp;

  // error: no match for ‘operator!=’
  //ASSERT_NE(time_stamp, start);

  ASSERT_LT(start, time_stamp);

  // error: no match for ‘operator>’
  //ASSERT_GT(time_stamp, start);

  PRINTF("All good at WolfTestTimeStamp::TimeStampInequality !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
