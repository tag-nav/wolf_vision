/*
 * gtest_capture_base.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: asantamaria
 */


#include "utils_gtest.h"

#include "processor_base.h"

using namespace wolf;
using namespace Eigen;

class KFPackBufferTest : public testing::Test
{
    public:

        KFPackBuffer kfpackbuffer;
        FrameBasePtr f10, f20, f21, f28;
        Scalar tt10, tt20, tt21, tt28;
        TimeStamp timestamp;
        Scalar timetolerance;

        void SetUp(void)
        {
            f10 = std::make_shared<FrameBase>(TimeStamp(10),nullptr,nullptr,nullptr);
            f20 = std::make_shared<FrameBase>(TimeStamp(20),nullptr,nullptr,nullptr);
            f21 = std::make_shared<FrameBase>(TimeStamp(21),nullptr,nullptr,nullptr);
            f28 = std::make_shared<FrameBase>(TimeStamp(28),nullptr,nullptr,nullptr);

            tt10 = 0.1;
            tt20 = 0.2;
            tt21 = 0.21;
            tt28 = 0.28;
        };
};

TEST_F(KFPackBufferTest, add)
{
    kfpackbuffer.add(f10, tt10);
    ASSERT_EQ(kfpackbuffer.size(),1);
    kfpackbuffer.add(f20, tt20);
    ASSERT_EQ(kfpackbuffer.size(),2);
}

TEST_F(KFPackBufferTest, removeUpTo)
{
    kfpackbuffer.add(f10, tt10);
    kfpackbuffer.add(f20, tt20);
    kfpackbuffer.add(f21, tt21);
    ASSERT_EQ(kfpackbuffer.size(),3);

    // it should remove f20 and f10, thus size should be 1 after removal
    // Specifically, only f21 should remain
    KFPackPtr pack20 = std::make_shared<KFPack>(f20,tt20);
    kfpackbuffer.removeUpTo( pack20 );
    ASSERT_EQ(kfpackbuffer.size(),1);
    ASSERT_TRUE(kfpackbuffer.selectPack(f10->getTimeStamp(),tt10)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f20->getTimeStamp(),tt20)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f21->getTimeStamp(),tt21)!=nullptr);

    // Chech removal of an imprecise time stamp
    // Specifically, only f28 should remain
    kfpackbuffer.add(f28, tt28);
    ASSERT_EQ(kfpackbuffer.size(),2);
    FrameBasePtr f22 = std::make_shared<FrameBase>(TimeStamp(22),nullptr,nullptr,nullptr);
    KFPackPtr pack22 = std::make_shared<KFPack>(f22,5);
    kfpackbuffer.removeUpTo( pack22 );
    ASSERT_EQ(kfpackbuffer.size(),1);
    ASSERT_TRUE(kfpackbuffer.selectPack(f21->getTimeStamp(),tt21)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f28->getTimeStamp(),tt28)!=nullptr);
}

//TEST_F(KFPackBufferTest, selectPack)
//{
//    kfpackbuffer.add(f10, tt10);
//    kfpackbuffer.add(f20, tt20);
//    kfpackbuffer.add(f21, tt21);
//
//    // It should return f20
//    timestamp = 18;
//    timetolerance = 2;
//
//    KFPackPtr pack = kfpackbuffer.selectPack(timestamp, timetolerance);
//
//    std::cout << pack->key_frame->getTimeStamp() << "   " << pack->time_tolerance << std::endl;
//}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

