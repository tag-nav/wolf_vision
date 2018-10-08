/*
 * gtest_KF_pack_buffer.cpp
 *
 *  Created on: Mar 5, 2018
 *      Author: jsola
 */
//Wolf
#include "utils_gtest.h"

#include "processor_odom_2D.h"
#include "sensor_odom_2D.h"

#include "processor_tracker_feature_dummy.h"
#include "capture_void.h"

#include "problem.h"

// STL
#include <iterator>
#include <iostream>

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

            tt10 = 1.0;
            tt20 = 1.0;
            tt21 = 1.0;
            tt28 = 1.0;
        };
};

TEST_F(KFPackBufferTest, empty)
{
    ASSERT_TRUE(kfpackbuffer.empty());
}

TEST_F(KFPackBufferTest, add)
{
    kfpackbuffer.add(f10, tt10);
    ASSERT_EQ(kfpackbuffer.size(), (unsigned int) 1);
    kfpackbuffer.add(f20, tt20);
    ASSERT_EQ(kfpackbuffer.size(), (unsigned int) 2);
}

TEST_F(KFPackBufferTest, clear)
{
    kfpackbuffer.add(f10, tt10);
    kfpackbuffer.add(f20, tt20);
    ASSERT_EQ(kfpackbuffer.size(), (unsigned int) 2);
    kfpackbuffer.clear();
    ASSERT_TRUE(kfpackbuffer.empty());
}

//TEST_F(KFPackBufferTest, print)
//{
//    kfpackbuffer.clear();
//    kfpackbuffer.add(f10, tt10);
//    kfpackbuffer.add(f20, tt20);
//    kfpackbuffer.print();
//}

TEST_F(KFPackBufferTest, checkTimeTolerance)
{
    kfpackbuffer.clear();
    kfpackbuffer.add(f10, tt10);
    kfpackbuffer.add(f20, tt20);
    // min time tolerance  > diff between time stamps. It should return true
    ASSERT_TRUE(kfpackbuffer.checkTimeTolerance(10, 20, 20, 20));
    // min time tolerance  < diff between time stamps. It should return true
    ASSERT_FALSE(kfpackbuffer.checkTimeTolerance(10, 1, 20, 20));
}

TEST_F(KFPackBufferTest, selectPack)
{
    // Evaluation using two packs (p1,p2)
    // with different time tolerances (tp1,tp2)
    // using a query pack (q) with also different time tolerances
    // depending on these tolerances we will get one (p1) or the other (p2)
    // packages from the buffer (res).
    // This can be summarized in the table hereafter:
    //
    //  p1 p2 q | resulting pack time stamp
    // --------------------------------
    //  2  2  2 | nullptr
    //  2  2  5 | nullptr
    //  2  2  7 | nullptr
    //  2  7  2 | nullptr
    //  2  7  5 | 20
    //  2  7  7 | 20
    //  7  2  2 | nullptr
    //  7  2  5 | nullptr
    //  7  2  7 | 10
    //  7  7  2 | nullptr
    //  7  7  5 | 20
    //  7  7  7 | 20

    kfpackbuffer.clear();

    // input packages
    std::vector<int> p1 = {2, 7}; // Pack 1 time tolerances
    std::vector<int> p2 = {2, 7}; // Pack 2 time tolerances
    std::vector<int> q = {2, 5, 7}; // Query pack time tolerances

    // Solution matrix
    Eigen::VectorXi res = Eigen::VectorXi::Zero(12);
    res(4) = 20;
    res(5) = 20;
    res(8) = 10;
    res(10) = 20;
    res(11) = 20;

    // test
    for (unsigned int ip1=0;ip1<p1.size();++ip1)
    {
        for (unsigned int ip2=0;ip2<p2.size();++ip2)
        {
            kfpackbuffer.add(f10, p1[ip1]);
            kfpackbuffer.add(f20, p2[ip2]);
            for (unsigned int iq=0;iq<q.size();++iq)
            {
                KFPackPtr packQ = kfpackbuffer.selectPack(16, q[iq]);
                if (packQ!=nullptr)
                    ASSERT_EQ(packQ->key_frame->getTimeStamp(),res(ip1*6+ip2*3+iq));
            }
            kfpackbuffer.clear();
        }
    }
}

TEST_F(KFPackBufferTest, selectPackBefore)
{
    kfpackbuffer.clear();

    kfpackbuffer.add(f10, tt10);
    kfpackbuffer.add(f20, tt20);
    kfpackbuffer.add(f21, tt21);

    // input time stamps
    std::vector<TimeStamp> q_ts = {9.5, 9.995, 10.005, 19.5, 20.5, 21.5};
    Scalar tt = 0.01;

    // Solution matrix
    // q_ts  |  pack
    //=================
    // first time
    //-----------------
    // 9.5      nullptr
    // 9.995    10
    // 10,005   10
    // 19.5     10
    // 20.5     10
    // 21.5     10
    // second time
    //-----------------
    // 9.5      nullptr
    // 9.995    null
    // 10,005   null
    // 19.5     null
    // 20.5     20
    // 21.5     20
    // third time
    //-----------------
    // 9.5      nullptr
    // 9.995    null
    // 10,005   null
    // 19.5     null
    // 20.5     null
    // 21.5     21

    Eigen::MatrixXs truth(3,6), res(3,6);
    truth << 0,10,10,10,10,10,  0,0,0,0,20,20,  0,0,0,0,0,21;
    res.setZero();

    for (int i=0; i<3; i++)
    {
        KFPackPtr packQ;
        int j = 0;
        for (auto ts : q_ts)
        {
            packQ = kfpackbuffer.selectPackBefore(ts, tt);
            if (packQ)
                res(i,j) = packQ->key_frame->getTimeStamp().get();

            j++;
        }
        kfpackbuffer.removeUpTo(packQ->key_frame->getTimeStamp());
    }

    ASSERT_MATRIX_APPROX(res, truth, 1e-6);
}

TEST_F(KFPackBufferTest, removeUpTo)
{
    // Small time tolerance for all test asserts
    Scalar tt = 0.1;
    kfpackbuffer.clear();
    kfpackbuffer.add(f10, tt10);
    kfpackbuffer.add(f20, tt20);
    kfpackbuffer.add(f21, tt21);

    // it should remove f20 and f10, thus size should be 1 after removal
    // Specifically, only f21 should remain
    KFPackPtr pack20 = std::make_shared<KFPack>(f20,tt20);
    kfpackbuffer.removeUpTo( pack20->key_frame->getTimeStamp() );
    ASSERT_EQ(kfpackbuffer.size(), (unsigned int) 1);
    ASSERT_TRUE(kfpackbuffer.selectPack(f10->getTimeStamp(),tt)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f20->getTimeStamp(),tt)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f21->getTimeStamp(),tt)!=nullptr);

    // Chech removal of an imprecise time stamp
    // Specifically, only f28 should remain
    kfpackbuffer.add(f28, tt28);
    ASSERT_EQ(kfpackbuffer.size(), (unsigned int) 2);
    FrameBasePtr f22 = std::make_shared<FrameBase>(TimeStamp(22),nullptr,nullptr,nullptr);
    KFPackPtr pack22 = std::make_shared<KFPack>(f22,5);
    kfpackbuffer.removeUpTo( pack22->key_frame->getTimeStamp() );
    ASSERT_EQ(kfpackbuffer.size(), (unsigned int) 1);
    ASSERT_TRUE(kfpackbuffer.selectPack(f21->getTimeStamp(),tt)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f28->getTimeStamp(),tt)!=nullptr);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

