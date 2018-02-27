/*
 * gtest_capture_base.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: asantamaria
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
    ASSERT_EQ(kfpackbuffer.size(),1);
    kfpackbuffer.add(f20, tt20);
    ASSERT_EQ(kfpackbuffer.size(),2);
}

TEST_F(KFPackBufferTest, clear)
{
    kfpackbuffer.add(f10, tt10);
    kfpackbuffer.add(f20, tt20);
    ASSERT_EQ(kfpackbuffer.size(),2);
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
    for (auto ip1=0;ip1<p1.size();++ip1)
    {
        for (auto ip2=0;ip2<p2.size();++ip2)
        {
            kfpackbuffer.add(f10, p1[ip1]);
            kfpackbuffer.add(f20, p2[ip2]);
            for (auto iq=0;iq<q.size();++iq)
            {
                KFPackPtr packQ = kfpackbuffer.selectPack(16, q[iq]);
                if (packQ!=nullptr)
                    ASSERT_EQ(packQ->key_frame->getTimeStamp(),res(ip1*6+ip2*3+iq));
            }
            kfpackbuffer.clear();
        }
    }
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
    ASSERT_EQ(kfpackbuffer.size(),1);
    ASSERT_TRUE(kfpackbuffer.selectPack(f10->getTimeStamp(),tt)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f20->getTimeStamp(),tt)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f21->getTimeStamp(),tt)!=nullptr);

    // Chech removal of an imprecise time stamp
    // Specifically, only f28 should remain
    kfpackbuffer.add(f28, tt28);
    ASSERT_EQ(kfpackbuffer.size(),2);
    FrameBasePtr f22 = std::make_shared<FrameBase>(TimeStamp(22),nullptr,nullptr,nullptr);
    KFPackPtr pack22 = std::make_shared<KFPack>(f22,5);
    kfpackbuffer.removeUpTo( pack22->key_frame->getTimeStamp() );
    ASSERT_EQ(kfpackbuffer.size(),1);
    ASSERT_TRUE(kfpackbuffer.selectPack(f21->getTimeStamp(),tt)==nullptr);
    ASSERT_TRUE(kfpackbuffer.selectPack(f28->getTimeStamp(),tt)!=nullptr);
}


TEST(ProcessorBase, KeyFrameCallback)
{

    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    using Eigen::Vector2s;

    Scalar dt = 0.01;

    // Wolf problem
    ProblemPtr problem = Problem::create("PO 2D");

    // Install tracker (sensor and processor)
    SensorBasePtr sens_trk = make_shared<SensorBase>("FEATURE", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)),
                                                     std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)),
                                                     std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);
    shared_ptr<ProcessorTrackerFeatureDummy> proc_trk = make_shared<ProcessorTrackerFeatureDummy>(5, 5);
    proc_trk->setTimeTolerance(dt/2);

    problem->addSensor(sens_trk);
    sens_trk->addProcessor(proc_trk);

    // Install odometer (sensor and processor)
    SensorBasePtr sens_odo = problem->installSensor("ODOM 2D", "odometer", Vector3s(0,0,0), "");
    ProcessorParamsOdom2DPtr proc_odo_params = make_shared<ProcessorParamsOdom2D>();
    ProcessorBasePtr proc_odo = problem->installProcessor("ODOM 2D", "odometer", sens_odo, proc_odo_params);
    proc_odo->setTimeTolerance(dt/2);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    // Sequence to test KeyFrame creations (callback calls)

    // initialize
    TimeStamp   t(0.0);
    Vector3s    x(0,0,0);
    Matrix3s    P = Matrix3s::Identity() * 0.1;
    problem->setPrior(x, P, t, dt/2);             // KF1

    CaptureOdom2DPtr capt_odo = make_shared<CaptureOdom2D>(t, sens_odo, Vector2s(0.5,0));

    // Track
    CaptureVoidPtr capt_trk(make_shared<CaptureVoid>(t, sens_trk));
    proc_trk->process(capt_trk);

    for (size_t ii=0; ii<10; ii++ )
    {
        // Move
        t = t+dt;
        capt_odo->setTimeStamp(t);
        proc_odo->process(capt_odo);

        // Track
        capt_trk = make_shared<CaptureVoid>(t, sens_trk);
        proc_trk->process(capt_trk);

//        problem->print(4,1,1,0);
        WOLF_INFO("-------------------------------------------------");

        // Only odom creating KFs
        ASSERT_TRUE( problem->getLastKeyFramePtr()->getType().compare("PO 2D")==0 );
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

