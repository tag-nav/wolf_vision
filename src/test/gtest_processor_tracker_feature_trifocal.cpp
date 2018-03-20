#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processors/processor_tracker_feature_trifocal.h"
#include "processor_odom_3D.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

// Use the following in case you want to initialize tests with predefines variables or methods.
//class ProcessorTrackerFeatureTrifocal_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};



//TEST(ProcessorTrackerFeatureTrifocal, Constructor)
//{
//  std::cout << "\033[1;33m [WARN]:\033[0m gtest for ProcessorTrackerFeatureTrifocal Constructor is empty." << std::endl;
//}

//TEST(ProcessorTrackerFeatureTrifocal, Destructor)
//{
//  std::cout << "\033[1;33m [WARN]:\033[0m gtest for ProcessorTrackerFeatureTrifocal Destructor is empty." << std::endl;
//}

////[Class methods]
//TEST(ProcessorTrackerFeatureTrifocal, trackFeatures)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal trackFeatures is empty." << std::endl;
//}
//
//TEST(ProcessorTrackerFeatureTrifocal, correctFeatureDrift)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal correctFeatureDrift is empty." << std::endl;
//}
//
//TEST(ProcessorTrackerFeatureTrifocal, voteForKeyFrame)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal voteForKeyFrame is empty." << std::endl;
//}
//
//TEST(ProcessorTrackerFeatureTrifocal, detectNewFeatures)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal detectNewFeatures is empty." << std::endl;
//}
//
//TEST(ProcessorTrackerFeatureTrifocal, createConstraint)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal createConstraint is empty." << std::endl;
//}

TEST(ProcessorBase, KeyFrameCallback)
{

    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    using Eigen::Vector2s;

    Scalar dt = 0.01;

    // Wolf problem
    ProblemPtr problem = Problem::create("PO 3D");

    // Install tracker (sensor and processor)
    SensorBasePtr sens_trk = make_shared<SensorBase>("FEATURE", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)),
                                                     std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)),
                                                     std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);
    shared_ptr<ProcessorTrackerFeatureTrifocal> proc_trk = make_shared<ProcessorTrackerFeatureTrifocal>(dt/2, 5, 5);

    problem->addSensor(sens_trk);
    sens_trk->addProcessor(proc_trk);

    // Install odometer (sensor and processor)
    SensorBasePtr sens_odo = problem->installSensor("ODOM 3D", "odometer", Vector3s(0,0,0), "");
    ProcessorParamsOdom3DPtr proc_odo_params = make_shared<ProcessorParamsOdom3D>();
    ProcessorBasePtr proc_odo = problem->installProcessor("ODOM 3D", "odometer", sens_odo, proc_odo_params);
    proc_odo->setTimeTolerance(dt/2);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    // Sequence to test KeyFrame creations (callback calls)

    // initialize
    TimeStamp   t(0.0);
    Vector3s    x(0,0,0);
    Matrix3s    P = Matrix3s::Identity() * 0.1;
    problem->setPrior(x, P, t, dt/2);             // KF1

    CaptureOdom3DPtr capt_odo = make_shared<CaptureOdom3D>(t, sens_odo, Vector2s(0.5,0));

    // Track
    CaptureVoidPtr capt_trk(make_shared<CaptureVoid>(t, sens_trk));
    proc_trk->process(capt_trk);

    for (size_t ii=0; ii<10; ii++ )
    {
        // Move
        t = t+dt;
        WOLF_INFO("----------------------- ts: ", t , " --------------------------");

        capt_odo->setTimeStamp(t);
        proc_odo->process(capt_odo);

        // Track
        capt_trk = make_shared<CaptureVoid>(t, sens_trk);
        proc_trk->process(capt_trk);

//        problem->print(4,1,1,0);

        // Only odom creating KFs
        ASSERT_TRUE( problem->getLastKeyFramePtr()->getType().compare("PO 3D")==0 );
    }
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

