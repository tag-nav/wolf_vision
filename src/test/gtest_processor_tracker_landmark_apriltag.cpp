#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processors/processor_tracker_landmark_apriltag.h"

#include "features/feature_apriltag.h"
#include "landmark_apriltag.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

//// Use the following in case you want to initialize tests with predefines variables or methods.
//class ProcessorTrackerLandmarkApriltag_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//            std::string wolf_root = _WOLF_ROOT_DIR;
//
//            ProblemPtr   problem = Problem::create("PO 3D");
//            SensorBasePtr    sen = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_camera.yaml");
//            ProcessorBasePtr prc = problem->installProcessor("TRACKER LANDMARK APRILTAG", "apriltags", "camera", wolf_root + "/src/examples/processor_tracker_landmark_apriltag.yaml");
//        }
//};

TEST(ProcessorTrackerLandmarkApriltag, Constructor)
{
    ProcessorParamsTrackerLandmarkApriltagPtr params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();
    params->tag_family_ = "tag36h11";
    params->name        = params->tag_family_;
    ProcessorTrackerLandmarkApriltagPtr p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "tag36h10";
    params->name        = params->tag_family_;
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "tag36artoolkit";
    params->name        = params->tag_family_;
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "tag25h9";
    params->name        = params->tag_family_;
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "tag25h7";
    params->name        = params->tag_family_;
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "wrong_family";
    params->name        = params->tag_family_;
    ASSERT_DEATH( { std::make_shared<ProcessorTrackerLandmarkApriltag>(params); }, "" );
}

TEST(ProcessorTrackerLandmarkApriltag, voteForKeyFrame)
{
    ProcessorParamsTrackerLandmarkApriltagPtr params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();
    params->tag_family_ = "tag36h11";
    ProcessorTrackerLandmarkApriltagPtr p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);

    ASSERT_FALSE(p->voteForKeyFrame());
}

TEST(ProcessorTrackerLandmarkApriltag, detectNewFeatures)
{
    ProcessorParamsTrackerLandmarkApriltagPtr params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();
    params->tag_family_ = "tag36h11";
    ProcessorTrackerLandmarkApriltagPtr p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);

    // No detected features
    FeatureBaseList features_out;
    p->detectNewFeatures(1, features_out);
    ASSERT_EQ(features_out.size(), 0);

    // Some detected features TODO
}

TEST(ProcessorTrackerLandmarkApriltag, createLandmark)
{
    ProcessorParamsTrackerLandmarkApriltagPtr params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();
    params->tag_family_ = "tag36h11";
    ProcessorTrackerLandmarkApriltagPtr p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);

    apriltag_detection_t det;
    det.id = 1;

    FeatureApriltagPtr feature = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), det);

    // no links in the wolf tree (it needs sensor, frame, capture):
    ASSERT_DEATH({
        LandmarkBasePtr lmk = p->createLandmark(feature);
        LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);
        ASSERT_TRUE(lmk_april->getType() == "APRILTAG");
    },"");
}

TEST(ProcessorTrackerLandmarkApriltag, createConstraint)
{
    std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerLandmarkApriltag createConstraint is empty." << std::endl;
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

