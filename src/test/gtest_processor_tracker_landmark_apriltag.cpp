#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processors/processor_tracker_landmark_apriltag.h"
#include "features/feature_apriltag.h"
#include "landmark_apriltag.h"
#include "capture_pose.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

class ProcessorTrackerLandmarkApriltag_Wrapper : public ProcessorTrackerLandmarkApriltag
{
    public:
        ProcessorTrackerLandmarkApriltag_Wrapper(ProcessorParamsTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag) :
            ProcessorTrackerLandmarkApriltag(_params_tracker_landmark_apriltag){};
        ~ProcessorTrackerLandmarkApriltag_Wrapper(){}
        void setLastPtr(const CaptureBasePtr _last_ptr) { last_ptr_ = _last_ptr; }
};

// Use the following in case you want to initialize tests with predefined variables or methods.
class ProcessorTrackerLandmarkApriltag_class : public testing::Test{
    public:
        virtual void SetUp()
        {
            wolf_root = _WOLF_ROOT_DIR;

            problem = Problem::create("PO 3D");
            sen = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_params_canonical.yaml");

            WOLF_TRACE("The line below needs to be uncommented after adding Factory stuff to processor apriltag");
            prc     = problem->installProcessor("TRACKER LANDMARK APRILTAG", "apriltags", "camera", wolf_root + "/src/examples/processor_tracker_landmark_apriltag.yaml");
            prc_apr = std::static_pointer_cast<ProcessorTrackerLandmarkApriltag>(prc);

            problem->setPrior(Vector7s(), Matrix6s::Identity(), 0.0, 0.1);

//            F1 = problem->emplaceFrame(KEY_FRAME, 0.0);
//            C1 = std::make_shared<CapturePose>(0.0, sen, Vector7s(), Matrix6s());
//            C1 = prc_apr->getLastPtr();
//            F1 = C1->getFramePtr();
//            F1->addCapture(C1);
        }
    public:
        std::string wolf_root;
        ProblemPtr   problem;
        SensorBasePtr    sen;
        ProcessorBasePtr prc;
        ProcessorTrackerLandmarkApriltagPtr prc_apr;
        FrameBasePtr     F1;
        CaptureBasePtr   C1;
};

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
    FeatureBaseList features_in;
    Eigen::Vector3s pos;
    Eigen::Vector3s ori; //Euler angles in rad
    Eigen::Quaternions quat;
    Eigen::Vector7s pose;
    Eigen::Matrix6s meas_cov( (p->getVarVec()).asDiagonal() );
    int tag_id;

    // feature 0
    pos << 0,2,0;
    ori << M_TORAD * 0, M_TORAD * 0, M_TORAD * 0;
    quat = e2q(ori);
    pose << pos, quat.coeffs();
    tag_id = 0;
    FeatureBasePtr detected_feature0 = std::make_shared<FeatureApriltag>(pose, meas_cov, tag_id);

    // feature 1
    pos << 1,2,0;
    ori << M_TORAD * 0, M_TORAD * 0, M_TORAD * 0;
    quat = e2q(ori);
    pose << pos, quat.coeffs();
    tag_id = 0;
    FeatureBasePtr detected_feature1 = std::make_shared<FeatureApriltag>(pose, meas_cov, tag_id);

    features_in.push_back(detected_feature0);
    features_in.push_back(detected_feature0);


}

TEST_F(ProcessorTrackerLandmarkApriltag_class, createLandmark)
{
    FeatureApriltagPtr f1 = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 1);

    // need to set at least last_ptr in processor p before this DEATH can be removed
    ASSERT_DEATH({
        C1->addFeature(f1);
        problem->print(4,1,1,1);
        LandmarkBasePtr lmk = prc_apr->createLandmark(f1);
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

