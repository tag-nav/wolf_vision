#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processors/processor_tracker_landmark_apriltag.h"
#include "features/feature_apriltag.h"
#include "landmark_apriltag.h"
#include "capture_pose.h"
#include "processor_factory.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;


////////////////////////////////////////////////////////////////
/*
 * Wrapper class to be able to have setOriginPtr() and setLastPtr() in ProcessorTrackerLandmarkApriltag
 */
WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkApriltag_Wrapper);
class ProcessorTrackerLandmarkApriltag_Wrapper : public ProcessorTrackerLandmarkApriltag
{
    public:
        ProcessorTrackerLandmarkApriltag_Wrapper(ProcessorParamsTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag) :
            ProcessorTrackerLandmarkApriltag(_params_tracker_landmark_apriltag)
        {
            setType("TRACKER LANDMARK APRILTAG WRAPPER");
        };
        ~ProcessorTrackerLandmarkApriltag_Wrapper(){}
        void setOriginPtr(const CaptureBasePtr _origin_ptr) { origin_ptr_ = _origin_ptr; }
        void setLastPtr  (const CaptureBasePtr _last_ptr)   { last_ptr_ = _last_ptr; }
        // for factory
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr)
        {
            std::shared_ptr<ProcessorParamsTrackerLandmarkApriltag> prc_apriltag_params;
            if (_params)
                prc_apriltag_params = std::static_pointer_cast<ProcessorParamsTrackerLandmarkApriltag>(_params);
            else
                prc_apriltag_params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();

            ProcessorTrackerLandmarkApriltag_WrapperPtr prc_ptr = std::make_shared<ProcessorTrackerLandmarkApriltag_Wrapper>(prc_apriltag_params);
            prc_ptr->setName(_unique_name);
            return prc_ptr;
        }
};
namespace wolf{
// Register in the Factories
WOLF_REGISTER_PROCESSOR("TRACKER LANDMARK APRILTAG WRAPPER", ProcessorTrackerLandmarkApriltag_Wrapper);
}
////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////
/*
 * Test class to prepare a little wolf problem to test the class ProcessorTrackerLandmarkApriltag
 *
 * The class ProcessorTrackerLandmarkApriltag is sometimes tested via the wrapper ProcessorTrackerLandmarkApriltag_Wrapper
 */
// Use the following in case you want to initialize tests with predefined variables or methods.
class ProcessorTrackerLandmarkApriltag_class : public testing::Test{
    public:
        virtual void SetUp()
        {
            wolf_root = _WOLF_ROOT_DIR;

            problem = Problem::create("PO 3D");
            sen = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_params_canonical.yaml");

            prc     = problem->installProcessor("TRACKER LANDMARK APRILTAG WRAPPER", "apriltags_wrapper", "camera", wolf_root + "/src/examples/processor_tracker_landmark_apriltag.yaml");
            prc_apr = std::static_pointer_cast<ProcessorTrackerLandmarkApriltag_Wrapper>(prc);

            F1 = problem->setPrior(Vector7s(), Matrix6s::Identity(), 0.0, 0.1);

            C1 = std::make_shared<CapturePose>(1.0, sen, Vector7s(), Matrix6s());
            F1->addCapture(C1);
            prc_apr->setOriginPtr(C1);
            prc_apr->setLastPtr(C1);
        }
    public:
        std::string wolf_root;
        ProblemPtr   problem;
        SensorBasePtr    sen;
        ProcessorBasePtr prc;
        ProcessorTrackerLandmarkApriltag_WrapperPtr prc_apr;
        FrameBasePtr     F1;
        CaptureBasePtr   C1;
};
////////////////////////////////////////////////////////////////



/////////////////// TESTS START HERE ///////////////////////////
//                                                            //
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

TEST_F(ProcessorTrackerLandmarkApriltag_class, voteForKeyFrame)
{
    ASSERT_FALSE(prc_apr->voteForKeyFrame());
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, detectNewFeatures)
{
    // No detected features
    FeatureBaseList features_out;
    prc_apr->detectNewFeatures(1, features_out);
    ASSERT_EQ(features_out.size(), 0);

    // Some detected features TODO
    FeatureBaseList features_in;
    Eigen::Vector3s pos;
    Eigen::Vector3s ori; //Euler angles in rad
    Eigen::Quaternions quat;
    Eigen::Vector7s pose;
    Eigen::Matrix6s meas_cov( (prc_apr->getVarVec()).asDiagonal() );
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

    C1->addFeature(f1);
    LandmarkBasePtr lmk = prc_apr->createLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);
    ASSERT_TRUE(lmk_april->getType() == "APRILTAG");
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, createConstraint)
{
    FeatureApriltagPtr f1 = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 1);

    C1->addFeature(f1);
    LandmarkBasePtr lmk = prc_apr->createLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);

    ConstraintBasePtr ctr = prc_apr->createConstraint(f1, lmk);

    ASSERT_TRUE(ctr->getType() == "AUTODIFF APRILTAG");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

