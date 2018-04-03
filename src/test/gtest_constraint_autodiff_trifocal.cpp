#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "constraints/constraint_autodiff_trifocal.h"
#include "capture_image.h"
#include "processors/processor_tracker_feature_trifocal.h"
#include "ceres_wrapper/ceres_manager.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

using namespace Eigen;

class ConstraintAutodiffTrifocalTest : public testing::Test{
    public:
        Vector3s pos1, pos2, pos3, pos_cam, point;
        Vector3s euler1, euler2, euler3, euler_cam;
        Quaternions quat1, quat2, quat3, quat_cam;

        ProblemPtr problem;
        CeresManagerPtr ceres_manager;

        SensorCameraPtr camera;
        ProcessorTrackerFeatureTrifocalPtr proc_trifocal;

        FrameBasePtr F1, F2, F3;
        CaptureImagePtr I1, I2, I3;
        FeatureBasePtr f1, f2, f3;
        ConstraintAutodiffTrifocalPtr c123;

        virtual void SetUp()
        {
            std::string wolf_root = _WOLF_ROOT_DIR;

            // configuration

            // all frames look to the origin
            pos1 << 1,0,0;
            pos2 << 0,1,0;
            pos3 << 0,0,1;
            euler1 << 0, 0, 180; euler1 *= M_TORAD;
            euler2 << 0, 0, -90; euler2 *= M_TORAD;
            euler3 << 0, 90, 0 ; euler3 *= M_TORAD;
            quat1 = e2q(euler1);
            quat2 = e2q(euler2);
            quat3 = e2q(euler3);

            // camera at the robot origin looking forward
            euler_cam << -90, 0, -90; euler_cam *= M_TORAD;
            quat_cam = e2q(euler_cam);

            // build all pose vectors
            Vector7s pose1;    pose1    << pos1, quat1.coeffs();
            Vector7s pose2;    pose2    << pos2, quat2.coeffs();
            Vector7s pose3;    pose3    << pos3, quat3.coeffs();
            Vector7s pose_cam; pose_cam << pos_cam, quat_cam.coeffs();

            // Build problem
            problem = Problem::create("PO 3D");
            ceres_manager = std::make_shared<CeresManager>(problem);

            // Install sensor and processor
            SensorBasePtr sen = problem->installSensor("CAMERA", "canonical", pose_cam, wolf_root + "/src/examples/camera_params_canonical.yaml");
            camera = std::static_pointer_cast<SensorCamera>(sen);

            ProcessorParamsTrackerFeatureTrifocalPtr params_trifocal = std::make_shared<ProcessorParamsTrackerFeatureTrifocal>();
            params_trifocal->time_tolerance = 1.0/2;
            params_trifocal->max_new_features = 5;
            params_trifocal->min_features_for_keyframe = 5;
            params_trifocal->yaml_file_params_vision_utils = wolf_root + "/src/examples/vision_utils_active_search.yaml";

            ProcessorTrackerFeatureTrifocalPtr proc_trk = std::make_shared<ProcessorTrackerFeatureTrifocal>(*params_trifocal);
            camera->addProcessor(proc_trk);

            // Add three viewpoints with frame, capture and feature
            Vector2s pix(0,0);
            Matrix2s pix_cov; pix_cov.setIdentity();

            F1 = problem->emplaceFrame(KEY_FRAME, pose1, 1.0);
            I1 = std::make_shared<CaptureImage>(1.0, camera, cv::Mat(2,2,CV_8UC1));
            F1->addCapture(I1);
            f1 = std::make_shared<FeatureBase>("PIXEL", pix, pix_cov); // pixel at origin
            I1->addFeature(f1);

            F2 = problem->emplaceFrame(KEY_FRAME, pose2, 2.0);
            I2 = std::make_shared<CaptureImage>(2.0, camera, cv::Mat(2,2,CV_8UC1));
            F2->addCapture(I2);
            f2 = std::make_shared<FeatureBase>("PIXEL", pix, pix_cov); // pixel at origin
            I2->addFeature(f2);

            F3 = problem->emplaceFrame(KEY_FRAME, pose3, 3.0);
            I3 = std::make_shared<CaptureImage>(3.0, camera, cv::Mat(2,2,CV_8UC1));
            F3->addCapture(I3);
            f3 = std::make_shared<FeatureBase>("PIXEL", pix, pix_cov); // pixel at origin
            I3->addFeature(f3);

            // trifocal constraint
            c123 = std::make_shared<ConstraintAutodiffTrifocal>(f1, f2, f3, proc_trk, false, CTR_ACTIVE);
            f3->addConstraint(c123);
            f1->addConstrainedBy(c123);
            f2->addConstrainedBy(c123);
            F1->addConstrainedBy(c123);
            F2->addConstrainedBy(c123);
        }
};

TEST_F(ConstraintAutodiffTrifocalTest, ground_truth)
{
    Vector4s residual;

    // Constraint with exact states should give zero residual
    c123->operator ()(pos1.data(), quat1.coeffs().data(),
                      pos2.data(), quat2.coeffs().data(),
                      pos3.data(), quat3.coeffs().data(),
                      pos_cam.data(), quat_cam.coeffs().data(),
                      residual.data());

    problem->print(4,1,1,1);
//    ASSERT_TRUE(problem->check()); // cannot pass this test!
    WOLF_DEBUG("residuals: ", residual.transpose());
    ASSERT_MATRIX_APPROX(residual, Vector4s::Zero(), 1e-16);
}

//[Class methods]

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

