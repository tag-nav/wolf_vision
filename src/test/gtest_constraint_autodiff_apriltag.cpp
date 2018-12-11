#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "ceres_wrapper/ceres_manager.h"
#include "processors/processor_tracker_landmark_apriltag.h"
#include "capture_image.h"
#include "constraints/constraint_autodiff_apriltag.h"
#include "processor_factory.h"

#include <apriltag.h>

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
        void setIncomingPtr  (const CaptureBasePtr _incoming_ptr)   { incoming_ptr_ = _incoming_ptr; }

        // for factory
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr)
        {
            std::shared_ptr<ProcessorParamsTrackerLandmarkApriltag> prc_apriltag_params_;
            if (_params)
                prc_apriltag_params_ = std::static_pointer_cast<ProcessorParamsTrackerLandmarkApriltag>(_params);
            else
                prc_apriltag_params_ = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();

            ProcessorTrackerLandmarkApriltag_WrapperPtr prc_ptr = std::make_shared<ProcessorTrackerLandmarkApriltag_Wrapper>(prc_apriltag_params_);
            prc_ptr->setName(_unique_name);
            return prc_ptr;
        }

};
namespace wolf{
// Register in the Factories
WOLF_REGISTER_PROCESSOR("TRACKER LANDMARK APRILTAG WRAPPER", ProcessorTrackerLandmarkApriltag_Wrapper);
}
////////////////////////////////////////////////////////////////

// Use the following in case you want to initialize tests with predefines variables or methods.
/*class ConstraintAutodiffApriltag_class : public testing::Test{
    public:
        Vector3s    pos_camera,   pos_robot,   pos_landmark;
        Vector3s    euler_camera, euler_robot, euler_landmark;
        Quaternions quat_camera,  quat_robot,  quat_landmark;
        Vector4s    vquat_camera, vquat_robot, vquat_landmark; // quaternions as vectors
        Vector7s    pose_camera,  pose_robot,  pose_landmark;

        ProblemPtr      problem;
        CeresManagerPtr ceres_manager;

        SensorCameraPtr camera;
        ProcessorTrackerLandmarkApriltagPtr proc_apriltag;

        SensorBasePtr   S;
        FrameBasePtr    F1;
        CaptureImagePtr I1;
        FeatureApriltagPtr  f1;
        ConstraintAutodiffApriltagPtr c_tag;

        virtual void SetUp()
        {
            std::string wolf_root = _WOLF_ROOT_DIR;

            // configuration

             * We have three poses to take into account:
             *  - pose of the camera (extrinsincs)
             *  - pose of the landmark
             *  - pose of the robot (Keyframe)
             *
             * The measurement provides the pose of the landmark wrt camera's current pose in the world.
             * Camera's current pose in World is the composition of the robot pose with the camera extrinsics.
             *
             * The robot has a camera looking forward
             *   S: pos = (0,0,0), ori = (-pi/1, 0, -pi/1)
             *
             * There is a point at the origin
             *   P: pos = (0,0,0)
             *
             * The camera is canonical
             *   K = Id.
             *
             * Therefore, P projects exactly at the origin of the camera,
             *   f: p = (0,0)
             *
             * We form a Wolf tree with 1 frames F1, 11 capture C1,
             * 1 feature f1 (measurement), 11 landmark l and 11 apriltag constraint c1:
             *
             *   Frame F1, Capture C1, feature f1, landmark l1, constraint c1
             *
             * The frame pose F1, and the camera pose S
             * in the robot frame are variables subject to optimization
             *
             * We perform a number of tests based on this configuration.


            // camera is at origin of robot and looking forward
            // robot is at (0,0,0)
            // landmark is right in front of camera. Its orientation wrt camera is identity.
            pos_camera      << 0,0,0;
            pos_robot       << 0,0,0; //robot is at origin
            pos_landmark    << 0,1,0;
            euler_camera    << -M_PI_2, 0, -M_PI_2;
            euler_robot     << 0,0,0;
            euler_landmark  << 0,0,0;
            quat_camera     = e2q(euler_camera);
            quat_robot      = e2q(euler_robot);
            quat_landmark   = e2q(euler_landmark);
            vquat_camera    = quat_camera.coeffs();
            vquat_robot     = quat_robot.coeffs();
            vquat_landmark  = quat_landmark.coeffs();
            pose_camera     << pos_camera, vquat_camera;
            pose_robot      << pos_robot, vquat_robot;
            pose_landmark   << pos_landmark, vquat_landmark;

            // Build problem
            problem = Problem::create("PO 3D");
            ceres::Solver::Options options;
            ceres_manager = std::make_shared<CeresManager>(problem, options);

            // Install sensor and processor
            S      = problem->installSensor("CAMERA", "canonical", pose_camera, wolf_root + "/src/examples/camera_params_canonical.yaml");
            camera = std::static_pointer_cast<SensorCamera>(S);

            ProcessorParamsTrackerLandmarkApriltagPtr params_tracker_landmark_apriltag = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();
             Need to set some parameters ? do it now !
            params_tracker_landmark_apriltag->                = ;


            ProcessorBasePtr proc = problem->installProcessor("TRACKER LANDMARK APRILTAG", "apriltag", camera, params_tracker_landmark_apriltag);
            proc_apriltag = std::static_pointer_cast<ProcessorTrackerLandmarkApriltag>(proc);

            //create feature and landmark
            // F1 should be origin KF
            F1 = proc_apriltag->getOriginPtr()->getFramePtr();
            I1 = std::make_shared<CaptureImage>(1.0, camera, cv::Mat(2,2,CV_8UC1));
            F1-> addCapture(I1);
            // the sensor is at origin as well as the robot. The measurement matches with the pose of the tag wrt camera (and also wrt robot and world)
            // FeatureApriltag(Vector7s & _measurement,Matrix6s & _meas_covariance, apriltag_detection_t & _det
            Eigen::Matrix6s meas_cov(Eigen::Matrix6s::Identity());
            meas_cov.topLeftCorner(3,3)     *= 1e-2;
            meas_cov.bottomRightCorner(3,3) *= 1e-3;

            //we also need a detection object. This object is defined in Apriltag library...
            //maybe we can just delete the detection obkect from FeatureApriltag class !

            //apriltag_detection_t detection;
            //detection.id        = 1;
            //detection.hamming   = 1;
            //detection.goodness  = 1.0;
            //detection.decision_margin = 1.0;

            //define the homography matrix...
            matd_t R = *matd_create(3,3);
            MATD_EL(&R, 0, 0) = c;
            MATD_EL(&R, 0, 1) = -s;
            MATD_EL(&R, 1, 0) = s;
            MATD_EL(&R, 1, 1) = c;
            MATD_EL(&R, 2, 2) = 1;
            detection.H = R;

            // f1 = std::make_shared<FeatureApriltag>(pose_landmark, meas_cov, det);
        }
};*/


TEST(ConstraintAutodiffApriltag, Constructor)
{
    std::cout << "\033[1;33m [WARN]:\033[0m gtest for ConstraintAutodiffApriltag Constructor is empty." << std::endl;
}

TEST(ConstraintAutodiffApriltag, Destructor)
{
    std::cout << "\033[1;33m [WARN]:\033[0m gtest for ConstraintAutodiffApriltag Destructor is empty." << std::endl;
}

//[Class methods]

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

