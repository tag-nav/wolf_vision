/*
 * gtest_factor_pixelHP.cpp
 *
 *  Created on: May 16, 2019
 *      Author: ovendrell
 */
#include "vision/factor/factor_pixel_hp.h"
#include "vision/landmark/landmark_hp.h"
#include "vision/capture/capture_image.h"
#include "vision/processor/processor_bundle_adjustment.h"
#include "vision/internal/config.h"
#include "vision/math/pinhole_tools.h"

#include <core/utils/utils_gtest.h>
#include <core/factor/factor_block_absolute.h>
#include <core/factor/factor_quaternion_absolute.h>
#include <core/ceres_wrapper/ceres_manager.h>
#include <core/ceres_wrapper/ceres_manager.h>

using namespace wolf;
using namespace Eigen;

std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;


class FactorPixelHpTest : public testing::Test{
    public:
        Vector3d    pos1,   pos2,   pos3,   pos_cam, point;
        Vector3d    euler1, euler2, euler3, euler_cam;
        Quaterniond quat1,  quat2,  quat3,  quat_cam;
        Vector4d    vquat1, vquat2, vquat3, vquat_cam; // quaternions as vectors
        Vector7d    pose1,  pose2,  pose3,  pose_cam;
        Vector4d    lmkHP1, lmkHP2, lmkHP3, lmkHP4;

        ProblemPtr      problem;
        CeresManagerPtr ceres_manager;


        SensorCameraPtr camera;
        ProcessorBundleAdjustmentPtr proc_bundle_adjustment;

        SensorBasePtr   S;
        FrameBasePtr    F1, F2, F3;
        CaptureImagePtr I1, I2, I3;
        FeaturePointImagePtr  f11, f12, f13, f14;
        FeaturePointImagePtr  f21, f22, f23, f24;
        FeaturePointImagePtr  f31, f32, f33, f34;

        LandmarkHpPtr L1; //only one landmark L1 is initialized
        LandmarkHpPtr L2;
        LandmarkHpPtr L3;
        LandmarkHpPtr L4;

        FactorPixelHpPtr c11, c12, c13, c14;
        FactorPixelHpPtr c21, c22, c23, c24;
        FactorPixelHpPtr c31, c32, c33, c34;

        double pixel_noise_std;

        virtual ~FactorPixelHpTest()
        {
            std::cout << "destructor\n";
        }

        virtual void SetUp() override
        {
            // configuration
            /*
             * We have three robot poses, in three frames, with cameras C1, C2, C3
             * looking towards the origin of coordinates:
             *
             *              Z
             *              |
             *           ________  C3
             *          /   |   /
             *         ---------           /| C2
             *              |             / |
             *              |____________/_ | ___ Y
             *             /             |  /
             *            /              | /
             *      --------- C1         |/
             *      |   /   |
             *      ---------
             *        /
             *       X
             *
             * Each robot pose is at one axis, facing the origin:
             *   F1: pos = (1,0,0), ori = (0,0,pi)
             *   F2: pos = (0,1,0), ori = (0,0,-pi/2)
             *   F3: pos = (0,0,1), ori = (0,pi/2,pi)
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
             * Therefore, P projects exactly at the origin on each camera,
             * creating three features:
             *   f1: p1 = (0,0)
             *   f2: p2 = (0,0)
             *   f3: p3 = (0,0)
             *
             * We form a Wolf tree with three frames, three captures,
             * three features, one landmark and 3 factors:
             *
             *   Frame F1, Capture C1, feature f1, factor c1
             *   Frame F2, Capture C2, feature f2, factor c2
             *   Frame F3, Capture C3, feature f3, factor c3
             *   Landmark L1
             *
             * The three frame poses F1, F2, F3, the camera pose S in the robot frame
             * and the Landmark position are variables subject to optimization
             *
             * We perform a number of tests based on this configuration.
             */

            // all frames look to the origin
            pos1   << 1, 0, 0;
            pos2   << 0, 1, 0;
            pos3   << 0, 0, 1;
            euler1 << 0, 0     ,  M_PI   ;
            euler2 << 0, 0     , -M_PI_2 ;
            euler3 << 0, M_PI_2,  M_PI   ;
            quat1  =  e2q(euler1);
            quat2  =  e2q(euler2);
            quat3  =  e2q(euler3);
            vquat1 =  quat1.coeffs();
            vquat2 =  quat2.coeffs();
            vquat3 =  quat3.coeffs();
            pose1  << pos1, vquat1;
            pose2  << pos2, vquat2;
            pose3  << pos3, vquat3;

            //landmarks
            lmkHP1 << 0, 0, 0, 1;

            // camera at the robot origin looking forward
            pos_cam   <<  0, 0, 0;
            euler_cam << -M_PI_2, 0, -M_PI_2;
            quat_cam  =  e2q(euler_cam);
            vquat_cam =  quat_cam.coeffs();
            pose_cam  << pos_cam, vquat_cam;

            // Build problem
            problem = Problem::create("PO", 3);
            ceres::Solver::Options options;
            options.function_tolerance  = 1e-6;
            options.max_num_iterations  = 200;
            ceres_manager = std::make_shared<CeresManager>(problem, options);


            // Install sensor and processor
        	ParamsSensorCameraPtr intr      = std::make_shared<ParamsSensorCamera>();
            intr->pinhole_model_raw       = Eigen::Vector4d(320,240,320,320);
            intr->pinhole_model_rectified = Eigen::Vector4d(320,240,320,320);
        	intr->width  = 640;
        	intr->height = 480;
            S      = problem->installSensor("SensorCamera", "camera", pose_cam, intr);
            camera = std::static_pointer_cast<SensorCamera>(S);

            // Add three viewpoints with frame, capture and feature
            pixel_noise_std = 1.0; //params->pixel_noise_std;
            Vector2d pix(0,0);
            Matrix2d pix_cov(Matrix2d::Identity() * pow(pixel_noise_std, 2));

        	// Point
        	cv::Point2f p = cv::Point2f(intr->width /2, intr->height/2);
        	cv::KeyPoint kp = cv::KeyPoint(p, 32.0f);
        	cv::Mat des = cv::Mat::zeros(1,8, CV_8UC1);

            F1 = problem->emplaceFrame(KEY, 1.0, pose1);
            I1 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F1, 1.0, camera, cv::Mat(intr->width,intr->height,CV_8UC1)));
            f11 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp, 0, des, pix_cov)); // pixel at origin

            F2 = problem->emplaceFrame(KEY, 2.0, pose2);
            I2 = std::static_pointer_cast<CaptureImage>((CaptureBase::emplace<CaptureImage>(F2, 2.0, camera, cv::Mat(intr->width,intr->height,CV_8UC1))));
            f21 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I2, kp, 0, des, pix_cov));  // pixel at origin

            F3 = problem->emplaceFrame(KEY, 3.0, pose3);
            I3 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F3, 3.0, camera, cv::Mat(intr->width,intr->height,CV_8UC1)));
            f31 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I3, kp, 0, des, pix_cov));  // pixel at origin

            // create Landmark
        	L1 = std::static_pointer_cast<LandmarkHp>(LandmarkBase::emplace<LandmarkHp>(problem->getMap(),lmkHP1, camera, des));

            // factors
            c11 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f11, f11, L1, nullptr /*proc*/, false /*use loss function*/));
            c21 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f21, f21, L1, nullptr /*proc*/, false /*use loss function*/));
            c31 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f31, f31, L1, nullptr /*proc*/, false /*use loss function*/));
        }
};

TEST(ProcessorFactorPixelHp, testZeroResidual)
{
    //Build problem
    ProblemPtr problem_ptr = Problem::create("PO", 3);
    CeresManagerPtr ceres_mgr = std::make_shared<CeresManager>(problem_ptr);

    // Install sensor
    ParamsSensorCameraPtr intr      = std::make_shared<ParamsSensorCamera>();
    intr->pinhole_model_raw       = Eigen::Vector4d(0,0,1,1);
    intr->pinhole_model_rectified = Eigen::Vector4d(320,240,320,320);
    intr->width  = 640;
    intr->height = 480;
    auto sens_cam = problem_ptr->installSensor("SensorCamera", "camera", (Eigen::Vector7d() << 0,0,0,  0,0,0,1).finished(), intr);
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sens_cam);

    // Install processor
    ParamsProcessorBundleAdjustmentPtr params = std::make_shared<ParamsProcessorBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_vision_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc = problem_ptr->installProcessor("ProcessorBundleAdjustment", "processor", sens_cam, params);
    ProcessorBundleAdjustmentPtr proc_bundle_adj = std::static_pointer_cast<ProcessorBundleAdjustment>(proc);

    // Frame
    FrameBasePtr frm0 = problem_ptr->emplaceFrame(KEY, 0.0, problem_ptr->stateZero());

    // Capture
    auto cap0 = std::static_pointer_cast<CaptureImage>(CaptureImage::emplace<CaptureImage>(frm0, TimeStamp(0), camera, cv::Mat::zeros(480,640, 1)));

    // Feature
    cv::Point2f p = cv::Point2f(320, 240);
    cv::KeyPoint kp = cv::KeyPoint(p, 32.0f);
    cv::Mat des = cv::Mat::zeros(1,8, CV_8U);

    FeatureBasePtr fea0 = FeatureBase::emplace<FeaturePointImage>(cap0, kp, 0, des, Eigen::Matrix2d::Identity()* pow(1, 2));

    // Landmark
    LandmarkBasePtr lmk = proc_bundle_adj->emplaceLandmark(fea0);
    LandmarkHpPtr lmk_hp = std::static_pointer_cast<LandmarkHp>(lmk);

    // Factor
    auto fac0 = FactorBase::emplace<FactorPixelHp>(fea0, fea0, lmk_hp, proc, false);
    auto fac_ptr = std::static_pointer_cast<FactorPixelHp>(fac0);

    ASSERT_TRUE(problem_ptr->check(0));

    //ASSERT the expectation of the landmark should be equal to the measurement
    Eigen::VectorXd expect = fac_ptr->expectation();
    //	std::cout << expect << std::endl;
    ASSERT_FLOAT_EQ(expect(0,0),fea0->getMeasurement()(0,0));
    ASSERT_FLOAT_EQ(expect(1,0),fea0->getMeasurement()(1,0));
}

TEST_F(FactorPixelHpTest, testSolveLandmark)
{
    ASSERT_TRUE(problem->check(0));

    S->fix();
    F1->fix();
    F2->fix();
    F3->fix();
    L1->unfix();

    auto orig = L1->point();
    std::string report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);

    std::cout << report << std::endl;

    std::cout << orig.transpose() << std::endl;
    std::cout << L1->point().transpose() << std::endl;

    ASSERT_MATRIX_APPROX(L1->point(), orig, 1e-6);

}

TEST_F(FactorPixelHpTest, testSolveLandmarkAltered)
{
    ASSERT_TRUE(problem->check(0));

    S->fix();
    F1->fix();
    F2->fix();
    F3->fix();
    L1->unfix();

    auto orig = L1->point();
    L1->getP()->setState(L1->getState().vector("P") + Vector4d::Random());
    std::string report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);

    std::cout << report << std::endl;

    std::cout << orig.transpose() << std::endl;
    std::cout << L1->point().transpose() << std::endl;

    ASSERT_MATRIX_APPROX(L1->point(), orig, 1e-6);

}

TEST_F(FactorPixelHpTest, testSolveFramePosition2ObservableDoF)
{
    ASSERT_TRUE(problem->check(0));

    S->fix();
    F2->fix();
    F3->fix();
    L1->fix();

    auto orig = F1->getP()->getState();

    //change state
    Vector3d position;
    position << 2.0, 2.0, 2.0;
    auto ori = F1->getO()->getState();
    Vector7d state;
    state << position, ori;
    F1->setState(VectorComposite(state, "PO", {3,4}));

    F1->getO()->fix();
    F1->getP()->unfix();

    std::string report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);

    std::cout << report << std::endl;

    std::cout << orig.transpose() << std::endl;
    std::cout << F1->getP()->getState().transpose() << std::endl;

    // This test is no good because it checks 3 DoF and only 2doF are observable.
    //ASSERT_MATRIX_APPROX(F1->getP()->getState(), orig, 1e-6);
    // We use the following alternative:
    // Frame must be in the X axis, so Y=0 and Z=0
    ASSERT_MATRIX_APPROX(F1->getP()->getState().tail(2), orig.tail(2), 1e-6);

    Eigen::VectorXd expect = c11->expectation();
    ASSERT_FLOAT_EQ(expect(0,0),f11->getMeasurement()(0,0));
    ASSERT_FLOAT_EQ(expect(1,0),f11->getMeasurement()(1,0));

}

TEST_F(FactorPixelHpTest, testSolveFramePosition)
{
    //landmark homogeneous coordinates in world reference
    lmkHP2 << 0.5, 0.5, 0.5, 1;
    lmkHP3 << -0.5, 0.75, 0.5, 1;
    lmkHP4 << 0.5, 0, -0.5, 1;

    //landmarks to camera coordinates
    Transform<double,3,Isometry> T_w_r
    = Translation<double,3>(F1->getP()->getState())
    * Quaterniond(F1->getO()->getState().data());
    Transform<double,3,Isometry> T_r_c
    = Translation<double,3>(I1->getSensorP()->getState())
    * Quaterniond(I1->getSensorO()->getState().data());
    Eigen::Matrix<double, 4, 1> lmkHP2_c =  T_r_c.inverse() * T_w_r.inverse() * lmkHP2;
    Eigen::Matrix<double, 4, 1> lmkHP3_c =  T_r_c.inverse() * T_w_r.inverse() * lmkHP3;
    Eigen::Matrix<double, 4, 1> lmkHP4_c =  T_r_c.inverse() * T_w_r.inverse() * lmkHP4;

    //landmark point in camera coord
    Vector3d lmk_point_2_c = lmkHP2_c.head<3>()/lmkHP2_c(3);
    Vector3d lmk_point_3_c = lmkHP3_c.head<3>()/lmkHP3_c(3);
    Vector3d lmk_point_4_c = lmkHP4_c.head<3>()/lmkHP4_c(3);

    //points projected to image plane
    Vector2d point2 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_2_c);
    Vector2d point3 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_3_c);
    Vector2d point4 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_4_c);
    cv::KeyPoint kp2 = cv::KeyPoint(cv::Point2f(point2(0),point2(1)), 32.0f);
    cv::KeyPoint kp3 = cv::KeyPoint(cv::Point2f(point3(0),point3(1)), 32.0f);
    cv::KeyPoint kp4 = cv::KeyPoint(cv::Point2f(point4(0),point4(1)), 32.0f);
    cv::Mat des = cv::Mat::zeros(1,8, CV_8UC1);
    pixel_noise_std = 1.0; //params->pixel_noise_std;
    Vector2d pix(0,0);
    Matrix2d pix_cov(Matrix2d::Identity() * pow(pixel_noise_std, 2));

    //create features
    f12 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp2, 0, des, pix_cov));
    f13 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp3, 0, des, pix_cov));
    f14 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp4, 0, des, pix_cov));

    //create landmarks
    L2 = std::static_pointer_cast<LandmarkHp>(LandmarkBase::emplace<LandmarkHp>(problem->getMap(),lmkHP2, camera, des));
    L3 = std::static_pointer_cast<LandmarkHp>(LandmarkBase::emplace<LandmarkHp>(problem->getMap(),lmkHP3, camera, des));
    L4 = std::static_pointer_cast<LandmarkHp>(LandmarkBase::emplace<LandmarkHp>(problem->getMap(),lmkHP4, camera, des));

    //create factors
    c12 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f12, f12, L2, nullptr /*proc*/, false /*use loss function*/));
    c13 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f13, f13, L3, nullptr /*proc*/, false /*use loss function*/));
    c14 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f14, f14, L4, nullptr /*proc*/, false /*use loss function*/));

    ASSERT_TRUE(problem->check());

    S->fix();
    F2->fix();
    F3->fix();
    L1->fix();
    L2->fix();
    L3->fix();
    L4->fix();

    auto orig = F1->getP()->getState();

    //change state
    Vector3d position;
    position << Vector3d::Random()*100;//2.0, 2.0, 2.0;
    auto ori = F1->getO()->getState();
//    Vector7d state;
//    state << position, ori;
    F1->setState("PO", {position, ori});

    F1->getO()->fix();
    F1->getP()->unfix();

    std::string report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);

    std::cout << report << std::endl;

    std::cout << orig.transpose() << std::endl;
    std::cout << F1->getP()->getState().transpose() << std::endl;

    // This test checks 3 DoF (3doF are observable).
    ASSERT_MATRIX_APPROX(F1->getP()->getState(), orig, 1e-6);

    Eigen::VectorXd expect = c11->expectation();
    ASSERT_FLOAT_EQ(expect(0,0),f11->getMeasurement()(0,0));
    ASSERT_FLOAT_EQ(expect(1,0),f11->getMeasurement()(1,0));
}


TEST_F(FactorPixelHpTest, testSolveBundleAdjustment)
{


    // Go around a weird bug: if we do not perturbate this LMK then the test fails.
    // EDIT 2020/06/07: it seems to work
    // todo remove this if working fine long after this edit
    // todo close issue wolf/#245 related to this
    //    L1->perturb(1e-12);




    //landmark homogeneous coordinates in world reference
    lmkHP2 << 0.5, 0.5, 0.5, 1;
    lmkHP3 << -0.5, 0.75, 0.5, 1;
    lmkHP4 << 0.5, 0, -0.5, 1;

    //landmarks to camera coordinates
    Transform<double,3,Isometry> T_w_r1
        = Translation<double,3>(F1->getP()->getState())
        * Quaterniond(F1->getO()->getState().data());
    Transform<double,3,Isometry> T_r_c
		= Translation<double,3>(I1->getSensorP()->getState())
        * Quaterniond(I1->getSensorO()->getState().data());
    Eigen::Matrix<double, 4, 1> lmkHP2_c1 =  T_r_c.inverse() * T_w_r1.inverse() * lmkHP2;
    Eigen::Matrix<double, 4, 1> lmkHP3_c1 =  T_r_c.inverse() * T_w_r1.inverse() * lmkHP3;
    Eigen::Matrix<double, 4, 1> lmkHP4_c1 =  T_r_c.inverse() * T_w_r1.inverse() * lmkHP4;

    //landmark point in camera coord
	Vector3d lmk_point_2_c1 = lmkHP2_c1.head<3>()/lmkHP2_c1(3);
	Vector3d lmk_point_3_c1 = lmkHP3_c1.head<3>()/lmkHP3_c1(3);
	Vector3d lmk_point_4_c1 = lmkHP4_c1.head<3>()/lmkHP4_c1(3);

    //landmarks to camera coordinates
    Transform<double,3,Isometry> T_w_r2
        = Translation<double,3>(F2->getP()->getState())
        * Quaterniond(F2->getO()->getState().data());
    Eigen::Matrix<double, 4, 1> lmkHP2_c2 =  T_r_c.inverse() * T_w_r2.inverse() * lmkHP2;
    Eigen::Matrix<double, 4, 1> lmkHP3_c2 =  T_r_c.inverse() * T_w_r2.inverse() * lmkHP3;
    Eigen::Matrix<double, 4, 1> lmkHP4_c2 =  T_r_c.inverse() * T_w_r2.inverse() * lmkHP4;

    //landmark point in camera coord
	Vector3d lmk_point_2_c2 = lmkHP2_c2.head<3>()/lmkHP2_c2(3);
	Vector3d lmk_point_3_c2 = lmkHP3_c2.head<3>()/lmkHP3_c2(3);
	Vector3d lmk_point_4_c2 = lmkHP4_c2.head<3>()/lmkHP4_c2(3);

    //landmarks to camera coordinates
    Transform<double,3,Isometry> T_w_r3
        = Translation<double,3>(F3->getP()->getState())
        * Quaterniond(F3->getO()->getState().data());
    Eigen::Matrix<double, 4, 1> lmkHP2_c3 =  T_r_c.inverse() * T_w_r3.inverse() * lmkHP2;
    Eigen::Matrix<double, 4, 1> lmkHP3_c3 =  T_r_c.inverse() * T_w_r3.inverse() * lmkHP3;
    Eigen::Matrix<double, 4, 1> lmkHP4_c3 =  T_r_c.inverse() * T_w_r3.inverse() * lmkHP4;

    //landmark point in camera coord
	Vector3d lmk_point_2_c3 = lmkHP2_c3.head<3>()/lmkHP2_c3(3);
	Vector3d lmk_point_3_c3 = lmkHP3_c3.head<3>()/lmkHP3_c3(3);
	Vector3d lmk_point_4_c3 = lmkHP4_c3.head<3>()/lmkHP4_c3(3);

	//points projected to image plane
	Vector2d point12 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_2_c1);
	Vector2d point13 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_3_c1);
	Vector2d point14 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_4_c1);
	cv::KeyPoint kp12 = cv::KeyPoint(cv::Point2f(point12(0),point12(1)), 32.0f);
	cv::KeyPoint kp13 = cv::KeyPoint(cv::Point2f(point13(0),point13(1)), 32.0f);
	cv::KeyPoint kp14 = cv::KeyPoint(cv::Point2f(point14(0),point14(1)), 32.0f);

	Vector2d point22 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_2_c2);
	Vector2d point23 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_3_c2);
	Vector2d point24 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_4_c2);
	cv::KeyPoint kp22 = cv::KeyPoint(cv::Point2f(point22(0),point22(1)), 32.0f);
	cv::KeyPoint kp23 = cv::KeyPoint(cv::Point2f(point23(0),point23(1)), 32.0f);
	cv::KeyPoint kp24 = cv::KeyPoint(cv::Point2f(point24(0),point24(1)), 32.0f);

	Vector2d point32 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_2_c3);
	Vector2d point33 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_3_c3);
	Vector2d point34 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_4_c3);
	cv::KeyPoint kp32 = cv::KeyPoint(cv::Point2f(point32(0),point32(1)), 32.0f);
	cv::KeyPoint kp33 = cv::KeyPoint(cv::Point2f(point33(0),point33(1)), 32.0f);
	cv::KeyPoint kp34 = cv::KeyPoint(cv::Point2f(point34(0),point34(1)), 32.0f);

	cv::Mat des = cv::Mat::zeros(1,8, CV_8UC1);
    pixel_noise_std = 1.0;
    Vector2d pix(0,0);
    Matrix2d pix_cov(Matrix2d::Identity() * pow(pixel_noise_std, 2));

    //create features
    f12 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp12, 0, des, pix_cov));
	f13 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp13, 0, des, pix_cov));
	f14 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp14, 0, des, pix_cov));

    f22 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I2, kp22, 0, des, pix_cov));
	f23 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I2, kp23, 0, des, pix_cov));
	f24 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I2, kp24, 0, des, pix_cov));

	f32 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I3, kp32, 0, des, pix_cov));
	f33 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I3, kp33, 0, des, pix_cov));
	f34 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I3, kp34, 0, des, pix_cov));

	//create landmarks
	L2 = std::static_pointer_cast<LandmarkHp>(LandmarkBase::emplace<LandmarkHp>(problem->getMap(),lmkHP2, camera, des));
	L3 = std::static_pointer_cast<LandmarkHp>(LandmarkBase::emplace<LandmarkHp>(problem->getMap(),lmkHP3, camera, des));
	L4 = std::static_pointer_cast<LandmarkHp>(LandmarkBase::emplace<LandmarkHp>(problem->getMap(),lmkHP4, camera, des));

	//create factors
    c12 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f12, f12, L2, nullptr /*proc*/, false /*use loss function*/));
    c13 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f13, f13, L3, nullptr /*proc*/, false /*use loss function*/));
    c14 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f14, f14, L4, nullptr /*proc*/, false /*use loss function*/));

    c22 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f22, f22, L2, nullptr /*proc*/, false /*use loss function*/));
    c23 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f23, f23, L3, nullptr /*proc*/, false /*use loss function*/));
    c24 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f24, f24, L4, nullptr /*proc*/, false /*use loss function*/));

    c32 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f32, f32, L2, nullptr /*proc*/, false /*use loss function*/));
    c33 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f33, f33, L3, nullptr /*proc*/, false /*use loss function*/));
    c34 = std::static_pointer_cast<FactorPixelHp>(FactorBase::emplace<FactorPixelHp>(f34, f34, L4, nullptr /*proc*/, false /*use loss function*/));

	ASSERT_TRUE(problem->check());

	S->fix();
	F1->fix();
	F2->unfix();
	F3->unfix();
	L1->fix();
	L2->unfix();
	L3->unfix();
	L4->unfix();

	auto p1 = F1->getP()->getState();
	auto p2 = F2->getP()->getState();
	auto p3 = F3->getP()->getState();
	auto q1 = F1->getO()->getState();
	auto q2 = F2->getO()->getState();
	auto q3 = F3->getO()->getState();
	auto l1 = L1->getP()->getState();
	auto l2 = L2->getP()->getState();
	auto l3 = L3->getP()->getState();
	auto l4 = L4->getP()->getState();


	std::string report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);
	std::cout << report << std::endl;

	//
    ASSERT_MATRIX_APPROX(c11->expectation(), f11->getMeasurement(), 0.1); // 0.1 pixel error allowed
    ASSERT_MATRIX_APPROX(c12->expectation(), f12->getMeasurement(), 0.1);
    ASSERT_MATRIX_APPROX(c13->expectation(), f13->getMeasurement(), 0.1);
    ASSERT_MATRIX_APPROX(c14->expectation(), f14->getMeasurement(), 0.1);

    ASSERT_MATRIX_APPROX(F1->getP()->getState(), p1, 1e-6);
    ASSERT_MATRIX_APPROX(F2->getP()->getState(), p2, 1e-6);
    ASSERT_MATRIX_APPROX(F3->getP()->getState(), p3, 1e-6);
    ASSERT_MATRIX_APPROX(F1->getO()->getState(), q1, 1e-6);
    ASSERT_MATRIX_APPROX(F2->getO()->getState(), q2, 1e-6);
    ASSERT_MATRIX_APPROX(F3->getO()->getState(), q3, 1e-6);

    ASSERT_MATRIX_APPROX(L1->getP()->getState(), l1, 1e-6);
    ASSERT_MATRIX_APPROX(L2->getP()->getState(), l2, 1e-6);
    ASSERT_MATRIX_APPROX(L3->getP()->getState(), l3, 1e-6);
    ASSERT_MATRIX_APPROX(L4->getP()->getState(), l4, 1e-6);

	// perturb states

    // kfs
	for (auto kf : problem->getTrajectory()->getFrameList())
	{
		if (kf == F1) continue;

		if (!kf->getP()->isFixed())
		    kf->getP()->setState(kf->getP()->getState() + 0.2*Vector3d::Random());
        if (!kf->getO()->isFixed())
            kf->getO()->setState((Quaterniond(kf->getO()->getState().data()) * exp_q(0.2*Vector3d::Random())).coeffs());
	}

	// lmks
	for (auto lmk : problem->getMap()->getLandmarkList())
	{
		if (lmk == L1) continue;

        if (!lmk->isFixed())
            lmk->getP()->setState(lmk->getP()->getState() + 0.2*Vector4d::Random());
	}

	// solve again
    problem->print(1,0,1,1);

    report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);
	std::cout << report << std::endl;

	problem->print(1,0,1,1);
	for (auto lmk : problem->getMap()->getLandmarkList())
	{
		std::cout << "L" << lmk->id()<< ": " << std::static_pointer_cast<LandmarkHp>(lmk)->point().transpose() << std::endl;
	}


	// This test checks 3 DoF (3doF are observable).
    ASSERT_MATRIX_APPROX(F1->getP()->getState(), p1, 1e-6);
    ASSERT_MATRIX_APPROX(F2->getP()->getState(), p2, 1e-6);
    ASSERT_MATRIX_APPROX(F3->getP()->getState(), p3, 1e-6);
    ASSERT_QUATERNION_APPROX(Quaterniond(F1->getO()->getState().data()), Quaterniond(q1.data()), 1e-6);
    ASSERT_QUATERNION_APPROX(Quaterniond(F2->getO()->getState().data()), Quaterniond(q2.data()), 1e-6);
    ASSERT_QUATERNION_APPROX(Quaterniond(F3->getO()->getState().data()), Quaterniond(q3.data()), 1e-6);

    ASSERT_MATRIX_APPROX(L1->point(), lmkHP1.head<3>()/lmkHP1(3), 1e-6);
    ASSERT_MATRIX_APPROX(L2->point(), lmkHP2.head<3>()/lmkHP2(3), 1e-6);
    ASSERT_MATRIX_APPROX(L3->point(), lmkHP3.head<3>()/lmkHP3(3), 1e-6);
    ASSERT_MATRIX_APPROX(L4->point(), lmkHP4.head<3>()/lmkHP4(3), 1e-6);

    ASSERT_MATRIX_APPROX(c11->expectation(), f11->getMeasurement(), 0.1); // 0.1 pixel error allowed
    ASSERT_MATRIX_APPROX(c12->expectation(), f12->getMeasurement(), 0.1);
    ASSERT_MATRIX_APPROX(c13->expectation(), f13->getMeasurement(), 0.1);
    ASSERT_MATRIX_APPROX(c14->expectation(), f14->getMeasurement(), 0.1);


}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
//  ::testing::GTEST_FLAG(filter) = "FactorPixelHpTest.testSolveBundleAdjustment*";

  return RUN_ALL_TESTS();
}

