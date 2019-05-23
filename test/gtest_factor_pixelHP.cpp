/*
 * gtest_factor_pixelHP.cpp
 *
 *  Created on: May 16, 2019
 *      Author: ovendrell
 */
#include "vision/factor/factor_pixelHP.h"
#include "vision/landmark/landmark_HP.h"
#include "vision/capture/capture_image.h"
#include "vision/processor/processor_bundle_adjustment.h"
#include "vision/internal/config.h"
#include <core/utils/utils_gtest.h>

#include <core/factor/factor_block_absolute.h>
#include <core/factor/factor_quaternion_absolute.h>
#include <core/ceres_wrapper/ceres_manager.h>
#include <core/math/pinhole_tools.h>
#include <core/ceres_wrapper/ceres_manager.h>

using namespace wolf;
using namespace Eigen;

std::string wolf_root = _WOLF_VISION_ROOT_DIR;


class FactorPixelHPTest : public testing::Test{
    public:
        Vector3s    pos1,   pos2,   pos3,   pos_cam, point;
        Vector3s    euler1, euler2, euler3, euler_cam;
        Quaternions quat1,  quat2,  quat3,  quat_cam;
        Vector4s    vquat1, vquat2, vquat3, vquat_cam; // quaternions as vectors
        Vector7s    pose1,  pose2,  pose3,  pose_cam;
        Vector4s    lmkHP1, lmkHP2, lmkHP3, lmkHP4;

        ProblemPtr      problem;
        CeresManagerPtr ceres_manager;

        SensorCameraPtr camera;
        ProcessorBundleAdjustmentPtr proc_bundle_adjustment;

        SensorBasePtr   S;
        FrameBasePtr    F1, F2, F3;
        CaptureImagePtr I1, I2, I3;
        FeaturePointImagePtr  f1, f2, f3;
        FeaturePointImagePtr  f12, f13, f14;

        LandmarkHPPtr L1; //only one landmark L1 is initialized
        LandmarkHPPtr L2;
        LandmarkHPPtr L3;
        LandmarkHPPtr L4;

        FactorPixelHPPtr c1, c12, c13, c14;
        FactorPixelHPPtr c2;
        FactorPixelHPPtr c3;

        Scalar pixel_noise_std;

        virtual ~FactorPixelHPTest()
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
            ceres_manager = std::make_shared<CeresManager>(problem, options);

            // Install sensor and processor
        	IntrinsicsCameraPtr intr = std::make_shared<IntrinsicsCamera>();
        	intr->pinhole_model_raw = Eigen::Vector4s(320,240,320,320);
        	intr->width  = 640;
        	intr->height = 480;
            S      = problem->installSensor("CAMERA", "camera", pose_cam, intr);
            camera = std::static_pointer_cast<SensorCamera>(S);

//        	ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
//        	params->delete_ambiguities = true;
//        	params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
//        	params->pixel_noise_std                = 1.0;
//        	params->min_track_length_for_factor = 3;
//        	params->voting_active = true;
//        	params->max_new_features = 5;
//
//            ProcessorBasePtr proc = problem->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", camera, params);
//            proc_bundle_adjustment = std::static_pointer_cast<ProcessorBundleAdjustment>(proc);

            // Add three viewpoints with frame, capture and feature
            pixel_noise_std = 1.0; //params->pixel_noise_std;
            Vector2s pix(0,0);
            Matrix2s pix_cov(Matrix2s::Identity() * pow(pixel_noise_std, 2));

        	// Point
        	cv::Point2f p = cv::Point2f(intr->width /2, intr->height/2);
        	cv::KeyPoint kp = cv::KeyPoint(p, 32.0f);
        	cv::Mat des = cv::Mat::zeros(1,8, CV_8UC1);

            F1 = problem->emplaceFrame(KEY, pose1, 1.0);
            I1 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F1, 1.0, camera, cv::Mat(intr->width,intr->height,CV_8UC1)));
            f1 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp, 0, des, pix_cov)); // pixel at origin

            F2 = problem->emplaceFrame(KEY, pose2, 2.0);
            I2 = std::static_pointer_cast<CaptureImage>((CaptureBase::emplace<CaptureImage>(F2, 2.0, camera, cv::Mat(intr->width,intr->height,CV_8UC1))));
            f2 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I2, kp, 0, des, pix_cov));  // pixel at origin

            F3 = problem->emplaceFrame(KEY, pose3, 3.0);
            I3 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F3, 3.0, camera, cv::Mat(intr->width,intr->height,CV_8UC1)));
            f3 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I3, kp, 0, des, pix_cov));  // pixel at origin

            //create Landmark -- with f1, f2 or f3
        	//LandmarkBasePtr lmk = proc_bundle_adjustment->createLandmark(f1);
        	L1 = std::static_pointer_cast<LandmarkHP>(LandmarkBase::emplace<LandmarkHP>(problem->getMap(),lmkHP1, camera, des));
//        	problem->addLandmark(L1);
//        	lmk->link(problem->getMap());

            // factors
            c1 = std::static_pointer_cast<FactorPixelHP>(FactorBase::emplace<FactorPixelHP>(f1, f1, L1, nullptr /*proc*/));
            c2 = std::static_pointer_cast<FactorPixelHP>(FactorBase::emplace<FactorPixelHP>(f2, f2, L1, nullptr /*proc*/));
            c3 = std::static_pointer_cast<FactorPixelHP>(FactorBase::emplace<FactorPixelHP>(f3, f3, L1, nullptr /*proc*/));
        }
};

TEST(ProcessorFactorPixelHP, testZeroResidual)
{
	//Build problem
	ProblemPtr problem_ptr = Problem::create("PO", 3);
	CeresManagerPtr ceres_mgr = std::make_shared<CeresManager>(problem_ptr);

	// Install sensor
	IntrinsicsCameraPtr intr = std::make_shared<IntrinsicsCamera>();
	intr->pinhole_model_raw = Eigen::Vector4s(0,0,1,1);
	intr->width  = 640;
	intr->height = 480;
	auto sens_cam = problem_ptr->installSensor("CAMERA", "camera", (Eigen::Vector7s() << 0,0,0,  0,0,0,1).finished(), intr);
	SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sens_cam);
	// Install processor
	ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
	params->delete_ambiguities = true;
	params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
	params->pixel_noise_std                = 1.0;
	params->min_track_length_for_factor = 3;
	params->voting_active = true;
	params->max_new_features = 5;
	auto proc = problem_ptr->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);
	ProcessorBundleAdjustmentPtr proc_bundle_adj = std::static_pointer_cast<ProcessorBundleAdjustment>(proc);

    // Frame
	FrameBasePtr frm0 = problem_ptr->emplaceFrame(KEY, problem_ptr->zeroState(), TimeStamp(0));

	// Capture
	auto cap0 = std::static_pointer_cast<CaptureImage>(CaptureImage::emplace<CaptureImage>(frm0, TimeStamp(0), camera, cv::Mat::zeros(480,640, 1)));

	// Feature
	cv::Point2f p = cv::Point2f(240, 320);
	cv::KeyPoint kp = cv::KeyPoint(p, 32.0f);
	cv::Mat des = cv::Mat::zeros(1,8, CV_8U);

	FeaturePointImagePtr fea0 = std::make_shared<FeaturePointImage>(kp, 0, des, Eigen::Matrix2s::Identity()* pow(1, 2));
	fea0->setCapture(cap0);
	cap0->addFeature(fea0);
	fea0->link(cap0);

	// Landmark
	LandmarkBasePtr lmk = proc_bundle_adj->createLandmark(fea0);
	LandmarkHPPtr lmk_hp = std::static_pointer_cast<LandmarkHP>(lmk);
	problem_ptr->addLandmark(lmk_hp);
	lmk->link(problem_ptr->getMap());

	// Factor
	auto fac0 = FactorBase::emplace<FactorPixelHP>(fea0, fea0, lmk_hp, proc);
	auto fac_ptr = std::static_pointer_cast<FactorPixelHP>(fac0);

	ASSERT_TRUE(problem_ptr->check(0));

	//ASSERT the expectation of the landmark should be equal to the measurement
	Eigen::VectorXs expect = fac_ptr->expectation();
//	std::cout << expect << std::endl;
	ASSERT_FLOAT_EQ(expect(0,0),fea0->getMeasurement()(0,0));
	ASSERT_FLOAT_EQ(expect(1,0),fea0->getMeasurement()(1,0));
}

TEST_F(FactorPixelHPTest, testSolveLandmark)
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

TEST_F(FactorPixelHPTest, testSolveLandmarkAltered)
{
	ASSERT_TRUE(problem->check(0));

	S->fix();
	F1->fix();
	F2->fix();
	F3->fix();
	L1->unfix();

	auto orig = L1->point();
	L1->getP()->setState(L1->getState() + Vector4s::Random());
	std::string report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);

	std::cout << report << std::endl;

	std::cout << orig.transpose() << std::endl;
	std::cout << L1->point().transpose() << std::endl;

	ASSERT_MATRIX_APPROX(L1->point(), orig, 1e-6);

}

TEST_F(FactorPixelHPTest, testSolveFramePosition2ObservableDoF)
{
	ASSERT_TRUE(problem->check(0));

	S->fix();
	F2->fix();
	F3->fix();
	L1->fix();

	auto orig = F1->getP()->getState();

	//change state
	Vector3s position;
	position << 2.0, 2.0, 2.0;
	auto ori = F1->getO()->getState();
	Vector7s state;
	state << position, ori;
	F1->setState(state);

	F1->getO()->fix();
	F1->getP()->unfix();

	std::string report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);

	std::cout << report << std::endl;

	std::cout << orig.transpose() << std::endl;
	std::cout << F1->getP()->getState().transpose() << std::endl;

	// This test is no good because it checks 3 DoF and only 2DoF are observable.
    //ASSERT_MATRIX_APPROX(F1->getP()->getState(), orig, 1e-6);
	// We use the following alternative:
	// Frame must be in the X axis, so Y=0 and Z=0
    ASSERT_MATRIX_APPROX(F1->getP()->getState().tail(2), orig.tail(2), 1e-6);

	Eigen::VectorXs expect = c1->expectation();
	ASSERT_FLOAT_EQ(expect(0,0),f1->getMeasurement()(0,0));
	ASSERT_FLOAT_EQ(expect(1,0),f1->getMeasurement()(1,0));

}

TEST_F(FactorPixelHPTest, testSolveFramePosition)
{
    //landmark homogeneous coordinates in world reference
    lmkHP2 << 0.5, 0.5, 0.5, 1;
    lmkHP3 << -0.5, 0.75, 0.5, 1;
    lmkHP4 << 0.5, 0, -0.5, 1;

    //landmarks to camera coordinates
    Transform<Scalar,3,Isometry> T_w_r
        = Translation<Scalar,3>(F1->getP()->getState())
        * Quaternions(F1->getO()->getState().data());
    Transform<Scalar,3,Isometry> T_r_c
		= Translation<Scalar,3>(I1->getSensorP()->getState())
        * Quaternions(I1->getSensorO()->getState().data());
    Eigen::Matrix<Scalar, 4, 1> lmkHP2_c =  T_r_c.inverse() * T_w_r.inverse() * lmkHP2;
    Eigen::Matrix<Scalar, 4, 1> lmkHP3_c =  T_r_c.inverse() * T_w_r.inverse() * lmkHP3;
    Eigen::Matrix<Scalar, 4, 1> lmkHP4_c =  T_r_c.inverse() * T_w_r.inverse() * lmkHP4;

    //landmark point in camera coord
	Vector3s lmk_point_2_c = lmkHP2_c.head<3>()/lmkHP2_c(3);
	Vector3s lmk_point_3_c = lmkHP3_c.head<3>()/lmkHP3_c(3);
	Vector3s lmk_point_4_c = lmkHP4_c.head<3>()/lmkHP4_c(3);

	//points projected to image plane
	Vector2s point2 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_2_c);
	Vector2s point3 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_3_c);
	Vector2s point4 = pinhole::projectPoint(camera->getIntrinsic()->getState(), camera->getDistortionVector(), lmk_point_4_c);
	cv::KeyPoint kp2 = cv::KeyPoint(cv::Point2f(point2(0),point2(1)), 32.0f);
	cv::KeyPoint kp3 = cv::KeyPoint(cv::Point2f(point3(0),point3(1)), 32.0f);
	cv::KeyPoint kp4 = cv::KeyPoint(cv::Point2f(point4(0),point4(1)), 32.0f);
	cv::Mat des = cv::Mat::zeros(1,8, CV_8UC1);
    pixel_noise_std = 1.0; //params->pixel_noise_std;
    Vector2s pix(0,0);
    Matrix2s pix_cov(Matrix2s::Identity() * pow(pixel_noise_std, 2));

    //create features
    f12 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp2, 0, des, pix_cov));
	f13 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp3, 0, des, pix_cov));
	f14 = std::static_pointer_cast<FeaturePointImage>(FeatureBase::emplace<FeaturePointImage>(I1, kp4, 0, des, pix_cov));

	//create landmarks
	L2 = std::static_pointer_cast<LandmarkHP>(LandmarkBase::emplace<LandmarkHP>(problem->getMap(),lmkHP2, camera, des));
	L3 = std::static_pointer_cast<LandmarkHP>(LandmarkBase::emplace<LandmarkHP>(problem->getMap(),lmkHP3, camera, des));
	L4 = std::static_pointer_cast<LandmarkHP>(LandmarkBase::emplace<LandmarkHP>(problem->getMap(),lmkHP4, camera, des));

	//create factors
    c12 = std::static_pointer_cast<FactorPixelHP>(FactorBase::emplace<FactorPixelHP>(f12, f12, L2, nullptr /*proc*/));
    c13 = std::static_pointer_cast<FactorPixelHP>(FactorBase::emplace<FactorPixelHP>(f13, f13, L3, nullptr /*proc*/));
    c14 = std::static_pointer_cast<FactorPixelHP>(FactorBase::emplace<FactorPixelHP>(f14, f14, L4, nullptr /*proc*/));

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
	Vector3s position;
	position << Vector3s::Random()*100;//2.0, 2.0, 2.0;
	auto ori = F1->getO()->getState();
	Vector7s state;
	state << position, ori;
	F1->setState(state);

	F1->getO()->fix();
	F1->getP()->unfix();

	std::string report = ceres_manager->solve(wolf::SolverManager::ReportVerbosity::FULL);

	std::cout << report << std::endl;

	std::cout << orig.transpose() << std::endl;
	std::cout << F1->getP()->getState().transpose() << std::endl;

	// This test checks 3 DoF (3DoF are observable).
    ASSERT_MATRIX_APPROX(F1->getP()->getState(), orig, 1e-6);

	Eigen::VectorXs expect = c1->expectation();
	ASSERT_FLOAT_EQ(expect(0,0),f1->getMeasurement()(0,0));
	ASSERT_FLOAT_EQ(expect(1,0),f1->getMeasurement()(1,0));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

