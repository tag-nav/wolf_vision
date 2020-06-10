#include <core/utils/utils_gtest.h>

#include "core/utils/logging.h"

#include "core/ceres_wrapper/ceres_manager.h"
#include "vision/processor/processor_tracker_feature_trifocal.h"
#include "vision/capture/capture_image.h"
#include "vision/factor/factor_trifocal.h"
#include "vision/internal/config.h"

using namespace Eigen;
using namespace wolf;

class FactorTrifocalTest : public testing::Test{
    public:
        Vector3d    pos1,   pos2,   pos3,   pos_cam, point;
        Vector3d    euler1, euler2, euler3, euler_cam;
        Quaterniond quat1,  quat2,  quat3,  quat_cam;
        Vector4d    vquat1, vquat2, vquat3, vquat_cam; // quaternions as vectors
        Vector7d    pose1,  pose2,  pose3,  pose_cam;

        ProblemPtr      problem;
        CeresManagerPtr ceres_manager;

        SensorCameraPtr camera;
        ProcessorTrackerFeatureTrifocalPtr proc_trifocal;

        SensorBasePtr   S;
        FrameBasePtr    F1, F2, F3;
        CaptureImagePtr I1, I2, I3;
        FeatureBasePtr  f1, f2, f3;
        FactorTrifocalPtr c123;

        double pixel_noise_std;

        ~FactorTrifocalTest() override
        {
            std::cout << "destructor\n";
        }

        void SetUp() override
        {
            std::string wolf_root = _WOLF_VISION_ROOT_DIR;

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
             *       Y
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
             * three features, and one trifocal factor:
             *
             *   Frame F1, Capture C1, feature f1
             *   Frame F2, Capture C2, feature f2
             *   Frame F3, Capture C3, feature f3, factor c123
             *
             * The three frame poses F1, F2, F3 and the camera pose S
             * in the robot frame are variables subject to optimization
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

            // camera at the robot origin looking forward
            pos_cam   <<  0, 0, 0;
            euler_cam << -M_PI_2, 0, -M_PI_2;// euler_cam *= M_TORAD;
            quat_cam  =  e2q(euler_cam);
            vquat_cam =  quat_cam.coeffs();
            pose_cam  << pos_cam, vquat_cam;

            // Build problem
            problem = Problem::create("PO", 3);
            ceres::Solver::Options options;
            ceres_manager = std::make_shared<CeresManager>(problem, options);

            // Install sensor and processor
            S      = problem->installSensor("SensorCamera", "canonical", pose_cam, wolf_root + "/demos/camera_params_canonical.yaml");
            camera = std::static_pointer_cast<SensorCamera>(S);

            ParamsProcessorTrackerFeatureTrifocalPtr params_tracker_feature_trifocal_trifocal = std::make_shared<ParamsProcessorTrackerFeatureTrifocal>();
            params_tracker_feature_trifocal_trifocal->time_tolerance                = 1.0/2;
            params_tracker_feature_trifocal_trifocal->max_new_features              = 5;
            params_tracker_feature_trifocal_trifocal->min_features_for_keyframe     = 5;
            params_tracker_feature_trifocal_trifocal->yaml_file_params_vision_utils = wolf_root + "/demos/processor_tracker_feature_trifocal_vision_utils.yaml";

            ProcessorBasePtr proc = problem->installProcessor("ProcessorTrackerFeatureTrifocal", "trifocal", camera, params_tracker_feature_trifocal_trifocal);
            proc_trifocal = std::static_pointer_cast<ProcessorTrackerFeatureTrifocal>(proc);

            // Add three viewpoints with frame, capture and feature
            pixel_noise_std = 2.0;
            Vector2d pix(0,0);
            Matrix2d pix_cov(Matrix2d::Identity() * pow(pixel_noise_std, 2));

            F1 = problem->emplaceFrame(KEY, 1.0, pose1);
            I1 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F1, 1.0, camera, cv::Mat(2,2,CV_8UC1)));
            f1 = FeatureBase::emplace<FeatureBase>(I1, "PIXEL", pix, pix_cov); // pixel at origin

            F2 = problem->emplaceFrame(KEY, 2.0, pose2);
            I2 = std::static_pointer_cast<CaptureImage>((CaptureBase::emplace<CaptureImage>(F2, 2.0, camera, cv::Mat(2,2,CV_8UC1))));
            f2 = FeatureBase::emplace<FeatureBase>(I2, "PIXEL", pix, pix_cov); // pixel at origin

            F3 = problem->emplaceFrame(KEY, 3.0, pose3);
            I3 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F3, 3.0, camera, cv::Mat(2,2,CV_8UC1)));
            f3 = FeatureBase::emplace<FeatureBase>(I3, "PIXEL", pix, pix_cov); // pixel at origin

            // trifocal factor
            // c123 = std::make_shared<FactorTrifocal>(f1, f2, f3, proc_trifocal, false, FAC_ACTIVE);
            c123 = std::static_pointer_cast<FactorTrifocal>(FactorBase::emplace<FactorTrifocal>(f3, f1, f2, f3, proc_trifocal, false, FAC_ACTIVE));
            // f3   ->addFactor   (c123);
            // f1   ->addConstrainedBy(c123);
            // f2   ->addConstrainedBy(c123);
        }
};

TEST_F(FactorTrifocalTest, InfoMatrix)
{
    /** Ground truth covariance. Rationale:
     * Due to the orthogonal configuration (see line 40 and onwards), we have:
     *   Let s = pixel_noise_std.
     *   Let d = 1 the distance from the cameras to the 3d point
     *   Let k be a proportionality constant related to the projection and pixellization process
     *   Let S = k*d*s
     *   The pixel on camera 1 retroprojects a conic PDF with width S = k*s*d
     *   The line on camera 2 retroprojects a plane aperture of S = k*s*d
     *   The product (ie intersection) of cone and plane aperture PDFs is a sphere of radius S
     *   Projection of the sphere to camera 3 is a circle of S/k/d=s pixels
     *   This is the projected covariance: s^2 pixel^2
     *   The measurement has a std dev of s pixel --> cov is s^2 pixel^2
     *   The total cov is s^2 pix^2 + s^2 pix^2 = 2s^2 pix^2
     *   The info matrix is 0.5 s^-2 pix^-2
     *   The sqrt info matrix is 1/s/sqrt(2) pix^-1
     */
    Matrix3d sqrt_info_gt = Matrix3d::Identity() / pixel_noise_std / sqrt(2.0);

    ASSERT_MATRIX_APPROX(c123->getSqrtInformationUpper(), sqrt_info_gt, 1e-8);

}

TEST_F(FactorTrifocalTest, expectation)
{
    //    Homogeneous transform C2 wrt C1
    Matrix4d _c1Hc2; _c1Hc2 <<
               0,  0, -1,  1,
               0,  1,  0,  0,
               1,  0,  0,  1,
               0,  0,  0,  1;

    // rotation and translation
    Matrix3d _c1Rc2 = _c1Hc2.block(0,0,3,3);
    Vector3d _c1Tc2 = _c1Hc2.block(0,3,3,1);

    // Essential matrix, ground truth (fwd and bkwd)
    Matrix3d _c1Ec2 = wolf::skew(_c1Tc2) * _c1Rc2;
    Matrix3d _c2Ec1 = _c1Ec2.transpose();

    // Expected values
    vision_utils::TrifocalTensor tensor;
    Matrix3d c2Ec1;
    c123->expectation(pos1, quat1, pos2, quat2, pos3, quat3, pos_cam, quat_cam, tensor, c2Ec1);

    // check trilinearities

    // Elements computed using the tensor
    Matrix3d T0, T1, T2;
    tensor.getLayers(T0,T1,T2);
    Vector3d _m1 (0,0,1),
             _m2 (0,0,1),
             _m3 (0,0,1); // ground truth
    Matrix3d m1Tt = _m1(0)*T0.transpose()
                  + _m1(1)*T1.transpose()
                  + _m1(2)*T2.transpose();

    // Projective line: l = (nx ny dn), with (nx ny): normal vector; dn: distance to origin times norm(nx,ny)
    Vector3d _l2(0,1,0),
             _p2(1,0,0),
             _p3(0,1,0); // ground truth
    Vector3d l2;
    l2 = c2Ec1 * _m1;

    // check epipolar lines (check only director vectors for equal direction)
    ASSERT_MATRIX_APPROX(l2/l2(1), _l2/_l2(1), 1e-8);

    // check perpendicular lines (check only director vectors for orthogonal direction)
    ASSERT_NEAR(l2(0)*_p2(0) + l2(1)*_p2(1), 0, 1e-8);

    // Verify trilinearities

    // Point-line-line
    Matrix1d pll = _p3.transpose() * m1Tt * _p2;
    ASSERT_TRUE(pll(0)<1e-5);

    // Point-line-point
    Vector3d plp = wolf::skew(_m3) * m1Tt * _p2;
    ASSERT_MATRIX_APPROX(plp, Vector3d::Zero(), 1e-8);

    // Point-point-line
    Vector3d ppl = _p3.transpose() * m1Tt * wolf::skew(_m2);
    ASSERT_MATRIX_APPROX(ppl, Vector3d::Zero(), 1e-8);

    // Point-point-point
    Matrix3d ppp = wolf::skew(_m3) * m1Tt * wolf::skew(_m2);
    ASSERT_MATRIX_APPROX(ppp, Matrix3d::Zero(), 1e-8);

    // check epipolars
    ASSERT_MATRIX_APPROX(c2Ec1/c2Ec1(0,1), _c2Ec1/_c2Ec1(0,1), 1e-8);
}

TEST_F(FactorTrifocalTest, residual)
{
    vision_utils::TrifocalTensor tensor;
    Matrix3d c2Ec1;
    Vector3d residual;

    // Nominal values
    c123->expectation(pos1, quat1, pos2, quat2, pos3, quat3, pos_cam, quat_cam, tensor, c2Ec1);
    residual = c123->residual(tensor, c2Ec1);

    ASSERT_MATRIX_APPROX(residual, Vector3d::Zero(), 1e-8);
}

TEST_F(FactorTrifocalTest, error_jacobians)
{
    vision_utils::TrifocalTensor tensor;
    Matrix3d c2Ec1;
    Vector3d residual, residual_pert;
    Vector3d pix0, pert, pix_pert;
    double epsilon = 1e-8;

    // Nominal values
    c123->expectation(pos1, quat1, pos2, quat2, pos3, quat3, pos_cam, quat_cam, tensor, c2Ec1);
    residual = c123->residual(tensor, c2Ec1);

    Matrix<double, 3, 3> J_e_m1, J_e_m2, J_e_m3, J_r_m1, J_r_m2, J_r_m3;
    c123->error_jacobians(tensor, c2Ec1, J_e_m1, J_e_m2, J_e_m3);
    J_r_m1 = c123->getSqrtInformationUpper() * J_e_m1;
    J_r_m2 = c123->getSqrtInformationUpper() * J_e_m2;
    J_r_m3 = c123->getSqrtInformationUpper() * J_e_m3;

    // numerical jacs
    Matrix<double,3,3> Jn_r_m1, Jn_r_m2, Jn_r_m3;

    // jacs wrt m1
    pix0 = c123->getPixelCanonical1();
    for (int i=0; i<3; i++)
    {
        pert.setZero();
        pert(i)  = epsilon;
        pix_pert = pix0 + pert;
        c123->setPixelCanonical1(pix_pert); // m1
        residual_pert = c123->residual(tensor, c2Ec1);

        Jn_r_m1.col(i) = (residual_pert - residual) / epsilon;
    }
    c123->setPixelCanonical1(pix0);

    ASSERT_MATRIX_APPROX(J_r_m1, Jn_r_m1, 1e-6);

    // jacs wrt m2
    pix0 = c123->getPixelCanonical2();
    for (int i=0; i<3; i++)
    {
        pert.setZero();
        pert(i)  = epsilon;
        pix_pert = pix0 + pert;
        c123->setPixelCanonical2(pix_pert); // m2
        residual_pert = c123->residual(tensor, c2Ec1);

        Jn_r_m2.col(i) = (residual_pert - residual) / epsilon;
    }
    c123->setPixelCanonical2(pix0);

    ASSERT_MATRIX_APPROX(J_r_m2, Jn_r_m2, 1e-6);

    // jacs wrt m3
    pix0 = c123->getPixelCanonical3();
    for (int i=0; i<3; i++)
    {
        pert.setZero();
        pert(i)  = epsilon;
        pix_pert = pix0 + pert;
        c123->setPixelCanonical3(pix_pert); // m3
        residual_pert = c123->residual(tensor, c2Ec1);

        Jn_r_m3.col(i) = (residual_pert - residual) / epsilon;
    }
    c123->setPixelCanonical3(pix0);

    ASSERT_MATRIX_APPROX(J_r_m3, Jn_r_m3, 1e-6);

}

TEST_F(FactorTrifocalTest, operator_parenthesis)
{
    Vector3d res;

    // Factor with exact states should give zero residual
    c123->operator ()(pos1.data(), vquat1.data(),
                      pos2.data(), vquat2.data(),
                      pos3.data(), vquat3.data(),
                      pos_cam.data(), vquat_cam.data(),
                      res.data());

    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);
}

TEST_F(FactorTrifocalTest, solve_F1)
{
    F1->setState("PO", { pose1.head(3), pose1.tail(4) } );
    F2->setState("PO", { pose2.head(3), pose2.tail(4) } );
    F3->setState("PO", { pose3.head(3), pose3.tail(4) } );
    S ->getP()->setState(pos_cam);
    S ->getO()->setState(vquat_cam);
    // Residual with prior

    Vector3d res;

    Eigen::VectorXd F1_p = F1->getP()->getState();
    Eigen::VectorXd F1_o = F1->getO()->getState();
    Eigen::VectorXd F2_p = F2->getP()->getState();
    Eigen::VectorXd F2_o = F2->getO()->getState();
    Eigen::VectorXd F3_p = F3->getP()->getState();
    Eigen::VectorXd F3_o = F3->getO()->getState();
    Eigen::VectorXd S_p  = S ->getP()->getState();
    Eigen::VectorXd S_o  = S ->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("Initial state:              ", F1->getState().transpose());
    WOLF_DEBUG("residual before perturbing: ", res.transpose());
    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);

    // Residual with perturbated state

//    Vector7d pose_perturbated = F1->getState() + 0.1 * Vector7d::Random();
//    pose_perturbated.segment(3,4).normalize();
//    F1->setState(pose_perturbated);
    F1->perturb(0.1);
//    VectorXd pose_perturbated = F1->getState().vector("PO");

    F1_p = F1->getP()->getState();
    F1_o = F1->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("perturbed state:            ", F1->getState().vector("PO").transpose());
    WOLF_DEBUG("residual before solve:      ", res.transpose());

    // Residual with solved state

    S ->fixExtrinsics();
    F1->unfix();
    F2->fix();
    F3->fix();

    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF);

    F1_p = F1->getP()->getState();
    F1_o = F1->getO()->getState();
    F2_p = F2->getP()->getState();
    F2_o = F2->getO()->getState();
    F3_p = F3->getP()->getState();
    F3_o = F3->getO()->getState();
    S_p  = S ->getP()->getState();
    S_o  = S ->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("solved state:               ", F1->getState().vector("PO").transpose());
    WOLF_DEBUG("residual after solve:       ", res.transpose());

    WOLF_DEBUG(report, " AND UNION");

    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);

}

TEST_F(FactorTrifocalTest, solve_F2)
{
    F1->setState("PO", { pose1.head(3), pose1.tail(4) } );
    F2->setState("PO", { pose2.head(3), pose2.tail(4) } );
    F3->setState("PO", { pose3.head(3), pose3.tail(4) } );
    S ->getP()->setState(pos_cam);
    S ->getO()->setState(vquat_cam);

    // Residual with prior

    Vector3d res;

    Eigen::VectorXd F1_p = F1->getP()->getState();
    Eigen::VectorXd F1_o = F1->getO()->getState();
    Eigen::VectorXd F2_p = F2->getP()->getState();
    Eigen::VectorXd F2_o = F2->getO()->getState();
    Eigen::VectorXd F3_p = F3->getP()->getState();
    Eigen::VectorXd F3_o = F3->getO()->getState();
    Eigen::VectorXd S_p  = S ->getP()->getState();
    Eigen::VectorXd S_o  = S ->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("Initial state:              ", F2->getState().vector("PO").transpose());
    WOLF_DEBUG("residual before perturbing: ", res.transpose());
    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);

    // Residual with perturbated state

    F2->perturb(0.1);

    F2_p = F2->getP()->getState();
    F2_o = F2->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("perturbed state:            ", F2->getState().vector("PO").transpose());
    WOLF_DEBUG("residual before solve:      ", res.transpose());

    // Residual with solved state

    S ->fixExtrinsics();
    F1->fix();
    F2->unfix();
    F3->fix();

    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF);

    F1_p = F1->getP()->getState();
    F1_o = F1->getO()->getState();
    F2_p = F2->getP()->getState();
    F2_o = F2->getO()->getState();
    F3_p = F3->getP()->getState();
    F3_o = F3->getO()->getState();
    S_p  = S ->getP()->getState();
    S_o  = S ->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("solved state:               ", F2->getState().vector("PO").transpose());
    WOLF_DEBUG("residual after solve:       ", res.transpose());

    WOLF_DEBUG(report, " AND UNION");

    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);

}

TEST_F(FactorTrifocalTest, solve_F3)
{
    F1->setState("PO", { pose1.head(3), pose1.tail(4) } );
    F2->setState("PO", { pose2.head(3), pose2.tail(4) } );
    F3->setState("PO", { pose3.head(3), pose3.tail(4) } );
    S ->getP()->setState(pos_cam);
    S ->getO()->setState(vquat_cam);

    // Residual with prior

    Vector3d res;

    Eigen::VectorXd F1_p = F1->getP()->getState();
    Eigen::VectorXd F1_o = F1->getO()->getState();
    Eigen::VectorXd F2_p = F2->getP()->getState();
    Eigen::VectorXd F2_o = F2->getO()->getState();
    Eigen::VectorXd F3_p = F3->getP()->getState();
    Eigen::VectorXd F3_o = F3->getO()->getState();
    Eigen::VectorXd S_p  = S ->getP()->getState();
    Eigen::VectorXd S_o  = S ->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("Initial state:              ", F3->getState().vector("PO").transpose());
    WOLF_DEBUG("residual before perturbing: ", res.transpose());
    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);

    // Residual with perturbated state

    F3->perturb(0.1);

    F3_p = F3->getP()->getState();
    F3_o = F3->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("perturbed state:            ", F3->getState().vector("PO").transpose());
    WOLF_DEBUG("residual before solve:      ", res.transpose());
    ASSERT_NEAR(res(2), 0, 1e-8); // Epipolar c2-c1 should be respected when perturbing F3

    // Residual with solved state

    S ->fixExtrinsics();
    F1->fix();
    F2->fix();
    F3->unfix();

    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF);

    F1_p = F1->getP()->getState();
    F1_o = F1->getO()->getState();
    F2_p = F2->getP()->getState();
    F2_o = F2->getO()->getState();
    F3_p = F3->getP()->getState();
    F3_o = F3->getO()->getState();
    S_p  = S ->getP()->getState();
    S_o  = S ->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("solved state:               ", F3->getState().vector("PO").transpose());
    WOLF_DEBUG("residual after solve:       ", res.transpose());

    WOLF_DEBUG(report, " AND UNION");

    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);

}

TEST_F(FactorTrifocalTest, solve_S)
{
    F1->setState(pose1, "PO", {3,4});
    F2->setState(pose2, "PO", {3,4});
    F3->setState(pose3, "PO", {3,4});
    S ->getP()->setState(pos_cam);
    S ->getO()->setState(vquat_cam);

    // Residual with prior

    Vector3d res;

    Eigen::VectorXd F1_p = F1->getP()->getState();
    Eigen::VectorXd F1_o = F1->getO()->getState();
    Eigen::VectorXd F2_p = F2->getP()->getState();
    Eigen::VectorXd F2_o = F2->getO()->getState();
    Eigen::VectorXd F3_p = F3->getP()->getState();
    Eigen::VectorXd F3_o = F3->getO()->getState();
    Eigen::VectorXd S_p  = S ->getP()->getState();
    Eigen::VectorXd S_o  = S ->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("Initial state:              ", S->getP()->getState().transpose(), " ", S->getO()->getState().transpose());
    WOLF_DEBUG("residual before perturbing: ", res.transpose());
    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);

    // Residual with perturbated state

    S->perturb(0.1);

    S_p = S->getP()->getState();
    S_o = S->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("perturbed state:            ", S->getState().vector("PO").transpose());
    WOLF_DEBUG("residual before solve:      ", res.transpose());

    // Residual with solved state

    S ->unfixExtrinsics();
    F1->fix();
    F2->fix();
    F3->fix();

    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF);

    F1_p = F1->getP()->getState();
    F1_o = F1->getO()->getState();
    F2_p = F2->getP()->getState();
    F2_o = F2->getO()->getState();
    F3_p = F3->getP()->getState();
    F3_o = F3->getO()->getState();
    S_p  = S ->getP()->getState();
    S_o  = S ->getO()->getState();

    c123->operator ()(F1_p.data(), F1_o.data(),
                      F2_p.data(), F2_o.data(),
                      F3_p.data(), F3_o.data(),
                      S_p. data(), S_o. data(),
                      res.data());

    WOLF_DEBUG("solved state:               ", S->getP()->getState().transpose(), " ", S->getO()->getState().transpose());
    WOLF_DEBUG("residual after solve:       ", res.transpose());

    WOLF_DEBUG(report, " AND UNION");

    ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);

}

class FactorTrifocalMultiPointTest : public FactorTrifocalTest
{
        /*
         * In this test class we add 8 more points and perform optimization on the camera frames.
         *
         * C1 is not optimized as it acts as the reference
         * S  is not optimized either as observability is compromised
         * C2.pos is fixed to set the unobservable scale
         * C2.ori and all C3 are optimized
         *
         */

    public:
        std::vector<FeatureBasePtr> fv1, fv2, fv3;
        std::vector<FactorTrifocalPtr> cv123;

        void SetUp() override
        {
            FactorTrifocalTest::SetUp();

            Matrix<double, 2, 9> c1p_can;
            c1p_can <<
                    0,   -1/3.00,   -1/3.00,    1/3.00,    1/3.00,   -1.0000,   -1.0000, 1.0000,    1.0000,
                    0,    1/3.00,   -1/3.00,    1/3.00,   -1/3.00,    1.0000,   -1.0000, 1.0000,   -1.0000;

            Matrix<double, 2, 9> c2p_can;
            c2p_can <<
                    0,    1/3.00,    1/3.00,    1.0000,    1.0000,   -1/3.00,   -1/3.00,   -1.0000,   -1.0000,
                    0,    1/3.00,   -1/3.00,    1.0000,   -1.0000,    1/3.00,   -1/3.00,    1.0000,   -1.0000;

            Matrix<double, 2, 9> c3p_can;
            c3p_can <<
                    0,   -1/3.00,   -1.0000,    1/3.00,    1.0000,   -1/3.00,   -1.0000,   1/3.00,    1.0000,
                    0,   -1/3.00,   -1.0000,   -1/3.00,   -1.0000,    1/3.00,    1.0000,   1/3.00,    1.0000;

            Matrix2d pix_cov; pix_cov.setIdentity(); //pix_cov *= 1e-4;

            // for i==0 we already have them
            fv1.push_back(f1);
            fv2.push_back(f2);
            fv3.push_back(f3);
            cv123.push_back(c123);
            for (size_t i=1; i<c1p_can.cols() ; i++)
            {
                // fv1.push_back(std::make_shared<FeatureBase>("PIXEL", c1p_can.col(i), pix_cov));
                auto f  = FeatureBase::emplace<FeatureBase>(I1, "PIXEL", c1p_can.col(i), pix_cov);
                fv1.push_back(f);
                // I1->addFeature(fv1.at(i));

                // fv2.push_back(std::make_shared<FeatureBase>("PIXEL", c2p_can.col(i), pix_cov));
                auto f2  = FeatureBase::emplace<FeatureBase>(I2, "PIXEL", c2p_can.col(i), pix_cov);
                fv2.push_back(f2);
                // I2->addFeature(fv2.at(i));

                // fv3.push_back(std::make_shared<FeatureBase>("PIXEL", c3p_can.col(i), pix_cov));
                auto f3  = FeatureBase::emplace<FeatureBase>(I3, "PIXEL", c3p_can.col(i), pix_cov);
                fv3.push_back(f3);
                // I3->addFeature(fv3.at(i));

                auto ff = std::static_pointer_cast<FactorTrifocal>(FactorBase::emplace<FactorTrifocal>(fv3.at(i), fv1.at(i), fv2.at(i), fv3.at(i), proc_trifocal, false, FAC_ACTIVE));
                cv123.push_back(ff);
                // fv3.at(i)->addFactor(cv123.at(i));
                // fv1.at(i)->addConstrainedBy(cv123.at(i));
                // fv2.at(i)->addConstrainedBy(cv123.at(i));
            }

        }

};

TEST_F(FactorTrifocalMultiPointTest, solve_multi_point)
{
    /*
     * In this test we add 8 more points and perform optimization on the camera frames.
     *
     * C1 is not optimized as it acts as the reference
     * S  is not optimized either as observability is compromised
     * C2.pos is fixed to set the unobservable scale
     * C2.ori and all C3 are optimized
     *
     */

    S ->getP()->fix(); // do not calibrate sensor pos
    S ->getO()->fix(); // do not calibrate sensor ori
    F1->getP()->fix(); // Cam 1 acts as reference
    F1->getO()->fix(); // Cam 1 acts as reference
    F2->getP()->fix(); // This fixes the scale
    F2->getO()->unfix(); // Estimate Cam 2 ori
    F3->getP()->unfix(); // Estimate Cam 3 pos
    F3->getO()->unfix(); // Estimate Cam 3 ori

    // Perturbate states, keep scale
    F1->getP()->setState( pos1   );
    F1->getO()->setState( vquat1 );
    F2->getP()->setState( pos2   ); // this fixes the scale
    F2->getO()->setState((vquat2 + 0.2*Vector4d::Random()).normalized());
    F3->getP()->setState( pos3   + 0.2*Vector3d::Random());
    F3->getO()->setState((vquat3 + 0.2*Vector4d::Random()).normalized());

    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF);

    // Print results
    WOLF_DEBUG("report: ", report);
    problem->print(1,0,1,0);

    // Evaluate final states
    ASSERT_MATRIX_APPROX(F2->getP()->getState(), pos2  , 1e-6);
    ASSERT_MATRIX_APPROX(F2->getO()->getState(), vquat2, 1e-6);
    ASSERT_MATRIX_APPROX(F3->getP()->getState(), pos3  , 1e-6);
    ASSERT_MATRIX_APPROX(F3->getO()->getState(), vquat3, 1e-6);

    Eigen::VectorXd F1_p = F1->getP()->getState();
    Eigen::VectorXd F1_o = F1->getO()->getState();
    Eigen::VectorXd F2_p = F2->getP()->getState();
    Eigen::VectorXd F2_o = F2->getO()->getState();
    Eigen::VectorXd F3_p = F3->getP()->getState();
    Eigen::VectorXd F3_o = F3->getO()->getState();
    Eigen::VectorXd S_p  = S ->getP()->getState();
    Eigen::VectorXd S_o  = S ->getO()->getState();

    // evaluate residuals
    Vector3d res;
    for (size_t i=0; i<cv123.size(); i++)
    {
        cv123.at(i)->operator ()(F1_p.data(), F1_o.data(),
                                 F2_p.data(), F2_o.data(),
                                 F3_p.data(), F3_o.data(),
                                 S_p. data(), S_o. data(),
                                 res.data());

        ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);
    }

}

TEST_F(FactorTrifocalMultiPointTest, solve_multi_point_scale)
{
    /*
     * In this test we add 8 more points and perform optimization on the camera frames.
     *
     * C1 is not optimized as it acts as the reference
     * S  is not optimized either as observability is compromised
     * C2.pos is fixed to set the unobservable scale
     * C2.ori and all C3 are optimized
     *
     */

    S ->getP()->fix(); // do not calibrate sensor pos
    S ->getO()->fix(); // do not calibrate sensor ori
    F1->getP()->fix(); // Cam 1 acts as reference
    F1->getO()->fix(); // Cam 1 acts as reference
    F2->getP()->fix(); // This fixes the scale
    F2->getO()->unfix(); // Estimate Cam 2 ori
    F3->getP()->unfix(); // Estimate Cam 3 pos
    F3->getO()->unfix(); // Estimate Cam 3 ori

    // Perturbate states, change scale
    F1->getP()->setState( 2 * pos1 );
    F1->getO()->setState(   vquat1 );
    F2->getP()->setState( 2 * pos2 );
    F2->getO()->setState((  vquat2 + 0.2*Vector4d::Random()).normalized());
    F3->getP()->setState( 2 * pos3 + 0.2*Vector3d::Random());
    F3->getO()->setState((  vquat3 + 0.2*Vector4d::Random()).normalized());

    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF);

    // Print results
    WOLF_DEBUG("report: ", report);
    problem->print(1,0,1,0);

    // Evaluate final states
    ASSERT_MATRIX_APPROX(F2->getP()->getState(), 2 * pos2, 1e-8);
    ASSERT_MATRIX_APPROX(F2->getO()->getState(),   vquat2, 1e-8);
    ASSERT_MATRIX_APPROX(F3->getP()->getState(), 2 * pos3, 1e-8);
    ASSERT_MATRIX_APPROX(F3->getO()->getState(),   vquat3, 1e-8);

    Eigen::VectorXd F1_p = F1->getP()->getState();
    Eigen::VectorXd F1_o = F1->getO()->getState();
    Eigen::VectorXd F2_p = F2->getP()->getState();
    Eigen::VectorXd F2_o = F2->getO()->getState();
    Eigen::VectorXd F3_p = F3->getP()->getState();
    Eigen::VectorXd F3_o = F3->getO()->getState();
    Eigen::VectorXd S_p  = S ->getP()->getState();
    Eigen::VectorXd S_o  = S ->getO()->getState();

    // evaluate residuals
    Vector3d res;
    for (size_t i=0; i<cv123.size(); i++)
    {
        cv123.at(i)->operator ()(F1_p.data(), F1_o.data(),
                                 F2_p.data(), F2_o.data(),
                                 F3_p.data(), F3_o.data(),
                                 S_p. data(), S_o. data(),
                                 res.data());

        ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);
    }
}

#include "core/factor/factor_autodiff_distance_3d.h"

TEST_F(FactorTrifocalMultiPointTest, solve_multi_point_distance)
{
    /*
     * In this test we add 8 more points and perform optimization on the camera frames.
     *
     * C1 is not optimized as it acts as the reference
     * S  is not optimized either as observability is compromised
     * C2 and C3 are optimized
     * The scale is observed through a distance factor
     *
     */

    S ->getP()->fix(); // do not calibrate sensor pos
    S ->getO()->fix(); // do not calibrate sensor ori
    F1->getP()->fix(); // Cam 1 acts as reference
    F1->getO()->fix(); // Cam 1 acts as reference
    F2->getP()->unfix(); // Estimate Cam 2 pos
    F2->getO()->unfix(); // Estimate Cam 2 ori
    F3->getP()->unfix(); // Estimate Cam 3 pos
    F3->getO()->unfix(); // Estimate Cam 3 ori

    // Perturbate states, change scale
    F1->getP()->setState( pos1   );
    F1->getO()->setState( vquat1 );
    F2->getP()->setState( pos2   + 0.2*Vector3d::Random() );
    F2->getO()->setState((vquat2 + 0.2*Vector4d::Random()).normalized());
    F3->getP()->setState( pos3   + 0.2*Vector3d::Random());
    F3->getO()->setState((vquat3 + 0.2*Vector4d::Random()).normalized());

    // Add a distance factor to fix the scale
    double distance     = sqrt(2.0);
    double distance_std = 0.1;

    auto Cd = CaptureBase::emplace<CaptureBase>(F3, "DISTANCE", F3->getTimeStamp());
    // CaptureBasePtr Cd = std::make_shared<CaptureBase>("DISTANCE", F3->getTimeStamp());
    // F3->addCapture(Cd);
    auto fd = FeatureBase::emplace<FeatureBase>(Cd, "DISTANCE", Vector1d(distance), Matrix1d(distance_std * distance_std));
    // FeatureBasePtr fd = std::make_shared<FeatureBase>("DISTANCE", Vector1d(distance), Matrix1d(distance_std * distance_std));
    // Cd->addFeature(fd);
    auto cd = FactorBase::emplace<FactorAutodiffDistance3d>(fd, fd, F1, nullptr, false, FAC_ACTIVE);
    // FACTORAUTODIFFDISTANCE3dPTR cd = std::make_shared<FactorAutodiffDistance3d>(fd, F1, nullptr, false, FAC_ACTIVE);
    // fd->addFactor(cd);
    // F1->addConstrainedBy(cd);

    cd->setStatus(FAC_INACTIVE);
    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF);
    WOLF_DEBUG("DISTANCE CONSTRAINT INACTIVE: \n", report);

    problem->print(1,0,1,0);

    cd->setStatus(FAC_ACTIVE);
    report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF);

    // Print results
    WOLF_DEBUG("DISTANCE CONSTRAINT ACTIVE: \n", report);
    problem->print(1,0,1,0);

    // Evaluate final states
    ASSERT_MATRIX_APPROX(F2->getP()->getState(), pos2  , 1e-8);
    ASSERT_MATRIX_APPROX(F2->getO()->getState(), vquat2, 1e-8);
    ASSERT_MATRIX_APPROX(F3->getP()->getState(), pos3  , 1e-8);
    ASSERT_MATRIX_APPROX(F3->getO()->getState(), vquat3, 1e-8);

    Eigen::VectorXd F1_p = F1->getP()->getState();
    Eigen::VectorXd F1_o = F1->getO()->getState();
    Eigen::VectorXd F2_p = F2->getP()->getState();
    Eigen::VectorXd F2_o = F2->getO()->getState();
    Eigen::VectorXd F3_p = F3->getP()->getState();
    Eigen::VectorXd F3_o = F3->getO()->getState();
    Eigen::VectorXd S_p  = S ->getP()->getState();
    Eigen::VectorXd S_o  = S ->getO()->getState();

    // evaluate residuals
    Vector3d res;
    for (size_t i=0; i<cv123.size(); i++)
    {
        cv123.at(i)->operator ()(F1_p.data(), F1_o.data(),
                                 F2_p.data(), F2_o.data(),
                                 F3_p.data(), F3_o.data(),
                                 S_p. data(), S_o. data(),
                                 res.data());

        ASSERT_MATRIX_APPROX(res, Vector3d::Zero(), 1e-8);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    //    ::testing::GTEST_FLAG(filter) = "FactorTrifocalTest.solve_F1";
    //    ::testing::GTEST_FLAG(filter) = "FactorTrifocalTest.solve_F2";
    //    ::testing::GTEST_FLAG(filter) = "FactorTrifocalTest.solve_F3";
    //    ::testing::GTEST_FLAG(filter) = "FactorTrifocalTest.solve_S";
    //    ::testing::GTEST_FLAG(filter) = "FactorTrifocalTest.solve_multi_point";
    //    ::testing::GTEST_FLAG(filter) = "FactorTrifocalMultiPointTest.solve_multi_point_distance";
    return RUN_ALL_TESTS();
}

