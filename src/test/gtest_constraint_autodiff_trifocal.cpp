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

        SensorBasePtr S;
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
            euler1 << 0, 0, M_PI; //euler1 *= M_TORAD;
            euler2 << 0, 0, -M_PI_2; //euler2 *= M_TORAD;
            euler3 <<0, M_PI_2, M_PI ;// euler3 *= M_TORAD;
            quat1 = e2q(euler1);
            quat2 = e2q(euler2);
            quat3 = e2q(euler3);

            // camera at the robot origin looking forward
            pos_cam << 0,0,0;
            euler_cam << -M_PI_2, 0, -M_PI_2;// euler_cam *= M_TORAD;
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
            S = problem->installSensor("CAMERA", "canonical", pose_cam, wolf_root + "/src/examples/camera_params_canonical.yaml");
            camera = std::static_pointer_cast<SensorCamera>(S);

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
        }
};

TEST_F(ConstraintAutodiffTrifocalTest, expectation)
{
    // ground truth essential matrices

    //    Homogeneous transform C2 wrt C1
    Matrix4s _c1Hc2; _c1Hc2 <<
               0,  0, -1,  1,
               0,  1,  0,  0,
               1,  0,  0,  1,
               0,  0,  0,  1;

    Matrix3s _c1Rc2 = _c1Hc2.block(0,0,3,3);

    Vector3s _c1Tc2 = _c1Hc2.block(0,3,3,1);

    Matrix3s _c1Ec2 = wolf::skew(_c1Tc2) * _c1Rc2;

    Matrix3s _c2Ec1 = _c1Ec2.transpose();


    // expected values
    vision_utils::TrifocalTensor tensor;
    Matrix3s c2Ec1;
    c123->expectation(pos1, quat1, pos2, quat2, pos3, quat3, pos_cam, quat_cam, tensor, c2Ec1);

    // check trilinearities

    // Elements computed using the tensor
    Eigen::Matrix3s T0, T1, T2;
    tensor.getLayers(T0,T1,T2);
    Vector3s _m1 (0,0,1), _m2 (0,0,1), _m3 (0,0,1); // ground truth
    Matrix3s m1Tt = _m1(0)*T0.transpose() + _m1(1)*T1.transpose() + _m1(2)*T2.transpose();

    // Projective line: l = (nx ny dn), with (nx ny): normal vector; dn: distance to origin times norm(nx,ny)
    Vector3s _l2(0,1,0), _p2(1,0,0), _p3(0,1,0); // ground truth
    Vector3s l2;
    l2 = c2Ec1 * _m1;

    // check epipolar lines (check only director vectors for equal direction)
    ASSERT_MATRIX_APPROX(l2/l2(1), _l2/_l2(1), 1e-8);

    // check perpendicular lines (check only director vectors for orthogonal direction)
    ASSERT_NEAR(l2(0)*_p2(0) + l2(1)*_p2(1), 0, 1e-8);

    // Verify trilinearities

    // Point-line-line
    Eigen::Matrix1s pll = _p3.transpose() * m1Tt * _p2;
    ASSERT_TRUE(pll(0)<1e-5);

    // Point-line-point
    Eigen::Vector3s plp = wolf::skew(_m3) * m1Tt * _p2;
    ASSERT_MATRIX_APPROX(plp, Vector3s::Zero(), 1e-8);

    // Point-point-line
    Eigen::Vector3s ppl = _p3.transpose() * m1Tt * wolf::skew(_m2);
    ASSERT_MATRIX_APPROX(ppl, Vector3s::Zero(), 1e-8);

    // Point-point-point
    Eigen::Matrix3s ppp = wolf::skew(_m3) * m1Tt * wolf::skew(_m2);
    ASSERT_MATRIX_APPROX(ppp, Matrix3s::Zero(), 1e-8);

    // check epipolars
    ASSERT_MATRIX_APPROX(c2Ec1/c2Ec1(0,1), _c2Ec1/_c2Ec1(0,1), 1e-8);
}

TEST_F(ConstraintAutodiffTrifocalTest, residual)
{
    vision_utils::TrifocalTensor tensor;
    Matrix3s c2Ec1;
    Vector3s residual;

    // Nominal values
    c123->expectation(pos1, quat1, pos2, quat2, pos3, quat3, pos_cam, quat_cam, tensor, c2Ec1);
    residual = c123->residual(tensor, c2Ec1);

    ASSERT_MATRIX_APPROX(residual, Vector3s::Zero(), 1e-8);
}

TEST_F(ConstraintAutodiffTrifocalTest, error_jacobians)
{
    vision_utils::TrifocalTensor tensor;
    Matrix3s c2Ec1;
    Vector3s residual, residual_pert;
    Vector3s pix0, pert, pix_pert;
    Scalar epsilon = 1e-8;

    // Nominal values
    c123->expectation(pos1, quat1, pos2, quat2, pos3, quat3, pos_cam, quat_cam, tensor, c2Ec1);
    residual = c123->residual(tensor, c2Ec1);

    Matrix<Scalar, 3, 3> J_e_m1, J_e_m2, J_e_m3, J_r_m1, J_r_m2, J_r_m3;
    c123->error_jacobians(tensor, c2Ec1, J_e_m1, J_e_m2, J_e_m3);
    J_r_m1            = c123->getSqrtInformationUpper() * J_e_m1;
    J_r_m2            = c123->getSqrtInformationUpper() * J_e_m2;
    J_r_m3            = c123->getSqrtInformationUpper() * J_e_m3;

    // numerical jacs
    Matrix<Scalar,3,3> Jn_r_m1, Jn_r_m2, Jn_r_m3;

    // jacs wrt m1
    pix0 = c123->getPixelCanonicalPrev();
    for (int i=0; i<3; i++)
    {
        pert.setZero();
        pert(i)  = epsilon;
        pix_pert = pix0 + pert;
        c123->setPixelCanonicalPrev(pix_pert); // m1
        residual_pert = c123->residual(tensor, c2Ec1);

        Jn_r_m1.col(i) = (residual_pert - residual) / epsilon;
    }
    c123->setPixelCanonicalPrev(pix0);

    ASSERT_MATRIX_APPROX(J_r_m1, Jn_r_m1, 1e-6);


    // jacs wrt m2
    pix0 = c123->getPixelCanonicalOrigin();
    for (int i=0; i<3; i++)
    {
        pert.setZero();
        pert(i)  = epsilon;
        pix_pert = pix0 + pert;
        c123->setPixelCanonicalOrigin(pix_pert); // m2
        residual_pert = c123->residual(tensor, c2Ec1);

        Jn_r_m2.col(i) = (residual_pert - residual) / epsilon;
    }
    c123->setPixelCanonicalOrigin(pix0);

    ASSERT_MATRIX_APPROX(J_r_m2, Jn_r_m2, 1e-6);


    // jacs wrt m3
    pix0 = c123->getPixelCanonicalLast();
    for (int i=0; i<3; i++)
    {
        pert.setZero();
        pert(i)  = epsilon;
        pix_pert = pix0 + pert;
        c123->setPixelCanonicalLast(pix_pert); // m3
        residual_pert = c123->residual(tensor, c2Ec1);

        Jn_r_m3.col(i) = (residual_pert - residual) / epsilon;
    }
    c123->setPixelCanonicalLast(pix0);

    ASSERT_MATRIX_APPROX(J_r_m3, Jn_r_m3, 1e-6);

}

TEST_F(ConstraintAutodiffTrifocalTest, operator_parenthesis)
{
    Vector3s res;

    // Constraint with exact states should give zero residual
    c123->operator ()(pos1.data(), quat1.coeffs().data(),
                      pos2.data(), quat2.coeffs().data(),
                      pos3.data(), quat3.coeffs().data(),
                      pos_cam.data(), quat_cam.coeffs().data(),
                      res.data());

    ASSERT_MATRIX_APPROX(res, Vector3s::Zero(), 1e-8);
}

TEST_F(ConstraintAutodiffTrifocalTest, solve_F3)
{
    // Residual with prior

    Vector3s res;

    c123->operator ()(F1->getPPtr()->getPtr(), F1->getOPtr()->getPtr(),
                      F2->getPPtr()->getPtr(), F2->getOPtr()->getPtr(),
                      F3->getPPtr()->getPtr(), F3->getOPtr()->getPtr(),
                      S ->getPPtr()->getPtr(), S ->getOPtr()->getPtr(),
                      res.data());

    WOLF_INFO("Initial state:   ", F3->getState().transpose());
    WOLF_INFO("residual before perturbing: ", res.transpose());
    ASSERT_MATRIX_APPROX(res, Vector3s::Zero(), 1e-8);

    // Residual with perturbated state

    Vector7s pose_perturbated = F3->getState() + 0.1 * Vector7s::Random();
    pose_perturbated.segment(3,4).normalize();
    F3->setState(pose_perturbated);

    c123->operator ()(F1->getPPtr()->getPtr(), F1->getOPtr()->getPtr(),
                      F2->getPPtr()->getPtr(), F2->getOPtr()->getPtr(),
                      F3->getPPtr()->getPtr(), F3->getOPtr()->getPtr(),
                      S ->getPPtr()->getPtr(), S ->getOPtr()->getPtr(),
                      res.data());

    WOLF_INFO("perturbed state: ", pose_perturbated.transpose());
    WOLF_INFO("residual before solve:      ", res.transpose());
    ASSERT_NEAR(res(2), 0, 1e-8); // Epipolar c2-c1 should be respected when perturbing F3

    // Residual with solved state

    S ->fix();
    F1->fix();
    F2->fix();

    std::string report = ceres_manager->solve(1);

    c123->operator ()(F1->getPPtr()->getPtr(), F1->getOPtr()->getPtr(),
                      F2->getPPtr()->getPtr(), F2->getOPtr()->getPtr(),
                      F3->getPPtr()->getPtr(), F3->getOPtr()->getPtr(),
                      S ->getPPtr()->getPtr(), S ->getOPtr()->getPtr(),
                      res.data());

    WOLF_INFO("solved state:    ", F3->getState().transpose());
    WOLF_INFO("residual after solve:       ", res.transpose());

    WOLF_INFO(report, " AND UNION");

    ASSERT_MATRIX_APPROX(res, Vector3s::Zero(), 1e-8);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
//    ::testing::GTEST_FLAG(filter) = "ConstraintAutodiffTrifocalTest.residual_jacobians";
    return RUN_ALL_TESTS();
}

