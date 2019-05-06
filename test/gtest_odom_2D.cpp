/**
 * \file test_odom_2D.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */

#include "utils_gtest.h"

// Classes under test
#include "base/processor/processor_odom_2D.h"
#include "base/factor/factor_odom_2D.h"

// Wolf includes
#include "base/sensor/sensor_odom_2D.h"
#include "base/state_block/state_block.h"
#include "base/common/wolf.h"
#include "base/ceres_wrapper/ceres_manager.h"

// STL includes
#include <map>
#include <list>
#include <algorithm>
#include <iterator>

// General includes
#include <iostream>
#include <iomanip>      // std::setprecision
#include "base/capture/capture_pose.h"

using namespace wolf;
using namespace Eigen;

VectorXs plus(const VectorXs& _pose, const Vector2s& _data)
{
    VectorXs _pose_plus_data(3);
    _pose_plus_data(0) = _pose(0) + cos(_pose(2) + _data(1) / 2) * _data(0);
    _pose_plus_data(1) = _pose(1) + sin(_pose(2) + _data(1) / 2) * _data(0);
    _pose_plus_data(2) = pi2pi(_pose(2) + _data(1));
    return _pose_plus_data;
}

MatrixXs plus_jac_u(const VectorXs& _pose, const Vector2s& _data)
{
    MatrixXs Ju(3,2);
    Ju(0, 0) =  cos(_pose(2) + _data(1) / 2);
    Ju(0, 1) = -sin(_pose(2) + _data(1) / 2) * _data(0) / 2 ;
    Ju(1, 0) =  sin(_pose(2) + _data(1) / 2);
    Ju(1, 1) =  cos(_pose(2) + _data(1) / 2) * _data(0) / 2 ;
    Ju(2, 0) = 0.0;
    Ju(2, 1) = 1.0;
    return Ju;
}

MatrixXs plus_jac_x(const VectorXs& _pose, const Vector2s& _data)
{
    Matrix3s Jx;
    Jx(0, 0) = 1.0;
    Jx(0, 1) = 0.0;
    Jx(0, 2) = -sin(_pose(2) + _data(1) / 2) * _data(0);
    Jx(1, 0) = 0.0;
    Jx(1, 1) = 1.0;
    Jx(1, 2) =  cos(_pose(2) + _data(1) / 2) * _data(0);
    Jx(2, 0) = 0.0;
    Jx(2, 1) = 0.0;
    Jx(2, 2) = 1.0;
    return Jx;
}

void show(const ProblemPtr& problem)
{
    using std::cout;
    using std::endl;
    cout << std::setprecision(4);

    for (FrameBasePtr F : problem->getTrajectory()->getFrameList())
    {
        if (F->isKey())
        {
            cout << "----- Key Frame " << F->id() << " -----" << endl;
            if (!F->getCaptureList().empty())
            {
                auto C = F->getCaptureList().front();
                if (!C->getFeatureList().empty())
                {
                    auto f = C->getFeatureList().front();
                    cout << "  feature measure: \n"
                            << F->getCaptureList().front()->getFeatureList().front()->getMeasurement().transpose()
                            << endl;
                    cout << "  feature covariance: \n"
                            << F->getCaptureList().front()->getFeatureList().front()->getMeasurementCovariance() << endl;
                }
            }
            cout << "  state: \n" << F->getState().transpose() << endl;
            Eigen::MatrixXs cov;
            problem->getFrameCovariance(F,cov);
            cout << "  covariance: \n" << cov << endl;
        }
    }
}

TEST(Odom2D, FactorFix_and_FactorOdom2D)
{
    using std::cout;
    using std::endl;

    // RATIONALE:
    // We build this tree (a is `absolute`, m is `motion`):
    // KF0 -- m -- KF1 -- m -- KF2
    //  |
    //  a
    //  |
    // GND
    // `absolute` is made with FactorFix
    // `motion`   is made with FactorOdom2D

    std::cout << std::setprecision(4);

    TimeStamp t0(0.0),  t = t0;
    Scalar              dt = .01;
    Vector3s            x0   (0,0,0);
    Eigen::Matrix3s     P0 = Eigen::Matrix3s::Identity() * 0.1;
    Vector3s            delta (2,0,0);
    Matrix3s delta_cov; delta_cov << .02, 0, 0, 0, .025, .02, 0, .02, .02;

    ProblemPtr          Pr = Problem::create("PO 2D");
    CeresManager        ceres_manager(Pr);

    // KF0 and absolute prior
    FrameBasePtr        F0 = Pr->setPrior(x0, P0,t0, dt/2);

    // KF1 and motion from KF0
    t += dt;
    FrameBasePtr        F1 = Pr->emplaceFrame(KEY_FRAME, Vector3s::Zero(), t);
    // CaptureBasePtr      C1 = F1->addCapture(std::make_shared<CaptureBase>("ODOM 2D", t));
    auto C1 = CaptureBase::emplace<CaptureBase>(F1, "ODOM 2D", t);
    // FeatureBasePtr      f1 = C1->addFeature(std::make_shared<FeatureBase>("ODOM 2D", delta, delta_cov));
    auto f1 = FeatureBase::emplace<FeatureBase>(C1, "ODOM 2D", delta, delta_cov);
    // FactorBasePtr   c1 = f1->addFactor(std::make_shared<FactorOdom2D>(f1, F0, nullptr));
    // F0->addConstrainedBy(c1);
    auto c1 = FactorBase::emplace<FactorOdom2D>(f1, f1, F0, nullptr);

    // KF2 and motion from KF1
    t += dt;
    FrameBasePtr        F2 = Pr->emplaceFrame(KEY_FRAME, Vector3s::Zero(), t);
    // CaptureBasePtr      C2 = F2->addCapture(std::make_shared<CaptureBase>("ODOM 2D", t));
    auto C2 = CaptureBase::emplace<CaptureBase>(F2, "ODOM 2D", t);
    // FeatureBasePtr      f2 = C2->addFeature(std::make_shared<FeatureBase>("ODOM 2D", delta, delta_cov));
    auto f2 = FeatureBase::emplace<FeatureBase>(C2, "ODOM 2D", delta, delta_cov);
    // FactorBasePtr   c2 = f2->addFactor(std::make_shared<FactorOdom2D>(f2, F1, nullptr));
    // F1->addConstrainedBy(c2);
    auto c2 = FactorBase::emplace<FactorOdom2D>(f2, f2, F1, nullptr);

    ASSERT_TRUE(Pr->check(0));

//    cout << "===== full ====" << endl;
    F0->setState(Vector3s(1,2,3));
    F1->setState(Vector3s(2,3,1));
    F2->setState(Vector3s(3,1,2));
    std::string report = ceres_manager.solve(SolverManager::ReportVerbosity::FULL);
//    std::cout << report << std::endl;

    ceres_manager.computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);
//    show(Pr);

    Matrix3s P1, P2;
    P1 << 0.12, 0,     0,
          0,    0.525, 0.22,
          0,    0.22,  0.12;
    P2 << 0.14, 0,    0,
          0,    1.91, 0.48,
          0,    0.48, 0.14;

    // get covariances
    MatrixXs P0_solver, P1_solver, P2_solver;
    ASSERT_TRUE(Pr->getFrameCovariance(F0, P0_solver));
    ASSERT_TRUE(Pr->getFrameCovariance(F1, P1_solver));
    ASSERT_TRUE(Pr->getFrameCovariance(F2, P2_solver));

    ASSERT_POSE2D_APPROX(F0->getState(), Vector3s(0,0,0), 1e-6);
    ASSERT_MATRIX_APPROX(P0_solver, P0, 1e-6);
    ASSERT_POSE2D_APPROX(F1->getState(), Vector3s(2,0,0), 1e-6);
    ASSERT_MATRIX_APPROX(P1_solver, P1, 1e-6);
    ASSERT_POSE2D_APPROX(F2->getState(), Vector3s(4,0,0), 1e-6);
    ASSERT_MATRIX_APPROX(P2_solver, P2, 1e-6);
}

TEST(Odom2D, VoteForKfAndSolve)
{
    std::cout << std::setprecision(4);
    // time
    TimeStamp t0(0.0), t = t0;
    Scalar dt = .01;
    // Origin frame:
    Vector3s x0(0,0,0);
    Eigen::Matrix3s P0 = Eigen::Matrix3s::Identity() * 0.1;
    // motion data
    VectorXs data(Vector2s(1, M_PI_4) ); // advance 1m turn pi/4 rad (45 deg). Need 8 steps for a complete turn
    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2, 2) * 0.01;
    int N = 7; // number of process() steps

    // Create Wolf tree nodes
    ProblemPtr problem = Problem::create("PO 2D");
    SensorBasePtr sensor_odom2d = problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0));
    ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
    params->voting_active   = true;
    params->dist_traveled   = 100;
    params->angle_turned    = 6.28;
    params->max_time_span   = 2.5*dt; // force KF at every third process()
    params->cov_det         = 100;
    params->unmeasured_perturbation_std = 0.00;
    Matrix3s unmeasured_cov = params->unmeasured_perturbation_std*params->unmeasured_perturbation_std*Matrix3s::Identity();
    ProcessorBasePtr prc_base = problem->installProcessor("ODOM 2D", "odom", sensor_odom2d, params);
    ProcessorOdom2DPtr processor_odom2d = std::static_pointer_cast<ProcessorOdom2D>(prc_base);

    // NOTE: We integrate and create KFs as follows:
    // i=    0    1    2    3    4    5    6
    // KF -- * -- * -- KF - * -- * -- KF - *

    // Ceres wrapper
    CeresManager ceres_manager(problem);

    // Origin Key Frame
    FrameBasePtr origin_frame = problem->setPrior(x0, P0, t0, dt/2);
    ceres_manager.solve(SolverManager::ReportVerbosity::BRIEF);
    ceres_manager.computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);

    //    std::cout << "Initial pose : " << problem->getCurrentState().transpose() << std::endl;
    //    std::cout << "Initial covariance : " << std::endl << problem->getLastKeyFrameCovariance() << std::endl;
    //    std::cout << "Motion data  : " << data.transpose() << std::endl;

    // Check covariance values
    Eigen::Vector3s integrated_pose      = x0;
    Eigen::Matrix3s integrated_cov       = P0;
    Eigen::Vector3s integrated_delta     = Eigen::Vector3s::Zero();
    Eigen::Matrix3s integrated_delta_cov = Eigen::Matrix3s::Zero();
    Eigen::MatrixXs Ju(3, 2);
    Eigen::Matrix3s Jx;
    std::vector<Eigen::VectorXs> integrated_pose_vector;
    std::vector<Eigen::MatrixXs> integrated_cov_vector;

//    std::cout << "\nIntegrating data..." << std::endl;

    t += dt;
    // Capture to use as container for all incoming data
    CaptureMotionPtr capture = std::make_shared<CaptureOdom2D>(t, sensor_odom2d, data, data_cov, nullptr);

    for (int n=1; n<=N; n++)
    {
        //        std::cout << "-------------------\nStep " << i << " at time " << t << std::endl;
        // re-use capture with updated timestamp
        capture->setTimeStamp(t);

        // Processor
        sensor_odom2d->process(capture);
        ASSERT_TRUE(problem->check(0));
//        Matrix3s odom2d_delta_cov = processor_odom2d->integrateBufferCovariance(processor_odom2d->getBuffer());
        Matrix3s odom2d_delta_cov = processor_odom2d->getMotion().delta_integr_cov_;
        //        std::cout << "State(" << (t - t0) << ") : " << processor_odom2d->getCurrentState().transpose() << std::endl;
        //        std::cout << "PRC  cov: \n" << odom2d_delta_cov << std::endl;

        // Integrate Delta
        if (n==3 || n==6) // keyframes
        {
            integrated_delta.setZero();
            integrated_delta_cov.setZero();
        }
        else
        {
            Ju = plus_jac_u(integrated_delta, data);
            Jx = plus_jac_x(integrated_delta, data);
            integrated_delta = plus(integrated_delta, data);
            integrated_delta_cov = Jx * integrated_delta_cov * Jx.transpose() + Ju * data_cov * Ju.transpose() + unmeasured_cov;
        }

        WOLF_DEBUG("n: ", n, " t:", t);
        WOLF_DEBUG("wolf delta: ", processor_odom2d->getMotion().delta_integr_.transpose());
        WOLF_DEBUG("test delta: ", integrated_delta                           .transpose());

        ASSERT_POSE2D_APPROX(processor_odom2d->getMotion().delta_integr_, integrated_delta, 1e-6);
        ASSERT_MATRIX_APPROX(odom2d_delta_cov, integrated_delta_cov, 1e-6);

        // Integrate pose
        Ju = plus_jac_u(integrated_pose, data);
        Jx = plus_jac_x(integrated_pose, data);
        integrated_pose = plus(integrated_pose, data);
        integrated_cov = Jx * integrated_cov * Jx.transpose() + Ju * data_cov * Ju.transpose() + unmeasured_cov;

        ASSERT_POSE2D_APPROX(processor_odom2d->getCurrentState(), integrated_pose, 1e-6);

        integrated_pose_vector.push_back(integrated_pose);
        integrated_cov_vector.push_back(integrated_cov);

        t += dt;
    }

    // Solve
    std::string report = ceres_manager.solve(SolverManager::ReportVerbosity::BRIEF);
//    std::cout << report << std::endl;
    ceres_manager.computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);

    MatrixXs computed_cov;
    ASSERT_TRUE(problem->getLastKeyFrameCovariance(computed_cov));
    ASSERT_MATRIX_APPROX(computed_cov , integrated_cov_vector[5], 1e-6);
}

TEST(Odom2D, KF_callback)
{
    using std::cout;
    using std::endl;

    std::cout << std::setprecision(4);
    // time
    TimeStamp t0(0.0), t = t0;
    Scalar dt = .01;
    Vector3s x0(0,0,0);
    Eigen::Matrix3s x0_cov = Eigen::Matrix3s::Identity() * 0.1;
    VectorXs data(Vector2s(1, M_PI/4) ); // advance 1m
    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2, 2) * 0.01;
    int N = 16; // number of process() steps

    // NOTE: We integrate and create KFs as follows:
    //  n =   0    1    2    3    4    5    6    7    8
    //  t =   0    dt  2dt  3dt  4dt  5dt  6dt  7dt  8dt
    //       KF8-- * -- * -- * -- * -- * -- * -- * -- * -->  : no keyframes during integration
    // And we create KFs as follows:
    //                                              KF10
    //                          KF11

    // Create Wolf tree nodes
    ProblemPtr problem = Problem::create("PO 2D");
    SensorBasePtr sensor_odom2d = problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0));
    ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
    params->dist_traveled   = 100;
    params->angle_turned    = 6.28;
    params->max_time_span   = 100;
    params->cov_det         = 100;
    params->unmeasured_perturbation_std = 0.000001;
    Matrix3s unmeasured_cov = params->unmeasured_perturbation_std*params->unmeasured_perturbation_std*Matrix3s::Identity();
    ProcessorBasePtr prc_base = problem->installProcessor("ODOM 2D", "odom", sensor_odom2d, params);
    ProcessorOdom2DPtr processor_odom2d = std::static_pointer_cast<ProcessorOdom2D>(prc_base);
    processor_odom2d->setTimeTolerance(dt/2);

    // Ceres wrapper
    CeresManager ceres_manager(problem);

    // Origin Key Frame
    FrameBasePtr keyframe_0 = problem->setPrior(x0, x0_cov, t0, dt/2);

    // Check covariance values
    Eigen::Vector3s integrated_pose = x0;
    Eigen::Matrix3s integrated_cov = x0_cov;
    Eigen::Vector3s integrated_delta = Eigen::Vector3s::Zero();
    Eigen::Matrix3s integrated_delta_cov = Eigen::Matrix3s::Zero();
    Eigen::MatrixXs Ju(3, 2);
    Eigen::Matrix3s Jx;
    std::vector<Eigen::VectorXs> integrated_pose_vector;
    std::vector<Eigen::MatrixXs> integrated_cov_vector;

    // Store integrals
    integrated_pose_vector.push_back(integrated_pose);
    integrated_cov_vector.push_back(integrated_cov);

//    std::cout << "\nIntegrating data..." << std::endl;

    // Capture to use as container for all incoming data
    CaptureMotionPtr capture = std::make_shared<CaptureMotion>("ODOM 2D", t, sensor_odom2d, data, data_cov, 3, 3, nullptr);

    std::cout << "t: " << t << std::endl;
    for (int n=1; n<=N; n++)
    {
        t += dt;

        // re-use capture with updated timestamp
        capture->setTimeStamp(t);
        std::cout << "capture ts: " << capture->getTimeStamp() << " - " << capture->getTimeStamp().get();
        std::cout << "nsec:        " << capture->getTimeStamp().getNanoSeconds() << std::endl;
        std::cout << "filled nsec: " << std::setfill('0') << std::setw(9) << std::right << capture->getTimeStamp().getNanoSeconds() << std::endl;
        std::cout << std::setfill(' ');

        // Processor
        sensor_odom2d->process(capture);
        ASSERT_TRUE(problem->check(0));

        // Integrate Delta
        Ju = plus_jac_u(integrated_delta, data);
        Jx = plus_jac_x(integrated_delta, data);
        integrated_delta = plus(integrated_delta, data);
        integrated_delta_cov = Jx * integrated_delta_cov * Jx.transpose() + Ju * data_cov * Ju.transpose() + unmeasured_cov;

        // Integrate pose
        Ju = plus_jac_u(integrated_pose, data);
        Jx = plus_jac_x(integrated_pose, data);
        integrated_pose = plus(integrated_pose, data);
        integrated_cov = Jx * integrated_cov * Jx.transpose() + Ju * data_cov * Ju.transpose() + unmeasured_cov;

        // Store integrals
        integrated_pose_vector.push_back(integrated_pose);
        integrated_cov_vector.push_back(integrated_cov);

    }

//    std::cout << "=============================" << std::endl;

    ////////////////////////////////////////////////////////////////
    // Split after the last keyframe, exact timestamp
    int n_split = 8;
    TimeStamp t_split = t0 + n_split*dt;

    std::cout << "-----------------------------\nSplit after last KF; time: " << t_split << std::endl;

    Vector3s x_split = processor_odom2d->getState(t_split);
    FrameBasePtr keyframe_2 = problem->emplaceFrame(KEY_FRAME, x_split, t_split);

    ASSERT_TRUE(problem->check(0));
    processor_odom2d->keyFrameCallback(keyframe_2, dt/2);
    ASSERT_TRUE(problem->check(0));
    t += dt;
    capture->setTimeStamp(t);
    processor_odom2d->process(capture);
    ASSERT_TRUE(problem->check(0));

    CaptureMotionPtr key_capture_n = std::static_pointer_cast<CaptureMotion>(keyframe_2->getCaptureList().front());

    MotionBuffer key_buffer_n = key_capture_n->getBuffer();

    std::string report = ceres_manager.solve(SolverManager::ReportVerbosity::BRIEF);
//    std::cout << report << std::endl;
    ceres_manager.computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);

    ASSERT_POSE2D_APPROX(problem->getLastKeyFrame()->getState() , integrated_pose_vector[n_split], 1e-6);
    MatrixXs computed_cov;
    ASSERT_TRUE(problem->getLastKeyFrameCovariance(computed_cov));
    ASSERT_MATRIX_APPROX(computed_cov, integrated_cov_vector [n_split], 1e-6);

    ////////////////////////////////////////////////////////////////
    // Split between keyframes, exact timestamp
    int m_split = 4;
    t_split = t0 + m_split*dt;
    std::cout << "-----------------------------\nSplit between KFs; time: " << t_split << std::endl;

    problem->print(4,1,1,1);

    x_split = processor_odom2d->getState(t_split);
    FrameBasePtr keyframe_1 = problem->emplaceFrame(KEY_FRAME, x_split, t_split);

    ASSERT_TRUE(problem->check(0));
    processor_odom2d->keyFrameCallback(keyframe_1, dt/2);
    ASSERT_TRUE(problem->check(0));
    t += dt;
    capture->setTimeStamp(t);
    processor_odom2d->process(capture);
    ASSERT_TRUE(problem->check(0));

    CaptureMotionPtr key_capture_m = std::static_pointer_cast<CaptureMotion>(keyframe_1->getCaptureList().front());
    MotionBuffer key_buffer_m = key_capture_m->getBuffer();

    // solve
//    cout << "===== full ====" << endl;
    keyframe_0->setState(Vector3s(1,2,3));
    keyframe_1->setState(Vector3s(2,3,1));
    keyframe_2->setState(Vector3s(3,1,2));

    report = ceres_manager.solve(SolverManager::ReportVerbosity::BRIEF);
    ceres_manager.computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);

    problem->print(4,1,1,1);

    // check the split KF
    MatrixXs KF1_cov;
    ASSERT_TRUE(problem->getFrameCovariance(keyframe_1, KF1_cov));
    ASSERT_POSE2D_APPROX(keyframe_1->getState(), integrated_pose_vector[m_split], 1e-6);
    ASSERT_MATRIX_APPROX(KF1_cov,                integrated_cov_vector [m_split], 1e-6);

    // check other KF in the future of the split KF
    MatrixXs KF2_cov;
    ASSERT_TRUE(problem->getFrameCovariance(keyframe_2, KF2_cov));
    ASSERT_POSE2D_APPROX(problem->getLastKeyFrame()->getState(), integrated_pose_vector[n_split], 1e-6);
    ASSERT_MATRIX_APPROX(KF2_cov,                                integrated_cov_vector [n_split], 1e-6);

    // Check full trajectory
    t = t0;
    for (int n=1; n<=N; n++)
    {
        t += dt;
        //        WOLF_DEBUG("   estimated(", t, ") = ", problem->getState(t).transpose());
        //        WOLF_DEBUG("ground truth(", t, ") = ", integrated_pose_vector[n].transpose());
        EXPECT_POSE2D_APPROX(problem->getState(t), integrated_pose_vector[n], 1e-6);
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

