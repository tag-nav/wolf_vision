/**
 * \file test_motion_2d.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */

#include "utils_gtest.h"

// Classes under test
#include "../processor_odom_2D.h"

// Wolf includes
#include "../capture_fix.h"
#include "../state_block.h"
#include "../wolf.h"
#include "../ceres_wrapper/ceres_manager.h"

// STL includes
#include <map>
#include <list>
#include <algorithm>
#include <iterator>

// General includes
#include <iostream>
#include <iomanip>      // std::setprecision

using namespace wolf;
using namespace Eigen;


void showBuffer(const MotionBuffer& _buffer, std::string _label = "", const TimeStamp _t0 = TimeStamp(0.0))
{
    std::cout << _label << " <";
    for (const auto &s : _buffer.get())
        std::cout << s.ts_ - _t0 << ' ';
    std::cout << ">" << std::endl;
}

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
    for (FrameBasePtr frm : problem->getTrajectoryPtr()->getFrameList())
    {
        if (frm->isKey())
        {
            std::cout << "===== Key Frame " << frm->id() << " ======" << std::endl;
            std::cout << "  feature measure: \n"
                    << frm->getCaptureList().front()->getFeatureList().front()->getMeasurement().transpose()
                    << std::endl;
            std::cout << "  feature covariance: \n"
                    << frm->getCaptureList().front()->getFeatureList().front()->getMeasurementCovariance() << std::endl;
            std::cout << "  state: \n" << frm->getState().transpose() << std::endl;
            std::cout << "  covariance: \n" << problem->getFrameCovariance(frm) << std::endl;
        }
    }
}
TEST(ProcessorMotion2D, VoteForKfAndSolve)
{
    std::cout << std::setprecision(3);
    // time
    TimeStamp t0(0.0), t = t0;
    Scalar dt = .01;
    // Origin frame:
    Vector3s x0(.5, -.5 -sqrt(.5), M_PI_4);
    Eigen::Matrix3s x0_cov = Eigen::Matrix3s::Identity() * 0.1;
    // motion data
    VectorXs data(Vector2s(1, M_PI_4) ); // advance 1m turn pi/4 rad (45 deg). Need 8 steps for a complete turn
    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2, 2) * 0.01;
    int N = 7; // number of process() steps

    // Create Wolf tree nodes
    ProblemPtr problem = Problem::create(FRM_PO_2D);
    SensorBasePtr sensor_odom2d = problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0));
    ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
    params->dist_traveled_th_   = 100;
    params->elapsed_time_th_    = 2.5*dt; // force KF at every third process()
    params->cov_det_th_         = 100;
    ProcessorBasePtr prc_base = problem->installProcessor("ODOM 2D", "odom", sensor_odom2d, params);
    ProcessorOdom2DPtr processor_odom2d = std::static_pointer_cast<ProcessorOdom2D>(prc_base);

    // NOTE: We integrate and create KFs as follows:
    // i=    0    1    2    3    4    5    6
    // KF -- * -- * -- KF - * -- * -- KF - *

    // Ceres wrapper
    CeresManager ceres_manager(problem);

    // Origin Key Frame
    FrameBasePtr origin_frame = problem->setPrior(x0, x0_cov, t0);
    ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    //    std::cout << "Initial pose : " << problem->getCurrentState().transpose() << std::endl;
    //    std::cout << "Initial covariance : " << std::endl << problem->getLastKeyFrameCovariance() << std::endl;
    //    std::cout << "Motion data  : " << data.transpose() << std::endl;

    // Check covariance values
    Eigen::Vector3s integrated_pose = x0;
    Eigen::Matrix3s integrated_cov = x0_cov;
    Eigen::Vector3s integrated_delta = Eigen::Vector3s::Zero();
    Eigen::Matrix3s integrated_delta_cov = Eigen::Matrix3s::Zero();
    Eigen::MatrixXs Ju(3, 2);
    Eigen::Matrix3s Jx;
    std::vector<Eigen::VectorXs> integrated_pose_vector;
    std::vector<Eigen::MatrixXs> integrated_cov_vector;

    std::cout << "\nIntegrating data..." << std::endl;

    t += dt;
    // Capture to use as container for all incoming data
    CaptureMotionPtr capture = std::make_shared<CaptureMotion>(t, sensor_odom2d, data, data_cov, nullptr);

    for (int i=0; i<N; i++)
    {
        //        std::cout << "-------------------\nStep " << i << " at time " << t << std::endl;
        // re-use capture with updated timestamp
        capture->setTimeStamp(t);

        // Processor
        sensor_odom2d->process(capture);
        Matrix3s odom2d_delta_cov = processor_odom2d->integrateBufferCovariance(processor_odom2d->getBuffer());
        //        std::cout << "State(" << (t - t0) << ") : " << processor_odom2d->getCurrentState().transpose() << std::endl;
        //        std::cout << "PRC  cov: \n" << odom2d_delta_cov << std::endl;

        // Integrate Delta
        if (i==2 || i==5) // keyframes
        {
            integrated_delta.setZero();
            integrated_delta_cov.setZero();
        }
        else
        {
            Ju = plus_jac_u(integrated_delta, data);
            Jx = plus_jac_x(integrated_delta, data);
            integrated_delta = plus(integrated_delta, data);
            integrated_delta_cov = Jx * integrated_delta_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();
        }
        //        std::cout << "TEST cov: \n" << integrated_delta_cov << std::endl;

//        ASSERT_EIGEN_APPROX(processor_odom2d->getMotion().delta_, integrated_delta);
        ASSERT_EIGEN_APPROX(odom2d_delta_cov, integrated_delta_cov);

        // Integrate pose
        Ju = plus_jac_u(integrated_pose, data);
        Jx = plus_jac_x(integrated_pose, data);
        integrated_pose = plus(integrated_pose, data);
        integrated_cov = Jx * integrated_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();

        ASSERT_POSE2D_APPROX(processor_odom2d->getCurrentState(), integrated_pose)

        integrated_pose_vector.push_back(integrated_pose);
        integrated_cov_vector.push_back(integrated_cov);

        t += dt;
    }

    // Solve
    ceres::Solver::Summary summary = ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    //    std::cout << "After solving the problem, covariance of the last keyframe:" << std::endl;
    //    std::cout << "WOLF:\n"      << problem->getLastKeyFrameCovariance() << std::endl;
    //    std::cout << "REFERENCE:\n" << integrated_cov_vector[5] << std::endl;

    ASSERT_EIGEN_APPROX(problem->getLastKeyFrameCovariance() , integrated_cov_vector[5]);
}

TEST(ProcessorMotion2D, SplitAndSolve)
{
    std::cout << std::setprecision(3);
    // time
    TimeStamp t0(0.0), t = t0;
    Scalar dt = .01;
    Vector3s x0(0,0,0);
    Eigen::Matrix3s x0_cov = Eigen::Matrix3s::Identity() * 0.1;
    VectorXs data(Vector2s(1, 0) ); // advance 1m
    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2, 2) * 0.01;
    int N = 8; // number of process() steps

    // NOTE: We integrate and create KFs as follows:
    // i=    0    1    2    3    4    5    6    7
    // KF -- * -- * -- * -- * -- * -- * -- * -- *   : no keyframes during integration
    // And we split as follows
    //                           s
    //                 s


    // Create Wolf tree nodes
    ProblemPtr problem = Problem::create(FRM_PO_2D);
    SensorBasePtr sensor_odom2d = problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0));
    ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
    params->dist_traveled_th_   = 100; // don't make keyframes
    params->elapsed_time_th_    = 100;
    params->cov_det_th_         = 100;
    ProcessorBasePtr prc_base = problem->installProcessor("ODOM 2D", "odom", sensor_odom2d, params);
    ProcessorOdom2DPtr processor_odom2d = std::static_pointer_cast<ProcessorOdom2D>(prc_base);

    // Ceres wrapper
    CeresManager ceres_manager(problem);

    // Origin Key Frame
    FrameBasePtr origin_frame = problem->setPrior(x0, x0_cov, t0);
    //    std::cout << "Initial pose : " << problem->getCurrentState().transpose() << std::endl;
    //    std::cout << "Motion data  : " << data.transpose() << std::endl;

    // Check covariance values
    Eigen::Vector3s integrated_pose = x0;
    Eigen::Matrix3s integrated_cov = x0_cov;
    Eigen::Vector3s integrated_delta = Eigen::Vector3s::Zero();
    Eigen::Matrix3s integrated_delta_cov = Eigen::Matrix3s::Zero();
    Eigen::MatrixXs Ju(3, 2);
    Eigen::Matrix3s Jx;
    std::vector<Eigen::VectorXs> integrated_pose_vector;
    std::vector<Eigen::MatrixXs> integrated_cov_vector;

    std::cout << "\nIntegrating data..." << std::endl;

    t += dt;
    // Capture to use as container for all incoming data
    CaptureMotionPtr capture = std::make_shared<CaptureMotion>(t, sensor_odom2d, data, data_cov, nullptr);

    for (int i=0; i<N; i++)
    {
        // re-use capture with updated timestamp
        capture->setTimeStamp(t);

        // Processor
        sensor_odom2d->process(capture);

        // Integrate Delta
        Ju = plus_jac_u(integrated_delta, data);
        Jx = plus_jac_x(integrated_delta, data);
        integrated_delta = plus(integrated_delta, data);
        integrated_delta_cov = Jx * integrated_delta_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();

        // Integrate pose
        Ju = plus_jac_u(integrated_pose, data);
        Jx = plus_jac_x(integrated_pose, data);
        integrated_pose = plus(integrated_pose, data);
        integrated_cov = Jx * integrated_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();

        // Store integrals
        integrated_pose_vector.push_back(integrated_pose);
        integrated_cov_vector.push_back(integrated_cov);

        t += dt;
    }

    std::cout << "=============================" << std::endl;
    showBuffer(processor_odom2d->getBuffer(), "Original buffer:", t0);

    ////////////////////////////////////////////////////////////////
    // Split after the last keyframe,
    int n_split = 4;
    TimeStamp t_split (t0 + n_split*dt);

    std::cout << "-----------------------------\nSplit after last KF; time: " << t_split - t0 << std::endl;

    Vector3s x_split = processor_odom2d->getState(t_split);
    FrameBasePtr keyframe_split_n = problem->emplaceFrame(KEY_FRAME, x_split, t_split);

    processor_odom2d->keyFrameCallback(keyframe_split_n, 0);

    CaptureMotionPtr key_capture_n = std::static_pointer_cast<CaptureMotion>(keyframe_split_n->getCaptureList().front());

    MotionBuffer key_buffer_n = key_capture_n->getBuffer();

    showBuffer(key_buffer_n,                  "New keyframe's capture buffer: ", t0);
    showBuffer(processor_odom2d->getBuffer(), "Current processor buffer     : ", t0);

    ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    ASSERT_POSE2D_APPROX(problem->getLastKeyFramePtr()->getState(), integrated_pose_vector[n_split - 1]);
    ASSERT_EIGEN_APPROX(problem->getLastKeyFrameCovariance()     , integrated_cov_vector [n_split - 1]);

    ////////////////////////////////////////////////////////////////
    // Split between keyframes
    int m_split = 2;
    t_split = t0 + m_split*dt;
    std::cout << "-----------------------------\nSplit between KFs; time: " << t_split - t0 << std::endl;


    x_split = processor_odom2d->getState(t_split);
    FrameBasePtr keyframe_split_m = problem->emplaceFrame(KEY_FRAME, x_split, t_split);

    processor_odom2d->keyFrameCallback(keyframe_split_m, 0);

    CaptureMotionPtr key_capture_m = std::static_pointer_cast<CaptureMotion>(keyframe_split_m->getCaptureList().front());
    MotionBuffer key_buffer_m = key_capture_m->getBuffer();

    //    showBuffer(key_buffer_m,                  "New keyframe's capture buffer: ", t0);
    //    std::cout << "delta:\n" << key_buffer_m.get().back().delta_integr_.transpose() << std::endl;
    //    std::cout << "cov:\n" << processor_odom2d->integrateBufferCovariance(key_buffer_m) << std::endl;
    //    showBuffer(key_capture_n->getBuffer(),    "Last KF's buffer             : ", t0);
    //    std::cout << "delta:\n" << key_capture_n->getBuffer().get().back().delta_integr_.transpose() << std::endl;
    //    std::cout << "cov:\n" << processor_odom2d->integrateBufferCovariance(key_capture_n->getBuffer()) << std::endl;
    //    showBuffer(processor_odom2d->getBuffer(), "Current processor buffer     : ", t0);
    //    std::cout << "delta:\n" << processor_odom2d->getBuffer().get().back().delta_integr_.transpose() << std::endl;
    //    std::cout << "cov:\n" << processor_odom2d->integrateBufferCovariance(processor_odom2d->getBuffer()) << std::endl;

    keyframe_split_n->setState(Vector3s(3,2,1));
    keyframe_split_m->setState(Vector3s(1,2,3));
    ceres::Solver::Summary summary = ceres_manager.solve();
    std::cout << summary.BriefReport() << std::endl;
    ceres_manager.computeCovariances(ALL_MARGINALS);


    // check the split KF
    ASSERT_POSE2D_APPROX(keyframe_split_m->getState()                 , integrated_pose_vector[m_split - 1]);
    ASSERT_EIGEN_APPROX(problem->getFrameCovariance(keyframe_split_m) , integrated_cov_vector [m_split - 1]);

    // check other KF in the future of the split KF
    ASSERT_POSE2D_APPROX(problem->getLastKeyFramePtr()->getState()    , integrated_pose_vector[n_split - 1]);
    EXPECT_EIGEN_APPROX(problem->getFrameCovariance(keyframe_split_n) , integrated_cov_vector [n_split - 1]);

//    problem->print(4,1,1,1);
    std::cout << problem->check(1) << std::endl;
//    show(problem);

}



TEST(motion, dummy)
{
    std::cout << std::setprecision(3);

    TimeStamp t0(0.0), t = t0;
    Scalar dt = .01;
    Vector3s x0(0,0,0);
    Eigen::Matrix3s x0_cov = Eigen::Matrix3s::Identity() * 0.1;

    Vector3s delta(2,0,0);
    //    Matrix3s delta_cov; delta_cov << 1.0, 0.0, 0.0, 0.0, 0.25, 0.5, 0.0, 0.5, 1.0; delta_cov /= 100; delta_cov += Matrix3s::Identity()*0.0000001;
    Matrix3s delta_cov; delta_cov << .02, 0, 0, 0, .025, .02, 0, .02, .02;

    ProblemPtr problem = Problem::create(FRM_PO_2D);
    FrameBasePtr F0 = problem->setPrior(x0, x0_cov,t0);
    SensorBasePtr sensor = problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0));
    CeresManager ceres_manager(problem);

    t += dt;
    FrameBasePtr F1 = problem->emplaceFrame(KEY_FRAME, 0*delta, t);
    CaptureBasePtr C1 = F1->addCapture(std::make_shared<CaptureBase>("ODOM 2D", t, sensor));
    FeatureBasePtr f1 = C1->addFeature(std::make_shared<FeatureBase>("ODOM 2D", delta, delta_cov));
    ConstraintBasePtr c1 = f1->addConstraint(std::make_shared<ConstraintOdom2D>(f1, F0));
    F0->addConstrainedBy(c1);

    t += dt;
    FrameBasePtr F2 = problem->emplaceFrame(KEY_FRAME, 0*delta, t);
    CaptureBasePtr C2 = F2->addCapture(std::make_shared<CaptureBase>("ODOM 2D", t, sensor));
    FeatureBasePtr f2 = C2->addFeature(std::make_shared<FeatureBase>("ODOM 2D", delta, delta_cov));
    ConstraintBasePtr c2 = f2->addConstraint(std::make_shared<ConstraintOdom2D>(f2, F1));
    F1->addConstrainedBy(c2);

    ceres::Solver::Summary summary = ceres_manager.solve();
    std::cout << summary.BriefReport() << std::endl;
    ceres_manager.computeCovariances(ALL_MARGINALS);

//    problem->print(4,1,1,1);
    std::cout << problem->check(1) << std::endl;
//    show(problem);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

