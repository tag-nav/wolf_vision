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

    std::cout << "Initial pose : " << problem->getCurrentState().transpose() << std::endl;
    std::cout << "Initial covariance : " << std::endl << problem->getLastKeyFrameCovariance() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

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
        std::cout << "-------------------\nStep " << i << " at time " << t << std::endl;
        // re-use capture with updated timestamp
        capture->setTimeStamp(t);

        // Processor
        sensor_odom2d->process(capture);
        Matrix3s odom2d_delta_cov = processor_odom2d->integrateBufferCovariance(processor_odom2d->getBuffer());
        std::cout << "State(" << (t - t0) << ") : " << processor_odom2d->getCurrentState().transpose() << std::endl;
        std::cout << "PRC  cov: \n" << odom2d_delta_cov << std::endl;

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
        std::cout << "TEST cov: \n" << integrated_delta_cov << std::endl;

//        ASSERT_EIGEN_APPROX(processor_odom2d->getMotion().delta_, integrated_delta);
        ASSERT_EIGEN_APPROX(odom2d_delta_cov, integrated_delta_cov);

        // Integrate pose
        Ju = plus_jac_u(integrated_pose, data);
        Jx = plus_jac_x(integrated_pose, data);
        integrated_pose = plus(integrated_pose, data);
        integrated_cov = Jx * integrated_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();

        // Pose error -- fix spurious error from pi to -pi
        Vector3s error_pose = processor_odom2d->getCurrentState() - integrated_pose;
        error_pose(2) = pi2pi(error_pose(2));
        ASSERT_EIGEN_APPROX(error_pose, Vector3s::Zero());

        integrated_pose_vector.push_back(integrated_pose);
        integrated_cov_vector.push_back(integrated_cov);

        t += dt;
    }

    // Solve
    ceres::Solver::Summary summary = ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    std::cout << "After solving the problem, covariance of the last keyframe:" << std::endl;
    std::cout << "WOLF:\n"      << problem->getLastKeyFrameCovariance() << std::endl;
    std::cout << "REFERENCE:\n" << integrated_cov_vector[5] << std::endl;

    ASSERT_EIGEN_APPROX(problem->getLastKeyFrameCovariance() , integrated_cov_vector[5]);
}

TEST(ProcessorMotion2D, SplitAndSolve)
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
    int N = 8; // number of process() steps

    // NOTE: We integrate and create KFs as follows:
    // i=    0    1    2    3    4    5    6
    // KF -- * -- * -- * -- * -- * -- * -- * : no keyframes during integration
    // And we split as follows
    //                           s          : exact time stamp t_split     = 5*dt
    //              s                       : fractional time stamp t_split = 2.2*dt


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
    std::cout << "Initial pose : " << problem->getCurrentState().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

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

//        if (t - t0 == 0.05){
//            std::cout << "State(" << (t - t0) << ") : " << processor_odom2d->getCurrentState().transpose() << std::endl;
//            std::cout << "WOLF delta cov: \n" << odom2d_delta_cov << std::endl;
//            std::cout << "REF delta cov: \n" << integrated_delta_cov << std::endl;
//            std::cout << "REF cov: \n" << integrated_cov << std::endl;
//        }

        integrated_pose_vector.push_back(integrated_pose);
        integrated_cov_vector.push_back(integrated_cov);

        t += dt;
    }


    showBuffer(processor_odom2d->getBuffer(), "Original buffer:", t0);

    ////////////////////////////////////////////////////////////////
    // Split after the last keyframe, t_split = n_split*dt;
    int n_split = 5;
    TimeStamp t_split (t0 + n_split*dt);

    std::cout << "### Split after last KF; time: " << t_split - t0 << std::endl;

    Vector3s x_split = processor_odom2d->getState(t_split);
    FrameBasePtr keyframe_split_n = problem->emplaceFrame(KEY_FRAME, x_split, t_split);

    processor_odom2d->keyFrameCallback(keyframe_split_n, 0);

    CaptureMotionPtr key_capture_n = std::static_pointer_cast<CaptureMotion>(keyframe_split_n->getCaptureList().front());

    MotionBuffer key_buffer_n = key_capture_n->getBuffer();

    showBuffer(key_buffer_n,                  "New keyframe's capture buffer: ", t0);
    showBuffer(processor_odom2d->getBuffer(), "Current processor buffer     : ", t0);

    ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    ASSERT_EIGEN_APPROX(problem->getLastKeyFramePtr()->getState(), integrated_pose_vector[n_split - 1]);
    ASSERT_EIGEN_APPROX(problem->getLastKeyFrameCovariance()     , integrated_cov_vector [n_split - 1]);

    ////////////////////////////////////////////////////////////////
    // Split between keyframes
    int m_split = 3; // now we have KF at n=0 and n=5
    t_split = t0 + m_split*dt;
    std::cout << "### Split between KFs; time: " << t_split - t0 << std::endl;


    x_split = processor_odom2d->getState(t_split);
    FrameBasePtr keyframe_split_m = problem->emplaceFrame(KEY_FRAME, x_split, t_split);

    processor_odom2d->keyFrameCallback(keyframe_split_m, 0);

    CaptureMotionPtr key_capture_m = std::static_pointer_cast<CaptureMotion>(keyframe_split_m->getCaptureList().front());
    MotionBuffer key_buffer_m = key_capture_m->getBuffer();

    showBuffer(key_buffer_m,                  "New keyframe's capture buffer: ", t0);
    std::cout << "cov:\n" << processor_odom2d->integrateBufferCovariance(key_buffer_m) << std::endl;
    showBuffer(key_capture_n->getBuffer(),    "Last KF's buffer             : ", t0);
    std::cout << "cov:\n" << processor_odom2d->integrateBufferCovariance(key_capture_n->getBuffer()) << std::endl;
    showBuffer(processor_odom2d->getBuffer(), "Current processor buffer     : ", t0);
    std::cout << "cov:\n" << processor_odom2d->integrateBufferCovariance(processor_odom2d->getBuffer()) << std::endl;

    ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);
    problem->print();

    //    FrameBasePtr keyframe_last = problem->getLastKeyFramePtr();
    //    FrameBasePtr keyframe_previous = keyframe_last->getPreviousFrame();
    //    std::cout << "Last  KF -> P: " << keyframe_last.get()     << " - V: " << keyframe_split_n.get() << std::endl;
    //    std::cout << "Split KF -> P: " << keyframe_previous.get() << " - V: " << keyframe_split_m.get() << std::endl;

    // check the split KF
    EXPECT_EIGEN_APPROX(keyframe_split_m->getState(), integrated_pose_vector[m_split - 1]);
    EXPECT_EIGEN_APPROX(problem->getFrameCovariance(keyframe_split_m) , integrated_cov_vector [m_split - 1]);

    // check other KF in the future of the split KF
    EXPECT_EIGEN_APPROX(problem->getLastKeyFramePtr()->getState(), integrated_pose_vector[n_split - 1]);
    EXPECT_EIGEN_APPROX(problem->getLastKeyFrameCovariance()     , integrated_cov_vector [n_split - 1]);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

