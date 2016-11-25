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

TEST(ProcessorMotion, Motion2D)
{
    std::cout << std::setprecision(3);

    using namespace wolf;

    // time
    TimeStamp t0, t;
    t0.setToNow();
    t = t0;
    Scalar dt = .01;

    // Origin frame:
    Eigen::Vector2s p0;
    p0 << 0.5, -0.5 - sqrt(0.5);
    Eigen::Vector1s o0(Eigen::Vector1s::Constant(M_PI_4));
    Eigen::Vector3s x0;
    x0 << p0, o0;
    Eigen::Matrix3s init_cov = Eigen::Matrix3s::Identity() * 0.1;

    // motion data
    Eigen::VectorXs data(2);
    data << 1, M_PI_4;  // advance 1m turn pi/4 rad (45 deg). Need 8 steps for a complete turn
    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2, 2) * 0.01;

    // Create Wolf tree nodes
    ProblemPtr problem_ptr = Problem::create(FRM_PO_2D);
    SensorBasePtr sensor_odom_ptr = std::make_shared< SensorBase>("ODOM 2D", std::make_shared<StateBlock>(Eigen::Vector2s::Zero(), true),
                                            std::make_shared<StateBlock>(Eigen::Vector1s::Zero(), true),
                                            std::make_shared<StateBlock>(Eigen::VectorXs::Zero(0), true), 0);
    SensorBasePtr sensor_fix_ptr = std::make_shared< SensorBase>("ABSOLUTE POSE", nullptr, nullptr, nullptr, 0);
    ProcessorOdom2DPtr odom2d_ptr = std::make_shared< ProcessorOdom2D>(100,100,100);
    // Assemble Wolf tree by linking the nodes
    sensor_odom_ptr->addProcessor(odom2d_ptr);
    problem_ptr->addSensor(sensor_odom_ptr);
    problem_ptr->addSensor(sensor_fix_ptr);

    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_ptr = new CeresManager(problem_ptr, ceres_options);


    // Origin Key Frame
    FrameBasePtr origin_frame = problem_ptr->emplaceFrame(KEY_FRAME, x0, t0);

    // Prior covariance
    CaptureFixPtr initial_covariance = std::make_shared<CaptureFix>(TimeStamp(0), sensor_fix_ptr, x0, init_cov);
    origin_frame->addCapture(initial_covariance);
    initial_covariance->process();

    // Initialize processor motion
    odom2d_ptr->setOrigin(origin_frame);

    std::cout << "Initial pose : " << problem_ptr->getCurrentState().transpose() << std::endl;
    std::cout << "Initial covariance : " << std::endl << problem_ptr->getCurrentState().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    std::cout << "\nIntegrating states at synchronous time values..." << std::endl;

    std::cout << "State(" << (t - t0) << ") : " << odom2d_ptr->getCurrentState().transpose() << std::endl;
    // Capture to use as container for all incoming data
    t += dt;
    CaptureMotionPtr cap_ptr = std::make_shared<CaptureMotion>(t, sensor_odom_ptr, data, data_cov, nullptr);

    // Check covariance values
    Eigen::Vector3s integrated_x = x0;
    Eigen::Vector3s integrated_delta = Eigen::Vector3s::Zero();
    Eigen::Matrix3s integrated_covariance = init_cov;
    Eigen::Matrix3s integrated_delta_covariance = Eigen::Matrix3s::Zero();
    Eigen::MatrixXs Ju(3, 2);
    Eigen::Matrix3s Jx = Eigen::Matrix3s::Identity();
    std::vector<Eigen::VectorXs> integrated_x_vector;
    std::vector<Eigen::MatrixXs> integrated_covariance_vector;

    for (int i = 0; i <= 20; i++)
    {
        WOLF_TRACE("");
        // Processor
        odom2d_ptr->process(cap_ptr);

        // Delta Reference
        Ju(0, 0) = cos(integrated_delta(2) + data(1) / 2);
        Ju(1, 0) = sin(integrated_delta(2) + data(1) / 2);
        Ju(2, 0) = 0;
        Ju(0, 1) = -data(0) / 2 * sin(integrated_delta(2) + data(1) / 2);
        Ju(1, 1) = data(0) / 2 * cos(integrated_delta(2) + data(1) / 2);
        Ju(2, 1) = 1;

        Jx(0, 2) = -sin(integrated_delta(2) + data(1) / 2) * data(0);
        Jx(1, 2) = cos(integrated_delta(2) + data(1) / 2) * data(0);

        integrated_delta_covariance = Jx * integrated_delta_covariance * Jx.transpose() + Ju * data_cov * Ju.transpose();

        integrated_delta(0) = integrated_delta(0) + cos(integrated_delta(2) + data(1) / 2) * data(0);
        integrated_delta(1) = integrated_delta(1) + sin(integrated_delta(2) + data(1) / 2) * data(0);
        integrated_delta(2) = integrated_delta(2) + data(1);

        // Absolute Reference
        Ju(0, 0) = cos(integrated_x(2) + data(1) / 2);
        Ju(1, 0) = sin(integrated_x(2) + data(1) / 2);
        Ju(2, 0) = 0;
        Ju(0, 1) = -data(0) / 2 * sin(integrated_x(2) + data(1) / 2);
        Ju(1, 1) = data(0) / 2 * cos(integrated_x(2) + data(1) / 2);
        Ju(2, 1) = 1;

        Jx(0, 2) = -sin(integrated_x(2) + data(1) / 2) * data(0);
        Jx(1, 2) = cos(integrated_x(2) + data(1) / 2) * data(0);

        integrated_covariance = Jx * integrated_covariance * Jx.transpose() + Ju * data_cov * Ju.transpose();

        integrated_x(0) = integrated_x(0) + cos(integrated_x(2) + data(1) / 2) * data(0);
        integrated_x(1) = integrated_x(1) + sin(integrated_x(2) + data(1) / 2) * data(0);
        integrated_x(2) = integrated_x(2) + data(1);

        integrated_x_vector.push_back(integrated_x);
        integrated_covariance_vector.push_back(integrated_covariance);

//        if ((odom2d_ptr->getCurrentState() - integrated_x).norm() > Constants::EPS)
//        {
//            std::cout << "----------- PROCESSOR:" << std::endl;
//            std::cout << "State(" << (t - t0) << ") : " << odom2d_ptr->getCurrentState().transpose() << std::endl;
//            std::cout << "Covariance(" << (t - t0) << ") : " << std::endl
//                      << odom2d_ptr->getBuffer().get().back().delta_integr_cov_ << std::endl;
//            std::cout << "REFERENCE:" << std::endl;
//            std::cout << "State(" << (t - t0) << ") : " << integrated_x.transpose() << std::endl;
//            std::cout << "Covariance(" << (t - t0) << ") : " << std::endl << integrated_delta_covariance << std::endl;
//            std::cout << "ERROR:" << std::endl;
//            std::cout << "State error(" << (t - t0) << ") : " << odom2d_ptr->getCurrentState().transpose() - integrated_x.transpose() << std::endl;
//            std::cout << "Covariance error(" << (t - t0) << ") : " << std::endl
//                      << odom2d_ptr->getBuffer().get().back().delta_integr_cov_ - integrated_delta_covariance << std::endl;
//
//            std::cout << "TEST DELTA CHECK ------> ERROR: Integrated state different from reference." << std::endl;
//        }
        EXPECT_TRUE((odom2d_ptr->getCurrentState() - integrated_x).isMuchSmallerThan(1.0,Constants::EPS));

//        if ((odom2d_ptr->getBuffer().get().back().delta_integr_cov_ - integrated_delta_covariance).array().abs().maxCoeff() > Constants::EPS)
//        {
//            std::cout << "----------- PROCESSOR:" << std::endl;
//            std::cout << "State(" << (t - t0) << ") : " << odom2d_ptr->getCurrentState().transpose() << std::endl;
//            std::cout << "Covariance(" << (t - t0) << ") : " << std::endl
//                      << odom2d_ptr->getBuffer().get().back().delta_integr_cov_ << std::endl;
//            std::cout << "REFERENCE:" << std::endl;
//            std::cout << "State(" << (t - t0) << ") : " << integrated_x.transpose() << std::endl;
//            std::cout << "Covariance(" << (t - t0) << ") : " << std::endl << integrated_delta_covariance << std::endl;
//            std::cout << "ERROR:" << std::endl;
//            std::cout << "State error(" << (t - t0) << ") : " << odom2d_ptr->getCurrentState().transpose() - integrated_x.transpose() << std::endl;
//            std::cout << "Covariance error(" << (t - t0) << ") : " << std::endl
//                      << odom2d_ptr->getBuffer().get().back().delta_integr_cov_ - integrated_delta_covariance << std::endl;
//
//            std::cout << "TEST COVARIANCE CHECK ------> ERROR: Integrated covariance different from reference." << std::endl;
//        }
        EXPECT_TRUE((odom2d_ptr->getBuffer().get().back().delta_integr_cov_ - integrated_delta_covariance).isMuchSmallerThan(1.0,Constants::EPS));

        // Timestamp
        t += dt;
        cap_ptr->setTimeStamp(t);
    }

    std::cout << "\nQuery states at asynchronous time values..." << std::endl;

    t = t0;
    dt = 0.0045; // new dt
    for (int i = 1; i <= 25; i++)
    {
        std::cout << "State(" << (t - t0) << ") = " << odom2d_ptr->getState(t).transpose() << std::endl;
        //std::cout << "Covariance(" << (t - t0) << ") : " << std::endl
        //        << odom2d_ptr->getBufferPtr()->getMotion(t).delta_integr_cov_ << std::endl;
        t += dt;
    }
    std::cout << "       ^^^^^^^   After the last time-stamp the buffer keeps returning the last member." << std::endl;

    // Split the buffer

    std::cout << "\nSplitting the buffer!\n---------------------" << std::endl;
    std::cout << "Original buffer:           < ";
    for (const auto &s : odom2d_ptr->getBuffer().get())
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    // first split at exact timestamp
    TimeStamp t_split = t0 + 0.13;
    std::cout << "Split time:                  " << t_split - t0 << std::endl;

    FrameBasePtr new_keyframe_ptr = problem_ptr->emplaceFrame(KEY_FRAME, odom2d_ptr->getState(t_split), t_split);

    odom2d_ptr->keyFrameCallback(new_keyframe_ptr, 0);

    std::cout << "New buffer: oldest part:   < ";
    for (const auto &s : (std::static_pointer_cast<CaptureMotion>(new_keyframe_ptr->getCaptureList().front()))->getBuffer().get())
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    std::cout << "Original keeps the newest: < ";
    for (const auto &s : odom2d_ptr->getBuffer().get())
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    std::cout << "Processor measurement of new keyframe: " << std::endl;
    std::cout << "Delta: " << new_keyframe_ptr->getCaptureList().front()->getFeatureList().front()->getMeasurement().transpose() << std::endl;
    std::cout << "Covariance: " << std::endl << new_keyframe_ptr->getCaptureList().front()->getFeatureList().front()->getMeasurementCovariance() << std::endl;

    std::cout << "getState with TS previous than the last keyframe: " << t_split-0.5 << std::endl;
    std::cout << odom2d_ptr->getState(t_split-0.5) << std::endl;

    // Solve
    ceres::Solver::Summary summary = ceres_manager_ptr->solve();
    //std::cout << summary.FullReport() << std::endl;
    ceres_manager_ptr->computeCovariances(ALL_MARGINALS);

//    if ((problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[12]).array().abs().maxCoeff() > 1e-10)
//    {
//        std::cout << "After solving the problem, covariance of new keyframe:" << std::endl;
//        std::cout << "WOLF:" << std::endl << problem_ptr->getFrameCovariance(new_keyframe_ptr) << std::endl;
//        std::cout << "REFERENCE:" << std::endl << integrated_covariance_vector[12] << std::endl;
//        std::cout << "ERROR:" << std::endl << problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[12] << std::endl;
////        throw std::runtime_error("Integrated covariance different from reference.");
//        std::cout << "1st TEST COVARIANCE CHECK ------> ERROR!: Integrated covariance different from reference." << std::endl;
//
//    }
    EXPECT_TRUE((problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[12]).isMuchSmallerThan(1.0, Constants::EPS));


    // second split as non-exact timestamp
    t_split = t0 + 0.062;
    std::cout << "New split time (assyncronous and older than previous keyframe):  " << t_split - t0 << std::endl;

    new_keyframe_ptr = problem_ptr->emplaceFrame(KEY_FRAME, odom2d_ptr->getState(t_split), t_split);
    odom2d_ptr->keyFrameCallback(new_keyframe_ptr, 0);

    // Solve
    summary = ceres_manager_ptr->solve();
    //std::cout << summary.FullReport() << std::endl;
    ceres_manager_ptr->computeCovariances(ALL_MARGINALS);

//    if ((problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[5]).array().abs().maxCoeff() > 1e-10)
//    {
//        std::cout << "After solving the problem, covariance of new keyframe:" << std::endl;
//        std::cout << "WOLF:" << std::endl << problem_ptr->getFrameCovariance(new_keyframe_ptr) << std::endl;
//        std::cout << "REFERENCE:" << std::endl << integrated_covariance_vector[5] << std::endl;
//        std::cout << "ERROR:" << std::endl << problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[5] << std::endl;
////        throw std::runtime_error("Integrated covariance different from reference.");
//        std::cout << "2nd TEST COVARIANCE CHECK ------> ERROR!: Integrated covariance different from reference." << std::endl;
//
//    }
    EXPECT_TRUE((problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[5]).isMuchSmallerThan(1.0, Constants::EPS));


    std::cout << "All in one row:            < ";
    for (const auto &s : (std::static_pointer_cast<CaptureMotion>(new_keyframe_ptr->getCaptureList().front()))->getBuffer().get())
        std::cout << s.ts_ - t0 << ' ';
    std::cout << "> " << t_split - t0 << " < ";
    for (const auto &s : odom2d_ptr->getBuffer().get())
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    // Free allocated memory
    delete ceres_manager_ptr;
    std::cout << "ceres manager deleted" << std::endl;
    problem_ptr.reset();
    std::cout << "problem deleted" << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

