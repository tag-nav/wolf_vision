/*
 * gtest_processor_motion.cpp
 *
 *  Created on: Sep 27, 2017
 *      Author: jsola
 */

#include "utils_gtest.h"

#include "core/common/wolf.h"
#include "core/utils/logging.h"

#include "core/sensor/sensor_odom_2D.h"
#include "core/processor/processor_odom_2D.h"
#include "core/ceres_wrapper/ceres_manager.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

class ProcessorMotion_test : public ProcessorOdom2D, public testing::Test{
    public:
        Scalar              dt;
        ProblemPtr          problem;
        SensorOdom2DPtr     sensor;
        ProcessorOdom2DPtr  processor;
        CaptureMotionPtr    capture;
        Vector2s            data;
        Matrix2s            data_cov;

        ProcessorMotion_test() :
            ProcessorOdom2D(std::make_shared<ProcessorParamsOdom2D>()),
            dt(0)
        { }

        virtual void SetUp()
        {
            dt                      = 1.0;
            problem = Problem::create("PO", 2);
            ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
            params->time_tolerance  = 0.25;
            params->dist_traveled   = 100;
            params->angle_turned    = 6.28;
            params->max_time_span   = 2.5*dt; // force KF at every third process()
            params->cov_det         = 100;
            params->unmeasured_perturbation_std = 0.001;
            sensor = static_pointer_cast<SensorOdom2D>(problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0)));
            processor = static_pointer_cast<ProcessorOdom2D>(problem->installProcessor("ODOM 2D", "odom", sensor, params));
            capture = std::make_shared<CaptureMotion>("ODOM 2D", 0.0, sensor, data, data_cov, 3, 3, nullptr);

            Vector3s x0; x0 << 0, 0, 0;
            Matrix3s P0; P0.setIdentity();
            problem->setPrior(x0, P0, 0.0, 0.01);
        }

        Motion interpolate(const Motion& _ref, Motion& _second, TimeStamp& _ts)
        {
            return ProcessorMotion::interpolate(_ref, _second, _ts);
        }

        Motion interpolate(const Motion& _ref1, const Motion& _ref2, const TimeStamp& _ts, Motion& _second)
        {
            return ProcessorMotion::interpolate(_ref1, _ref2, _ts, _second);
        }


        virtual void TearDown(){}

};

TEST_F(ProcessorMotion_test, IntegrateStraight)
{
    data << 1, 0; // advance straight
    data_cov.setIdentity();
    TimeStamp t(0.0);

    for (int i = 0; i<9; i++)
    {
        t += dt;
        capture->setTimeStamp(t);
        capture->setData(data);
        capture->setDataCovariance(data_cov);
        processor->process(capture);
        WOLF_DEBUG("t: ", t, "  x: ", problem->getCurrentState().transpose());
    }

    ASSERT_MATRIX_APPROX(problem->getCurrentState(), (Vector3s()<<9,0,0).finished(), 1e-8);
}

TEST_F(ProcessorMotion_test, IntegrateCircle)
{
    data << 1, 2*M_PI/10; // advance in circle
    data_cov.setIdentity();
    TimeStamp t(0.0);

    for (int i = 0; i<10; i++) // one full turn exactly
    {
        t += dt;
        capture->setTimeStamp(t);
        capture->setData(data);
        capture->setDataCovariance(data_cov);
        processor->process(capture);
        WOLF_DEBUG("t: ", t, "  x: ", problem->getCurrentState().transpose());
    }

    ASSERT_MATRIX_APPROX(problem->getCurrentState(), (Vector3s()<<0,0,0).finished(), 1e-8);
}

TEST_F(ProcessorMotion_test, Interpolate)
{
    data << 1, 2*M_PI/10; // advance in turn
    data_cov.setIdentity();
    TimeStamp t(0.0);
    std::vector<Motion> motions;
    motions.push_back(motionZero(t));

    for (int i = 0; i<10; i++) // one full turn exactly
    {
        t += dt;
        capture->setTimeStamp(t);
        capture->setData(data);
        capture->setDataCovariance(data_cov);
        processor->process(capture);
        motions.push_back(processor->getMotion(t));
        WOLF_DEBUG("t: ", t, "  x: ", problem->getCurrentState().transpose());
    }

    TimeStamp tt    = 2.2;
    Motion ref      = motions[2];
    Motion second   = motions[3];
    Motion interp   = interpolate(ref, second, tt);

    ASSERT_NEAR(         interp.ts_.get()       , 2.2                       , 1e-8);
    ASSERT_MATRIX_APPROX(interp.data_           , VectorXs::Zero(2)         , 1e-8);
    ASSERT_MATRIX_APPROX(interp.delta_          , VectorXs::Zero(3)         , 1e-8);
    ASSERT_MATRIX_APPROX(interp.delta_integr_   , motions[2].delta_integr_  , 1e-8);

    tt      = 2.6;
    interp  = interpolate(ref, second, tt);

    ASSERT_NEAR(         interp.ts_.get()       , 2.6                       , 1e-8);
    ASSERT_MATRIX_APPROX(interp.data_           , motions[3].data_          , 1e-8);
    ASSERT_MATRIX_APPROX(interp.delta_          , motions[3].delta_         , 1e-8);
    ASSERT_MATRIX_APPROX(interp.delta_integr_   , motions[3].delta_integr_  , 1e-8);
}

//TEST_F(ProcessorMotion_test, Interpolate_alternative)
//{
//    data << 1, 2*M_PI/10; // advance in turn
//    data_cov.setIdentity();
//    TimeStamp t(0.0);
//    std::vector<Motion> motions;
//    motions.push_back(motionZero(t));
//
//    for (int i = 0; i<10; i++) // one full turn exactly
//    {
//        t += dt;
//        capture->setTimeStamp(t);
//        capture->setData(data);
//        capture->setDataCovariance(data_cov);
//        processor->process(capture);
//        motions.push_back(processor->getMotion(t));
//        WOLF_DEBUG("t: ", t, "  x: ", problem->getCurrentState().transpose());
//    }
//
//    TimeStamp tt    = 2.2;
//    Motion ref1     = motions[2];
//    Motion ref2     = motions[3];
//    Motion second(0.0, 2, 3, 3, 0);
//    Motion interp   = interpolate(ref1, ref2, tt, second);
//
//    ASSERT_NEAR(         interp.ts_.get()       , 2.2                       , 1e-8);
//    ASSERT_MATRIX_APPROX(interp.data_           , VectorXs::Zero(2)         , 1e-8);
//    ASSERT_MATRIX_APPROX(interp.delta_          , VectorXs::Zero(3)         , 1e-8);
//    ASSERT_MATRIX_APPROX(interp.delta_integr_   , motions[2].delta_integr_  , 1e-8);
//
//    ASSERT_NEAR(         second.ts_.get()       , 3.0                       , 1e-8);
//    ASSERT_MATRIX_APPROX(second.data_           , motions[3].data_          , 1e-8);
//    ASSERT_MATRIX_APPROX(second.delta_          , motions[3].delta_         , 1e-8);
//    ASSERT_MATRIX_APPROX(second.delta_integr_   , motions[3].delta_integr_  , 1e-8);
//
//    tt      = 2.6;
//    interp  = interpolate(ref1, ref2, tt, second);
//
//    ASSERT_NEAR(         interp.ts_.get()       , 2.6                       , 1e-8);
//    ASSERT_MATRIX_APPROX(interp.data_           , motions[3].data_          , 1e-8);
//    ASSERT_MATRIX_APPROX(interp.delta_          , motions[3].delta_         , 1e-8);
//    ASSERT_MATRIX_APPROX(interp.delta_integr_   , motions[3].delta_integr_  , 1e-8);
//
//    ASSERT_NEAR(         second.ts_.get()       , 3.0                       , 1e-8);
//    ASSERT_MATRIX_APPROX(second.data_           , VectorXs::Zero(2)         , 1e-8);
//    ASSERT_MATRIX_APPROX(second.delta_          , VectorXs::Zero(3)         , 1e-8);
//    ASSERT_MATRIX_APPROX(second.delta_integr_   , motions[3].delta_integr_  , 1e-8);
//}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

