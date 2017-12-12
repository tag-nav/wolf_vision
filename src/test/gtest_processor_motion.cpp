/*
 * gtest_processor_motion.cpp
 *
 *  Created on: Sep 27, 2017
 *      Author: jsola
 */


#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "sensor_odom_2D.h"
#include "processor_odom_2D.h"
#include "ceres_wrapper/ceres_manager.h"

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

        virtual void SetUp()
        {
            dt                          = 1.0;
            problem = Problem::create("PO 2D");
            ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
            params->dist_traveled_th_   = 100;
            params->theta_traveled_th_  = 6.28;
            params->elapsed_time_th_    = 2.5*dt; // force KF at every third process()
            params->cov_det_th_         = 100;
            params->unmeasured_perturbation_std_ = 0.001;
            sensor = static_pointer_cast<SensorOdom2D>(problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0)));
            processor = static_pointer_cast<ProcessorOdom2D>(problem->installProcessor("ODOM 2D", "odom", sensor, params));
            capture = std::make_shared<CaptureMotion>(0.0, sensor, data, data_cov, 3, 3, nullptr);

            Vector3s x0; x0 << 0, 0, 0;
            processor->setOrigin(x0, 0.0);
        }

        virtual void TearDown(){}

        virtual Motion interpolate(const Motion& _ref,
                                   Motion& _second,
                                   TimeStamp& _ts)
        {
            return ProcessorMotion::interpolate(_ref, _second, _ts);
        }

        virtual Motion motionZero(TimeStamp& t)
        {
            return ProcessorOdom2D::motionZero(t);
        }


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

    for (int i = 0; i<10; i++) // one full turn
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
    data << 1, 2*M_PI/10; // advance straight
    data_cov.setIdentity();
    TimeStamp t(0.0);
    std::vector<Motion> motions;
    motions.push_back(motionZero(t));

    for (int i = 0; i<10; i++)
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

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

