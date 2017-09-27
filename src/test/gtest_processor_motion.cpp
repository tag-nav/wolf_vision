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

class ProcessorMotion_test : public testing::Test{
    public:
        Scalar              dt;
        ProblemPtr          problem;
        SensorOdom2DPtr     sensor;
        ProcessorOdom2DPtr  processor;
        CaptureMotionPtr    capture;
        Vector2s            data;
        Matrix2s            data_cov;
        Matrix3s            unmeasured_cov;

        virtual void SetUp()
        {
            dt = 0.1;
            problem = Problem::create("PO 2D");
            ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
            params->dist_traveled_th_   = 100;
            params->theta_traveled_th_  = 6.28;
            params->elapsed_time_th_    = 2.5*dt; // force KF at every third process()
            params->cov_det_th_         = 100;
            params->unmeasured_perturbation_std_ = 0.001;
            sensor = static_pointer_cast<SensorOdom2D>(problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0)));
            processor = static_pointer_cast<ProcessorOdom2D>(problem->installProcessor("ODOM 2D", "odom", sensor, params));
            unmeasured_cov = params->unmeasured_perturbation_std_ * params->unmeasured_perturbation_std_ * Matrix3s::Identity();
        }

        virtual void TearDown(){}

};


TEST_F(ProcessorMotion_test, DummyTestExample)
{
    TimeStamp t = 0.0;
    data << 1, 0; // advance straight
    data_cov.setIdentity();
    capture = std::make_shared<CaptureMotion>(t, sensor, data, data_cov, 3, 3, 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

