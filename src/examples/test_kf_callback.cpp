/*
 * test_kf_callback.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: jsola
 */



#include "../sensor_odom_2D.h"
#include "../processor_odom_2D.h"
#include "../processor_tracker_feature_dummy.h"
#include "../capture_void.h"

int main()
{
    using namespace wolf;
    using namespace Eigen;
    using namespace std;

    ProblemPtr problem = Problem::create("PO 2D");

    SensorBasePtr sen_odo    = problem->installSensor   ("ODOM 2D", "main odometer", (Vector3s()<<0,0,0).finished(),"");
    ProcessorParamsOdom2DPtr params_odo = std::make_shared<ProcessorParamsOdom2D>();
    params_odo->elapsed_time_th_ = 2;
    params_odo->theta_traveled_th_ = M_PI; // 180 degrees turn
    ProcessorBasePtr prc_odo = problem->installProcessor("ODOM 2D", "odometry integrator", sen_odo, params_odo);
    prc_odo->setTimeTolerance(0.1);

    SensorBasePtr sen_ftr    = problem->installSensor   ("ODOM 2D", "other odometer", (Vector3s()<<0,0,0).finished(),"");
    shared_ptr<ProcessorTrackerFeatureDummy> prc_ftr = make_shared<ProcessorTrackerFeatureDummy>(0.5, 7, 4);
    prc_ftr->setName("tracker");
    sen_ftr->addProcessor(prc_ftr);
    prc_ftr->setTimeTolerance(0.1);

    cout << "Motion sensor    : " << problem->getProcessorMotionPtr()->getSensorPtr()->getName() << endl;
    cout << "Motion processor : " << problem->getProcessorMotionPtr()->getName() << endl;

    TimeStamp t(0);
    cout << "=======================\n>> TIME: " << t.get() << endl;
    Vector3s x({0,0,0});
    Matrix3s P; P.setZero();
    problem->setPrior(x, P, t, 0.01);

    cout << "x(" << t.get() << ") = " << problem->getCurrentState().transpose() << endl;

    Vector2s odo_data;  odo_data << .1, (M_PI / 10);

    problem->print(2, false, true, true); // print(level, constr_by, metric, state_blocks)

    Scalar dt = 1;
    for (auto i = 0; i < 4; i++)
    {

        cout << "Tracker----------------" << endl;
        sen_ftr->process(make_shared<CaptureVoid>(t, sen_ftr));
        problem->print(2, false, true, true); // print(level, constr_by, metric, state_blocks)

        t += dt;
        cout << "=======================\n>> TIME: " << t.get() << endl;

        cout << "Motion-----------------" << endl;
        sen_odo->process(make_shared<CaptureMotion>(t, sen_odo, odo_data, 3, 3, nullptr));
        cout << "x(" << t.get() << ") = " << problem->getCurrentState().transpose() << endl;
        problem->print(2, false, true, true); // print(level, constr_by, metric, state_blocks)

    }

    return 0;
}
