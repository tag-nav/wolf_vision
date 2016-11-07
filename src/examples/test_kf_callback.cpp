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

    ProblemPtr problem = Problem::create(FRM_PO_2D);

    SensorBasePtr sen_odo    = problem->installSensor("ODOM 2D", "main odometer", (Vector3s()<<0,0,0).finished(),"");
    ProcessorBasePtr prc_odo = problem->installProcessor("ODOM 2D", "odometry integrator", "main odometer", "");
    prc_odo->setTimeTolerance(0.1);

    SensorBasePtr sen_ftr    = problem->installSensor("ODOM 2D", "other odometer", (Vector3s()<<0,0,0).finished(),"");
    shared_ptr<ProcessorTrackerFeatureDummy> prc_ftr = make_shared<ProcessorTrackerFeatureDummy>(7, 4);
    prc_ftr->setName("tracker");
    sen_ftr->addProcessor(prc_ftr);
    prc_ftr->setTimeTolerance(0.1);

    cout << "Motion sensor    : " << problem->getProcessorMotionPtr()->getSensorPtr()->getName() << endl;
    cout << "Motion processor : " << problem->getProcessorMotionPtr()->getName() << endl;

    TimeStamp t(0);
    Vector3s x; x << 0,0,0;
    WOLF_DEBUG_HERE
    problem->setOrigin(x, (Matrix3s::Identity()), t);
    WOLF_DEBUG_HERE

    cout << "x(0) = " << problem->getCurrentState().transpose() << endl;

    Vector2s odo_data;  odo_data << .1, (M_PI / 2);

    problem->print(2);

    Scalar dt = 1;
    for (auto i = 0; i < 4; i++)
    {
        t += dt;
        cout << "=======================\n>> TIME: " << t.get() << endl;

        cout << "Tracker----------------" << endl;
        sen_ftr->addCapture(make_shared<CaptureVoid>(t, sen_ftr));
        problem->print(2);

        cout << "Motion-----------------" << endl;
        sen_odo->addCapture(make_shared<CaptureMotion>(t, sen_odo, odo_data));
        cout << "x(" << t.get() << ") = " << problem->getCurrentState().transpose() << endl;
        problem->print(2);

    }



    return 0;
}
