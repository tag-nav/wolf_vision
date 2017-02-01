/**
 * \file gtest_ceres.cpp
 *
 *  Created on: Jan 18, 2017
 *      \author: Dinesh Atchuthan
 */


#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processor_odom_3D.h"
#include "processor_imu.h"
#include "wolf.h"
#include "problem.h"
#include "ceres_wrapper/ceres_manager.h"
#include "state_quaternion.h"
#include "sensor_imu.h"

#include <iostream>

using namespace Eigen;
using namespace std;
using namespace wolf;

TEST(ProcessorOdom3D, static_ceresOptimiszation)
{
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", "odom");
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    TimeStamp t(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    /* We do not need to create features and frames and constraints here. Everything is done in wolf.
    Features and constraint at created automatically when a new Keyframe is generated. Whether a new keyframe should be created or not, this is
    handled by voteForKeyFrame() function for this processorMotion
    */

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/
     ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
     std::cout << summary.FullReport() << std::endl;

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

TEST(ProcessorIMU, static_ceresOptimiszation_fixBias)
{
    //With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
    // We must add an odometry to make covariances observable Or... we could fix all bias stateBlocks
    //First we will try to fix bias stateBlocks

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

    SensorBasePtr sen_imu = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorBasePtr processor_ptr = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml");

    //setting origin
    Eigen::VectorXs x0(16);
    TimeStamp t(0);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.001,  0,0,.002;
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE IMU CLASSES  **************/
    Eigen::Vector6s data;
    //data << 0.15,0.10,9.88, 0.023,0.014,0.06;
    data << 0.0,0.0,9.81, 0.0,0.0,0.0;
    Scalar dt = t.get();
    TimeStamp ts(0);
    while((dt-t.get())<=std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan()){
    // Time and data variables
    dt += 0.001;
    ts.set(dt);

    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    // process data in capture
    sen_imu->process(imu_ptr);
    }

    //Fix all biases StateBlocks
    for(FrameBasePtr it : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList()){
        ( std::static_pointer_cast<FrameIMU>(it) )->getAccBiasPtr()->fix();
        ( std::static_pointer_cast<FrameIMU>(it) )->getGyroBiasPtr()->fix();
    }

    //Check and print wolf tree
    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }
    
                                             /************** SOLVER PART  **************/
     ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
     std::cout << summary.FullReport() << std::endl;
     
    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

TEST(ProcessorIMU, static_ceresOptimisation_Odom0)
{
    //With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
    // We must add an odometry to make covariances observable

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


    // SENSOR + PROCESSOR IMU
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml");
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

    // SET ORIGIN AND FIX ORIGIN KEYFRAME
    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    //wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();


    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", "odom", wolf_root + "/src/examples/processor_odom_3D.yaml");
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // There should be a FrameIMU at origin as KeyFrame + 1 FrameIMU and 1 FrameOdom Non-KeyFrame
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);

    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU + CaptureFix due to setting problem origin before installing processors
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),3);
    /*for ( for CaptureBasePtr C : (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList() )
    {

    }*/
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    //data << 0.0019, 0.0001, 9.8122, 0.1022, 0.1171, -0.0413;
    data << 0.00, 0.000, 9.81, 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*2) ){
        
        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    wolf_problem_ptr_->setProcessorMotion(processor_ptr_odom3D);
    data_odom3D << 0,0,0, 0,0,0;
    //Add an Odom3D constraint
    //dt += 0.001;
    ts.set(dt);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    //Check and print wolf tree
    //wolf_problem_ptr_->print(4,1,1,1);
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/
    std::cout << "print...\n" << std::endl;
    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

TEST(ProcessorIMU, static_ceresOptimisation_Odom1)
{
    //In this test we will process both IMU and Odom3D data at the same time (in a same loop).
    //difference with test above, we don't wait for a KeyFrame to be created y processorIMU to process Odom data'

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


    // SENSOR + PROCESSOR IMU
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml");
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

    // SET ORIGIN AND FIX ORIGIN KEYFRAME
    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    //wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();


    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", "odom", wolf_root + "/src/examples/processor_odom_3D.yaml");
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // There should be a FrameIMU at origin as KeyFrame + 1 FrameIMU and 1 FrameOdom Non-KeyFrame
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);

    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU + CaptureFix due to setting problem origin before installing processors
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),3);
    /*for ( for CaptureBasePtr C : (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList() )
    {

    }*/
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data, data_odom3D;
    data << 0.0019, 0.0001, 9.8122, 0.1022, 0.1171, -0.0413;
    //data << 0.00, 0.000, 9.81, 0.0, 0.0, 0.0;
    data_odom3D << 0,0,0, 0,0,0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);
    unsigned int iter = 0;

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*2) ){
        
        // PROCESS IMU DATA
        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(iter == 100) //every 100 ms
        {
            // PROCESS ODOM 3D DATA
            mot_ptr->setTimeStamp(ts);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);
        }
    }

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    //Check and print wolf tree
    //wolf_problem_ptr_->print(4,1,1,1);
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/
    std::cout << "print...\n" << std::endl;
    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}