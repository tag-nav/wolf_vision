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
#include <fstream>

using namespace Eigen;
using namespace std;
using namespace wolf;

//Global variables

//used in pure_translation test
const char * filename_pure_tranlation_imu_data;
const char * filename_pure_tranlation_odom;

TEST(ProcessorOdom3D, static_ceresOptimisation)
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

TEST(ProcessorIMU, static_ceresOptimisation_fixBias)
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

TEST(ProcessorIMU, Pure_translation)
{
    //In this test we will process both IMU and Odom3D data at the same time (in a same loop).
    //difference with test above, we don't wait for a KeyFrame to be created y processorIMU to process Odom data'

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    //load files containing accelerometer and gyroscope data
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

        imu_data_input.open(filename_pure_tranlation_imu_data);
        odom_data_input.open(filename_pure_tranlation_odom);
        std::cout << "pure translation imu file: " << filename_pure_tranlation_imu_data << std::endl;
        std::cout << "pure translation odom: " << filename_pure_tranlation_odom << std::endl;

        std::string dummy;
        getline(imu_data_input, dummy); getline(odom_data_input, dummy);

        if(!imu_data_input.is_open() || !odom_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }

    //prepare creation of file if DEBUG_RESULTS activated
#ifdef DEBUG_RESULTS
    std::ofstream debug_results;
    debug_results.open("debug_results.dat");
    if(debug_results)
        debug_results << "%%TimeStamp\t"
                      << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                      << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                      << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
#endif


    //===================================================== SETTING PROBLEM
    /*std::string wolf_root = _WOLF_ROOT_DIR;

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
    /*std::cout << "print...\n" << std::endl;
    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;*/

    //===================================================== END{SOLVER PART}
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "*static_ceresOptimisation*"; //default : use all test for static optimisation (not using any input)
  if (argc < 3)
    {
        std::cout << "Missing input argument to run pure_translation test! : needs 2 arguments (path to accelerometer file and path to gyroscope data)." << std::endl;
        ADD_FAILURE(); //Generates a non fatal failure
    }
 else{
     filename_pure_tranlation_imu_data = argv[1];
     filename_pure_tranlation_odom = argv[2];
     ::testing::GTEST_FLAG(filter) = "*static_ceresOptimisation*:*Pure_translation*"; //if arguments given, run test for static_optimisation + pure_translation
 }
  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}