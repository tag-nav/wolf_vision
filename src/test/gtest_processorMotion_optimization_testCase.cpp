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
#include "rotations.h"

// wolf yaml
#include "../yaml/yaml_conversion.h"
// yaml-cpp library
#include <../yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;
using namespace wolf;

//Global variables

//used in pure_translation test
char * filename_motion_imu_data;
char * filename_motion_odom;
unsigned int number_of_KF = 2; //determine the number of final keyframes that will be created (except origin, so n>=1) in some of processorIMU tests

class ProcessorIMU_Odom_tests : public testing::Test
{
    public:
        wolf::TimeStamp t;
        Eigen::Vector6s data_;
        wolf::Scalar dt;
        SensorIMUPtr sen_imu;
        SensorOdom3DPtr sen_odom3D;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        ProcessorOdom3DPtr processor_ptr_odom3D;


    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;
        ASSERT_TRUE(number_of_KF>0) << "number_of_KF (number of Keyframe created) must be int >0";

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
        t.set(0);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


        // SENSOR + PROCESSOR IMU
        //We want a processorIMU with a specific max_time_span (1s) forour test
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
        prc_imu_params->max_time_span = 1;
        prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_imu_params->dist_traveled = 1000000000;
        prc_imu_params->angle_turned = 1000000000;

        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 1;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

        //ORIGIN MUST BE SET IN THE TEST

    //===================================================== END{SETTING PROBLEM}
    }

    virtual void TearDown()
    {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
        /*
            You can do deallocation of resources in TearDown or the destructor routine. 
                However, if you want exception handling you must do it only in the TearDown code because throwing an exception 
                from the destructor results in undefined behavior.
            The Google assertion macros may throw exceptions in platforms where they are enabled in future releases. 
                Therefore, it's a good idea to use assertion macros in the TearDown code for better maintenance.
        */
    }
};


TEST(ProcessorOdom3D, static_ceresOptimisation_Odom_PO)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve.
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1]
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);
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

     //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
     ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
     
     //This is a static test so we are not supposed to have moved from origin to last KeyFrame
     ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
     ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
     EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

TEST(ProcessorOdom3D, static_ceresOptimisation_convergenceOdom_PO)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve. 
     *
     * But before solving, we change the state of final KeyFrame.
     * First we change only Px, then Py, then Pz, then all of them
     * Second : we change Ox, then Oy, then Oz, then all of them
     * Third : we change everything
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1]
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);
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

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/

     /* ___________________________________________ CHANGING FINAL FRAME BEFORE OPTIMIZATION ___________________________________________*/
    
    //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
    EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    //get pointer to the last KeyFrame (which is at t = 2s)
    EXPECT_EQ(t.get(),2);
    FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);

    // FIRST SOLVER TEST WITHOUT CHANGING ANYTHING - WE DID NOT MOVE

    std::cout << "______ solving...______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved !______" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;


    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
    

                                                    /*********************/
                                                    //CHANGE PX AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<30,0,0,0,0,0,1).finished());

    std::cout << "______ solving... Px changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Px changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Px is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PY AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<0,30,0,0,0,0,1).finished());

    std::cout << "______ solving... Py changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Py changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Py is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Py is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Py changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Py changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PZ AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<0,0,30,0,0,0,1).finished());

    std::cout << "______ solving... Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Pz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Pz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Pz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Pz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE PX, Py AND PZ AND SOLVE//
                                                    /********************************/

    last_KF->setState((Vector7s()<<25,20,30,0,0,0,1).finished());

    std::cout << "______ solving... Px, Py and Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Px, Py and Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Px, Py and Pz are changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px, Py and Pz are changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px, Py and Pz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px, Py and Pz changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OX AND SOLVE//
                                                    /*********************/
    Eigen::Vector3s o_initial_guess;
    Eigen::Quaternions q_init_guess;

    o_initial_guess << 40,0,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Ox changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Ox changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Ox is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OY AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,40,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Oy changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Oy changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oy is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Oy is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oy changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oy changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OZ AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,0,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Oz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE OX, OY AND OZ AND SOLVE//
                                                    /********************************/
    o_initial_guess << 80,50,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Ox, Oy and Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Ox, Oy and Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox, Oy and Oz changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Ox, Oy and Oz changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox, Oy and Oz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox, Oy and Oz changed" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);
}


TEST(ProcessorOdom3D, static_ceresOptimisation_convergenceOdom_POV)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve. 
     *
     * But before solving, we change the state of final KeyFrame.
     * First we change only Px, then Py, then Pz, then all of them
     * Second : we change Ox, then Oy, then Oz, then all of them
     * Third : we change everything
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1 ,0,0,0] -->Checked with Assertions
     *
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_POV_3D);
    Eigen::VectorXs x0(10);
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    t.set(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/
    wolf_problem_ptr_->print(4,1,1,1);

                                             /************** SOLVER PART  **************/

    //If we want the covariances to be computed, then we need to fix all Velocity StateBlocks because they cannot be observed we Odometry measurements only
    /*for(FrameBasePtr Frame_ptr : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList())
    {
        Frame_ptr->getVPtr()->fix();
    }*/

     /* ___________________________________________ CHANGING FINAL FRAME BEFORE OPTIMIZATION ___________________________________________*/
    
    //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
    EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    //get pointer to the last KeyFrame (which is at t = 2s)
    EXPECT_EQ(t.get(),2);
    FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
    Eigen::Matrix<wolf::Scalar, 10, 1> kf2_state = last_KF->getState(); //This state vector will be used to get the velocity state

    // FIRST SOLVER TEST WITHOUT CHANGING ANYTHING - WE DID NOT MOVE

    std::cout << "\n\t\t\t______ solving...______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved !______" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;


    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
    

                                                    /*********************/
                                                    //CHANGE PX AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<30,0,0,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Px changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Px changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Px is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PY AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,30,0,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Py changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Py changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Py is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Py is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Py changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Py changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PZ AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,30,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Pz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Pz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Pz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Pz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE PX, Py AND PZ AND SOLVE//
                                                    /********************************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<25,20,30,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Px, Py and Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Px, Py and Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Px, Py and Pz are changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px, Py and Pz are changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\n\t\t\t ______computing covariances______ Px, Py and Pz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px, Py and Pz changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OX AND SOLVE//
                                                    /*********************/
    Eigen::Vector3s o_initial_guess;
    Eigen::Quaternions q_init_guess;

    o_initial_guess << 40,0,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Ox changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Ox changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Ox is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OY AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,40,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Oy changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Oy changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oy is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Oy is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oy changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oy changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OZ AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,0,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Oz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE OX, OY AND OZ AND SOLVE//
                                                    /********************************/
    o_initial_guess << 80,50,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Ox, Oy and Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Ox, Oy and Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox, Oy and Oz changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Ox, Oy and Oz changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox, Oy and Oz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox, Oy and Oz changed" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);
}


TEST(ProcessorIMU, static_ceresOptimisation_fixBias)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Bias of all frames are fixed before we call the solver
     * 
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances observable Or... we could fix all bias stateBlocks
     * First we will try to fix bias stateBlocks
     */


    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

    SensorBasePtr sen_imu = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");

    // We want to create a processorIMU with a max_time_span of 1 seconds.
    // Default processorIMUParams is made so that a KeyFrame will be created at each step.
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen_imu, prc_imu_params);

    //setting origin and fixing origin KeyFrame
    Eigen::VectorXs x0(16);
    TimeStamp t(0);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0;
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
    data << 0.0,0.0,-wolf::gravity()(2), 0.0,0.0,0.0; //we use exactly the value of gravity defined in wolf.h

    //integrate IMU data until KeyFrame creation (until we reach max_time_span)
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

    //Fix all biases StateBlocks -> to make covariances computable
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

     wolf_problem_ptr_->print(4,1,1,1);

     //We check with assertions if Final KeyFrame has the same state as origin_KF
     FrameBasePtr origin_KF = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front();
     FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts);
     ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     
    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

//_______________________________________________________________________________________________________________
// ##################################### static_Optim_IMUOdom_2KF TESTS #####################################
//_______________________________________________________________________________________________________________

/*
 *  Tests below will be based on the following representation :
 *     KF0 ---- constraintIMU ---- KF1
 *        \____constraintOdom3D___/
 *
 *
 */

TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_2KF_Unfix1StateBlock)
{

    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     *
     *
     */

     //===================================================== END OF SETTINGS

     // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    Eigen::VectorXs initial_origin_state(16);
    Eigen::VectorXs initial_final_state(16);

    //store states before optimization so that we can reset the frames to their original state for other optimization tests
    origin_KF->getState(initial_origin_state);
    last_KF->getState(initial_final_state);

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);

    //get stateblocks from origin and last KF and concatenate them in one std::vector to unfix stateblocks
    std::vector<StateBlockPtr> originStateBlock_vec = origin_KF->getStateBlockVec();
    std::vector<StateBlockPtr> finalStateBlock_vec = last_KF->getStateBlockVec();
    std::vector<StateBlockPtr> allStateBlocks;

    allStateBlocks.reserve(originStateBlock_vec.size() + finalStateBlock_vec.size());
    allStateBlocks.insert( allStateBlocks.end(), originStateBlock_vec.begin(), originStateBlock_vec.end() );
    allStateBlocks.insert( allStateBlocks.end(), finalStateBlock_vec.begin(), finalStateBlock_vec.end() );

    /*std::cout << "\n\t ######### TEST 1 : ONLY 1 STATEBLOCK UNFIXED (BOTH KF FIXED) #########" << std::endl;
    for(int i = 0; i<allStateBlocks.size(); i++)
    {
        origin_KF->setState(initial_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we now unfix only one StateBlock
        allStateBlocks[i]->unfix();

        std::cout << "\t\t\t ______solving______ test number " << i <<  std::endl;
        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
    }

    std::cout << "\n\t ######### TEST 2 : ONLY 1 STATEBLOCK UNFIXED (ONLY ORIGIN_KF FIXED) #########" << std::endl;
    for(int i = 0; i<originStateBlock_vec.size(); i++)
    {
        origin_KF->setState(initial_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); //this fix the all keyframe

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        std::cout << "\t\t\t ______solving______ test number " << i <<  std::endl;
        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
    }

    std::cout << "\n\t ######### TEST 3 : ONLY 1 STATEBLOCK UNFIXED (ONLY LAST_KF FIXED) #########" << std::endl;
    for(int i = 0; i<finalStateBlock_vec.size(); i++)
    {
        origin_KF->setState(initial_origin_state);
        last_KF->setState(initial_final_state);

        last_KF->fix(); //this fix the all keyframe

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        std::cout << "\t\t\t ______solving______ test number " << i <<  std::endl;
        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
    }

    //we do the same but we unfix 2 state blocks

    std::cout << "\n\t ######### TEST 4 : 2 STATEBLOCK UNFIXED (BOTH KF FIXED) #########" << std::endl;
    for(int i = 0; i<allStateBlocks.size(); i++)
    {
        for(int j = 0; j<allStateBlocks.size(); j++)
        {   
            std::cout << "\t\t\t ______solving______ test number " << i << "." << j << std::endl;

            origin_KF->setState(initial_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix(); //this fix the all keyframe
            last_KF->fix();
            //we now unfix only one StateBlock
            allStateBlocks[i]->unfix();
            allStateBlocks[j]->unfix();

            ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
            std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
        }
    }

    std::cout << "\n\t ######### TEST 5 : 2 STATEBLOCK UNFIXED (ONLY ORIGIN KF FIXED) #########" << std::endl;
    for(int i = 0; i<originStateBlock_vec.size(); i++)
    {
        for(int j = 0; j<originStateBlock_vec.size(); j++)
        {   
            std::cout << "\t\t\t ______solving______ test number " << i << "." << j << std::endl;

            origin_KF->setState(initial_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix(); //this fix the all keyframe

            //we now unfix only one StateBlock
            originStateBlock_vec[i]->unfix();
            originStateBlock_vec[j]->unfix();

            ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
            std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
        }
    }

    std::cout << "\n\t ######### TEST 6 : 2 STATEBLOCK UNFIXED (ONLY LAST KF FIXED) #########" << std::endl;
    for(int i = 0; i<finalStateBlock_vec.size(); i++)
    {
        for(int j = 0; j<finalStateBlock_vec.size(); j++)
        {   
            std::cout << "\t\t\t ______solving______ test number " << i << "." << j << std::endl;

            origin_KF->setState(initial_origin_state);
            last_KF->setState(initial_final_state);

            last_KF->fix();

            //we now unfix only one StateBlock
            finalStateBlock_vec[i]->unfix();
            finalStateBlock_vec[j]->unfix();

            ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
            std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
        }
    }*/

    std::cout << "\n\t ######### TEST 1 : BOTH KF FIXED, UNFIX UP TO 5 STATEBLOCKS #########" << std::endl;
    for(int i = 0; i<allStateBlocks.size(); i++)
    {
        for(int j = 0; j<allStateBlocks.size(); j++)
        {   
            for(int k = 0; k<allStateBlocks.size(); k++)
            {
                for(int l = 0; l<allStateBlocks.size(); l++)
                {
                    for(int m = 0; m<allStateBlocks.size(); m++)
                    {
                        std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

                        origin_KF->setState(initial_origin_state);
                        last_KF->setState(initial_final_state);

                        origin_KF->fix(); //this fix the all keyframe
                        last_KF->fix();
                        //we now unfix only one StateBlock
                        allStateBlocks[i]->unfix();
                        allStateBlocks[j]->unfix();
                        allStateBlocks[k]->unfix();
                        allStateBlocks[l]->unfix();
                        allStateBlocks[m]->unfix();

                        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                        //std::cout << summary.BriefReport() << std::endl;

                        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
                        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                    }
                }
            }
        }
    }

    std::cout << "\n\t ######### TEST 2 : ORIGIN KF FIXED, UNFIX UP TO 3 STATEBLOCKS #########" << std::endl;

    last_KF->unfix();
    for(int i = 0; i<originStateBlock_vec.size(); i++)
    {
        for(int j = 0; j<originStateBlock_vec.size(); j++)
        {   
            for(int k = 0; k<originStateBlock_vec.size(); k++)
            {
                for(int l = 0; l<originStateBlock_vec.size(); l++)
                {
                    std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << std::endl;

                    origin_KF->setState(initial_origin_state);
                    last_KF->setState(initial_final_state);

                    origin_KF->fix(); //this fix the all keyframe

                    //we now unfix only one StateBlock
                    originStateBlock_vec[i]->unfix();
                    originStateBlock_vec[j]->unfix();
                    originStateBlock_vec[k]->unfix();
                    originStateBlock_vec[l]->unfix();

                    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                    //std::cout << summary.BriefReport() << std::endl;

                    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
                    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                }
            }
        }
    }

    //case with origin and last KF completely unfixed

    std::cout << "\n\t ######### TEST 3 : LAST KF FIXED, UNFIX UP TO 3 STATEBLOCKS #########" << std::endl;
    
    origin_KF->unfix();
    for(int i = 0; i<finalStateBlock_vec.size(); i++)
    {
        for(int j = 0; j<finalStateBlock_vec.size(); j++)
        {   
            for(int k = 0; k<finalStateBlock_vec.size(); k++)
            {
                for(int l = 0; l<finalStateBlock_vec.size(); l++)
                {
                    //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << std::endl;

                    origin_KF->setState(initial_origin_state);
                    last_KF->setState(initial_final_state);

                    last_KF->fix(); //this fix the all keyframe

                    //we now unfix only one StateBlock
                    finalStateBlock_vec[i]->unfix();
                    finalStateBlock_vec[j]->unfix();
                    finalStateBlock_vec[k]->unfix();
                    finalStateBlock_vec[l]->unfix();

                    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                    //std::cout << summary.BriefReport() << std::endl;

                    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
                    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                }
            }
        }
    }

    std::cout << "\t\t\t ______solving______ BOTH KF UNFIXED" << std::endl;
    origin_KF->setState(initial_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix(); //this fix the all keyframe
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU

    wolf_problem_ptr_->print(4,1,1,1);

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}

}

TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_2KF_perturbate_position)
{

    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     *
     * We perturbate a position (x,y or z or 2 of them or all of them before calling ceres)
     */

     //===================================================== END OF SETTINGS

     WOLF_WARN("THIS TEST IS VERY LONG TO COMPUTE...")

     // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    Eigen::VectorXs initial_origin_state(16);
    Eigen::VectorXs initial_final_state(16);

    //store states before optimization so that we can reset the frames to their original state for other optimization tests
    origin_KF->getState(initial_origin_state);
    last_KF->getState(initial_final_state);

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);

    //get stateblocks from origin and last KF and concatenate them in one std::vector to unfix stateblocks
    std::vector<StateBlockPtr> originStateBlock_vec = origin_KF->getStateBlockVec();
    std::vector<StateBlockPtr> finalStateBlock_vec = last_KF->getStateBlockVec();
    std::vector<StateBlockPtr> allStateBlocks;

    allStateBlocks.reserve(originStateBlock_vec.size() + finalStateBlock_vec.size());
    allStateBlocks.insert( allStateBlocks.end(), originStateBlock_vec.begin(), originStateBlock_vec.end() );
    allStateBlocks.insert( allStateBlocks.end(), finalStateBlock_vec.begin(), finalStateBlock_vec.end() );


    std::cout << "\n\t ### TEST 1 : BOTH KF FIXED, UNFIX 1 STATEBLOCK, PERTURBATE 1 ORIGIN POSITION (EXCEPT FINAL POSITION)###" << std::endl;

    for(int pert_index = 0; pert_index<3; pert_index++)
    {
        //perturate initial state
        Eigen::VectorXs perturbated_origin_state(16);
        perturbated_origin_state = initial_origin_state;
        perturbated_origin_state(pert_index) += 1.0;

        for(int i = 1; i<originStateBlock_vec.size(); i++)
        {

            //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix(); //this fix the all keyframe
            last_KF->fix();
            //we unfix origin position stateblock to let it converge
            originStateBlock_vec[0]->unfix();

            //we now unfix only one StateBlock
            originStateBlock_vec[i]->unfix();

            ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
            std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
        }

        for(int i = 1; i<finalStateBlock_vec.size(); i++)
        {

            //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix(); //this fix the all keyframe
            last_KF->fix();
            //we unfix origin position stateblock to let it converge
            originStateBlock_vec[0]->unfix();

            //we now unfix only one StateBlock
            finalStateBlock_vec[i]->unfix();

            ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
            std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
        }
    }

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 2 : BOTH KF FIXED, UNFIX 1 STATEBLOCKS, PERTURBATE 2 ORIGIN POSITIONS ###" << std::endl;

    for(int pert_index0 = 0; pert_index0<3; pert_index0++)
    {
        for(int pert_index1 = 1; pert_index1<3; pert_index1++)
        {
            //perturate initial state
            Eigen::VectorXs perturbated_origin_state(16);
            perturbated_origin_state = initial_origin_state;
            perturbated_origin_state(pert_index0) += 1.0;
            perturbated_origin_state(pert_index1) += 1.0;

            for(int i = 1; i<originStateBlock_vec.size(); i++)
            {

                //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix(); //this fix the all keyframe
                last_KF->fix();
                //we unfix origin position stateblock to let it converge
                originStateBlock_vec[0]->unfix();

                //we now unfix only one StateBlock
                originStateBlock_vec[i]->unfix();

                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
            }

            for(int i = 1; i<finalStateBlock_vec.size(); i++)
            {

                //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix(); //this fix the all keyframe
                last_KF->fix();
                //we unfix origin position stateblock to let it converge
                originStateBlock_vec[0]->unfix();

                //we now unfix only one StateBlock
                finalStateBlock_vec[i]->unfix();

                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
            }
        }
    }

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 3 : BOTH KF FIXED, UNFIX 1 STATEBLOCK, PERTURBATE ALL ORIGIN POSITIONS ###" << std::endl;

    //perturate initial state
    Eigen::VectorXs perturbated_origin_state(16);
    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(0) += 1.0;
    perturbated_origin_state(1) += 2.0;
    perturbated_origin_state(2) += 3.0;

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {
        std::cout << "\t\t\t ______solving______ test number " << i << std::endl;

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we unfix origin position stateblock to let it converge
        originStateBlock_vec[0]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {
        //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we unfix origin position stateblock to let it converge
        originStateBlock_vec[0]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 4 : BOTH KF FIXED, UNFIX 1 STATEBLOCK, PERTURBATE ALL FINAL POSITIONS ###" << std::endl;

    //perturate initial state
    Eigen::VectorXs perturbated_final_state(16);
    perturbated_final_state = initial_final_state;
    perturbated_final_state(0) += 1.0;
    perturbated_final_state(1) += 2.0;
    perturbated_final_state(2) += 3.0;

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {
        std::cout << "\t\t\t ______solving______ test number " << i << std::endl;

        origin_KF->setState(initial_origin_state);
        last_KF->setState(perturbated_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we unfix origin position stateblock to let it converge
        finalStateBlock_vec[0]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {
        //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we unfix origin position stateblock to let it converge
        finalStateBlock_vec[0]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        /*
         * Origin KF is fixed. Only 1 StateBlock is unfixed in last_KF.
         * Even with the odometry constraint imposing no movement. It is not sure that we can converge exactly to the pose of origin_KF.
         * otherwise, one of the conditions wil not be met.
         */

        EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
    }

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 5 : BOTH KF UNFIXED, PERTURBATE ALL ORIGIN POSITIONS ###" << std::endl;

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix(); //this fix the all keyframe
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
    /*ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));*/

    /* Here we gave an initial position and final position. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somwhere between both keyframe positions.
     *
     */

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 6 : BOTH KF UNFIXED, PERTURBATE ALL FINAL POSITIONS ###" << std::endl;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->unfix(); //this fix the all keyframe
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
    /*ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));*/

    /* Here we gave an initial position and final position. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somwhere between both keyframe positions.
     *
     */

    wolf_problem_ptr_->print(4,1,1,1);
}


TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_2KF_perturbate_velocity)
{

    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     *
     * We perturbate a velocity value (x,y or z or 2 of them or all of them before calling ceres)
     */

     //===================================================== END OF SETTINGS

     WOLF_WARN("THIS TEST IS VERY LONG TO COMPUTE...")

     // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    Eigen::VectorXs initial_origin_state(16);
    Eigen::VectorXs initial_final_state(16);

    //store states before optimization so that we can reset the frames to their original state for other optimization tests
    origin_KF->getState(initial_origin_state);
    last_KF->getState(initial_final_state);

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);

    //get stateblocks from origin and last KF and concatenate them in one std::vector to unfix stateblocks
    std::vector<StateBlockPtr> originStateBlock_vec = origin_KF->getStateBlockVec();
    std::vector<StateBlockPtr> finalStateBlock_vec = last_KF->getStateBlockVec();
    std::vector<StateBlockPtr> allStateBlocks;

    allStateBlocks.reserve(originStateBlock_vec.size() + finalStateBlock_vec.size());
    allStateBlocks.insert( allStateBlocks.end(), originStateBlock_vec.begin(), originStateBlock_vec.end() );
    allStateBlocks.insert( allStateBlocks.end(), finalStateBlock_vec.begin(), finalStateBlock_vec.end() );


    std::cout << "\n\t ### TEST 1 : BOTH KF FIXED, UNFIX 1 STATEBLOCK, PERTURBATE 1 ORIGIN VELOCITY (EXCEPT FINAL VELOCITY)###" << std::endl;

    for(int pert_index = 7; pert_index<10; pert_index++)
    {
        //perturate initial state
        Eigen::VectorXs perturbated_origin_state(16);
        perturbated_origin_state = initial_origin_state;
        perturbated_origin_state(pert_index) += 1.0;

        for(int i = 0; i<originStateBlock_vec.size(); i++)
        {

            //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix(); //this fix the all keyframe
            last_KF->fix();

            //we unfix origin velocity stateblock to let it converge
            originStateBlock_vec[2]->unfix();
            
            //we now unfix one StateBlock
            originStateBlock_vec[i]->unfix();

            ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
            std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
        }

        for(int i = 0; i<finalStateBlock_vec.size(); i++)
        {

            //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix(); //this fix the all keyframe
            last_KF->fix();
            //we unfix origin VELOCITY stateblock to let it converge
            originStateBlock_vec[2]->unfix();

            //we now unfix only one StateBlock
            finalStateBlock_vec[i]->unfix();

            ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
            std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
        }
    }

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 2 : BOTH KF FIXED, UNFIX 1 STATEBLOCKS, PERTURBATE 2 ORIGIN VELOCITY ###" << std::endl;

    for(int pert_index0 = 7; pert_index0<10; pert_index0++)
    {
        for(int pert_index1 = 8; pert_index1<10; pert_index1++)
        {
            //perturate initial state
            Eigen::VectorXs perturbated_origin_state(16);
            perturbated_origin_state = initial_origin_state;
            perturbated_origin_state(pert_index0) += 1.0;
            perturbated_origin_state(pert_index1) += 1.0;

            for(int i = 1; i<originStateBlock_vec.size(); i++)
            {

                std::cout << "\t\t\t ______solving______ test number " << i << std::endl;

                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix(); //this fix the all keyframe
                last_KF->fix();
                //we unfix origin VELOCITY stateblock to let it converge
                originStateBlock_vec[2]->unfix();

                //we now unfix only one StateBlock
                originStateBlock_vec[i]->unfix();

                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
                "origin orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n last orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
                "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
            }

            for(int i = 1; i<finalStateBlock_vec.size(); i++)
            {

                std::cout << "\t\t\t ______solving______ test number " << i << std::endl;

                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix(); //this fix the all keyframe
                last_KF->fix();
                //we unfix origin position stateblock to let it converge
                originStateBlock_vec[2]->unfix();

                //we now unfix only one StateBlock
                finalStateBlock_vec[i]->unfix();

                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
            }
        }
    }

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 3 : BOTH KF FIXED, UNFIX 1 STATEBLOCK, PERTURBATE ALL ORIGIN VELOCITIES ###" << std::endl;

    //perturate initial state
    Eigen::VectorXs perturbated_origin_state(16);
    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(7) += 1.0;
    perturbated_origin_state(8) += 2.0;
    perturbated_origin_state(9) += 3.0;

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {
        std::cout << "\t\t\t ______solving______ test number " << i << std::endl;

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we unfix origin VELOCITY stateblock to let it converge
        originStateBlock_vec[2]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {
        //std::cout << "\t\t\t ______solving______ test number " << i << "." << j << "." << k << "." << l << "." << m << std::endl;

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we unfix origin velocity stateblock to let it converge
        originStateBlock_vec[2]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 4 : BOTH KF FIXED, UNFIX 1 STATEBLOCK, PERTURBATE ALL FINAL VELOCITIES ###" << std::endl;

    //perturate initial state
    Eigen::VectorXs perturbated_final_state(16);
    perturbated_final_state = initial_final_state;
    perturbated_final_state(7) += 1.0;
    perturbated_final_state(8) += 2.0;
    perturbated_final_state(9) += 3.0;

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {
        std::cout << "\t\t\t ______solving______ test number " << i << std::endl;

        origin_KF->setState(initial_origin_state);
        last_KF->setState(perturbated_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we unfix origin velocity stateblock to let it converge
        finalStateBlock_vec[2]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
        "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {
        std::cout << "\t\t\t ______solving______ test number " << i << std::endl;

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();
        //we unfix origin velocity stateblock to let it converge
        finalStateBlock_vec[2]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        std::cout << summary.BriefReport() << std::endl;

        /*
         * Origin KF is fixed. Only 1 StateBlock is unfixed in last_KF.
         * Even with the odometry constraint imposing no movement. It is not sure that we can converge exactly to the velocity of origin_KF.
         * otherwise, one of the conditions is likely not to be met.
         */

        EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        EXPECT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
        "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
    }

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 5 : BOTH KF UNFIXED, PERTURBATE ALL ORIGIN VELOCITIES ###" << std::endl;

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_FALSE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
    /*ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));*/

    /* Here we gave an velocity position and final velocity. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somewhere between both keyframe velocities. The Robot couldhave done anything between both KF
     *
     * Robot could have done anything. Velocity changed. So acc bias could also change to anything... But gyroscope bias should still be 0
     *
     */

    wolf_problem_ptr_->print(4,1,1,1);

    std::cout << "\n\t ### TEST 6 : BOTH KF UNFIXED, PERTURBATE ALL FINAL VELOCITIES ###" << std::endl;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->unfix(); //this fix the all keyframe
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_FALSE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
    /*ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));*/

    /* Here we gave an initial velocity and final velocity. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. But we perturbated the final velocities before optimization.
     * There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somewhere between both keyframe velocities.
     *
     */

    wolf_problem_ptr_->print(4,1,1,1);
}


TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_2KF)
{

    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     *
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

     //===================================================== END OF SETTINGS

     // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}

}

//_______________________________________________________________________________________________________________
// END ##################################### static_Optim_IMUOdom_2KF TESTS #####################################
//_______________________________________________________________________________________________________________


TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_nKFs_biasUnfixed)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____________________________________constraintOdom3D___________________________/
     *
     * IMU BIAS UNFIXED
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */
     
    //===================================================== END OF SETTINGS

    // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;

    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*number_of_KF) ){
        
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
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_nKFs_biasFixed)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____________________________________constraintOdom3D___________________________/
     *
     * IMU BIAS UNFIXED
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

    //===================================================== END OF SETTINGS

    // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    //There should be 2 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*number_of_KF) ){
        
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
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

        //Fix all biases StateBlocks
    for(FrameBasePtr it : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList()){
        ( std::static_pointer_cast<FrameIMU>(it) )->getAccBiasPtr()->fix();
        ( std::static_pointer_cast<FrameIMU>(it) )->getGyroBiasPtr()->fix();
    }

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}



TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_SeveralKFs)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____constraintOdom3D___/   \____constraintOdom3D___/   \____constraintOdom3D___/
     *
     * data integration is done for 10s (10 * max_time_span)
     * IMU data are processed at 1 Khz (every ms)
     * Odom3D data are processed at 100 Hz (every 10 ms)
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //===================================================== END OF SETTINGS

    // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    //There should be 2 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data, data_odom3D;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data_odom3D << 0,0,0, 0,0,0;
    Scalar dt = t.get();
    TimeStamp ts(0.000);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);
    unsigned int iter = 0;
    const unsigned int odom_freq = 10; //processing odometry data every 10 ms

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*number_of_KF) ){
        
        // PROCESS IMU DATA
        // Time and data variables
        dt += 0.001;
        iter++;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(iter == odom_freq) //every 100 ms
        {
            // PROCESS ODOM 3D DATA
            mot_ptr->setTimeStamp(ts);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);

            iter = 0;
        }
    }

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    //wolf_problem_ptr_->print(4,1,1,1);
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}


TEST_F(ProcessorIMU_Odom_tests,Motion_IMU_and_Odom)
{
    /* In this test we will process both IMU and Odom3D data at the same time (in a same loop).
     * we use data simulating a perfect IMU doing some motion. This motion could be a pure translation or a pure rotation or anything
     *
     * HOWEVER : The test will use 2 data files which shall meet the following requirements :
     *
     * - a file containing only imu data in IMU frame will be provided. IMU data shall be written in the form [ax, ay, az, wx, wy, wz].
     *      The IMU measurements must include the measurement of the gravity.
     *      First line of this file will contain the initial condition in position, orientation and velocity only. (PQV formulation)
     *      Each line of the file correspond to a new set of IMU data.
     *      Each data will be separated from the previous one with a tabulation (\t).
     *      Finally, the following should give a clear idea of how the file is and summarizes the previous points :
     *          (line1) px_initial\t   py_initial\t   pz_initial\t   qx_initial\t   qy_initial\t   qz_initial\t   qw_initial\t   vx_initial\t   vy_initial\t   vz_initial\t 
     *          (line2) TimesTamp1\t   ax1\t   ay1\t   az1\t   wx1\t   wy1\t   wz1\t   
     *          (line3) TimesTamp2\t   ax2\t   ay2\t   az2\t   wx2\t   wy2\t   wz2\t   
     *          (.           .           .       .       .       .       .       .  )
     *          (lineN) TimesTampN\t   axN\t   ayN\t   azN\t   wxN\t   wyN\t   wzN\t
     *
     * - a file containing only odometry measurements will be provided. This file only contains odometry in the form PO (using orientation vector here !)
     *      Here is how the file should look like :
     *          (line1) TimeStamp1\t    px1\t   py1\t   pz1\t   ox1\t   oy1\t   oz1
     *          (line2) TimeStamp2\t    px2\t   py2\t   pz2\t   ox2\t   oy2\t   oz2
     *          (   .         .           .       .       .       .       .       . )
     *          (lineN) TimeStampN\t    pxN\t   pyN\t   pzN\t   oxN\t   oyN\t   ozN
     *
     *      TIMESTAMPS FROM IMU DATA FILE AND ODOM FILE MUST MATCH PERFECTLY !!!!
     *      This does not mean that for each IMU data you must provide an odometry data !
     *
     *      Integration ends once the end of imu or odom data file is reached.
     *
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    std::cout << "opening" << std::endl;
    imu_data_input.open(filename_motion_imu_data);
    odom_data_input.open(filename_motion_odom);
    std::cout << "pure translation imu file: " << filename_motion_imu_data << std::endl;
    std::cout << "pure translation odom: " << filename_motion_odom << std::endl;

    if(!imu_data_input.is_open() || !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }
    std::cout << "opened" << std::endl;

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
    std::string wolf_root = _WOLF_ROOT_DIR;

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() && !odom_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(ts == t_odom) //every 100 ms
        {
            // PROCESS ODOM 3D DATA
            mot_ptr->setTimeStamp(t_odom);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);

            //prepare next odometry measurement if there is any
            odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
            t_odom.set(input_clock);
        }
    }

    //closing file
    imu_data_input.close();
    odom_data_input.close();

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
    /*  LIST OF TESTS :
     *      - ProcessorOdom3D:
     *          - static_ceresOptimisation_Odom_PO
     *          - static_ceresOptimisation_convergenceOdom_PO
     *          - static_ceresOptimisation_convergenceOdom_POV
     *      - ProcessorIMU:
     *          - static_ceresOptimisation_fixBias
     *      - ProcessorIMU_Odom_tests:
     *          - static_Optim_IMUOdom_2KF
     *          - static_Optim_IMUOdom_nKFs_biasUnfixed
     *          - static_Optim_IMUOdom_nKFs_biasFixed
     *          - static_Optim_IMUOdom_SeveralKFs
     *          - Motion_IMU_and_Odom
     */

     /* Running simply ./gtest_processorMotion_optimization_testCase will run all tests that do not need any input file
      * If you want to run specific test, just modify booleans in 'gtest_processorMotion_optimization_testCase.yaml.yaml' to specify the tests you want to run
      * In this file you can also modify some values that will be used in the tests (example : number of keyframes)
      * to make it simple, we modify global variables given values in this file
      */

     //We use parser to determine from yaml which tests will be executed
     //initialize all variables
     //Default is to only run tests which do not need any file input
     bool static_ceresOptimisation_Odom_PO = true;
     bool static_ceresOptimisation_convergenceOdom_PO = true;
     bool static_ceresOptimisation_convergenceOdom_POV = true;
     bool static_ceresOptimisation_fixBias = true;
     bool static_Optim_IMUOdom_2KF = true;
     bool static_Optim_IMUOdom_nKFs_biasUnfixed = true;
     bool static_Optim_IMUOdom_nKFs_biasFixed = true;
     bool static_Optim_IMUOdom_SeveralKFs = true;
     bool Motion_IMU_and_Odom = false;

     std::string wolf_root = _WOLF_ROOT_DIR;
     std::string gtest_ceres_yaml_path;
     gtest_ceres_yaml_path = wolf_root + "/src/examples/gtest_processorMotion_optimization_testCase.yaml";
     std::string tests_to_run = "";

     //if first argument is --use_yaml then we parse yaml file to configure the execution of tests
     if (argc == 2)
     {
        const char * option = argv[1];
        std:: string option_string(option);
         
        if(option_string == "--use_yaml") // use yaml option detected
        {
            YAML::Node gtest_config = YAML::LoadFile(gtest_ceres_yaml_path);

            static_ceresOptimisation_Odom_PO               = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_Odom_PO"]               .as<bool>();
            static_ceresOptimisation_convergenceOdom_PO    = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_convergenceOdom_PO"]    .as<bool>();
            static_ceresOptimisation_convergenceOdom_POV   = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_convergenceOdom_POV"]   .as<bool>();

            static_ceresOptimisation_fixBias               = gtest_config["ProcessorIMU"]["static_ceresOptimisation_fixBias"]                  .as<bool>();
            static_Optim_IMUOdom_2KF                       = gtest_config["ProcessorIMU_Odom_tests"]["static_Optim_IMUOdom_2KF"]               .as<bool>();
            static_Optim_IMUOdom_nKFs_biasUnfixed          = gtest_config["ProcessorIMU_Odom_tests"]["static_Optim_IMUOdom_nKFs_biasUnfixed"]  .as<bool>();
            static_Optim_IMUOdom_nKFs_biasFixed            = gtest_config["ProcessorIMU_Odom_tests"]["static_Optim_IMUOdom_nKFs_biasFixed"]    .as<bool>();
            static_Optim_IMUOdom_SeveralKFs                = gtest_config["ProcessorIMU_Odom_tests"]["static_Optim_IMUOdom_SeveralKFs"]        .as<bool>();
            Motion_IMU_and_Odom                            = gtest_config["ProcessorIMU_Odom_tests"]["Motion_IMU_and_Odom"]                    .as<bool>();
            number_of_KF                                   = gtest_config["ProcessorIMU_Odom_tests"]["n_KF"]                                   .as<unsigned int>();

            std::string motion_imu_data_string    = gtest_config["ProcessorIMU_Odom_tests"]["motion_IMU_filepath"]                             .as<std::string>();
            std::string motion_odom_string        = gtest_config["ProcessorIMU_Odom_tests"]["motion_Odom_filepath"]                            .as<std::string>();

             //store filename in global variables
            filename_motion_imu_data   = new char[motion_imu_data_string.length() + 1];
            filename_motion_odom       = new char[motion_odom_string.length()     + 1];

            std::strcpy(filename_motion_imu_data, motion_imu_data_string.c_str());
            std::strcpy(filename_motion_odom, motion_odom_string.c_str());
        }
     }

     // use booleans to Filter all tests that will be executed
    if(static_ceresOptimisation_Odom_PO)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_Odom_PO";
    if(static_ceresOptimisation_convergenceOdom_PO)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_convergenceOdom_PO";
    if(static_ceresOptimisation_convergenceOdom_POV)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_convergenceOdom_POV";
    
    if(static_ceresOptimisation_fixBias)
        tests_to_run +=":ProcessorIMU.static_ceresOptimisation_fixBias";
    if(static_Optim_IMUOdom_2KF)
        tests_to_run +=":ProcessorIMU_Odom_tests.static_Optim_IMUOdom_2KF";
    if(static_Optim_IMUOdom_nKFs_biasUnfixed)
        tests_to_run +=":ProcessorIMU_Odom_tests.static_Optim_IMUOdom_nKFs_biasUnfixed";
    if(static_Optim_IMUOdom_nKFs_biasFixed)
        tests_to_run +=":ProcessorIMU_Odom_tests.static_Optim_IMUOdom_nKFs_biasFixed";
    if(static_Optim_IMUOdom_SeveralKFs)
        tests_to_run +=":ProcessorIMU_Odom_tests.static_Optim_IMUOdom_SeveralKFs";
    if(Motion_IMU_and_Odom)
        tests_to_run +=":ProcessorIMU_Odom_tests.Motion_IMU_and_Odom";


  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = tests_to_run;
  ::testing::GTEST_FLAG(filter) = "ProcessorIMU_Odom_tests.static_Optim_IMUOdom_2KF_perturbate_velocity";

  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}