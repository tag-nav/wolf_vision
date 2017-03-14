//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "sensor_odom_3D.h"
#include "capture_imu.h"
#include "constraint_odom_3D.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"
#include "processor_odom_3D.h"
#include "ceres_wrapper/ceres_manager.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

#define DEBUG_RESULTS

int _kbhit();

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    // LOADING DATA FILES (IMU + ODOM)
    // FOR IMU, file content is : Timestampt\t Ax\t Ay\t Az\t Wx\t Wy\t Wz
    // FOR ODOM, file content is : Timestampt\t Δpx\t  Δpy\t  Δpz\t  Δox\t  Δoy\t  Δoz
    
    
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;
    const char * filename_imu;
    const char * filename_odom;
    if (argc < 3)
        {
            std::cout << "Missing input argument! : needs 2 argument (path to imu and odom data files)." << std::endl;
            return 1;
        }
        else
        {
            filename_imu = argv[1];
            filename_odom = argv[2];

            imu_data_input.open(filename_imu);
            odom_data_input.open(filename_odom);

            std::cout << "file imu : " << filename_imu <<"\t file odom : " << filename_odom << std::endl;

            //std::string dummy; //this is needed only if first line is headers or useless data
            //getline(imu_data_input, dummy);
        }

        if(!imu_data_input.is_open() || !odom_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            return 1;
        }

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;
        
    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x_origin(16);
    Eigen::Vector6s origin_bias;
    x_origin << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00; //INITIAL CONDITIONS
    TimeStamp t(0);

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
    imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


    // SENSOR + PROCESSOR IMU
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 10;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;
    
    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
    prc_odom3D_params->max_time_span = 0.999;
    prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_odom3D_params->dist_traveled = 1000000000;
    prc_odom3D_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // reset origin of problem
    t.set(0);
    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state;
    
    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    odom_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                            expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];

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

    clock_t begin = clock();

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

        if(ts.get() == t_odom.get()) //every 100 ms
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

    clock_t end = clock();
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}

    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // Final state
    std::cout << "\nIntegration results ----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Initial    state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << x_origin.head(16).transpose() << std::endl;
    std::cout << "Integrated delta: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_.transpose() << std::endl;
    std::cout << "Integrated state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState().head(16).transpose() << std::endl;
    std::cout << "Integrated std  : " << std::fixed << std::setprecision(3) << std::setw(8)
    << (wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_cov_.diagonal().transpose()).array().sqrt() << std::endl;


    // Print statistics
    std::cout << "\nStatistics -----------------------------------------------------------------------------------" << std::endl;
    std::cout << "If you want meaningful CPU metrics, remove all couts in the loop / remove DEBUG_RESULTS definition variable, and compile in RELEASE mode!" << std::endl;

    TimeStamp t0, tf;
    t0 = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().front().ts_;
    tf = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
    int N = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().size();
    std::cout << "t0        : " << t0.get() << " s" << std::endl;
    std::cout << "tf        : " << tf.get() << " s" << std::endl;
    std::cout << "duration  : " << tf-t0 << " s" << std::endl;
    std::cout << "N samples : " << N << std::endl;
    std::cout << "frequency : " << (N-1)/(tf-t0) << " Hz" << std::endl;
    std::cout << "CPU time  : " << elapsed_secs << " s" << std::endl;
    std::cout << "s/integr  : " << elapsed_secs/(N-1)*1e6 << " us" << std::endl;
    std::cout << "integr/s  : " << (N-1)/elapsed_secs << " ips" << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);
    
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    return 0;

}

int _kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}