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

    std::cout << std::endl << "==================== test_imu_constraintAHP ======================" << std::endl;


    // LOADING DATA FILES (IMU + ODOM)
    // FOR IMU, file content is : Timestampt\t Ax\t Ay\t Az\t Wx\t Wy\t Wz
    // FOR ODOM, file content is : ΔT(current - last), Δpx, Δpy, Δpz, Δox, Δoy, Δoz
    
    
    std::ifstream data_file_imu;
    std::ifstream data_file_odom;
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

            data_file_imu.open(filename_imu);
            data_file_odom.open(filename_odom);

            std::cout << "file imu : " << filename_imu <<"\t file odom : " << filename_odom << std::endl;

            std::string dummy; //this is needed only if first line is headers or useless data
            getline(data_file_imu, dummy);
        }

        if(!data_file_imu.is_open() || !data_file_odom.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            return 1;
        }

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;
        
    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  1,2,2,  0,0,.00,  0,0,.00; //INITIAL CONDITIONS
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    //CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


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

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,9.806, 0,0,0;
    //data_imu << 0.00, 0.000, 9.81, 0.0, 0.0, 0.0;
    data_odom3D << 0,0,0, 0,0,0;
    Scalar input_clock;
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);
    unsigned int iter = 0;
    const unsigned int odom_freq = 50; //odom data generated every 50 ms

    clock_t begin = clock();

    while( (!data_file_imu.eof() && !data_file_odom.eof())){

        iter++;
        // PROCESS IMU DATA
        // Time and data variables
        data_file_imu >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz
        data_imu[2] += 9.806;
        //9.806 added in Az because gravity was not added in the perfect imu simulation
        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(iter == odom_freq)
        {
            // PROCESS ODOM 3D DATA
            data_file_odom >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
            mot_ptr->setTimeStamp(ts);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);
            iter = 0;
        }
    }

    //===================================================== END{PROCESS DATA}

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // Final state
    std::cout << "\nIntegration results ----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Initial    state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << x0.head(16).transpose() << std::endl;
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