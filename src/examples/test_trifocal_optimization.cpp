// Testing creating wolf tree from imported .graph file

//C includes for sleep, time and main args
#include "unistd.h"

//std includes
#include <iostream>

// Vision utils
#include "vision_utils.h"

//Wolf includes
#include "../processors/processor_tracker_feature_trifocal.h"
#include "../capture_image.h"
#include "../sensor_camera.h"
#include "../ceres_wrapper/ceres_manager.h"
#include "../rotations.h"

Eigen::VectorXs get_random_state(const double& _LO, const double& _HI)
{
    double range= _HI-_LO;
    Eigen::VectorXs x = Eigen::VectorXs::Random(7); // Vector filled with random numbers between (-1,1)
    x = (x + Eigen::VectorXs::Constant(7,1.0))*range/2.; // add 1 to the vector to have values between 0 and 2; multiply with range/2
    x = (x + Eigen::VectorXs::Constant(7,_LO)); //set LO as the lower bound (offset)
    x.segment(3,4).normalize(); // Normalize quaternion part
    return x;
}

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    using Eigen::Vector2s;

    std::string wolf_root = _WOLF_ROOT_DIR;

    // ===============================================
    // TEST IMAGES ===================================
    // ===============================================

    // x,y cm displacement, negatives values are directly added in the path string (row-wise).
    Eigen::MatrixXs img_pos = Eigen::MatrixXs::Zero(9,2);
    img_pos.row(0) << 0,0;
    img_pos.row(1) << 1,0;
    img_pos.row(2) << 2,0;
    img_pos.row(3) << 2,1;
    img_pos.row(4) << 2,2;
    img_pos.row(5) << 1,2;
    img_pos.row(6) << 0,2;
    img_pos.row(7) << 0,1;
    img_pos.row(8) << 0,0;

    // read image
    std::cout << std::endl << "-> Reading images from ground-truth movements..." << std::endl;
    std::vector<cv::Mat> images;
    for (unsigned int ii = 0; ii < img_pos.rows(); ++ii)
    {
        std::string img_path = wolf_root + "/src/examples/Test_gazebo_x-" + std::to_string((int)img_pos(ii,0)) + "0cm_y-" + std::to_string((int)img_pos(ii,1)) + "0cm.png";
        std::cout << " |->" << img_path << std::endl;
        images.push_back(cv::imread(img_path, CV_LOAD_IMAGE_GRAYSCALE));   // Read the file
        if(! images.at(ii).data )                              // Check for invalid input
        {
            std::cout <<  " X--Could not open or find the image: " << img_path << std::endl ;
            return -1;
        }
    }
    std::cout << std::endl;

    cv::imshow( "DEBUG VIEW", images.at(0) );  // Show our image inside it.
    cv::waitKey(0);                            // Wait for a keystroke in the window

    // ===============================================
    // CONFIG WOLF ===================================
    // ===============================================

    // Wolf problem
    ProblemPtr problem = Problem::create("PO 3D");

    // CERES WRAPPER
    CeresManagerPtr ceres_manager;
    ceres::Solver::Options ceres_options;
    ceres_options.max_num_iterations = 1000;
    ceres_manager = make_shared<CeresManager>(problem, ceres_options);

    // Install tracker (sensor and processor)
    Eigen::Vector7s cam_ext; cam_ext << 0.0,0.0,0.0, 0.0,0.0,0.0,1.0;
    std::string cam_intr_yaml = wolf_root + "/src/examples/camera_params_vga_ideal.yaml";
    SensorBasePtr sensor = problem->installSensor("CAMERA","camera",cam_ext,cam_intr_yaml);
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sensor);

    std::string proc_trifocal_params_yaml = wolf_root + "/src/examples/processor_tracker_feature_trifocal.yaml";
    ProcessorBasePtr processor = problem->installProcessor("TRACKER FEATURE TRIFOCAL","trifocal","camera", proc_trifocal_params_yaml);
    ProcessorTrackerFeatureTrifocalPtr tracker = std::static_pointer_cast<ProcessorTrackerFeatureTrifocal>(processor);

    // ===============================================
    // KF1 (PRIOR) ===================================
    // ===============================================

    // Set problem PRIOR
    Scalar dt = 0.01;
    TimeStamp   t(0.0);
    Vector7s    x; x <<  img_pos.row(0).transpose(), 0.0,   0.0, 0.0, 0.0, 1.0;
    x.segment(3,4).normalize();
    Matrix6s    P = Matrix6s::Identity() * 0.1;

    // ====== KF1 ======
    FrameBasePtr kf1 = problem->setPrior(x, P, t, dt/2);

    // Process capture
    CaptureImagePtr capture_1 = make_shared<CaptureImage>(t, camera, images.at(0));
    camera->process(capture_1);

    // Verify KFs
    FrameBasePtr kf1_check = capture_1->getFramePtr();
    assert(kf1->id()==kf1_check->id() && "Prior and KF1 are not the same!");

    problem->print(1,1,1,0);

    // ===============================================
    // KF2 ===========================================
    // ===============================================

    t += dt; // increment t
    x << img_pos.row(1).transpose(),0.0,    0.0,0.0,0.0,1.0; // ground truth position
    FrameBasePtr kf2 = problem->emplaceFrame(KEY_FRAME,x,t);
    problem->keyFrameCallback(kf2,nullptr,dt/2.0); // Ack KF creation

    CaptureImagePtr capture_2 = make_shared<CaptureImage>(t, camera, images.at(1));
    std::cout << "Capture 2 TS: " << capture_2->getTimeStamp() << std::endl;
    camera->process(capture_2);

    problem->print(2,1,1,0);

    cv::waitKey(0); // Wait for a keystroke in the window

    // ===============================================
    // KF3 ===========================================
    // ===============================================

    t += dt; // increment t
    x << img_pos.row(2).transpose(),0.0,    0.0,0.0,0.0,1.0; // ground truth position
    FrameBasePtr kf3 = problem->emplaceFrame(KEY_FRAME,x,t);
    problem->keyFrameCallback(kf3,nullptr,dt/2.0); // Ack KF creation

    CaptureImagePtr capture_3 = make_shared<CaptureImage>(t, camera, images.at(2));
    camera->process(capture_3);

    cv::waitKey(0); // Wait for a keystroke in the window

    // ===============================================
    // KF4 ===========================================
    // ===============================================

    t += dt; // increment t
    x << img_pos.row(3).transpose(),0.0,    0.0,0.0,0.0,1.0; // ground truth position
    FrameBasePtr kf4 = problem->emplaceFrame(KEY_FRAME,x,t);
    problem->keyFrameCallback(kf4,nullptr,dt/2.0); // Ack KF creation

    CaptureImagePtr capture_4 = make_shared<CaptureImage>(t, camera, images.at(3));
    camera->process(capture_4);

    cv::waitKey(0); // Wait for a keystroke in the window

    // ===============================================
    // SOLVE PROBLEM (1) =============================
    // ===============================================

    std::cout << "================== SOLVE 1rst TIME ========================" << std::endl;

    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::FULL);
    std::cout << report << std::endl;

    problem->print(1,1,1,0);

    // Print orientation states for all KFs
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
        std::cout << wolf::q2v(Eigen::Quaternions(kf->getOPtr()->getState().data())).transpose()*180.0/3.14159 << std::endl;

    // ===============================================
    // PERTURBATE STATES =============================
    // ===============================================

    // ADD PERTURBATION
    std::cout << "================== ADD PERTURBATION ========================" << std::endl;

    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
    {
        if (kf != kf1)
        {
//            std::cout << std::setprecision(5) << std::endl;
            std::cout << "*****" << std::endl;
            std::cout << kf->getState().transpose()(1) << std::endl;

            Eigen::Vector7s state_perturbed; state_perturbed << Vector3s::Random() * 0.5,    Vector4s::Random().normalized();
            kf->setState(state_perturbed);
            std::cout << wolf::q2v(Eigen::Quaternions(kf->getOPtr()->getState().data())).transpose()*180.0/3.14159 << std::endl;
        }
    }

    problem->print(1,1,1,0);

    // ===============================================
    // SOLVE PROBLEM (2) =============================
    // ===============================================

    // ===== SOLVE SECOND TIME =====
    report = ceres_manager->solve(SolverManager::ReportVerbosity::FULL);

    std::cout << report << std::endl;

    std::cout << "================== AFTER SOLVE 2nd TIME ========================" << std::endl;
    problem->print(2,1,1,0);

    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
        std::cout << wolf::q2v(Eigen::Quaternions(kf->getOPtr()->getState().data())).transpose()*180.0/3.14159 << std::endl;

    return 0;
}
