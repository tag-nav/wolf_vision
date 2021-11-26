//--------LICENSE_START--------
//
// Copyright (C) 2020,2021 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
// Testing creating wolf tree from imported .graph file

//C includes for sleep, time and main args
#include "unistd.h"

//std includes
#include <iostream>

// Vision utils
#include <vision_utils.h>

//Wolf includes
#include "../processors/processor_tracker_feature_trifocal.h"
#include "../capture_image.h"
#include "../sensor_camera.h"
#include "../ceres_wrapper/solver_ceres.h"
#include "../rotations.h"
#include "../capture_pose.h"
#include "../capture_void.h"
#include "../constraints/constraint_autodiff_distance_3D.h"

Eigen::VectorXd get_random_state(const double& _LO, const double& _HI)
{
    double range= _HI-_LO;
    Eigen::VectorXd x = Eigen::VectorXd::Random(7); // Vector filled with random numbers between (-1,1)
    x = (x + Eigen::VectorXd::Constant(7,1.0))*range/2.; // add 1 to the vector to have values between 0 and 2; multiply with range/2
    x = (x + Eigen::VectorXd::Constant(7,_LO)); //set LO as the lower bound (offset)
    x.segment(3,4).normalize(); // Normalize quaternion part
    return x;
}

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    using Eigen::Vector2d;

    std::string wolf_root = _WOLF_ROOT_DIR;

    // ===============================================
    // TEST IMAGES ===================================
    // ===============================================

    // x,y displacement, negatives values are directly added in the path string (row-wise).
    Eigen::MatrixXd img_pos = Eigen::MatrixXd::Zero(10,2);
    img_pos.row(0) <<  0, 0;
    img_pos.row(1) << -1, 0;
    img_pos.row(2) << -2, 0;
    img_pos.row(3) << -2,-1;
    img_pos.row(4) << -2,-2;
    img_pos.row(5) << -1,-2;
    img_pos.row(6) <<  0,-2;
    img_pos.row(7) <<  0,-1;
    img_pos.row(8) <<  0, 0;
    img_pos.row(9) << -1, 0;

    // read image
    std::cout << std::endl << "-> Reading images from ground-truth movements..." << std::endl;
    std::vector<cv::Mat> images;
    for (unsigned int ii = 0; ii < img_pos.rows(); ++ii)
    {
        std::string img_path = wolf_root + "/src/examples/Test_gazebo_x" + std::to_string((int)img_pos(ii,0)) + "0cm_y" + std::to_string((int)img_pos(ii,1)) + "0cm.jpg";
        std::cout << " |->" << img_path << std::endl;
        images.push_back(cv::imread(img_path, CV_LOAD_IMAGE_GRAYSCALE));   // Read the file
        if(! images.at(ii).data )                              // Check for invalid input
        {
            std::cout <<  " X--Could not open or find the image: " << img_path << std::endl ;
            return -1;
        }
    }
    std::cout << std::endl;


    // Scale ground truth priors
    img_pos *= 0.10;

    cv::imshow( "DEBUG VIEW", images.at(0) );  // Show our image inside it.
    cv::waitKey(1);                            // Wait for a keystroke in the window

    // ===============================================
    // CONFIG WOLF ===================================
    // ===============================================

    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);

    // CERES WRAPPER
    SolverCeresPtr solver = make_shared<SolverCeres>(problem, ceres_options);
    solver->getSolverOptions().max_num_iterations = 50;
    solver->getSolverOptions().function_tolerance = 1e-6;

    // Install tracker (sensor and processor)
    Eigen::Vector7d cam_ext; cam_ext << 0.0,0.0,0.0, 0.0,0.0,0.0,1.0;
    std::string cam_intr_yaml = wolf_root + "/src/examples/camera_params_1280x960_ideal.yaml";
    SensorBasePtr sensor = problem->installSensor("CAMERA","camera",cam_ext,cam_intr_yaml);
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sensor);

    std::string proc_trifocal_params_yaml = wolf_root + "/src/examples/processor_tracker_feature_trifocal.yaml";
    ProcessorBasePtr processor = problem->installProcessor("TRACKER FEATURE TRIFOCAL","trifocal","camera", proc_trifocal_params_yaml);
    ProcessorTrackerFeatureTrifocalPtr tracker = std::static_pointer_cast<ProcessorTrackerFeatureTrifocal>(processor);

    // ===============================================
    // KF1 (PRIOR) ===================================
    // ===============================================

    // Set problem PRIOR
    double dt = 0.01;
    TimeStamp   t(0.0);
    Vector7d    x; x <<  img_pos.row(0).transpose(), 0.0,   0.0, 0.0, 0.0, 1.0;
    x.segment(3,4).normalize();
    Matrix6d    P = Matrix6d::Identity() * 0.000001; // 1mm

    // ====== KF1 ======
    FrameBasePtr kf1 = problem->setPrior(x, P, t, dt/2);

    // Process capture
    CaptureImagePtr capture_1 = make_shared<CaptureImage>(t, camera, images.at(0));
    camera->process(capture_1);

    // Verify KFs
    FrameBasePtr kf1_check = capture_1->getFramePtr();
    assert(kf1->id()==kf1_check->id() && "Prior and KF1 are not the same!");

//    problem->print(2,0,1,0);


    // ===============================================
    // Other KFs =====================================
    // ===============================================
    int kf_total_num = img_pos.rows();
    std::vector<FrameBasePtr> kfs;
    kfs.push_back(kf1);
    std::vector<CaptureImagePtr> captures;
    captures.push_back(capture_1);

    for (int kf_id = 2; kf_id <= kf_total_num; ++kf_id)
    {
        t += dt; // increment t

        if ( (kf_id % 2 == 1) )
        {
            x << img_pos.row(kf_id-1).transpose(),0.0,    0.0,0.0,0.0,1.0; // ground truth position
            FrameBasePtr kf = problem->emplaceFrame(KEY_FRAME,x,t);
            std::cout << "KeyFrm " << kf->id() << " TS: " << kf->getTimeStamp() << std::endl;
            kfs.push_back(kf);
            problem->keyFrameCallback(kf,nullptr,dt/2.0); // Ack KF creation
        }

        CaptureImagePtr capture = make_shared<CaptureImage>(t, camera, images.at(kf_id-1));
        captures.push_back(capture);
        std::cout << "Capture " << kf_id << " TS: " << capture->getTimeStamp() << std::endl;
        camera->process(capture);

        cv::waitKey(1); // Wait for a keystroke in the window
    }

    // ==================================================
    // Establish Scale Factor (to see results scaled)
    // ==================================================

    std::cout << "================== ADD Scale constraint ========================" << std::endl;

    // Distance constraint
    Vector1d distance(0.2);      // 2x10cm distance -- this fixes the scale
    Matrix1d dist_cov(0.000001); // 1mm error

    CaptureBasePtr
    cap_dist  = problem->closestKeyFrameToTimeStamp(TimeStamp(2*dt))->addCapture(make_shared<CaptureVoid>(t,
                                                                                                          sensor));
    FeatureBasePtr
    ftr_dist = cap_dist->addFeature(make_shared<FeatureBase>("DISTANCE",
                                                              distance,
                                                              dist_cov));
    FactorBasePtr
    ctr_dist  = ftr_dist->addFactor(make_shared<FactorAutodiffDistance3D>(ftr_dist,
                                                                                  kfs.at(0),
                                                                                  nullptr));
    kfs.at(0)->addConstrainedBy(ctr_dist);

    problem->print(1,1,1,0);

    // ===============================================
    // SOLVE PROBLEM (1) =============================
    // ===============================================

    std::cout << "================== SOLVE 1rst TIME ========================" << std::endl;

    std::string report = solver->solve(SolverManager::ReportVerbosity::FULL);
    std::cout << report << std::endl;

    problem->print(1,1,1,0);

    // Print orientation states for all KFs
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
        std::cout << "KF" << kf->id() << " Euler deg " << wolf::q2e(kf->getOPtr()->getState()).transpose()*180.0/3.14159 << std::endl;


    // ===============================================
    // COVARIANCES ===================================
    // ===============================================
    // GET COVARIANCES of all states
    WOLF_TRACE("======== COVARIANCES OF SOLVED PROBLEM =======")
    solver->computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
        if (kf->isKey())
        {
            Eigen::MatrixXd cov = kf->getCovariance();
            WOLF_TRACE("KF", kf->id(), "_std (sigmas) = ", cov.diagonal().transpose().array().sqrt());
        }
    std::cout << std::endl;

    // ===============================================
    // PERTURBATE STATES =============================
    // ===============================================

    // ADD PERTURBATION
    std::cout << "================== ADD PERTURBATION ========================" << std::endl;

    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
    {
        if (kf != kf1)
        {
            Eigen::Vector7d perturbation; perturbation << Vector7d::Random() * 0.05;
            Eigen::Vector7d state_perturbed = kf->getState() + perturbation;
            state_perturbed.segment(3,4).normalize();
            kf->setState(state_perturbed);
            std::cout << "KF" << kf->id() << " Euler deg " << wolf::q2e(kf->getOPtr()->getState()).transpose()*180.0/3.14159 << std::endl;
        }
    }

    problem->print(1,1,1,0);

    // ===============================================
    // SOLVE PROBLEM (2) =============================
    // ===============================================

    // ===== SOLVE SECOND TIME =====
    report = solver->solve(SolverManager::ReportVerbosity::FULL);

    std::cout << report << std::endl;

    std::cout << "================== AFTER SOLVE 2nd TIME ========================" << std::endl;
    problem->print(1,1,1,0);

    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
        std::cout << "KF" << kf->id() << " Euler deg " << wolf::q2e(kf->getOPtr()->getState()).transpose()*180.0/3.14159 << std::endl;

    cv::waitKey(0); // Wait for a keystroke in the window

    return 0;
}
