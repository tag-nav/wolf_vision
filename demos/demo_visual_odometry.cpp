//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
//std
#include <iostream>
#include <fstream>

//Wolf
#include <core/ceres_wrapper/solver_ceres.h>
#include <core/solver/factory_solver.h>
#include <core/common/wolf.h>
#include <core/yaml/parser_yaml.h>
#include <core/problem/problem.h>
#include "vision/sensor/sensor_camera.h"
#include "vision/processor/processor_visual_odometry.h"
#include "vision/capture/capture_image.h"
#include "vision/internal/config.h"


using namespace wolf;

std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;

std::string getFilenameWithoutExtension(const std::string& filePath)
{
    size_t lastSlash = filePath.find_last_of('/');
    size_t lastDot = filePath.find_last_of('.');
    if (lastDot == std::string::npos || lastDot < lastSlash)
        return filePath.substr(lastSlash + 1);
    return filePath.substr(lastSlash + 1, lastDot - lastSlash - 1);
}

int main(int argc, char** argv)
{


    WOLF_INFO("======== CONFIGURE PROBLEM =======");

    // Config file to parse. Here is where all the problem is defined:
    std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;
    std::string config_file = "demos/demo_visual_odometry.yaml";

    // parse file into params server: each param will be retrievable from this params server:
    ParserYaml parser       = ParserYaml(config_file, wolf_vision_root);
    ParamsServer server     = ParamsServer(parser.getParams());
    // Wolf problem: automatically build the left branch of the wolf tree from the contents of the params server:
    ProblemPtr problem      = Problem::autoSetup(server);

    assert(problem->getDim() == 3);

    SolverManagerPtr ceres = FactorySolver::create("SolverCeres", problem, server);

    // Print problem to see its status before processing any sensor data
    problem->print(4,0,1,0);

    // recover sensor pointers and other stuff for later use (access by sensor name)
    SensorCameraPtr sensor_cam = std::dynamic_pointer_cast<SensorCamera>(problem->findSensor("sen cam"));
    ProcessorVisualOdometryPtr proc_vo = std::dynamic_pointer_cast<ProcessorVisualOdometry>(problem->findProcessor("proc vo"));

    std::cout << sensor_cam->getCategory() << std::endl;

    //    ==============================================================================

    // std::string euroc_data_folder = "/home/mfourmy/Documents/Phd_LAAS/data/Euroc/MH_01_easy/mav0/cam0/data/";
    std::string seq_name = "V101";
    cv::String euroc_data_path("/media/jlee/T7/data/EuRoC/"+seq_name+"/mav0/cam0/data/*.png");

    std::vector<cv::String> fn;
    cv::glob(euroc_data_path, fn, true); // recurse
    std::sort(fn.begin(), fn.end());

    TimeStamp tic = 0;
    unsigned int number_of_KFs = 0;

    // main loop
    for (size_t k=0; k < fn.size(); ++k)
    {
        cv::Mat img = cv::imread(fn[k], cv::IMREAD_GRAYSCALE);

        // Extract the filename without extension
        std::string ts_raw = getFilenameWithoutExtension(fn[k]);
        TimeStamp t = std::stod(ts_raw) / 1e9;
        
        if (k==0) tic = t;

        CaptureImagePtr image = std::make_shared<CaptureImage>(t, sensor_cam, img);
        sensor_cam->process(image);

        // problem->print(3,0,1,1);

        // solve only when new KFs are added
        if (problem->getTrajectory()->getFrameMap().size() > number_of_KFs)
        {
            number_of_KFs = problem->getTrajectory()->getFrameMap().size();
            std::string report = ceres->solve(wolf::SolverManager::ReportVerbosity::BRIEF);
            // std::cout << report << std::endl;
        }

        // cv::waitKey();

        // if (t.getSeconds() - tic.getSeconds() > 5.0) break;

    }
    // problem->print(3,0,1,1);
    
    // ADDED HERE BEGIN
    std::ofstream f;
    f.open("/home/jlee/workspace/wolf/vision/results/"+seq_name+"_vo.tum");
    for (const auto& e : problem->getTrajectory()->getFrameMap()) {
        
        auto ts = e.first;
        auto kf = e.second;

        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q;
        
        p = kf->getP()->getState();
        q = Eigen::Quaterniond(Eigen::Vector4d(kf->getO()->getState()));
        
        f << std::setprecision(19) << ts.get() << " " << std::setprecision(10) << p(0) << " " << p(1) << " " << p(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    f.close();
    // ADDED HERE END


    return 0;
}

