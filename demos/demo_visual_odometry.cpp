//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023,2024 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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

//Wolf
#include <core/common/wolf.h>
#include "core/yaml/parser_yaml.h"
#include <core/problem/problem.h>
#include "vision/sensor/sensor_camera.h"
#include "vision/processor/processor_visual_odometry.h"
#include "vision/capture/capture_image.h"
#include "vision/internal/config.h"


using namespace wolf;

std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;

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

    // Print problem to see its status before processing any sensor data
    problem->print(4,0,1,0);

    // recover sensor pointers and other stuff for later use (access by sensor name)
    SensorCameraPtr sensor_cam = std::dynamic_pointer_cast<SensorCamera>(problem->findSensor("sen cam"));
    // ProcessorVisualOdometryPtr proc_vo = std::dynamic_pointer_cast<ProcessorVisualOdometry>(problem->getProcessor("proc vo"));

    //    ==============================================================================

    // std::string euroc_data_folder = "/home/mfourmy/Documents/Phd_LAAS/data/Euroc/MH_01_easy/mav0/cam0/data/";
    cv::String euroc_data_path("/home/mfourmy/Documents/Phd_LAAS/data/Euroc/MH_01_easy/mav0/cam0/data/*.png");
    std::vector<cv::String> fn;
    cv::glob(euroc_data_path, fn, true); // recurse

    TimeStamp t = 0;
    // unsigned int number_of_KFs = 0;
    // main loop
    double dt = 0.05;

    std::cout << sensor_cam->getCategory() << std::endl;


    // for (size_t k=0; k < fn.size(); ++k)
    for (size_t k=0; k < 20; ++k)
    {
        cv::Mat img = cv::imread(fn[k], cv::IMREAD_GRAYSCALE);

        //////////////////////////
        // Correct img
        // 
        // ....... 
        //
        //////////////////////////

        CaptureImagePtr image = std::make_shared<CaptureImage>(t, sensor_cam, img);
        sensor_cam->process(image);

        // problem->print(3,0,1,1);


        // // solve only when new KFs are added
        // if (problem->getTrajectory()->getFrameMap().size() > number_of_KFs)
        // {
        //     number_of_KFs = problem->getTrajectory()->getFrameMap().size();
        //     std::string report = solver->solve(wolf::SolverManager::ReportVerbosity::BRIEF);
        //     std::cout << report << std::endl;
        //     if (number_of_KFs > 5)
        //     	break;
        // }

        // cv::waitKey();//1e7);

        t += dt;


    }
    problem->print(3,0,1,1);


    return 0;
}

