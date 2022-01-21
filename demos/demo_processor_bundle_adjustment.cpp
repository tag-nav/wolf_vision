//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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

#include "vision/processor/processor_bundle_adjustment.h"

//Wolf
#include <core/common/wolf.h>
#include <core/problem/problem.h>
#include <core/state_block/state_block.h>
#include "vision/sensor/sensor_camera.h"
#include "vision/capture/capture_image.h"
#include <core/ceres_wrapper/solver_ceres.h>
#include "vision/landmark/landmark_hp.h"
#include "vision/internal/config.h"


// Vision utils includes
#include "vision_utils/vision_utils.h"
#include "vision_utils/sensors.h"
#include "vision_utils/common_class/buffer.h"
#include "vision_utils/common_class/frame.h"

////Mvbluefox includes
//#include <iri/mvbluefox3/mvbluefox3.h>
//#include <unistd.h>
//#include <limits.h>
//#include <sstream>
//
//typedef bool( *SUPPORTED_DEVICE_CHECK )( const mvIMPACT::acquire::Device* const );
//
//// mvIMPACT::acquire::DeviceManager devMgr_;            // Device Manager.
//
//
//std::string GetSerialFromUser(void)
//{
//  mvIMPACT::acquire::DeviceManager devMgr; // Device Manager.
//  return getDeviceFromUserInput(devMgr)->serial.read();
//}
//
//std::string AvailableDevices(SUPPORTED_DEVICE_CHECK pSupportedDeviceCheckFn)
//{
//  mvIMPACT::acquire::DeviceManager devMgr; // Device Manager.
//  std::ostringstream devices;
//  devices << "Available device(s): ";
//  const unsigned int devCnt = devMgr.deviceCount();
//  if (devCnt == 0)
//  {
//    devices << "0.";
//  }
//  else
//  {
//    for( unsigned int i = 0; i < devCnt; i++ )
//    {
//      Device* pDev = devMgr[i];
//      if( pDev )
//      {
//        if( !pSupportedDeviceCheckFn || pSupportedDeviceCheckFn( pDev ) )
//        {
//          devices << " \n  - " << pDev->product.read() << ": " << pDev->serial.read();
//        }
//      }
//    }
//  }
//  return devices.str();
//}
//
//std::string GetPath()
//{
//  char result[ PATH_MAX ];
//  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
//  return std::string( result, (count > 0) ? count : 0 );
//}

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector6d;
using Eigen::Vector7d;

using namespace wolf;

std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;

int main(int argc, char** argv)
{
    std::cout << std::endl << "==================== processor bundle adjustment test ======================" << std::endl;

    vision_utils::SensorCameraPtr sen_ptr = vision_utils::askUserSource(argc, argv);
    if (sen_ptr==NULL)
        return 0;

    unsigned int buffer_size = 10;
    vision_utils::Buffer<vision_utils::FramePtr> frame_buff(buffer_size);
    frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), 0) );

    unsigned int img_width  = frame_buff.back()->getImage().cols;
    unsigned int img_height = frame_buff.back()->getImage().rows;
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

//    ===========================================================================

    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);
    SolverCeresPtr solver = std::make_shared<SolverCeres>(problem);
    solver->getSolverOptions().max_num_iterations = 100;
    solver->getSolverOptions().function_tolerance = 1e-4;


    // Install camera
//    ParamsSensorCameraPtr intr = std::make_shared<ParamsSensorCamera>(); // TODO init params or read from YAML
//    intr->pinhole_model_raw = Eigen::Vector4d(320,240,320,320);
//    intr->width  = img_width;
//    intr->height = img_height;
//    auto sens_cam = problem->installSensor("CAMERA", "camera", (Eigen::Vector7d() << 0,0,0,  0,0,0,1).finished(), intr);
    auto sens_cam = problem->installSensor("CAMERA", "camera", (Eigen::Vector7d() << 0,0,0,  0,0,0,1).finished(), wolf_vision_root + "/demos/calib_logitech_webcam_params.yaml");
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sens_cam);
    camera->setImgWidth(img_width);
    camera->setImgHeight(img_height);

    // Install processor
    ParamsProcessorBundleAdjustmentPtr params = std::make_shared<ParamsProcessorBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_vision_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 1;
    params->voting_active = true;
    params->max_new_features = 200;
    params->min_features_for_keyframe = 50;
    params->time_tolerance = 0.01;
    params->n_cells_h = 20;
    params-> n_cells_v = 17;
    params->min_response_new_feature = 10;
    auto proc = problem->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);
    ProcessorBundleAdjustmentPtr proc_bundle_adj = std::static_pointer_cast<ProcessorBundleAdjustment>(proc);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;


//    ==============================================================================

    TimeStamp t = 0;
    unsigned int number_of_KFs = 0;
    // main loop
    double dt = 0.04;
    for(int frame_count = 0; frame_count<10000; ++frame_count)
    {
        t += dt;

        // Image ---------------------------------------------
        frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), frame_count) );
        CaptureImagePtr image = std::make_shared<CaptureImage>(t, camera, frame_buff.back()->getImage());
        /* process */
        camera->process(image);

        // solve only when new KFs are added
        if (problem->getTrajectory()->getFrameMap().size() > number_of_KFs)
        {
            number_of_KFs = problem->getTrajectory()->getFrameMap().size();
            std::string report = solver->solve(wolf::SolverManager::ReportVerbosity::BRIEF);
            std::cout << report << std::endl;
            if (number_of_KFs > 5)
            	break;
        }
//        problem->print(4,1,1,0);

        cv::waitKey();//1e7);

    }

//    problem->print(1,0,1,0);

    return 0;
}

