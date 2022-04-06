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
/**
 * \file gtest_preprocess.cpp
 *
 *  Created on: March 31, 2022
 *      \author: cdebeunne
 */

#include <string>
#include <core/utils/utils_gtest.h>
#include <core/sensor/sensor_base.h>
#include <opencv2/imgcodecs.hpp>

#include "vision/processor/processor_visual_odometry.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/factor/factor_pixel_hp.h"
#include "vision/landmark/landmark_hp.h"
#include "vision/capture/capture_image.h"
#include "vision/internal/config.h"

using namespace wolf;
using namespace cv;

std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;

TEST(ProcessorVisualOdometry, kltTrack)
{
    cv::Mat img = cv::imread(wolf_vision_root + "/test/demo_ORB.png", cv::IMREAD_GRAYSCALE);

    // Create an image with a predefined optical flow
    cv::Mat img_flow = img; 
    int du = 5;
    int dv = 5;
    for (int x=0; x<img.size().height; x++){
        for (int y=0; y<img.size().width; y++){
            if (x+du < img.size().height && y+dv<img.size().width){
                img_flow.at<uchar>(x,y) = img.at<uchar>(x+du,y+dv);
            }
        }
    }

    
    ASSERT_TRUE(du=dv);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}