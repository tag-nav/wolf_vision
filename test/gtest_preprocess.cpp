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

class ProcessorVisualOdometryTest : public ProcessorVisualOdometry
{
	public:

		ProcessorVisualOdometryTest(ParamsProcessorVisualOdometryPtr& _params_vo):
		    ProcessorVisualOdometry(_params_vo)
	    {

	    }

        cv::Ptr<cv::FeatureDetector> getDetector()
        {
            return detector_;
        }
};

TEST(ProcessorVisualOdometry, kltTrack)
{
    cv::Mat img = cv::imread(wolf_vision_root + "/test/markers.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat img_flow = cv::imread(wolf_vision_root + "/test/markers.jpg", cv::IMREAD_GRAYSCALE);
    // Create an image with a predefined optical flow
    int du = 5;
    int dv = 5;
    for (int x=0; x<img.size().height; x++){
        for (int y=0; y<img.size().width; y++){
            if (x+du < img.size().height && y+dv<img.size().width){
                img_flow.at<uchar>(x,y) = img.at<uchar>(x+du,y+dv);
            }
        }
    }

    // Create a processor
    ParamsProcessorVisualOdometryPtr params = std::make_shared<ParamsProcessorVisualOdometry>();
    params->klt_params_.tracker_height_ = 21;
    params->klt_params_.tracker_width_ = 21;
    params->klt_params_.nlevels_pyramids_ = 3;
    params->klt_params_.klt_max_err_ = 1.0;
    params->klt_params_.crit_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    ProcessorVisualOdometryTest processor(params);
    cv::Ptr<cv::FeatureDetector> detector = processor.getDetector();
    
    std::vector<cv::KeyPoint> kps;
    detector->detect(img, kps);
    cv::Mat img_draw;


    // Create WkpMap 
    KeyPointsMap mwkps, mwkps_flow;
    for (auto kp : kps){
        WKeyPoint wkp(kp);
        mwkps[wkp.getId()] = wkp;
    }
    TracksMap tracks_flow = processor.kltTrack(img, img_flow, mwkps, mwkps_flow);
    Eigen::Vector2d delta(static_cast<float>(du), static_cast<float>(dv));
    for (auto track : tracks_flow){
        float du_flow = mwkps[track.first].getCvKeyPoint().pt.x - mwkps_flow[track.second].getCvKeyPoint().pt.x;
        float dv_flow = mwkps[track.first].getCvKeyPoint().pt.y - mwkps_flow[track.second].getCvKeyPoint().pt.y;
        Eigen::Vector2d delta_flow(du_flow,dv_flow);
        ASSERT_MATRIX_APPROX(delta, delta_flow, 0.1);
    }

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}