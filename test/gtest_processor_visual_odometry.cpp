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
#include <core/yaml/parser_yaml.h>
#include <opencv2/imgcodecs.hpp>

#include "core/math/rotations.h"

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

        TrackMatrix getTrackMatrix()
		{
			return track_matrix_;
		}

        void setCaptureLast(CaptureImagePtr capture_image_last){
            last_ptr_ = capture_image_last;
            capture_image_last_ = capture_image_last;
        }

        void setCaptureIncoming(CaptureImagePtr capture_image_incoming){
            incoming_ptr_ = capture_image_incoming;
            capture_image_incoming_ = capture_image_incoming;
        }

        void setCaptureOrigin(CaptureImagePtr capture_image_origin){
            origin_ptr_ = capture_image_origin;
            capture_image_origin_ = capture_image_origin;
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
            } else {
                img_flow.at<uchar>(x,y) = 255;
            }
        }
    }

    // Create a processor
    ParamsProcessorVisualOdometryPtr params = std::make_shared<ParamsProcessorVisualOdometry>();
    params->klt_params_.patch_height_ = 21;
    params->klt_params_.patch_width_ = 21;
    params->klt_params_.nlevels_pyramids_ = 3;
    params->klt_params_.klt_max_err_ = 1.0;
    params->klt_params_.criteria_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    params->fast_params_.threshold_fast_ = 30;
    params->fast_params_.non_max_suppresion_ = true;

    ProcessorVisualOdometryTest processor(params);
    cv::Ptr<cv::FeatureDetector> detector = processor.getDetector();
    
    std::vector<cv::KeyPoint> kps;
    detector->detect(img, kps);


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

//TEST(ProcessorVisualOdometry, preProcess)
//{
//    // Create an image pair
//    cv::Mat img_0 = cv::imread(wolf_vision_root + "/test/demo_gazebo_x00cm_y00cm.jpg", cv::IMREAD_GRAYSCALE);
//    cv::Mat img_1 = cv::imread(wolf_vision_root + "/test/demo_gazebo_x00cm_y-20cm.jpg", cv::IMREAD_GRAYSCALE);
//
//    // Create a processor
//    ParamsProcessorVisualOdometryPtr params = std::make_shared<ParamsProcessorVisualOdometry>();
//    params->klt_params_.patch_height_ = 21;
//    params->klt_params_.patch_width_ = 21;
//    params->klt_params_.nlevels_pyramids_ = 3;
//    params->klt_params_.klt_max_err_ = 0.2;
//    params->klt_params_.criteria_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
//    params->fast_params_.threshold_fast_ = 20;
//    params->fast_params_.non_max_suppresion_ = true;
//    params->min_features_for_keyframe = 50;
//    params->max_nb_tracks_ = 400;
//    params->min_track_length_for_landmark_ = 4;
//    ProcessorVisualOdometryTest processor(params);
//
//
//    // Install camera
//    ParamsSensorCameraPtr intr = std::make_shared<ParamsSensorCamera>();
//    intr->pinhole_model_raw = Eigen::Vector4d(640, 480, 640, 640);
//    intr->width  = 1280;
//    intr->height = 960;
//    SensorCameraPtr cam_ptr = std::make_shared<SensorCamera>((Eigen::Vector7d() << 0,0,0,  0,0,0,1).finished(), intr);
//    processor.configure(cam_ptr);
//
//    // ----------------------------------------
//    // TIME 0 : Let's process the first capture
//    // ----------------------------------------
//
//    TimeStamp t0(0.0);
//
//    // Create Capture
//    CaptureImagePtr capture_image_incoming = std::make_shared<CaptureImage>(t0, cam_ptr, img_0);
//    processor.setCaptureIncoming(capture_image_incoming);
//
//    // PreProcess
//    processor.preProcess();
//
//    // Kps visu
//    cv::Mat img_draw;
//    std::vector<cv::KeyPoint> kps;
//    for (auto mwkp : capture_image_incoming->getKeyPoints()){
//        kps.push_back(mwkp.second.getCvKeyPoint());
//    }
//    // cv::drawKeypoints(img_0, kps, img_draw);
//    // cv::imshow( "KeyPoints t = 0", img_draw);
//    // cv::waitKey(0);
//
//    // ----------------------------------------
//    // TIME 1 : Let's process the other one
//    // ----------------------------------------
//
//    TimeStamp t1(0.1);
//
//    // Edit captures
//    CaptureImagePtr capture_image_last = capture_image_incoming;
//    capture_image_incoming = std::make_shared<CaptureImage>(t1, cam_ptr, img_1);
//    processor.setCaptureIncoming(capture_image_incoming);
//    processor.setCaptureLast(capture_image_last);
//    processor.setCaptureOrigin(capture_image_last);
//
//    processor.preProcess();
//
//    // Kps visu
//    kps.clear();
//    for (auto mwkp : capture_image_incoming->getKeyPoints()){
//        kps.push_back(mwkp.second.getCvKeyPoint());
//    }
//    // cv::drawKeypoints(img_1, kps, img_draw);
//    // cv::imshow( "KeyPoints t = 1", img_draw);
//    // cv::waitKey(0);
//
//    // Check if 80% of tracks between frames
//    float track_ratio = static_cast<float>(capture_image_incoming->getTracksPrev().size()) /
//                        static_cast<float>(capture_image_incoming->getKeyPoints().size());
//    ASSERT_TRUE(track_ratio > 0.8);
//
//    // Check if tracks_prev and tracks_origin are similar
//    ASSERT_EQ(capture_image_incoming->getTracksPrev().size(), capture_image_incoming->getTracksOrigin().size());
//
//}
//
//TEST(ProcessorVisualOdometry, process)
//{
//    // Config file to parse. Here is where all the problem is defined:
//    std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;
//    std::string config_file = "test/gtest_visual_odometry.yaml";
//
//    // parse file into params server: each param will be retrievable from this params server:
//    ParserYaml parser       = ParserYaml(config_file, wolf_vision_root);
//    ParamsServer server     = ParamsServer(parser.getParams());
//    // Wolf problem: automatically build the left branch of the wolf tree from the contents of the params server:
//    ProblemPtr problem      = Problem::autoSetup(server);
//
//    // Get sensor and processor
//    SensorCameraPtr cam_ptr = std::dynamic_pointer_cast<SensorCamera>(problem->getSensor("sen cam"));
//    ProcessorVisualOdometryPtr proc_vo_ptr;
//    for (auto proc : problem->getSensor("sen cam")->getProcessorList()){
//        proc_vo_ptr = std::dynamic_pointer_cast<ProcessorVisualOdometry>(proc);
//    }
//
//    // Successive images
//    cv::Mat img_0 = cv::imread(wolf_vision_root + "/test/demo_gazebo_x00cm_y00cm.jpg", cv::IMREAD_GRAYSCALE);
//    cv::Mat img_1 = cv::imread(wolf_vision_root + "/test/demo_gazebo_x00cm_y-10cm.jpg", cv::IMREAD_GRAYSCALE);
//    cv::Mat img_2 = cv::imread(wolf_vision_root + "/test/demo_gazebo_x00cm_y-20cm.jpg", cv::IMREAD_GRAYSCALE);
//
//
//    // ----------------------------------------
//    // TIME 0 : Let's process the first capture
//    // ----------------------------------------
//
//    TimeStamp t0(0.0);
//    CaptureImagePtr capture_image_0 = std::make_shared<CaptureImage>(t0, cam_ptr, img_0);
//    capture_image_0->process();
//    problem->print(4,0,1,0);
//
//    ASSERT_EQ(proc_vo_ptr->getTrackMatrix().numTracks(), 0);
//	ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 0);
//
//    // ----------------------------------------
//    // TIME 1 : Second image
//    // ----------------------------------------
//
//    TimeStamp t1(0.1);
//    CaptureImagePtr capture_image_1 = std::make_shared<CaptureImage>(t1, cam_ptr, img_1);
//    capture_image_1->process();
//
//    ASSERT_EQ(proc_vo_ptr->getTrackMatrix().numTracks(),capture_image_1->getTracksPrev().size());
//    // Check if tracks_prev and tracks_origin are similar
//    ASSERT_EQ(capture_image_1->getTracksPrev().size(), capture_image_1->getTracksOrigin().size());
//
//    // ----------------------------------------
//    // TIME 3 : Third image
//    // ----------------------------------------
//
//    TimeStamp t2(0.2);
//    CaptureImagePtr capture_image_2 = std::make_shared<CaptureImage>(t2, cam_ptr, img_2);
//    capture_image_2->process();
//
//    ASSERT_EQ(proc_vo_ptr->getTrackMatrix().numTracks(),capture_image_2->getTracksOrigin().size());
//}

TEST(ProcessorVisualOdometry, mergeTracks)
{
    TracksMap tracks_orig_last, tracks_orig_inco, tracks_last_inco;

    /* tested tracks:
     * O - L - I
     * ---------
     * 1 - 2 - x
     * 3 - 4 - 5
     * 6 - 7 - 8
     * 9 - 10 - x
     * x - 11 - 12
     * x - 13 - x
     * x - x - 14
     * 15 - x - 16
     */
    tracks_orig_last[1] = 2;
    tracks_orig_last[3] = 4;
    tracks_orig_last[6] = 7;
    tracks_orig_last[9] = 10;

    tracks_last_inco[4] = 5;
    tracks_last_inco[7] = 8;
    tracks_last_inco[11] = 12;

    tracks_orig_inco = ProcessorVisualOdometry::mergeTracks(tracks_orig_last, tracks_last_inco);

    // correct tracks
    ASSERT_EQ(tracks_orig_inco.size(), 2);
    ASSERT_EQ(tracks_orig_inco[3], 5);
    ASSERT_EQ(tracks_orig_inco[6], 8);

    // tracks that should not be there
    ASSERT_EQ(tracks_orig_inco.count(1), 0);
    ASSERT_EQ(tracks_orig_inco.count(9), 0);
    ASSERT_EQ(tracks_orig_inco.count(15), 0);
    ASSERT_EQ(tracks_orig_inco.count(11), 0);
    ASSERT_EQ(tracks_orig_inco.count(13), 0);

}

cv::Point2f project(Eigen::Matrix3f K, Eigen::Vector3f p){
        Eigen::Vector3f ph = K * p;
        ph = ph / ph(2,0);
        return cv::Point2f(ph(0), ph(1));
}

TEST(ProcessorVisualOdometry, filterWithEssential)
{
    KeyPointsMap mwkps_prev, mwkps_curr;
    TracksMap tracks_prev_curr;
    cv::Mat E;

    // Create a simple camera model
    Eigen::Matrix3f K;
    K << 100, 0, 240,
         0, 100, 240,
         0, 0, 1;
    
    // Create a processor
    ParamsProcessorVisualOdometryPtr params = std::make_shared<ParamsProcessorVisualOdometry>();
    params->grid_params_.margin_ = 10;
    params->grid_params_.nbr_cells_h_ = 8;
    params->grid_params_.nbr_cells_v_ = 8;
    params->grid_params_.separation_ = 10;
    params->ransac_params_.ransac_prob_ = 0.999;  // 0.99 makes the gtest fail -> missing one point
    params->ransac_params_.ransac_thresh_ = 1.0;
    ProcessorVisualOdometryTest processor(params);

    // Install camera
    ParamsSensorCameraPtr intr = std::make_shared<ParamsSensorCamera>();
    intr->noise_std = Vector2d(1,1);
    intr->pinhole_model_raw = Eigen::Vector4d(100, 100, 240, 240);
    intr->width  = 480;
    intr->height = 480;
    SensorCameraPtr cam_ptr = std::make_shared<SensorCamera>((Eigen::Vector7d() << 0,0,0,  0,0,0,1).finished(), intr);
    processor.configure(cam_ptr);

    // Set 3D points on the previous view
    Eigen::Vector3f X1_prev(1.0, 1.0, 2.0);
    Eigen::Vector3f X2_prev(-1.0, -1.0, 2.0);
    Eigen::Vector3f X3_prev(-0.75, -0.75, 3.0);
    Eigen::Vector3f X4_prev(-0.75, 0.75, 2.5);
    Eigen::Vector3f X5_prev(0.75, -0.75, 2.0);
    Eigen::Vector3f X6_prev(0.0, 1.0, 2.0);
    Eigen::Vector3f X7_prev(0.1, -1.0, 3.0);
    Eigen::Vector3f X8_prev(0.75, 0.75, 2.0);

    // Project pixels in the previous view
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(0, WKeyPoint(cv::KeyPoint(project(K,X1_prev), 1))));
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(1, WKeyPoint(cv::KeyPoint(project(K,X2_prev), 1))));
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(2, WKeyPoint(cv::KeyPoint(project(K,X3_prev), 1))));
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(3, WKeyPoint(cv::KeyPoint(project(K,X4_prev), 1))));
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(4, WKeyPoint(cv::KeyPoint(project(K,X5_prev), 1))));
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(5, WKeyPoint(cv::KeyPoint(project(K,X6_prev), 1))));
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(6, WKeyPoint(cv::KeyPoint(project(K,X7_prev), 1))));
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(7, WKeyPoint(cv::KeyPoint(project(K,X8_prev), 1))));

    // Set the transformation between the two views
    Eigen::Vector3f t(0.1, 0.1, 0.0);
    // Eigen::Vector3f euler(0.0, 0.0, 0.0);  // degenerate case!
    Eigen::Vector3f euler(0.2, 0.1, 0.3);
    Eigen::Matrix3f R = e2R(euler);
    

    // Project pixels in the current view
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(0, WKeyPoint(cv::KeyPoint(project(K,R*X1_prev + t), 1))));
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(1, WKeyPoint(cv::KeyPoint(project(K,R*X2_prev + t), 1))));
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(2, WKeyPoint(cv::KeyPoint(project(K,R*X3_prev + t), 1))));
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(3, WKeyPoint(cv::KeyPoint(project(K,R*X4_prev + t), 1))));
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(4, WKeyPoint(cv::KeyPoint(project(K,R*X5_prev + t), 1))));
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(5, WKeyPoint(cv::KeyPoint(project(K,R*X6_prev + t), 1))));
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(6, WKeyPoint(cv::KeyPoint(project(K,R*X7_prev + t), 1))));
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(7, WKeyPoint(cv::KeyPoint(project(K,R*X8_prev + t), 1))));  

    // Build the tracksMap
    for (size_t i=0; i<mwkps_curr.size(); ++i)
    {
        tracks_prev_curr.insert(std::pair<size_t, size_t>(i,i));
    }

    // Let's try FilterWithEssential, all points here are inliers
    processor.filterWithEssential(mwkps_prev, mwkps_curr, tracks_prev_curr, E);
    ASSERT_EQ(tracks_prev_curr.size(), mwkps_curr.size());

    // We had here an outlier: a keyPoint that doesn't move
    mwkps_prev.insert(std::pair<size_t, WKeyPoint>(8, WKeyPoint(cv::KeyPoint(cv::Point2f(120,140), 1))));
    mwkps_curr.insert(std::pair<size_t, WKeyPoint>(8, WKeyPoint(cv::KeyPoint(cv::Point2f(120,140), 1))));
    tracks_prev_curr.insert(std::pair<size_t, size_t>(8,8)); 

    // point at 8 must be discarded
    processor.filterWithEssential(mwkps_prev, mwkps_curr, tracks_prev_curr, E);
    ASSERT_EQ(tracks_prev_curr.count(8), 0);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
