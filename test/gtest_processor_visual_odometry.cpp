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

#include <Eigen/Dense>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>

#include <core/utils/utils_gtest.h>
#include <core/sensor/sensor_base.h>
#include <core/yaml/parser_yaml.h>
#include <core/math/rotations.h>
#include <core/processor/track_matrix.h>

#include "vision/processor/processor_visual_odometry.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/factor/factor_pixel_hp.h"
#include "vision/landmark/landmark_hp.h"
#include "vision/capture/capture_image.h"
#include "vision/internal/config.h"


using namespace wolf;
using namespace cv;

std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;

WOLF_PTR_TYPEDEFS(ProcessorVisualOdometry_Wrapper);
class ProcessorVisualOdometry_Wrapper : public ProcessorVisualOdometry
{
	public:

		ProcessorVisualOdometry_Wrapper(ParamsProcessorVisualOdometryPtr& _params_vo):
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

// namespace wolf{
// // Register in the Factories
// WOLF_REGISTER_PROCESSOR(ProcessorVisualOdometry_Wrapper);
// }
// ////////////////////////////////////////////////////////////////

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
    params->klt.patch_height = 21;
    params->klt.patch_width = 21;
    params->klt.nlevels_pyramids = 3;
    params->klt.max_err = 1.0;
    params->klt.criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    params->fast.threshold = 30;
    params->fast.non_max_suppresion = true;

    ProcessorVisualOdometry_Wrapper processor(params);
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
//    params->klt_params.patch_height = 21;
//    params->klt_params.patch_width = 21;
//    params->klt_params.nlevels_pyramids = 3;
//    params->klt_params.klt_max_err = 0.2;
//    params->klt_params.criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
//    params->fast_params.threshold_fast = 20;
//    params->fast_params.non_max_suppresion = true;
//    params->min_features_for_keyframe = 50;
//    params->max_nb_tracks = 400;
//    params->min_track_length_for_landmark = 4;
//    ProcessorVisualOdometry_Wrapper processor(params);
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
//    // Check if tracks_c1 and tracks_origin are similar
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
//    SensorCameraPtr cam_ptr = std::dynamic_pointer_cast<SensorCamera>(problem->findSensor("sen cam"));
//    ProcessorVisualOdometryPtr proc_vo_ptr;
//    for (auto proc : problem->findSensor("sen cam")->getProcessorList()){
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
//    // Check if tracks_c1 and tracks_origin are similar
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





////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
// Now we deal with multiview geometry so it helps to
// create a map of landmarks to project them
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class ProcessorVOMultiView_class : public testing::Test{

    public:
        KeyPointsMap mwkps_c1, mwkps_c2;
        TracksMap tracks_c1_c2;
        ProcessorVisualOdometry_WrapperPtr processor;
        Eigen::Vector3d t_21;
        Eigen::Matrix3d R_21;
        Eigen::Vector4d k;
        cv::Mat Kcv;
        void SetUp() override
        {
            k << 376, 240, 460, 460;
            Kcv = (cv::Mat_<double>(3,3) << k(2), 0,    k(0),
                                            0,    k(3), k(1),
                                            0,    0,    1);
            // Create a processor
            ParamsProcessorVisualOdometryPtr params = std::make_shared<ParamsProcessorVisualOdometry>();
            params->grid.margin = 10;
            params->grid.nbr_cells_h = 8;
            params->grid.nbr_cells_v = 8;
            params->grid.separation = 10;
            params->ransac.prob = 0.999;  // 0.99 makes the gtest fail -> missing one point
            params->ransac.thresh = 1.0;
            processor = std::make_shared<ProcessorVisualOdometry_Wrapper>(params);

            // Install camera
            ParamsSensorCameraPtr intr = std::make_shared<ParamsSensorCamera>();
            intr->pinhole_model_raw = k;  // Necessary so that the internal Kcv camera matrix is configured properly
            intr->distortion = Eigen::Vector4d::Zero();
            intr->width  = 752;
            intr->height = 480;
            SensorCameraPtr cam_ptr = std::make_shared<SensorCamera>((Eigen::Vector7d() << 0,0,0,  0,0,0,1).finished(), intr);
            processor->configure(cam_ptr);

            // Set 3D points on the previous camera frame -> all in front of the camera is better
            Eigen::Vector3d X1_c1(1.0, 1.0, 2.0);
            Eigen::Vector3d X2_c1(-1.0, -1.0, 2.0);
            Eigen::Vector3d X3_c1(-0.75, -0.75, 3.0);
            Eigen::Vector3d X4_c1(-0.75, 0.75, 2.5);
            Eigen::Vector3d X5_c1(0.75, -0.75, 2.0);
            Eigen::Vector3d X6_c1(0.0, 1.0, 2.0);
            Eigen::Vector3d X7_c1(0.1, -1.0, 3.0);
            Eigen::Vector3d X8_c1(0.75, 0.75, 2.0);

            // Project pixels in the previous view
            mwkps_c1.insert(std::pair<size_t, WKeyPoint>(0, WKeyPoint(pinhole::projectPoint(k, intr->distortion, X1_c1))));
            mwkps_c1.insert(std::pair<size_t, WKeyPoint>(1, WKeyPoint(pinhole::projectPoint(k, intr->distortion, X2_c1))));
            mwkps_c1.insert(std::pair<size_t, WKeyPoint>(2, WKeyPoint(pinhole::projectPoint(k, intr->distortion, X3_c1))));
            mwkps_c1.insert(std::pair<size_t, WKeyPoint>(3, WKeyPoint(pinhole::projectPoint(k, intr->distortion, X4_c1))));
            mwkps_c1.insert(std::pair<size_t, WKeyPoint>(4, WKeyPoint(pinhole::projectPoint(k, intr->distortion, X5_c1))));
            mwkps_c1.insert(std::pair<size_t, WKeyPoint>(5, WKeyPoint(pinhole::projectPoint(k, intr->distortion, X6_c1))));
            mwkps_c1.insert(std::pair<size_t, WKeyPoint>(6, WKeyPoint(pinhole::projectPoint(k, intr->distortion, X7_c1))));
            mwkps_c1.insert(std::pair<size_t, WKeyPoint>(7, WKeyPoint(pinhole::projectPoint(k, intr->distortion, X8_c1))));

            // Set the transformation between views 1 and 2 
            t_21 = Vector3d(0.1, 0.1, 0.0);
            // Eigen::Vector3d euler(0.0, 0.0, 0.0);  // degenerate case!
            Eigen::Vector3d euler(0.2, 0.1, 0.3);
            R_21 = e2R(euler);

            mwkps_c2.insert(std::pair<size_t, WKeyPoint>(0, WKeyPoint(pinhole::projectPoint(k, intr->distortion, R_21*X1_c1 + t_21))));
            mwkps_c2.insert(std::pair<size_t, WKeyPoint>(1, WKeyPoint(pinhole::projectPoint(k, intr->distortion, R_21*X2_c1 + t_21))));
            mwkps_c2.insert(std::pair<size_t, WKeyPoint>(2, WKeyPoint(pinhole::projectPoint(k, intr->distortion, R_21*X3_c1 + t_21))));
            mwkps_c2.insert(std::pair<size_t, WKeyPoint>(3, WKeyPoint(pinhole::projectPoint(k, intr->distortion, R_21*X4_c1 + t_21))));
            mwkps_c2.insert(std::pair<size_t, WKeyPoint>(4, WKeyPoint(pinhole::projectPoint(k, intr->distortion, R_21*X5_c1 + t_21))));
            mwkps_c2.insert(std::pair<size_t, WKeyPoint>(5, WKeyPoint(pinhole::projectPoint(k, intr->distortion, R_21*X6_c1 + t_21))));
            mwkps_c2.insert(std::pair<size_t, WKeyPoint>(6, WKeyPoint(pinhole::projectPoint(k, intr->distortion, R_21*X7_c1 + t_21))));
            mwkps_c2.insert(std::pair<size_t, WKeyPoint>(7, WKeyPoint(pinhole::projectPoint(k, intr->distortion, R_21*X8_c1 + t_21))));


            // Build the tracksMap
            for (size_t i=0; i<mwkps_c2.size(); ++i)
            {
                tracks_c1_c2.insert(std::pair<size_t, size_t>(i,i));
            }
        }
};


TEST_F(ProcessorVOMultiView_class, filterWithEssential)
{
    cv::Mat E;

    // Let's try FilterWithEssential, all points here are inliers
    processor->filterWithEssential(mwkps_c1, mwkps_c2, tracks_c1_c2, E);
    ASSERT_EQ(tracks_c1_c2.size(), mwkps_c2.size());

    ////////////////////////////////////////////////////////////////////
    // We had here an outlier: a keyPoint that doesn't move
    mwkps_c1.insert(std::pair<size_t, WKeyPoint>(8, WKeyPoint(cv::KeyPoint(cv::Point2f(120,140), 1))));
    mwkps_c2.insert(std::pair<size_t, WKeyPoint>(8, WKeyPoint(cv::KeyPoint(cv::Point2f(120,140), 1))));
    tracks_c1_c2.insert(std::pair<size_t, size_t>(8,8)); 

    // point at 8 must be discarded
    processor->filterWithEssential(mwkps_c1, mwkps_c2, tracks_c1_c2, E);
    ASSERT_EQ(tracks_c1_c2.count(8), 0);

}


TEST_F(ProcessorVOMultiView_class, recoverPoseOpenCV)
{

    // Check that the computed essential matrix corresponds to the camera movement
    // -> recover the orientation and compare to groundtruth
    cv::Mat E;

    // Compute essential matrix, all points here are inliers
    processor->filterWithEssential(mwkps_c1, mwkps_c2, tracks_c1_c2, E);

    ////////////////////////////////////////////////////////////////////
    // can we retrieve the right orientation from the essential matrix?
    std::vector<cv::Point2f> pts_c1, pts_c2;
    for (int i=0; i < mwkps_c1.size(); i++){
        pts_c1.push_back(mwkps_c1.at(i).getCvKeyPoint().pt);
        pts_c2.push_back(mwkps_c2.at(i).getCvKeyPoint().pt);
    }


    cv::Mat R_out, t_out;
    cv::Mat mask;
    cv::recoverPose(E, pts_c1, pts_c2, Kcv, R_out, t_out, mask);

    Eigen::Matrix3d R_out_eig, R_21_eig;
    cv::cv2eigen(R_out, R_out_eig);
    ASSERT_MATRIX_APPROX(R_21, R_out_eig, 1e-4);

    //////////////////////////////////////////////////////
    // Can we also use it to detect outliers using cheirality check?
    // Does not seem so...
    // Maybe the outliers are not rightly chosen for this test:
    // inside recoverPose, triangulatePoints is called and then there are
    // checks on depth coordinate of the triangulated point in both camera frames:
    // if depth is negative or depth lower than a threshold, point considered as an "outlier"
    // Can simply mean an absence of movement, hard to tune threshold...

    // add outliers
    pts_c1.push_back(cv::Point2f(165.0, 190.0));
    pts_c2.push_back(cv::Point2f(200.0, 190.0));

    pts_c1.push_back(cv::Point2f(100.0, 250.0));
    pts_c2.push_back(cv::Point2f(100.0, 250.0));

    pts_c1.push_back(cv::Point2f(400.0, 70.0));
    pts_c2.push_back(cv::Point2f(400.0, 70.0));

    pts_c1.push_back(cv::Point2f(300.0, 300.0));
    pts_c2.push_back(cv::Point2f(100.0, 300.0));    
    
    
    cv::Mat triangulated_pts;
    double distance_threshold = 80.0;
    // cv::recoverPose(E, pts_c1, pts_c2, Kcv, R_out, t_out, mask);
    cv::recoverPose(E, pts_c1, pts_c2, Kcv, R_out, t_out, distance_threshold, mask, triangulated_pts);

    // triangulated_pts.size()
    std::cout << triangulated_pts.size() << std::endl;


    // std::cout << "mask" << std::endl;
    // std::cout << mask << std::endl;

    // std::cout << "R_out" << std::endl;
    // std::cout << R_out << std::endl;
    // std::cout << "t_out" << std::endl;
    // std::cout << t_out << std::endl;



}


// // SEFAULT!!!!!!!!
// TEST_F(ProcessorVOMultiView_class, tryTriangulationFromFeatures)
// {
//     ProblemPtr problem = Problem::create("PO", 3);

//     VectorComposite state1("P0");
//     state1['P'] = Vector3d::Zero();
//     state1['O'] = Quaterniond::Identity().coeffs();
//     FrameBasePtr KF1 = FrameBase::emplace<FrameBase>(problem->getTrajectory(), 0.0, "PO", state1);
//     CaptureBasePtr c1 = CaptureBase::emplace<CaptureImage>(KF1, 0.0, nullptr, cv::Mat());
//     FeatureBasePtr f1 = FeatureBase::emplace<FeaturePointImage>(c1, mwkps_c1.at(0), Matrix2d::Identity());
    
//     VectorComposite state2("P0");
//     state2['P'] = Vector3d::Zero();
//     state2['O'] = Quaterniond::Identity().coeffs();
//     FrameBasePtr KF2 = FrameBase::emplace<FrameBase>(problem->getTrajectory(), 1.0, "PO", state1);
//     CaptureBasePtr c2 = CaptureBase::emplace<CaptureImage>(KF2, 1.0, nullptr, cv::Mat());
//     FeatureBasePtr f2 = FeatureBase::emplace<FeaturePointImage>(c2, mwkps_c2.at(0), Matrix2d::Identity());

//     Track track;
//     track.insert(std::pair<TimeStamp, FeatureBasePtr>(0.0, f1));
//     track.insert(std::pair<TimeStamp, FeatureBasePtr>(1.0, f2));
    
//     Vector4d pt_c;
//     auto f2_pi = std::static_pointer_cast<FeaturePointImage>(f2);
//     processor->tryTriangulationFromFeatures(f2_pi, track, pt_c);
//     WOLF_INFO(pt_c);
// }

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
