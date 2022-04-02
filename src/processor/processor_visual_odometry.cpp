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


//standard
#include "vision/processor/processor_visual_odometry.h"


namespace wolf{

ProcessorVisualOdometry::ProcessorVisualOdometry(ParamsProcessorVisualOdometryPtr _params_visual_odometry) :
                ProcessorTracker("ProcessorVisualOdometry", "PO", 3, _params_visual_odometry),
                params_visual_odometry_(_params_visual_odometry),
                frame_count_(0)
{
    //////////////////
    // PARAMS of the keypoint detector
    npoints_ = 500;
    scale_factor_ = 1.2;
    nlevels_pyramids_ = 8;

    // PARAMS KLT tracker
    tracker_width_ = 21;
    tracker_height_ = 21;
    klt_max_err_ = 0.2;
    nlevels_pyramids_klt_ = 3;
    crit_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    search_width_ = 21;
    search_height_ = 21;
    pyramid_level_ = 3;
    //////////////////

    // Initialize the detector
    detector_ = cv::ORB::create(npoints_,
                                scale_factor_,
                                nlevels_pyramids_,
                                31, 0, 2, cv::ORB::FAST_SCORE, 31, 20);

    // detector_ = cv::FAST::create(npoints,
    //                             scale_factor,
    //                             nlevels_pyramids,
    //                             31, 0, 2, cv::ORB::FAST_SCORE, 31, 20);

}

void ProcessorVisualOdometry::configure(SensorBasePtr _sensor)
{
	//Initialize camera sensor pointer
	sen_cam_ = std::static_pointer_cast<SensorCamera>(_sensor);
    Eigen::Matrix3d K = sen_cam_->getIntrinsicMatrix();
    
    Kcv_ = cv::Mat(3, 3, CV_32F, K.data());
}

void ProcessorVisualOdometry::preProcess()
{
    // Get Capture
    capture_image_incoming_ = std::dynamic_pointer_cast<CaptureImage>(incoming_ptr_);
    assert(capture_image_incoming_ != nullptr && ("Capture type mismatch. Processor " + getName() + " can only process captures of type CaptureImage").c_str());

    // Preprocess image
    cv::Mat img_incoming = capture_image_incoming_->getImage();
    std::cout << "img_incoming size: " << img_incoming.size().width << ", " << img_incoming.size().height << std::endl;


    // greyscale if BGR, CLAHE etc...
    // once preprocessing is done, replace the original image (?)
    // capture_image_incoming_->setImage(img_incoming);


    // if first image, compute keypoints and return
    // if (last_ptr_ == nullptr){
    //     size_t nb_detect = 100;
    //     std::vector<cv::KeyPoint> kps_current;
    //     kps_current.reserve(nb_detect);

    //     detector_->detect(img_incoming, kps_current);
    //     capture_image_incoming_->setKeyPoints(kps_current);
        
    //     // Description
    //     // ...

    //     return;
    // }


    ////////////////
    // if too few tracks, compute new keypoints (after tracking no?)

    ////////////////

    ////////////////////////
    ////////////////////////
    // TRACKING
    // Proceeed to tracking the previous features
    // capture_image_last_ = std::dynamic_pointer_cast<CaptureImage>(last_ptr_);
    // cv::Mat img_last = capture_image_last_->getImage();

    // std::cout << "img_last size: " << img_last.size().width << ", " img_last.size().height << std::endl;

    // VectorKeyPointPtrPairs tracks_origin_last = capture_image_last_->tracks_origin_;

    // // Process one way: previous->current with current init with previous
    // std::vector<uchar> status;
    // std::vector<float> err;

    // std::vector<cv::Point2f> v_pix_last, v_pix_incoming;

    // // KeyPoints in the last image are all considered for tracking
    // v_kps_last = capture_image_last_->getKeyPoints();
    // for (size_t i = 0; i < v_kps_last.size(); i++){
    //     v_pix_last.push_back(v_kps_last[i].pt);
    //     v_pix_incoming.push_back(v_kps_incoming[i].pt);  // use the previous tracked keypoints as a prior
    // }

    // // Track features
    // cv::calcOpticalFlowPyrLK(
    //     img_last,
    //     img_incoming, 
    //     v_pix_last,
    //     v_pix_incoming,
    //     status, 
    //     err,
    //     {search_width_, search_height_}, 
    //     pyramid_level_,
    //     crit_,
    //     (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS)
    // );

    // // Delete point if KLT failed or if point is outside the image
    // cv::Size img_size = img_incoming.size();
    // std::vector<cv::Point2f> v_pix_last_klt_fwd, v_pix_incoming_klt_fwd;

    // std::vector<size_t> tracks_origin_after_fwd_klt;
    // tracks_origin_after_fwd_klt.reserve(status.size());  // reserve more than necessary
    // for (size_t i = 0; i < status.size(); i++) {
    //     // Invalid match if one of the OF failed or KLT error is too high
    //     if(!status.at(i) || (err.at(i) > klt_max_err_)) {
    //         continue;
    //     }

    //     // Check if tracked points in the second image are in the image
    //     if( (v_pix_incoming.at(i).x < 0) ||
    //         (v_pix_incoming.at(i).y < 0) ||
    //         (v_pix_incoming.at(i).x > img_size.width) ||
    //         (v_pix_incoming.at(i).y > img_size.height) ) {
    //         continue;
    //     }


    //     v_pix_last_klt_fwd.push_back(v_pix_last.at(i));  // for reverse tracking
    //     v_pix_incoming_klt_fwd.push_back(v_pix_incoming.at(i));


    //     tracks_origin_after_fwd_klt.push_back(tracks_origin_kps_last[i]);
    //     // tracks_origin_after_fwd_klt.push_back(i);
    // }

    // ////////////////////////


    // /////////////////////////
    // // OUTLIER REJECTION
    // // Check inliers with RANSAC essential matrix computation 

    // std::vector<size_t> tracks_origin_after_ransac;
    // tracks_origin_after_ransac.reserve(tracks_origin_after_fwd_klt.size());  // reserve more than necessary
    // cv::Mat mask_inliers;
    // cv::Mat E = cv::findEssentialMat(v_pix_last_klt_fwd, v_pix_incoming_klt_fwd, Kcv_, 
    //                                  cv::RANSAC, 0.99, 1.0, mask_inliers);
    // size_t nb_valid_tracks = cv::countNonZero(mask_inliers);
    // /////////////////////////

    // std::vector<cv::Point2f> v_pix_incoming_valid;
    // std::vector<cv::KeyPoint> v_kps_incoming_valid;
    // v_pix_incoming_valid.reserve(nb_valid_tracks);
    // for (size_t i=0; i < v_pix_last_klt_fwd.size(); i++){
    //     if (mask_inliers.at<bool>(i)){
    //         v_pix_incoming_valid.push_back(v_pix_incoming_klt_fwd[i]);
    //         cv::KeyPoint kp(v_pix_incoming_klt_fwd[i], 1, 0, 0, 0, 0);  // TODO: populate with more info or use new class
    //         v_kps_incoming_valid.push_back(kp);

    //         tracks_origin_after_ransac.push_back(tracks_origin_after_fwd_klt[i]);
    //     }
    // }

    // capture_image_incoming_.tracks_origin_kps_ = tracks_origin_after_ransac;



}


void ProcessorVisualOdometry::postProcess()
{

    frame_count_ ++;
}

bool ProcessorVisualOdometry::voteForKeyFrame() const
{

    return ((frame_count_ % 5) == 0);
}

unsigned int ProcessorVisualOdometry::processKnown()
{
    return 42;
}


unsigned int ProcessorVisualOdometry::processNew(const int& _max_features)
{
    return 42;
}


void ProcessorVisualOdometry::establishFactors()
{
    // TODO
}

void ProcessorVisualOdometry::advanceDerived()
{
    // TODO
}

void ProcessorVisualOdometry::resetDerived()
{
    // TODO
}



void ProcessorVisualOdometry::setParams(const ParamsProcessorVisualOdometryPtr _params)
{
    params_visual_odometry_ = _params;
}

} //namespace wolf

// Register in the FactoryProcessor
#include "core/processor/factory_processor.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR(ProcessorVisualOdometry)
WOLF_REGISTER_PROCESSOR_AUTO(ProcessorVisualOdometry)
} // namespace wolf

