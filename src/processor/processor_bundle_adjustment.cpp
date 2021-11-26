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
/*
 * processor_bundle_adjustment.cpp
 *
 *  Created on: May 3, 2019
 *      Author: ovendrell
 */

//wolf
#include "vision_utils/vision_utils.h"
#include "vision_utils/detectors.h"
#include "vision_utils/descriptors.h"
#include "vision_utils/matchers.h"

//standard
#include <memory>
#include "vision/processor/processor_bundle_adjustment.h"

namespace wolf{

ProcessorBundleAdjustment::ProcessorBundleAdjustment(ParamsProcessorBundleAdjustmentPtr _params_bundle_adjustment) :
                ProcessorTrackerFeature("ProcessorBundleAdjustment", "PO", 3, _params_bundle_adjustment),
                params_bundle_adjustment_(_params_bundle_adjustment),
                capture_image_last_(nullptr),
                capture_image_incoming_(nullptr),
                frame_count_(0)
{
	//Initialize detector-descriptor-matcher
	pixel_cov_ = Eigen::Matrix2d::Identity() * params_bundle_adjustment_->pixel_noise_std * params_bundle_adjustment_->pixel_noise_std;

    // Detector yaml file
    std::string det_name = vision_utils::readYamlType(params_bundle_adjustment_->yaml_file_params_vision_utils, "detector");
    // Create Detector
    det_ptr_ = vision_utils::setupDetector(det_name, det_name + " detector", params_bundle_adjustment_->yaml_file_params_vision_utils);

    // Descriptor yaml file
    std::string des_name = vision_utils::readYamlType(params_bundle_adjustment_->yaml_file_params_vision_utils, "descriptor");
    // Create Descriptor
    des_ptr_ = vision_utils::setupDescriptor(des_name, des_name + " descriptor", params_bundle_adjustment_->yaml_file_params_vision_utils);

    // Matcher yaml file
    std::string mat_name = vision_utils::readYamlType(params_bundle_adjustment_->yaml_file_params_vision_utils, "matcher");
    // Create Matcher
    mat_ptr_ = vision_utils::setupMatcher(mat_name, mat_name + " matcher", params_bundle_adjustment_->yaml_file_params_vision_utils);

    //Initialize rvec and tvec
    tvec_ = cv::Mat::zeros(cv::Size(3,1), CV_32F);
    rvec_ = cv::Mat::zeros(cv::Size(3,1), CV_32F);
//    Eigen::Vector3d last_pos = capture_image_last_->getFrame()->getP()->getState();
//    Eigen::Vector4d last_or = capture_image_last_->getFrame()->getO()->getState();
//    Eigen::Quaternion<double> last_q(last_or(0), last_or(1), last_or(2), last_or(3));
//
//    Eigen::VectorXd tvec_eigen = last_pos;
//    Eigen::VectorXd rvec_eigen = q2v(last_q);
//
//    cv::eigen2cv(tvec_eigen, tvec);
//    cv::eigen2cv(rvec_eigen, rvec);


}

void ProcessorBundleAdjustment::configure(SensorBasePtr _sensor)
{
    //TODO: Implement if needed
	//Initialize camera sensor pointer
	camera = std::static_pointer_cast<SensorCamera>(_sensor);

}

void ProcessorBundleAdjustment::preProcess()
{
    // This method implements all Vision algorithms concerning OpenCV, so wolf algorithm only has to manage the data obtained
    // Get Capture
    capture_image_incoming_ = std::dynamic_pointer_cast<CaptureImage>(incoming_ptr_);
    assert(capture_image_incoming_ != nullptr && ("Capture type mismatch. Processor " + getName() + " can only process captures of type CaptureImage").c_str());
    // capture_image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);

    // Detect KeyPoints
    capture_image_incoming_->keypoints_ = det_ptr_->detect(capture_image_incoming_->getImage());

    //Sort keypoints by response if needed
    if (!capture_image_incoming_->keypoints_.empty())
    	vision_utils::sortByResponse(capture_image_incoming_->keypoints_, CV_SORT_DESCENDING);

    // Compute Descriptors
    capture_image_incoming_->descriptors_ = des_ptr_->getDescriptor(capture_image_incoming_->getImage(), capture_image_incoming_->keypoints_);

    // Create and fill incoming grid
    capture_image_incoming_->grid_features_ = std::make_shared<vision_utils::FeatureIdxGrid>(capture_image_incoming_->getImage().rows, capture_image_incoming_->getImage().cols, params_bundle_adjustment_->n_cells_v, params_bundle_adjustment_->n_cells_h);

    capture_image_incoming_->grid_features_->insert(capture_image_incoming_->keypoints_);

    if (last_ptr_ != nullptr)
    {
        // Get Capture
        capture_image_last_ = std::static_pointer_cast<CaptureImage>(last_ptr_);
        // Match image
        capture_image_incoming_->matches_normalized_scores_ = mat_ptr_->robustMatch(capture_image_incoming_->keypoints_, capture_image_last_->keypoints_,
                                                                                    capture_image_incoming_->descriptors_, capture_image_last_->descriptors_,
                                                                                    des_ptr_->getSize(), capture_image_incoming_->matches_from_precedent_);


        //TODO: Get only the best ones
        if (params_bundle_adjustment_->delete_ambiguities) //filter ambiguities
        {
        	std::map<int,bool> ambiguities_map;
        	for (auto match : capture_image_incoming_->matches_from_precedent_)
        		if (ambiguities_map.count(match.trainIdx)) //if ambiguity
        			ambiguities_map[match.trainIdx] = true; //ambiguity true
        		else //if not ambiguity
        			ambiguities_map[match.trainIdx] = false; //ambiguity false
        	// Set capture map of match indices
        	for (auto match : capture_image_incoming_->matches_from_precedent_)
        	{
        		if (!ambiguities_map.at(match.trainIdx))
        			capture_image_last_->map_index_to_next_[match.trainIdx] = match.queryIdx; // map[last] = incoming
        	}
        }
        else
        {
        	// Set capture map of match indices
        	for (auto match : capture_image_incoming_->matches_from_precedent_)
        		capture_image_last_->map_index_to_next_[match.trainIdx] = match.queryIdx; // map[last] = incoming

        }


//        //get points 2d from inlier matches
//        PointVector pts1, pts2;
//        for (auto match : capture_image_incoming_->matches_from_precedent_)
//        {
//        	pts1.push_back(capture_image_incoming_->keypoints_[match.queryIdx].pt);
//        	pts2.push_back(capture_image_last_->keypoints_[match.trainIdx].pt);
//        }
//
//        //find Essential Matrix
//        std::vector<uchar> inliers(pts1.size(),0);
//
//        auto camera_mat = std::static_pointer_cast<SensorCamera>(capture_image_last_->getSensor())->getIntrinsicMatrix();
//        cv::Mat camera_mat_cv;
//        cv::eigen2cv(camera_mat, camera_mat_cv);
//
//        cv::Mat E = cv::findEssentialMat(pts1,
//        					pts2,
//							camera_mat_cv,
//							CV_RANSAC,
//							0.999,
//							1.0,
//							inliers);
//
//        //Compute rotation R and translation t from E
//        cv::Mat R_cv;
//        cv::Mat t_cv;
//
//        cv::recoverPose(E,pts1,pts2, camera_mat_cv, R_cv, t_cv, inliers);
//
//        //Convert R and t from OpenCV type to Eigen type and convert Rotation to a quaternion
//        Eigen::Matrix3d R;
//        cv::cv2eigen(R_cv, R);
////        auto q = R2q(R); //Quaternion q
//
//        Eigen::Vector3d t; //Translation t
//        cv::cv2eigen(t_cv,t);







//        cv::Mat img_mat = cv::Mat(capture_image_incoming_->getImage().rows, capture_image_incoming_->getImage().cols, capture_image_incoming_->getImage().type());
//
//        cv::drawMatches(capture_image_incoming_->getImage(), 		 capture_image_incoming_->getKeypoints(),
//        				capture_image_last_->getImage(),     capture_image_last_->getKeypoints(),
//						capture_image_incoming_->matches_from_precedent_, img_mat);
//
//        cv::namedWindow("MATCHES VIEW", cv::WINDOW_NORMAL);
//        cv::resizeWindow("MATCHES VIEW", 800,600);
//        cv::imshow("MATCHES VIEW", img_mat);
    }

//    std::cout << "preProcess() done " << std::endl;

}

void ProcessorBundleAdjustment::postProcess()
{

    WOLF_INFO("last ", last_ptr_, "camera ", camera);

    //=====================================Compute prior for the frame===============================================
//    if (capture_image_last_)
//	{
//
//		std::vector<cv::Point3f> landmark_points;
//		std::vector<cv::Point2f> image_points;
//
//		//get camera information
//		auto camera_mat = camera->getIntrinsicMatrix();
//
//		cv::Mat camera_mat_cv;
//		cv::eigen2cv(camera_mat, camera_mat_cv);
//
//		auto dist_coeffs = camera->getDistortionVector();
//
//		Eigen::Vector5d dist;
//		switch(dist_coeffs.size())
//		{
//		case 0:
//		{
//			dist << 0,0,0,0,0;
//			break;
//		}
//		case 1:
//		{
//			dist << dist_coeffs(0),0,0,0,0;
//			break;
//		}
//		case 2:
//		{
//			dist << dist_coeffs(0),dist_coeffs(1),0,0,0;
//			break;
//		}
//		case 3:
//		{
//			dist << dist_coeffs(0),dist_coeffs(1),0,0,dist_coeffs(2);
//			break;
//		}
//		default:
//			dist << 0,0,0,0,0;
//		}
//
//		cv::Mat dist_coeffs_cv;
//		cv::eigen2cv(dist, dist_coeffs_cv);
//
//
//
//		//fill points and landmarks list
//		int debug_counter = 0;
//
////		for (auto & feat : capture_image_last_->getFeatureList())
////		{
////			for (auto & fact : feat->getFactorList())
////			{
////				debug_counter++;
////				//check the number of factors to the landmark
////				if (fact->getLandmarkOther()->getConstrainedByList().size() >= 2)
////				{
////					//3d points
////					auto point3d = std::static_pointer_cast<LandmarkHp>(fact->getLandmarkOther())->point();
////					landmark_points.push_back(cv::Point3f(point3d(0),point3d(1),point3d(2)));
////					//2d points
////					auto point2d = feat->getMeasurement();
////					image_points.push_back(cv::Point2f(point2d(0), point2d(1)));
////				}
////			}
////		}
//
//		for (auto & feat : track_matrix_.snapshotAsList(capture_image_last_))
//		{
//			debug_counter++;
//			auto trkId = feat->trackId();
//			if (lmk_track_map_.count(feat->trackId()))
//			{
//				auto lmk_base = lmk_track_map_[trkId];
////				if (lmk_base->getConstrainedByList().size() >= 2)
//				{
//					//3d points
//					auto point3d = std::static_pointer_cast<LandmarkHp>(lmk_base)->point();
//					landmark_points.push_back(cv::Point3f(point3d(0),point3d(1),point3d(2)));
//					//2d points
//					auto point2d = feat->getMeasurement();
//					image_points.push_back(cv::Point2f(point2d(0), point2d(1)));
//				}
//			}
//		}
//
//		WOLF_INFO("Num lmks in last:", debug_counter,"lmks constrained by >= 2 fctrs: ", landmark_points.size());
//
//		//solvePnP
//		if (landmark_points.size() > 7)
//		{
//            WOLF_INFO("before PnP ", tvec, rvec);
//            WOLF_INFO("before PnP ", last_ptr_->getFrame()->getState().transpose());
//
//
//            cv::solvePnP(landmark_points, image_points, camera_mat_cv, dist_coeffs_cv, rvec, tvec, true);
//		    WOLF_INFO("Solve PnP Done")
//
//		    //Compute and set Robot state
//		    //q_w_s Camera quaternion
//		    Eigen::Vector3d rvec_eigen;
//		    cv::cv2eigen(rvec, rvec_eigen);
//		    Eigen::Quaternion<double> q_w_s = v2q(rvec_eigen);
//
//		    //p_w_s Camera position
//		    Eigen::Vector3d p_w_s;
//		    cv::cv2eigen(tvec, p_w_s);
//
//		    //Robot pose
//		    //	Eigen::Vector3d p_w_r = capture_image_last_->getFrame()->getP()->getState();
//		    //	Eigen::Quaternion<double> q_w_r = Quaterniond(capture_image_last_->getFrame()->getO()->getState().data());
//
//		    //Extrinsics (camera in robot reference frame)
//		    Eigen::Vector3d p_r_s = camera->getP()->getState();
//		    Eigen::Quaternion<double> q_r_s = Quaterniond(camera->getO()->getState().data());
//
//		    //Robot pose compositions
//		    Eigen::Quaternion<double> q_w_r = q_w_s * q_r_s.conjugate();
//		    Eigen::Vector3d p_w_r = p_w_s - q_w_r * p_r_s;
//
//
//
//		    Eigen::Vector7d prior_state;
//		    prior_state << p_w_r(0), p_w_r(1), p_w_r(2), q_w_r.x(), q_w_r.y(), q_w_r.z(), q_w_r.w();
//
//		    last_ptr_->getFrame()->setState(prior_state);
//
//            WOLF_INFO("after PnP  ", last_ptr_->getFrame()->getState().transpose());
//            WOLF_INFO("after PnP  ", tvec, rvec);
//
//		}
//
//		//=================================================================================================================
//	}

    //delete landmarks
    std::list<LandmarkBasePtr> lmks_to_remove;
    for (auto & lmk : getProblem()->getMap()->getLandmarkList())
    {
        if (lmk->getConstrainedByList().size()==1)
        {
            if (lmk->getConstrainedByList().front()->getFeature()->getCapture() != origin_ptr_)
            {
                WOLF_INFO("Removing landmark: ", lmk->id());
                lmk_track_map_.erase(lmk->getConstrainedByList().front()->getFeature()->trackId());
                WOLF_INFO("Lmk deleted from track map: ", lmk->id());
                lmks_to_remove.push_back(lmk);
           }
        }
    }
    for (auto & lmk : lmks_to_remove)
    {
        lmk->remove();
        WOLF_INFO("Lmk deleted: ", lmk->id());
    }
    getProblem()->check(0);


    // draw debug image ======================================================================================

    // image to draw
    CaptureBasePtr image_to_draw = last_ptr_;


    std::map<int,std::list<vision_utils::KeyPointEnhanced> > kp_enh_tracks;
    for (auto const & feat_base : image_to_draw->getFeatureList())
    {
        FeaturePointImagePtr feat = std::static_pointer_cast<FeaturePointImage>(feat_base);
        unsigned int feat_id = feat->id();
        unsigned int track_id = feat->trackId();
        // tracks
        std::list<vision_utils::KeyPointEnhanced> kp_enh_track;
        for (auto feat_base_track : track_matrix_.trackAsVector(track_id))
        {
            FeaturePointImagePtr feat_img = std::static_pointer_cast<FeaturePointImage>(feat_base_track);
            vision_utils::KeyPointEnhanced kp_enh(feat_img->getKeypoint(),
                                                  feat_id,
                                                  track_id,
                                                  track_matrix_.trackSize(track_id),
                                                  feat_img->getMeasurementCovariance());
            kp_enh_track.push_back(kp_enh);
        }

        kp_enh_tracks[feat_id] = kp_enh_track;
    }
    // DEBUG
    image_debug_ = vision_utils::buildImageProcessed((std::static_pointer_cast<CaptureImage>(image_to_draw))->getImage(), kp_enh_tracks);
//    Snapshot snapshot_last = track_matrix_.snapshot(last_ptr_);
////	getProblem()->print(4,1,1,0);
//    for (auto pair : track_matrix_.snapshot(origin_ptr_))
//    {
//    	if (snapshot_last.count(pair.first)==0)
//    	{
//    		if (!(pair.second->getFactorList()).empty())
//    		{
//				auto factor = pair.second->getFactorList().front();
//				if (factor)
//				{
//					auto lmk = factor->getLandmarkOther();
//					if (lmk)
//					{
////						lmk->remove();
////	    	    		track_matrix_.remove(pair.second->trackId());
//					}
//				}
//    		}
//    	}
//    }
//    getProblem()->print(0,0,1,0);

    list<FeatureBasePtr> snapshot = track_matrix_.snapshotAsList(image_to_draw);

    for (auto const & feat_base : snapshot)
    {
        FeaturePointImagePtr feat = std::static_pointer_cast<FeaturePointImage>(feat_base);
        unsigned int track_id = feat->trackId();
		if (lmk_track_map_.count(track_id))
        {
			Vector3d point3d = std::static_pointer_cast<LandmarkHp>(lmk_track_map_[track_id])->point();

			Transform<double,3,Isometry> T_w_r
		        = Translation<double,3>(feat_base->getFrame()->getP()->getState())
		        * Quaterniond(feat_base->getFrame()->getO()->getState().data());
		    Transform<double,3,Isometry> T_r_c
				= Translation<double,3>(feat_base->getCapture()->getSensorP()->getState())
		        * Quaterniond(feat_base->getCapture()->getSensorO()->getState().data());

		    Eigen::Matrix<double, 3, 1> point3d_c = T_r_c.inverse()
												   * T_w_r.inverse()
		                                           * point3d;

//		    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(getSensor());

		    Vector2d point2d = pinhole::projectPoint(getSensor()->getIntrinsic()->getState(), camera->getDistortionVector(), point3d_c);
    		cv::Point point = cv::Point(point2d(0), point2d(1));

    		cv::circle(image_debug_, point, 3, cv::Scalar(0,0,255) , 1 , 8);
    		std::string id = std::to_string(lmk_track_map_[track_id]->id());
    		cv::putText(image_debug_, id, point, cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,255,0),2);
		}
	}

    // frame counter for voting for KF --- if not used in voteForKf, this can be removed.
    frame_count_ ++;

//    cv::namedWindow("DEBUG VIEW", cv::WINDOW_NORMAL);
//    cv::resizeWindow("DEBUG VIEW", 800,600);
//    cv::imshow("DEBUG VIEW", image_debug_);
//    cv::waitKey(1);
}


unsigned int ProcessorBundleAdjustment::trackFeatures(const FeatureBasePtrList& _features_in,
                                                      const CaptureBasePtr& _capture,
                                                      FeatureBasePtrList& _features_out,
                                                      FeatureMatchMap& _feature_correspondences)
{
    for (auto feat_base: _features_in)
    {
        FeaturePointImagePtr feat_last = std::static_pointer_cast<FeaturePointImage>(feat_base);

        if (capture_image_last_->map_index_to_next_.count(feat_last->getIndexKeyPoint())) //If the track to incoming exists
        {
            int index_inc = capture_image_last_->map_index_to_next_.at(feat_last->getIndexKeyPoint());

            if (true)//capture_image_incoming_->matches_normalized_scores_.at(index_inc) > mat_ptr_->getParams()->min_norm_score )
            {
                // Get kp incoming and keypoint last
                cv::KeyPoint kp_inc = capture_image_incoming_->keypoints_.at(index_inc);
                cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(feat_last->getIndexKeyPoint());

                if (isInlier(kp_last, kp_inc))
                {
					FeatureBasePtr feat_inc = FeatureBase::emplace<FeaturePointImage>(_capture,
					                                                                  kp_inc,
					                                                                  index_inc,
					                                                                  capture_image_incoming_->descriptors_.row(index_inc),
					                                                                  pixel_cov_);

					_features_out.push_back(feat_inc);

					auto feature_match_ptr = std::make_shared<FeatureMatch>();
					feature_match_ptr->feature_ptr_= feat_last;
					feature_match_ptr->normalized_score_ = 1.0;//capture_image_incoming_->matches_normalized_scores_.at(index_inc);
					_feature_correspondences[feat_inc] = feature_match_ptr;


					// hit cell to acknowledge there's a tracked point in that cell
					capture_image_incoming_->grid_features_->hitTrackingCell(kp_inc);
                }
            }
        }
    }
//    return _feature_correspondences.size();
    return _features_out.size();
}

bool ProcessorBundleAdjustment::isInlier(const cv::KeyPoint& _kp_last, const cv::KeyPoint& _kp_incoming) const
{
    // List of conditions
    bool inlier = true;

    // A. Check euclidean norm
    inlier = inlier && (cv::norm(_kp_incoming.pt-_kp_last.pt) < mat_ptr_->getParams()->max_match_euclidean_dist);

    // B. add your new condition test here
    // inlier = inlier && ...

    return inlier;
}

bool ProcessorBundleAdjustment::is_tracked(int& _kp_idx) const
{

    for (auto ftr : known_features_incoming_)
    {
        FeaturePointImagePtr ftr_kp = std::static_pointer_cast<FeaturePointImage>(ftr);
        if (ftr_kp->getIndexKeyPoint() == _kp_idx)
        {
            return true;
        }
    }
    return false;
}

unsigned int ProcessorBundleAdjustment::detectNewFeatures(const int& _max_new_features,
                                                          const CaptureBasePtr& _capture,
                                                          FeatureBasePtrList& _features_out)
{
    //TODO: efficient implementation?

////Simpler method to detect new features without using Grid Features form vision_utils
//    typedef std::map<int,int>::iterator iter;
//
//    for (iter it = capture_image_last_->map_index_to_next_.begin(); it!=capture_image_last_->map_index_to_next_.end(); ++it)
//    {
//        if (_features_out.size() >= _max_new_features)
//        		break;
//
//        else if(!is_tracked(it->second))
//        {
//            //add to _features_out
//            int idx_last = it->first;
//            cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(idx_last);
//            FeaturePointImagePtr feat_last_ = std::make_shared<FeaturePointImage>(kp_last, idx_last,
//                                                                                 capture_image_last_->descriptors_.row(idx_last),
//                                                                                 pixel_cov_);
//            _features_out.push_back(feat_last_);
//
//        }
//    }
//
//    return _features_out.size();

	for (unsigned int n_iterations = 0; _max_new_features == -1 || n_iterations < _max_new_features; ++n_iterations)
	{
		Eigen::Vector2i cell_last;
		assert(capture_image_last_->grid_features_!=nullptr);
		if (capture_image_last_->grid_features_->pickEmptyTrackingCell(cell_last))
		{
			// Get best keypoint in cell
			vision_utils::FeatureIdxMap cell_feat_map = capture_image_last_->grid_features_->getFeatureIdxMap(cell_last(0), cell_last(1), params_bundle_adjustment_->min_response_new_feature);
			bool found_feature_in_cell = false;

			for (auto target_last_pair_response_idx : cell_feat_map)
			{
				// Get KeyPoint in last
				unsigned int index_last = target_last_pair_response_idx.second;
				cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(index_last);
				assert(target_last_pair_response_idx.first == kp_last.response && "[ProcessorTrackerFeatureTrifocal::detectNewFeatures]: response mismatch.");

				// Check if there is match with incoming, if not we do not want it
				if (capture_image_last_->map_index_to_next_.count(index_last))
				{
					// matching keypoint in incoming
                  	unsigned int index_incoming = capture_image_last_->map_index_to_next_[index_last];
                    cv::KeyPoint kp_incoming = capture_image_incoming_->keypoints_.at(index_incoming);

					// validate match with extra tests
					if (isInlier( kp_incoming, kp_last))
					{
						// Create WOLF feature
						FeatureBasePtr ftr_point_last = FeatureBase::emplace<FeaturePointImage>(_capture,
                                                                                                kp_last,
                                                                                                index_last,
                                                                                                capture_image_last_->descriptors_.row(index_last),
                                                                                                pixel_cov_);

						_features_out.push_back(ftr_point_last);

						// hit cell to acknowledge there's a tracked point in that cell
						capture_image_last_->grid_features_->hitTrackingCell(kp_last);

						found_feature_in_cell = true;

						break; // Good kp found for this grid. //Use to have only one keypoint per grid
                    }
				}
			}
			if (!found_feature_in_cell)
				capture_image_last_->grid_features_->blockTrackingCell(cell_last);
		}
		else
			break; // There are no empty cells
	}
	return _features_out.size();
}


bool ProcessorBundleAdjustment::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    //TODO: Implement if needed
    return true;
}

bool ProcessorBundleAdjustment::voteForKeyFrame() const
{



    return ((frame_count_ % 5) == 0);






	//    // A. crossing voting threshold with ascending number of features
	    bool vote_up = false;
	//    // 1. vote if we did not have enough features before
	//    vote_up = vote_up && (previousNumberOfTracks() < params_bundle_adjustment_->min_features_for_keyframe);
	//    // 2. vote if we have enough features now
	//    vote_up = vote_up && (incoming_ptr_->getFeatureList().size() >= params_bundle_adjustment_->min_features_for_keyframe);

	    // B. crossing voting threshold with descending number of features
	    bool vote_down = true;
	    // 1. vote if we had enough features before
	//    vote_down = vote_down && (last_ptr_->getFeatureList().size() >= params_bundle_adjustment_->min_features_for_keyframe);
	    // 2. vote if we have not enough features now
	    vote_down = vote_down && (incoming_ptr_->getFeatureList().size() < params_bundle_adjustment_->min_features_for_keyframe);

	//    // C. Time-based policies
	    bool vote_time = false;
	////    vote_time = vote_time || (incoming_ptr_->getTimeStamp()-origin_ptr_->getTimeStamp() > 1.0);
	//
	//    if (vote_up)
	//        WOLF_TRACE("VOTE UP");
	//    if (vote_down)
	//        WOLF_TRACE("VOTE DOWN");
	//    if (vote_time)
	//        WOLF_TRACE("VOTE TIME");
	    if (vote_down)
	    	WOLF_INFO("Creating KF. Number of features: ", incoming_ptr_->getFeatureList().size());

	    return vote_up || vote_down || vote_time;
}


FactorBasePtr ProcessorBundleAdjustment::emplaceFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
    /* This function is no creating any factor.
     *  We create factors with establishFactors()
     */
    return FactorBasePtr();
}

LandmarkBasePtr ProcessorBundleAdjustment::emplaceLandmark(FeatureBasePtr _feature_ptr)
{
    FeaturePointImagePtr feat_point_image_ptr = std::static_pointer_cast<FeaturePointImage>( _feature_ptr);
    Eigen::Vector2d point2d = _feature_ptr->getMeasurement();

    Eigen::Vector3d point3d;
    point3d = pinhole::backprojectPoint(
    		getSensor()->getIntrinsic()->getState(),
    		(std::static_pointer_cast<SensorCamera>(getSensor()))->getCorrectionVector(),
			point2d);



    //double distance = params_bundle_adjustment_->distance; // arbitrary value
    double distance = 1;
    Eigen::Vector4d vec_homogeneous_c;
    vec_homogeneous_c = {point3d(0),point3d(1),point3d(2),point3d.norm()/distance};
    vec_homogeneous_c.normalize();

    //TODO: lmk from camera to world coordinate frame.
    Transform<double,3,Isometry> T_w_r
        = Translation<double,3>(_feature_ptr->getFrame()->getP()->getState())
        * Quaterniond(_feature_ptr->getFrame()->getO()->getState().data());
    Transform<double,3,Isometry> T_r_c
		= Translation<double,3>(_feature_ptr->getCapture()->getSensorP()->getState())
        * Quaterniond(_feature_ptr->getCapture()->getSensorO()->getState().data());
    Eigen::Matrix<double, 4, 1> vec_homogeneous_w = T_w_r
                                           * T_r_c
                                           * vec_homogeneous_c;


//    std::cout << "Point2d: " << point2d.transpose() << std::endl;
//    std::cout << "Point3d: " << point3d.transpose() << std::endl;
//    std::cout << "Hmg_c: " << vec_homogeneous.transpose() << std::endl;
//    std::cout << "Hmg_w: " << vec_homogeneous_w.transpose() << std::endl;
    //vec_homogeneous_w = vec_homogeneous;

    LandmarkBasePtr lmk_hp_ptr = LandmarkBase::emplace<LandmarkHp>(getProblem()->getMap(), vec_homogeneous_w, getSensor(), feat_point_image_ptr->getDescriptor());

//    std::cout << "LANDMARKS CREATED AND ADDED to MAP: " << getProblem()->getMap()->getLandmarkList().size() << std::endl;

    _feature_ptr->setLandmarkId(lmk_hp_ptr->id());
    return lmk_hp_ptr;
}

void ProcessorBundleAdjustment::establishFactors()
{

//	if (origin_ptr_ == last_ptr_)
//		return;
//
//	TrackMatches matches_origin_inc = track_matrix_.matches(origin_ptr_, last_ptr_);
//
//    for (auto const & pair_trkid_pair : matches_origin_inc)
//    {
//        size_t trkid = pair_trkid_pair.first;
//        //if track size is lower than a minimum, don't establish any factor.
//        if (track_matrix_.trackSize(trkid) < params_bundle_adjustment_->min_track_length_for_factor)
//        	continue;
//
//        FeatureBasePtr feature_origin = pair_trkid_pair.second.first;
//        FeatureBasePtr feature_last   = pair_trkid_pair.second.second;
//
//        if (lmk_track_map_.count(trkid)==0) //if the track doesn't have landmark associated -> we need a map: map[track_id] = landmarkptr
//        {
//        	//emplaceLandmark
//        	LandmarkBasePtr lmk = emplaceLandmark(feature_origin);
//        	LandmarkHpPtr lmk_hp = std::static_pointer_cast<LandmarkHp>(lmk);
//
//        	//add only one Landmark to map: map[track_id] = landmarkptr
//        	lmk_track_map_[trkid] = lmk;
//
//        	//emplace a factor for each feature in the track (only for keyframes)
//        	Track full_track = track_matrix_.trackAtKeyframes(trkid);
//        	for (auto it=full_track.begin(); it!=full_track.end(); ++it)
//        	{
//        		FactorBase::emplace<FactorPixelHp>(it->second, it->second, lmk_hp, shared_from_this());
//        	}
//
//        }
//        else
//        {
//        	LandmarkBasePtr lmk = lmk_track_map_[trkid];
//        	LandmarkHpPtr lmk_hp = std::static_pointer_cast<LandmarkHp>(lmk);
//
//        	//Create factor
//        	FactorBase::emplace<FactorPixelHp>(feature_last, feature_last, lmk_hp, shared_from_this());
//        }
//
//    }



	for (auto & pair_id_ftr : track_matrix_.snapshot(last_ptr_))
	{
		auto & trkid = pair_id_ftr.first;
		auto & ftr = pair_id_ftr.second;

		if (lmk_track_map_.count(trkid)==0) //if the track doesn't have landmark associated -> we need a map: map[track_id] = landmarkptr
		{
			//emplaceLandmark
			LandmarkBasePtr lmk = emplaceLandmark(ftr);
			LandmarkHpPtr lmk_hp = std::static_pointer_cast<LandmarkHp>(lmk);

			//add only one Landmark to map: map[track_id] = landmarkptr
			lmk_track_map_[trkid] = lmk;

			//emplace a factor
			FactorBase::emplace<FactorPixelHp>(ftr, ftr, lmk_hp, shared_from_this(), params_->apply_loss_function);

		}
		else
		{
			// recover the landmark
			LandmarkBasePtr lmk = lmk_track_map_[trkid];
			LandmarkHpPtr lmk_hp = std::static_pointer_cast<LandmarkHp>(lmk);

			//emplace a factor
			FactorBase::emplace<FactorPixelHp>(ftr, ftr, lmk_hp, shared_from_this(), params_->apply_loss_function);
		}

	}
}

void ProcessorBundleAdjustment::setParams(const ParamsProcessorBundleAdjustmentPtr _params)
{
    params_bundle_adjustment_ = _params;
}

ProcessorBasePtr ProcessorBundleAdjustment::create(const std::string& _unique_name,
                                                         const ParamsProcessorBasePtr _params)
{
  const auto params = std::static_pointer_cast<ParamsProcessorBundleAdjustment>(_params);

  ProcessorBasePtr prc_ptr = std::make_shared<ProcessorBundleAdjustment>(params);
  prc_ptr->setName(_unique_name);
  return prc_ptr;
}

} //namespace wolf

// Register in the FactoryProcessor
#include "core/processor/factory_processor.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR(ProcessorBundleAdjustment)
} // namespace wolf

