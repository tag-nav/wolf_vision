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

ProcessorBundleAdjustment::ProcessorBundleAdjustment(ProcessorParamsBundleAdjustmentPtr _params_bundle_adjustment) :
                ProcessorTrackerFeature("TRACKER BUNDLE ADJUSTMENT", _params_bundle_adjustment),
                params_bundle_adjustment_(_params_bundle_adjustment),
                capture_image_last_(nullptr),
                capture_image_incoming_(nullptr)
{

	pixel_cov_ = Eigen::Matrix2s::Identity() * params_bundle_adjustment_->pixel_noise_std * params_bundle_adjustment_->pixel_noise_std;

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

}

void ProcessorBundleAdjustment::configure(SensorBasePtr _sensor)
{
    //TODO: Implement if needed
}

void ProcessorBundleAdjustment::preProcess()
{
    // This method implements all Vision algorithms concerning OpenCV, so wolf algorithm only has to manage the data obtained
    // Get Capture
    capture_image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);
    // Detect KeyPoints
    capture_image_incoming_->keypoints_ = det_ptr_->detect(capture_image_incoming_->getImage());

    //Sort keypoints by response if needed
    if (!capture_image_incoming_->keypoints_.empty())
    	vision_utils::sortByResponse(capture_image_incoming_->keypoints_, CV_SORT_DESCENDING);

    // Compute Descriptors
    capture_image_incoming_->descriptors_ = des_ptr_->getDescriptor(capture_image_incoming_->getImage(), capture_image_incoming_->keypoints_);

    std::cout << "before fill grid " << std::endl;

    // Create and fill incoming grid
    capture_image_incoming_->grid_features_ = std::make_shared<vision_utils::FeatureIdxGrid>(capture_image_incoming_->getImage().rows, capture_image_incoming_->getImage().cols, params_bundle_adjustment_->n_cells_v, params_bundle_adjustment_->n_cells_h);

    capture_image_incoming_->grid_features_->insert(capture_image_incoming_->keypoints_);

    std::cout << "after fill grid done " << std::endl;


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

    std::cout << "preProcess() done " << std::endl;

}

void ProcessorBundleAdjustment::postProcess()
{
    std::map<int,std::list<vision_utils::KeyPointEnhanced> > kp_enh_tracks;

    for (auto const & feat_base : last_ptr_->getFeatureList())
    {
        FeaturePointImagePtr feat = std::static_pointer_cast<FeaturePointImage>(feat_base);
        unsigned int feat_id = feat->id();
        unsigned int track_id = feat->trackId();

        // tracks
        std::list<vision_utils::KeyPointEnhanced> kp_enh_track;
        for (auto feat_base : track_matrix_.trackAsVector(track_id))
        {
            FeaturePointImagePtr feat_img = std::static_pointer_cast<FeaturePointImage>(feat_base);
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
    image_debug_ = vision_utils::buildImageProcessed((std::static_pointer_cast<CaptureImage>(last_ptr_))->getImage(), kp_enh_tracks);

    Snapshot snapshot_last = track_matrix_.snapshot(last_ptr_);
//	getProblem()->print(4,1,1,0);
    for (auto pair : track_matrix_.snapshot(origin_ptr_))
    {
    	if (snapshot_last.count(pair.first)==0)
    	{
    		if (!(pair.second->getFactorList()).empty())
    		{
				auto factor = pair.second->getFactorList().front();
				if (factor)
				{
					auto lmk = factor->getLandmarkOther();
					if (lmk)
					{
//						lmk->remove();
//	    	    		track_matrix_.remove(pair.second->trackId());
					}
				}
    		}
    	}
    }
	getProblem()->print(4,1,1,0);

    list<FeatureBasePtr> snapshot = track_matrix_.snapshotAsList(last_ptr_);
    for (auto const & feat_base : snapshot)
    {
        FeaturePointImagePtr feat = std::static_pointer_cast<FeaturePointImage>(feat_base);
        unsigned int track_id = feat->trackId();
		if (lmk_track_map_.count(track_id))
        {
			Vector3s point3D = std::static_pointer_cast<LandmarkHP>(lmk_track_map_[track_id])->point();
		    Transform<Scalar,3,Isometry> T_w_r
		        = Translation<Scalar,3>(feat_base->getFrame()->getP()->getState())
		        * Quaternions(feat_base->getFrame()->getO()->getState().data());
		    Transform<Scalar,3,Isometry> T_r_c
				= Translation<Scalar,3>(feat_base->getCapture()->getSensorP()->getState())
		        * Quaternions(feat_base->getCapture()->getSensorO()->getState().data());
		    Eigen::Matrix<Scalar, 3, 1> point3D_c = T_r_c.inverse()
												   * T_w_r.inverse()
		                                           * point3D;
    		SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(getSensor());
    		Vector2s point2D = pinhole::projectPoint(getSensor()->getIntrinsic()->getState(), camera->getDistortionVector(), point3D_c);
    		cv::Point point = cv::Point(point2D(0), point2D(1));
    		cv::circle(image_debug_, point, 3, cv::Scalar(0,0,255) , 1 , 8);
		}
	}
    cv::namedWindow("DEBUG VIEW", cv::WINDOW_NORMAL);
    cv::resizeWindow("DEBUG VIEW", 800,600);
    cv::imshow("DEBUG VIEW", image_debug_);
    cv::waitKey(1);
}


unsigned int ProcessorBundleAdjustment::trackFeatures(const FeatureBasePtrList& _features_last_in, FeatureBasePtrList& _features_incoming_out, FeatureMatchMap& _feature_correspondences)
{
    for (auto feat_base: _features_last_in)
    {
        FeaturePointImagePtr feat_last = std::static_pointer_cast<FeaturePointImage>(feat_base);

        if (capture_image_last_->map_index_to_next_.count(feat_last->getIndexKeyPoint())) //If the track to incoming exists
        {
            int index_inc = capture_image_last_->map_index_to_next_.at(feat_last->getIndexKeyPoint());

            if (true)//capture_image_incoming_->matches_normalized_scores_.at(index_inc) > mat_ptr_->getParams()->min_norm_score )
            {
                // Get kp incoming and keypoint last
                cv::KeyPoint kp_inc = capture_image_incoming_->keypoints_.at(index_inc);
//                cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(feat_last->getIndexKeyPoint());

//                if (isInlier(kp_last, kp_inc))
//                {
					FeaturePointImagePtr feat_inc = std::make_shared<FeaturePointImage>(kp_inc, index_inc,
																							capture_image_incoming_->descriptors_.row(index_inc),
																							pixel_cov_);

					_features_incoming_out.push_back(feat_inc);

					auto feature_match_ptr = std::make_shared<FeatureMatch>();
					feature_match_ptr->feature_ptr_= feat_last;
					feature_match_ptr->normalized_score_ = 1.0;//capture_image_incoming_->matches_normalized_scores_.at(index_inc);
					_feature_correspondences[feat_inc] = feature_match_ptr;


					// hit cell to acknowledge there's a tracked point in that cell
					capture_image_incoming_->grid_features_->hitTrackingCell(kp_inc);
//                }
            }
        }
    }
//    return _feature_correspondences.size();
    return _features_incoming_out.size();
}

bool ProcessorBundleAdjustment::isInlier(const cv::KeyPoint& _kp_last, const cv::KeyPoint& _kp_incoming)
{
    // List of conditions
    bool inlier = true;

    // A. Check euclidean norm
    inlier = inlier && (cv::norm(_kp_incoming.pt-_kp_last.pt) < mat_ptr_->getParams()->max_match_euclidean_dist);

    // B. add your new condition test here
    // inlier = inlier && ...

    return inlier;
}

bool ProcessorBundleAdjustment::is_tracked(int& _kp_idx)
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

unsigned int ProcessorBundleAdjustment::detectNewFeatures(const int& _max_new_features, FeatureBasePtrList& _features_last_out)
{
    //TODO: efficient implementation?

////Simpler method to detect new features without using Grid Features form vision_utils
//    typedef std::map<int,int>::iterator iter;
//
//    for (iter it = capture_image_last_->map_index_to_next_.begin(); it!=capture_image_last_->map_index_to_next_.end(); ++it)
//    {
//        if (_features_last_out.size() >= _max_new_features)
//        		break;
//
//        else if(!is_tracked(it->second))
//        {
//            //add to _features_last_out
//            int idx_last = it->first;
//            cv::KeyPoint kp_last = capture_image_last_->keypoints_.at(idx_last);
//            FeaturePointImagePtr feat_last_ = std::make_shared<FeaturePointImage>(kp_last, idx_last,
//                                                                                 capture_image_last_->descriptors_.row(idx_last),
//                                                                                 pixel_cov_);
//            _features_last_out.push_back(feat_last_);
//
//        }
//    }
//
//    return _features_last_out.size();

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
//                  	      unsigned int index_incoming = capture_image_last_->map_index_to_next_[index_last];
//                      	  cv::KeyPoint kp_incoming = capture_image_incoming_->keypoints_.at(index_incoming);

					// validate match with extra tests
//					if (isInlier( kp_incoming, kp_last))
//					{
						// Create WOLF feature
						FeaturePointImagePtr ftr_point_last = std::make_shared<FeaturePointImage>(
								kp_last,
								index_last,
								capture_image_last_->descriptors_.row(index_last),
								pixel_cov_);

						_features_last_out.push_back(ftr_point_last);

						// hit cell to acknowledge there's a tracked point in that cell
						capture_image_last_->grid_features_->hitTrackingCell(kp_last);

						found_feature_in_cell = true;

						break; // Good kp found for this grid.
//                    }
				}
			}
			if (!found_feature_in_cell)
				capture_image_last_->grid_features_->blockTrackingCell(cell_last);
		}
		else
			break; // There are no empty cells
	}
	return _features_last_out.size();
}


bool ProcessorBundleAdjustment::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    //TODO: Implement if needed
    return true;
}

bool ProcessorBundleAdjustment::voteForKeyFrame()
{
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


FactorBasePtr ProcessorBundleAdjustment::createFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
    /* This function is no creating any factor.
     *  We create factors with establishFactors()
     */
    return FactorBasePtr();
}

LandmarkBasePtr ProcessorBundleAdjustment::createLandmark(FeatureBasePtr _feature_ptr)
{
    FeaturePointImagePtr feat_point_image_ptr = std::static_pointer_cast<FeaturePointImage>( _feature_ptr);
    Eigen::Vector2s point2D = _feature_ptr->getMeasurement();

    Eigen::Vector3s point3D;
    point3D = pinhole::backprojectPoint(
    		getSensor()->getIntrinsic()->getState(),
    		(std::static_pointer_cast<SensorCamera>(getSensor()))->getCorrectionVector(),
			point2D);



    //Scalar distance = params_bundle_adjustment_->distance; // arbitrary value
    Scalar distance = 1;
    Eigen::Vector4s vec_homogeneous_c;
    vec_homogeneous_c = {point3D(0),point3D(1),point3D(2),point3D.norm()/distance};
    vec_homogeneous_c.normalize();

    //TODO: lmk from camera to world coordinate frame.
    Transform<Scalar,3,Isometry> T_w_r
        = Translation<Scalar,3>(_feature_ptr->getFrame()->getP()->getState())
        * Quaternions(_feature_ptr->getFrame()->getO()->getState().data());
    Transform<Scalar,3,Isometry> T_r_c
		= Translation<Scalar,3>(_feature_ptr->getCapture()->getSensorP()->getState())
        * Quaternions(_feature_ptr->getCapture()->getSensorO()->getState().data());
    Eigen::Matrix<Scalar, 4, 1> vec_homogeneous_w = T_w_r
                                           * T_r_c
                                           * vec_homogeneous_c;


//    std::cout << "Point2D: " << point2D.transpose() << std::endl;
//    std::cout << "Point3D: " << point3D.transpose() << std::endl;
//    std::cout << "Hmg_c: " << vec_homogeneous.transpose() << std::endl;
//    std::cout << "Hmg_w: " << vec_homogeneous_w.transpose() << std::endl;
    //vec_homogeneous_w = vec_homogeneous;

    LandmarkBasePtr lmk_hp_ptr = LandmarkBase::emplace<LandmarkHP>(getProblem()->getMap(), vec_homogeneous_w, getSensor(), feat_point_image_ptr->getDescriptor());

//    std::cout << "LANDMARKS CREATED AND ADDED to MAP: " << getProblem()->getMap()->getLandmarkList().size() << std::endl;

    _feature_ptr->setLandmarkId(lmk_hp_ptr->id());
    return lmk_hp_ptr;
}

void ProcessorBundleAdjustment::establishFactors()
{

	if (origin_ptr_ == last_ptr_)
		return;


	TrackMatches matches_origin_inc = track_matrix_.matches(origin_ptr_, last_ptr_);

    for (auto const & pair_trkid_pair : matches_origin_inc)
    {
        size_t trkid = pair_trkid_pair.first;
        if (track_matrix_.trackSize(trkid)<params_bundle_adjustment_->min_track_length_for_factor)
        	continue;
        FeatureBasePtr feature_origin = pair_trkid_pair.second.first;
        FeatureBasePtr feature_last   = pair_trkid_pair.second.second;

        assert(feature_origin!=feature_last && "feature: origin equal last");
        assert(feature_origin->getCapture()!=feature_last->getCapture() && "capture: origin equal last");

        if (lmk_track_map_.count(trkid)==0) //if the track doesn't have landmark associated -> we need a map: map[track_id] = landmarkptr
        {
        	//createLandmark
        	LandmarkBasePtr lmk = createLandmark(feature_origin);
        	assert(lmk!=nullptr);
        	LandmarkHPPtr lmk_hp = std::static_pointer_cast<LandmarkHP>(lmk);
        	//add Landmark to map: map[track_id] = landmarkptr
        	lmk_track_map_[trkid] = lmk;

        	//create factors
//        	assert(feature_origin->getCapture()->getSensor() != nullptr);
//        	assert(feature_origin->getCapture()->getSensorP() != nullptr);
//        	assert(feature_origin->getCapture()->getSensorO() != nullptr);
//        	assert(lmk->getP() != nullptr);
        	FactorBase::emplace<FactorPixelHP>(feature_origin, feature_origin, lmk_hp, shared_from_this());
        	FactorBase::emplace<FactorPixelHP>(feature_last, feature_last, lmk_hp, shared_from_this());
        }
        else
        {
        	LandmarkBasePtr lmk = lmk_track_map_[trkid];
        	LandmarkHPPtr lmk_hp = std::static_pointer_cast<LandmarkHP>(lmk);

        	//Create factor
        	FactorBase::emplace<FactorPixelHP>(feature_last, feature_last, lmk_hp, shared_from_this());
        }
    }
}

void ProcessorBundleAdjustment::setParams(const ProcessorParamsBundleAdjustmentPtr _params)
{
    params_bundle_adjustment_ = _params;
}

ProcessorBasePtr ProcessorBundleAdjustment::create(const std::string& _unique_name,
                                                         const ProcessorParamsBasePtr _params,
                                                         const SensorBasePtr _sensor_ptr)
{
  const auto params = std::static_pointer_cast<ProcessorParamsBundleAdjustment>(_params);

  ProcessorBasePtr prc_ptr = std::make_shared<ProcessorBundleAdjustment>(params);
  prc_ptr->setName(_unique_name);
  prc_ptr->configure(_sensor_ptr);
  return prc_ptr;
}

} //namespace wolf

// Register in the ProcessorFactory
#include "core/processor/processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("TRACKER BUNDLE ADJUSTMENT", ProcessorBundleAdjustment)
} // namespace wolf

