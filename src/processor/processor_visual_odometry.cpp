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
    
    // PARAMS KLT tracker
    tracker_width_ = 21;
    tracker_height_ = 21;
    nlevels_pyramids_klt_ = 3;
    klt_max_err_ = 0.2;
    crit_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    //////////////////

    int threshold_fast = 30;
    detector_ = cv::FastFeatureDetector::create(threshold_fast);

    pixel_cov_ = Eigen::Matrix2d::Identity();

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
    capture_image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);
    assert(capture_image_incoming_ != nullptr && ("Capture type mismatch. Processor " + getName() + " can only process captures of type CaptureImage").c_str());

    cv::Mat img_incoming = capture_image_incoming_->getImage();
    std::cout << "img_incoming size: " << img_incoming.size().width << ", " << img_incoming.size().height << std::endl;

    // Time to PREPreprocess the image if necessary: greyscale if BGR, CLAHE etc...
    // once preprocessing is done, replace the original image (?)

    // if first image, compute keypoints, add to capture incoming and return
    if (last_ptr_ == nullptr){
        std::cout << "last_ptr is nullptr, should be the case only once" << std::endl;
        // size_t nb_detect = 100;
        std::vector<cv::KeyPoint> kps_current;

        detector_->detect(img_incoming, kps_current);
        capture_image_incoming_->addKeyPoints(kps_current);
        
        return;
    }


    ////////////////////////
    ////////////////////////
    // TRACKING
    // Proceeed to tracking the previous features
    capture_image_last_ = std::static_pointer_cast<CaptureImage>(last_ptr_);
    cv::Mat img_last = capture_image_last_->getImage();

    KeyPointsMap mwkps_last = capture_image_last_->getKeyPoints();
    KeyPointsMap mwkps_incoming;  // init incoming


    ////////////////////////////////
    // FEATURE TRACKING
    // Update capture Incoming data
    //   - KeyPoints
    //   - tracks wrt. origin and last
    //   - descriptor
    //   - ...
    ////////////////////////////////

    ////////////////////////////////
    // if too few tracks left in incoming
    // detect new KeyPoints in last and track them to incoming
    ////////////////////////////////


    ////////////////////////////////
    // FAKE DATA, to replace
    // Create 1 fake detections each time that should be tracked over time
    cv::KeyPoint kp0 = cv::KeyPoint(1.0, 2.0, 0.0);
    WKeyPoint wkp0 = WKeyPoint(kp0);
    capture_image_incoming_->addKeyPoint(wkp0);

    size_t id_last_kp = wkp0.getId()-1;  // -1 is a HACK!!
    size_t id_incoming_kp = wkp0.getId();
    std::cout << "\nCreate Fake track " << id_last_kp << " -> " << id_incoming_kp << std::endl;
    auto tracks_map_li = std::pair<size_t, size_t>(id_last_kp, id_incoming_kp);

    auto temp = capture_image_incoming_->getTracksPrev();  // cannot "insert" here since getTracksPrev is "const" -> create temp variable
    temp.insert(tracks_map_li);


    ////////////////////////////////

}


unsigned int ProcessorVisualOdometry::processKnown()
{
    std::cout << "processKnown" << std::endl;
    // reinitilize the bookeeping to communicate info from processKnown to processNew
    tracks_map_li_matched_.clear();
    // Extend the process track matrix by using information stored in the incomping capture

    // Get tracks present at the last capture time (should be the most recent snapshot at this moment)
    std::list<FeatureBasePtr> tracks_snapshot_last = track_matrix_.snapshotAsList(last_ptr_);

    for (auto feature_tracked_last: tracks_snapshot_last){
        // check if the keypoint in the last capture is in the last->incoming TracksMap stored in the incoming capture
        FeaturePointImagePtr feat_pi_last = std::dynamic_pointer_cast<FeaturePointImage>(feature_tracked_last);
        size_t id_feat_last = feat_pi_last->getKeyPoint().getId();  
        std::cout << "id_feat_last: " << id_feat_last << std::endl;
        // if this feature id is in the last->incoming tracks of capture incoming, the track is continued
        // otherwise we store the pair as a newly detected track (for processNew)
        TracksMap tracks_map_li = capture_image_incoming_->getTracksPrev();
        if (tracks_map_li.count(id_feat_last)){
            std::cout << "A corresponding track has been found for id_feat_last " << id_feat_last  << std::endl;
            auto kp_track_li = tracks_map_li.find(id_feat_last);
            // create a feature using the corresponding WKeyPoint contained in incoming (hence the "second")
            auto feat_inco = FeatureBase::emplace<FeaturePointImage>(
                                                    capture_image_incoming_, 
                                                    capture_image_incoming_->getKeyPoints().at(kp_track_li->second), 
                                                    pixel_cov_);
            track_matrix_.add(feat_pi_last->trackId(), feat_inco);

            // add tracks_map_li to a vector so that it so that 
            // Very BAD to have to create a new std pair here -> find a better syntax but that's the idead
            auto kp_track_li_matched = std::pair<size_t, size_t>(kp_track_li->first, kp_track_li->second);
            tracks_map_li_matched_.insert(kp_track_li_matched);
        }
    }

    return 42;
}


unsigned int ProcessorVisualOdometry::processNew(const int& _max_features)
{
    ///////////////////////////////////////
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // problem in the logic for the moment:
    // this function will
    ///////////////////////////////////////

    std::cout << "processNew" << std::endl;
    // We have matched the tracks in the track matrix with the last->incoming tracks 
    // stored in the TracksMap from getTracksPrev()
    // Now we need to add new tracks in the track matrix for the NEW tracks.
    //
    // use book-keeping prepared in processKnown: the TracksMap that have been matched were stored in tracks_map_li_matched_
    // and here add tracks only for those that have not been matched

    for (std::pair<size_t,size_t> track_li: capture_image_incoming_->getTracksPrev()){
        // if track not matched, then create a new track in the track matrix etc.
        std::cout << "In incoming, track: " << track_li.first << " -> " << track_li.second << std::endl;
        if (!tracks_map_li_matched_.count(track_li.first)){
            std::cout << "A NEW track is born!" << std::endl;
            // create a new last feature, a new track and add the incoming feature to this track
            WKeyPoint kp_last = capture_image_last_->getKeyPoints().at(track_li.first);
            FeaturePointImagePtr feat_pi_last = FeatureBase::emplace<FeaturePointImage>(capture_image_last_, kp_last, pixel_cov_);
            track_matrix_.newTrack(feat_pi_last);
        }
    }

    return 42;
}





void ProcessorVisualOdometry::establishFactors()
{
    // Function only called when KF is created using last
    // Loop over the snapshot in corresponding to last capture. Does 2 things:
    //     1) for tracks already associated to a landmark, create a KF-Lmk factor between the last KF and the landmark.
    //     2) if the feature track is not associated to a landmark yet and is long enough, create a new landmark
    //        using triangulation as a prior, using previous KF current estimates. Create a KF-Lmk factor for all these KFs. 
    //        For bookkeeping, define the landmark id as the track id.

    int min_track_length = 5;  // minimum track length for it to be considered a valid track
    std::list<FeatureBasePtr> tracks_snapshot_last = track_matrix_.snapshotAsList(last_ptr_);
    for (auto feat: tracks_snapshot_last){
        auto feat_pi = std::static_pointer_cast<FeaturePointImage>(feat);

        // verify if a landmark is associated to this track (BAD implementation)
        LandmarkBasePtr associated_lmk = nullptr;
        for (auto lmk: getProblem()->getMap()->getLandmarkList()){
            if (lmk->id() == feat_pi->trackId()){
                associated_lmk = lmk;
                break;
            }
        }

        // 1) create a factor between new KF and assocatiated track landmark
        //    HYP: assuming the trackid are the same as the landmark ID -> BAD if other types of landmarks involved
        if (associated_lmk){
            LandmarkHpPtr associated_lmk_hp = std::dynamic_pointer_cast<LandmarkHp>(associated_lmk);
            FactorBase::emplace<FactorPixelHp>(feat_pi, feat_pi, associated_lmk_hp, shared_from_this(), true);
        }

        // 2) create landmark if track is not associated with one and has enough length
        else if(track_matrix_.trackSize(feat->trackId()) >= min_track_length){
            std::cout << "NEW valid track \\o/ -> create a new Landmark" << std::endl;
            LandmarkBasePtr lmk = emplaceLandmark(feat_pi);
            // Associate the track to the landmark
            // NOT the right way -> should be replaced by book-keeping (in the processor? Feature?)
            // maybe call feature->setLandmarkId() on all the features of the track?
            lmk->setId(feat_pi->trackId());  
            LandmarkHpPtr lmk_hp = std::dynamic_pointer_cast<LandmarkHp>(lmk);
            for (auto feat_kf: track_matrix_.trackAtKeyframes(feat->trackId())){
                FactorBase::emplace<FactorPixelHp>(feat_kf.second, feat_kf.second, lmk_hp, shared_from_this(), true);
            }
        }
    }
}

LandmarkBasePtr ProcessorVisualOdometry::emplaceLandmark(FeatureBasePtr _feat)
{
    // Taken from processor_bundle_adjustment
    // Initialize the landmark in its ray (based on pixel meas) and using a arbitrary distance

    FeaturePointImagePtr feat_pi = std::static_pointer_cast<FeaturePointImage>(_feat);
    Eigen::Vector2d point2d = _feat->getMeasurement();

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
        = Translation<double,3>(feat_pi->getFrame()->getP()->getState())
        * Quaterniond(_feat->getFrame()->getO()->getState().data());
    Transform<double,3,Isometry> T_r_c
		= Translation<double,3>(_feat->getCapture()->getSensorP()->getState())
        * Quaterniond(_feat->getCapture()->getSensorO()->getState().data());
    Eigen::Matrix<double, 4, 1> vec_homogeneous_w = T_w_r
                                           * T_r_c
                                           * vec_homogeneous_c;

    auto lmk_hp_ptr = LandmarkBase::emplace<LandmarkHp>(getProblem()->getMap(), 
                                                        vec_homogeneous_w, 
                                                        getSensor(), 
                                                        feat_pi->getKeyPoint().getDescriptor());

    return lmk_hp_ptr;
}


void ProcessorVisualOdometry::postProcess()
{
    frame_count_ ++;
}

bool ProcessorVisualOdometry::voteForKeyFrame() const
{
    std::cout << "VOTE" << std::endl;
    // simple vote based on frame count, should be changed to something that takes into account number of tracks alive, parallax, etc.
    return ((frame_count_ % 5) == 0);
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

TracksMap ProcessorVisualOdometry::klt_track(cv::Mat img_prev, cv::Mat img_curr, KeyPointsMap &mwkps_prev, KeyPointsMap &mwkps_curr)
{
    TracksMap tracks_prev_curr;

    // Create cv point list for tracking, we initialize optical flow with previous keypoints
    // We also need a list of indices to build the track map
    std::vector<cv::Point2f> p2f_prev;
    std::vector<size_t> indices_prev;
    for (auto & wkp : mwkps_prev){
        p2f_prev.push_back(wkp.second.getCvKeyPoint().pt);
        indices_prev.push_back(wkp.first);
    }
    std::vector<cv::Point2f> p2f_curr = p2f_prev;

    // Configure and process KLT optical flow research
    std::vector<uchar> status;
    std::vector<float> err;


    // Process one way: previous->current with current init with previous
    cv::calcOpticalFlowPyrLK(
            img_prev,
            img_curr, 
            p2f_prev,
            p2f_curr,
            status, err,
            {search_width_, search_height_}, 
            pyramid_level_,
            crit_,
            (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));
    
    // Process the other way: current->previous
    std::vector<uchar> status_back;
    std::vector<float> err_back;
    cv::calcOpticalFlowPyrLK(
            img_curr,
            img_prev, 
            p2f_curr,
            p2f_prev,
            status_back, err_back,
            {search_width_, search_height_}, 
            pyramid_level_,
            crit_,
            (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));
    
    // Delete point if KLT failed
    for (size_t j = 0; j < status_back.size(); j++) {

        if(!status_back.at(j)  ||  (err_back.at(j) > klt_max_err_) ||
           !status.at(j)  ||  (err.at(j) > klt_max_err_)) {
            continue;
        }

        // We keep the initial point and add the tracked point
        WKeyPoint wkp(cv::KeyPoint(p2f_curr.at(j), 1));
        mwkps_curr[wkp.getId()] = wkp;

        // Update the map
        tracks_prev_curr[indices_prev.at(j)] = wkp.getId();

        // Other checks? Distance between track points?
    }

    return tracks_prev_curr;
}


} //namespace wolf

// Register in the FactoryProcessor
#include "core/processor/factory_processor.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR(ProcessorVisualOdometry)
WOLF_REGISTER_PROCESSOR_AUTO(ProcessorVisualOdometry)
} // namespace wolf

