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

#include <opencv2/core/eigen.hpp>

#include <chrono>
#include <ctime>

namespace wolf{

using namespace SE3;

ProcessorVisualOdometry::ProcessorVisualOdometry(ParamsProcessorVisualOdometryPtr _params_vo) :
                ProcessorTracker("ProcessorVisualOdometry", "PO", 3, _params_vo),
                MotionProvider("PO", _params_vo),
                params_visual_odometry_(_params_vo)
{
    // Preprocessor stuff
    detector_ = cv::FastFeatureDetector::create(_params_vo->fast.threshold,
                                                _params_vo->fast.non_max_suppresion,
                                                cv::FastFeatureDetector::TYPE_9_16); // TYPE_5_8, TYPE_7_12, TYPE_9_16
    
    // Processor stuff
    // Set pixel noise covariance
    Eigen::Vector2d std_pix; std_pix << params_visual_odometry_->std_pix, params_visual_odometry_->std_pix;
    pixel_cov_ = std_pix.array().square().matrix().asDiagonal();

}

void ProcessorVisualOdometry::configure(SensorBasePtr _sensor)
{
	//Initialize camera sensor pointer
	sen_cam_ = std::static_pointer_cast<SensorCamera>(_sensor);
    
    // CV type for intrinsic matrix
    cv::eigen2cv(sen_cam_->getIntrinsicMatrix(), Kcv_);
    
    // Tessalation of the image
    cell_grid_ = ActiveSearchGrid(sen_cam_->getImgWidth(), sen_cam_->getImgHeight(),
                                  params_visual_odometry_->grid.nbr_cells_h,
                                  params_visual_odometry_->grid.nbr_cells_v,
                                  params_visual_odometry_->grid.margin,
                                  params_visual_odometry_->grid.separation);
}

void ProcessorVisualOdometry::preProcess()
{

    auto t1 = std::chrono::system_clock::now();
    
    // Get Capture
    capture_image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);

    cv::Mat img_incoming = capture_image_incoming_->getImage();

    equalize_img(img_incoming);


    // if first image, compute keypoints, add to capture incoming and return
    if (last_ptr_ == nullptr){
        detect_keypoints_empty_grid(img_incoming, capture_image_incoming_);

        WOLF_DEBUG( "Initially detected " , capture_image_incoming_->getKeyPoints().size(), " keypoints in incoming" );

        // Initialize the tracks data structure with a "dummy track" where the keypoint is pointing to itself
        TracksMap tracks_init;
        for (auto mwkp : capture_image_incoming_->getKeyPoints()){
            tracks_init[mwkp.first] = mwkp.first;
        }
        capture_image_incoming_->setTracksOrigin(tracks_init);
        capture_image_incoming_->setTracksPrev(tracks_init);

        auto __attribute__((unused)) dt_preprocess = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t1).count();
        WOLF_DEBUG( "dt_preprocess (ms): " , dt_preprocess );

        return;
    }


    ////////////////////////////////
    // FEATURE TRACKING
    // Update capture Incoming data
    //   - Track keypoints last->incoming
    //   - Merge tracks origin->last with last->incoming to get origin->incoming
    //   - Outlier rejection origin->incoming (essential matrix)
    //   - If too few keypoints in incoming, detect new keypoints and last and track them in last->incoming
    //   - All the results are stored in incoming capture for later Tree Processing
    ////////////////////////////////

    capture_image_last_     = std::static_pointer_cast<CaptureImage>(last_ptr_);
    capture_image_origin_   = std::static_pointer_cast<CaptureImage>(origin_ptr_);
    cv::Mat img_last        = capture_image_last_->getImage();

    KeyPointsMap mwkps_origin   = capture_image_origin_ ->getKeyPoints();
    KeyPointsMap mwkps_last     = capture_image_last_   ->getKeyPoints();
    KeyPointsMap mwkps_incoming;  // init incoming

    WOLF_DEBUG( "Tracking " , mwkps_last.size(), " keypoints in last" );

    // TracksMap between last and incoming
    // Input: ID of Wkp in last. Output: ID of the tracked Wkp in incoming.
    TracksMap tracks_last_incoming = kltTrack(img_last, img_incoming, mwkps_last, mwkps_incoming);

    // TracksMap between origin and last
    // Input: ID of Wkp in origin. Output: ID of the tracked Wkp in last.
    TracksMap tracks_origin_last = capture_image_last_->getTracksOrigin();

    // Merge tracks to get TracksMap between origin and incoming
    // Input: ID of Wkp in origin. Output: ID of the tracked Wkp in incoming.
    TracksMap tracks_origin_incoming = mergeTracks(tracks_origin_last, tracks_last_incoming);

    // Outliers rejection with essential matrix
    cv::Mat E;
    VectorComposite pose_ol("PO", {3,4});
    int __attribute__((unused)) track_nb_before = tracks_origin_incoming.size();
    bool success = filterWithEssential(mwkps_origin, mwkps_incoming, tracks_origin_incoming, E, pose_ol);
    WOLF_DEBUG("After RANSAC: ", track_nb_before, "->", tracks_origin_incoming.size())
    if (success){
        // store result of recoverPose if RANSAC was handled well
        buffer_pose_cam_ol_.emplace(origin_ptr_->getTimeStamp(), std::make_shared<VectorComposite>(pose_ol));
    }
    else{
        WOLF_WARN("!!!!!!Essential matrix RANSAC failed!!!!!!!!");
        // return;  // What should we do in this case? 
    }

    // Edit tracks prev with only inliers wrt origin
    // and remove also from mwkps_incoming all the keypoints that have not been tracked
    TracksMap tracks_last_incoming_filtered;
    KeyPointsMap mwkps_incoming_filtered;
    filter_last_incoming_tracks_from_ransac_result(tracks_last_incoming, mwkps_incoming, tracks_origin_incoming,
                                                   tracks_last_incoming_filtered, mwkps_incoming_filtered);
    WOLF_DEBUG( "Tracked " , mwkps_incoming_filtered.size(), " inliers in incoming" );


    ////////////////////////////////
    // if too few tracks left in incoming
    // detect new KeyPoints in last and track them to incoming
    ////////////////////////////////
    size_t n_tracks_origin = tracks_origin_incoming.size();


    if (n_tracks_origin < params_visual_odometry_->min_features_for_keyframe)
    {

        std::vector<cv::KeyPoint> kps_last_new;
        KeyPointsMap mwkps_last_filtered;
        detect_keypoints_in_empty_grid_cells(img_last, tracks_last_incoming_filtered, mwkps_last,
                                             kps_last_new, mwkps_last_filtered);

        // Create a map of wolf KeyPoints to track only the new ones
        KeyPointsMap mwkps_last_new, mwkps_incoming_new;
        for (auto & cvkp : kps_last_new){
            WKeyPoint wkp(cvkp);
            mwkps_last_new[wkp.getId()] = wkp;
        } 
        WOLF_DEBUG("Detected ", mwkps_last_new.size(), " new keypoints in last");

        TracksMap tracks_last_incoming_new = kltTrack(img_last, img_incoming, mwkps_last_new, mwkps_incoming_new);

        WOLF_DEBUG("Tracked ", mwkps_incoming_new.size(), " inliers in incoming");

        // Concatenation of old tracks and new tracks
        for (auto & track: tracks_last_incoming_new){
            tracks_last_incoming_filtered[track.first] = track.second;
            mwkps_last_filtered[track.first] = mwkps_last_new[track.first];
            mwkps_incoming_filtered[track.second] = mwkps_incoming_new[track.second];
        }

        // Outliers rejection with essential matrix
        // tracks that are not geometrically consistent are removed from tracks_last_incoming_new
        VectorComposite not_used;
        filterWithEssential(mwkps_last_filtered, mwkps_incoming_filtered, tracks_last_incoming_filtered, E, not_used);

        WOLF_DEBUG("New total : ", n_tracks_origin, " + ", mwkps_incoming_new.size(), " = ", tracks_last_incoming_filtered.size(), " tracks");

        // add a flag so that voteForKeyFrame use it to vote for a KeyFrame 
        capture_image_incoming_->setLastWasRepopulated(true);

        // Update captures
        capture_image_last_->addKeyPoints(mwkps_last_new);

    }

    // Update captures
    capture_image_incoming_->addKeyPoints(mwkps_incoming_filtered);
    capture_image_incoming_->setTracksPrev(tracks_last_incoming_filtered);
    capture_image_incoming_->setTracksOrigin(tracks_origin_incoming);

    auto __attribute__((unused)) dt_preprocess = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t1).count();
    WOLF_DEBUG( "dt_preprocess (ms): " , dt_preprocess );

    return;
}


TracksMap ProcessorVisualOdometry::mergeTracks(const TracksMap& tracks_prev_curr, const TracksMap& tracks_curr_next){
    // merge tracks prev->curr and curr->next to get prev->next
    TracksMap tracks_prev_next;
    for (auto &match : tracks_prev_curr){
        if (tracks_curr_next.count(match.second)){
            tracks_prev_next[match.first] = tracks_curr_next.at(match.second);
        }
    }
    return tracks_prev_next;
}


unsigned int ProcessorVisualOdometry::processKnown()
{
    auto t1 = std::chrono::system_clock::now();
    // Extend the process track matrix by using information stored in the incoming capture

    // Get tracks present at the last capture time (should be the most recent snapshot at this moment)
    std::list<FeatureBasePtr> tracks_snapshot_last = track_matrix_.snapshotAsList(last_ptr_);

    TracksMap tracks_map_li = capture_image_incoming_->getTracksPrev();
    for (auto feature_tracked_last: tracks_snapshot_last){
        // check if the keypoint in the last capture is in the last->incoming TracksMap stored in the incoming capture
        FeaturePointImagePtr feat_pi_last = std::dynamic_pointer_cast<FeaturePointImage>(feature_tracked_last);
        size_t id_feat_last = feat_pi_last->getKeyPoint().getId();  

        // if this feature id is in the last->incoming tracks of capture incoming, the track is continued
        // the matched pairs are stored in tracks_map_li_matched_ which is used in processNew to select the point that have NOT been match as "new"
        if (tracks_map_li.count(id_feat_last)){
            auto kp_track_li = tracks_map_li.find(id_feat_last);

            // create a feature using the corresponding WKeyPoint contained in incoming (hence the "second")
            auto feat_inco = FeatureBase::emplace<FeaturePointImage>(
                                                    capture_image_incoming_, 
                                                    capture_image_incoming_->getKeyPoints().at(kp_track_li->second), 
                                                    pixel_cov_);
            // the landmark id of the incoming feature is the same as the last one: 0 if not associated yet, the true landmark id if already associated
            feat_inco->setLandmarkId(feat_pi_last->landmarkId());
            // extend the track
            track_matrix_.add(feat_pi_last->trackId(), feat_inco);

            // add tracks_map_li to a vector so that it so that 
            // Very BAD to have to create a new std pair here -> find a better syntax but that's the idead
            auto kp_track_li_matched = std::pair<size_t, size_t>(kp_track_li->first, kp_track_li->second);
            tracks_map_li_matched_.insert(kp_track_li_matched);
        }
    }
    auto __attribute__((unused)) dt_processKnown = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t1).count();
    WOLF_DEBUG( "dt_processKnown (ms): " , dt_processKnown );

    // return number of successful tracks until incoming
    return tracks_map_li_matched_.size();
}


unsigned int ProcessorVisualOdometry::processNew(const int& _max_features)
{
    auto t1 = std::chrono::system_clock::now();

    // A new keyframe was detected:
    // last_ptr_ is going to become origin_ptr and
    // icoming_ptr_ is going to become last_ptr
    // So we need to reset the origin tracks of incoming used in preProcess so that they correspond to the future origin (currently last)  
    capture_image_incoming_->setTracksOrigin(capture_image_incoming_->getTracksPrev());

    // We have matched the tracks in the track matrix with the last->incoming tracks stored in the TracksMap from getTracksPrev()
    // Now we need to add new tracks in the track matrix for the NEW tracks.
    // Use book-keeping prepared in processKnown: the TracksMap that have been matched were stored in tracks_map_li_matched_
    // and here add tracks only for those that have not been matched

    unsigned int counter_new_tracks = 0;
    for (std::pair<size_t,size_t> track_li: capture_image_incoming_->getTracksPrev()){
        // if the track is not matched, then create a new track in the track matrix etc.
        if (!tracks_map_li_matched_.count(track_li.first)){
            // create a new last feature, a new track using this last feature and add the incoming feature to this track
            WKeyPoint kp_last = capture_image_last_->getKeyPoints().at(track_li.first);
            WKeyPoint kp_inco = capture_image_incoming_->getKeyPoints().at(track_li.second);
            FeaturePointImagePtr feat_pi_last = FeatureBase::emplace<FeaturePointImage>(capture_image_last_, kp_last, pixel_cov_);
            FeaturePointImagePtr feat_pi_inco = FeatureBase::emplace<FeaturePointImage>(capture_image_incoming_, kp_inco, pixel_cov_);
            track_matrix_.newTrack(feat_pi_last);  // newTrack set also the track id of the feature internally
            track_matrix_.add(feat_pi_last->trackId(), feat_pi_inco);
            counter_new_tracks++;
        }
    }

    auto __attribute__((unused)) dt_processNew = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t1).count();
    WOLF_DEBUG( "dt_processNew (ms): " , dt_processNew );

    return counter_new_tracks;
}


bool ProcessorVisualOdometry::new_landmark_is_viable(int track_id)
{
    // a track should be old enough so that the keypoints can be considered stable (stable)
    bool track_old_enough = track_matrix_.trackSize(track_id) >= params_visual_odometry_->lmk_creation.min_track_length_for_landmark;

    // Check if enough parallax has appeared while the camera moves through the scene so as the Landmark can be safely initialized.
    // As a heuristic, we can check that the pixel distance between first detection time and current time is over a threshold.
    double dist_pix = (track_matrix_.firstFeature(track_id)->getMeasurement() - track_matrix_.lastFeature(track_id)->getMeasurement()).norm();
    bool enough_parallax = dist_pix > params_visual_odometry_->lmk_creation.min_pixel_dist;

    return track_old_enough && enough_parallax;
}

void ProcessorVisualOdometry::establishFactors()
{
    // Function only called when a KF is created using at ts last
    // Loop over the snapshot of last capture. Does 2 things:
    //     1) for tracks already associated to a landmark, create a KF-Lmk factor between the last KF and the landmark.
    //     2) if the feature track is not associated to a landmark yet and is long enough, create a new landmark. 
    //        Create a KF-Lmk factor for all these KFs.

    WOLF_INFO("   establishFactors")

    std::list<FeatureBasePtr> tracks_snapshot_last = track_matrix_.snapshotAsList(last_ptr_);

    if(tracks_snapshot_last.empty())
    {
        WOLF_WARN("Trying to establish factors but no features exist in last Capture!");
        return;
    }

    for (auto feat: tracks_snapshot_last)
    {
        auto feat_pi = std::static_pointer_cast<FeaturePointImage>(feat);

        // verify if a landmark is associated to this track
        // TODO: use std::find_if instead
        LandmarkBasePtr associated_lmk = nullptr;

        // linear search on the landmark ids to get the right landmark -> BAD
        // An std::map for the Landmark Map would help greatly here
        // OPTIM: try to reverse iterate the map?
        for (auto lmk: getProblem()->getMap()->getLandmarkList())
        {
            if (lmk->id() == feat_pi->landmarkId()){
                associated_lmk = lmk;
                break;
            }
        }

        // 1) create a factor between new KF and assocatiated track landmark
        if (associated_lmk)
        {
            LandmarkHpPtr associated_lmk_hp = std::dynamic_pointer_cast<LandmarkHp>(associated_lmk);
            assert(associated_lmk_hp && "Selected landmark is not a visual odometry landmark!");
            FactorBase::emplace<FactorPixelHp>(feat_pi,
                                               feat_pi,
                                               associated_lmk_hp,
                                               shared_from_this(),
                                               params_visual_odometry_->apply_loss_function);
        }

        // 2) create a landmark if the track is not associated with one and meets certain criterias
        else if(new_landmark_is_viable(feat->trackId()))
        {
            Track track_kf = track_matrix_.trackAtKeyframes(feat->trackId());

            // LandmarkBasePtr lmk = emplaceLandmarkTriangulation(feat_pi, track_kf);
            LandmarkBasePtr lmk = emplaceLandmarkNaive(feat_pi);

            // Associate all features of the track to the landmark
            // (A simpler option would be to set the landmark ID equal to the track ID
            // but this would create conflict if landmarks are created by other processors -> not good)
            for (auto feat_track: track_matrix_.track(feat->trackId()))
            {
                feat_track.second->setLandmarkId(lmk->id());
            }

            for (auto feat_kf: track_kf)
            {
                LandmarkHpPtr lmk_hp = std::dynamic_pointer_cast<LandmarkHp>(lmk);
                FactorBase::emplace<FactorPixelHp>(feat_kf.second,
                                                   feat_kf.second,
                                                   lmk_hp, shared_from_this(),
                                                   params_visual_odometry_->apply_loss_function);
            }
        }
    }

    return;
}


LandmarkBasePtr ProcessorVisualOdometry::emplaceLandmarkTriangulation(FeaturePointImagePtr _feat, Track _track_kf)
{
    ///////////////
    // NOT USED YET
    ///////////////

    // at least 2 KF needed to triangulate
    if (_track_kf.size() < 2)
    {
        return nullptr;
    }
    Vector4d pt_c;
    bool success = tryTriangulationFromFeatures(_feat, _track_kf, pt_c);
    if (!success)
    {
        return nullptr;
    }

    WOLF_INFO("New lmk: ", pt_c.transpose())
    auto lmk_hp_ptr = LandmarkBase::emplace<LandmarkHp>(getProblem()->getMap(), 
                                                        pt_c, 
                                                        _feat->getKeyPoint().getDescriptor());

    // // Set all IDs equal to track ID
    // size_t track_id = _feat->trackId();
    // lmk_hp_ptr->setId(track_id);
    // _feat->setLandmarkId(track_id);

    return lmk_hp_ptr;
}


bool ProcessorVisualOdometry::tryTriangulationFromFeatures(FeaturePointImagePtr _feat, Track _track_kf, Vector4d& pt_c)
{
    // heuristic: use oldest feature/KF to triangulate with respect to current time
    FeaturePointImagePtr feat_pi1 = std::static_pointer_cast<FeaturePointImage>(_track_kf.begin()->second);

    cv::Vec2d pix1, pix2;
    // WOLF_INFO("TOTO")
    cv::eigen2cv(feat_pi1->getMeasurement(), pix1);
    cv::eigen2cv(_feat->getMeasurement(), pix2);
    // WOLF_INFO(pix1, pix2)
    
    // create 3x4 projection matrices [K|0] * Tcw 
    Matrix4d Tcw1 = getTcw(feat_pi1->getCapture()->getTimeStamp()).matrix();
    Matrix4d Tcw2 = getTcw(_feat->getCapture()->getTimeStamp()).matrix();
    Eigen::Matrix<double, 3, 4> P1 = sen_cam_->getProjectionMatrix() * Tcw1;
    Eigen::Matrix<double, 3, 4> P2 = sen_cam_->getProjectionMatrix() * Tcw2;
    cv::Mat P1_cv, P2_cv;
    cv::eigen2cv(P1, P1_cv);
    cv::eigen2cv(P2, P2_cv);
    // WOLF_INFO(P1)
    // WOLF_INFO(P1_cv)

    // perform triangulation of one landmark
    cv::Vec4d ptcv_w;  // output: homogeneous landmark point expressed in world frame
    cv::triangulatePoints(P1_cv, P2_cv, pix1, pix2, ptcv_w);
    // WOLF_INFO("YAY: ", ptcv_w)

    /////////////////////////////////////////////////////////
    // check that triangulation was done with enough parallax   
    /////////////////////////////////////////////////////////
    bool triangulation_is_a_success = true;  // Not implemented yet
    if(!triangulation_is_a_success)
    {
        return false;
    }

    // normalize to make equivalent to a unit quaternion
    cv::cv2eigen(ptcv_w, pt_c);
    
    // HACK: to avoid "nans" in residal, set Z = 1
    // pt_c(2) = 1 * pt_c(3);

    // Normalization necessary since homogeneous point stateblock is supposed to be unitary 
    pt_c.normalize();

    return true;
}


LandmarkBasePtr ProcessorVisualOdometry::emplaceLandmarkNaive(FeaturePointImagePtr _feat)
{
    // Taken from processor_bundle_adjustment
    // Initialize the landmark in its ray (based on pixel meas) and using a arbitrary distance

    Eigen::Vector2d point2d = _feat->getMeasurement();

    Eigen::Vector3d point3d;
    point3d = pinhole::backprojectPoint(
            getSensor()->getIntrinsic()->getState(),
            (std::static_pointer_cast<SensorCamera>(getSensor()))->getCorrectionVector(),
            point2d);

    // double distance = params_bundle_adjustment_->distance; // arbitrary value
    double distance = 1;
    Eigen::Vector4d vec_homogeneous_c;
    vec_homogeneous_c = {point3d(0),point3d(1),point3d(2),point3d.norm()/distance};

    // lmk from camera to world coordinate frame.
    Transform<double,3,Isometry> T_w_r
        = Translation<double,3>(_feat->getFrame()->getP()->getState())
        * Quaterniond(_feat->getFrame()->getO()->getState().data());
    Transform<double,3,Isometry> T_r_c
		= Translation<double,3>(_feat->getCapture()->getSensorP()->getState())
        * Quaterniond(_feat->getCapture()->getSensorO()->getState().data());
    Eigen::Matrix<double, 4, 1> vec_homogeneous_w = T_w_r
                                           * T_r_c
                                           * vec_homogeneous_c;

    // normalize to make equivalent to a unit quaternion
    vec_homogeneous_w.normalize();

    auto lmk_hp_ptr = LandmarkBase::emplace<LandmarkHp>(getProblem()->getMap(), 
                                                        vec_homogeneous_w, 
                                                        _feat->getKeyPoint().getDescriptor());
    return lmk_hp_ptr;
}


Eigen::Isometry3d ProcessorVisualOdometry::getTcw(TimeStamp _ts) const
{
    // get robot position and orientation at capture's timestamp
    const auto& state   = getProblem()->getState(_ts, "PO");
    const auto& pos     = state.at('P');
    const auto& ori     = state.at('O');

    // compute world-to-camera transform
    Eigen::Isometry3d T_w_r = Translation3d(pos) * Quaterniond(ori.data());
    Eigen::Isometry3d T_r_c = Translation3d(getSensor()->getP()->getState())
            * Quaterniond(getSensor()->getP()->getState().data());

    // invert transform to camera-to-world and return
    return (T_w_r * T_r_c).inverse();
}


void ProcessorVisualOdometry::postProcess()
{

    // Delete all dead tracks
    auto snap_last = track_matrix_.snapshot(capture_image_last_);
    for (const auto& track_id : track_matrix_.trackIds())
    {
        if (snap_last.count(track_id) == 0)
            track_matrix_.remove(track_id);
    }

    // print a bar with the number of active features in incoming
    unsigned int n = capture_image_incoming_->getKeyPoints().size();
    std::string s(n/2, '#');
    WOLF_INFO("FEATRS/2: ", n, " ", s);

    // print a bar with the number of active tracks
    n = track_matrix_.trackIds().size();
    s = std::string(n/4, 'o');
    WOLF_INFO("TRACKS/4: ", n, " ", s);

    // print a bar with the number of landmarks
    n = getProblem()->getMap()->getLandmarkList().size();
    s = std::string(n/2, '-');
    WOLF_INFO("LMARKS/2: ", n, " ", s);
}

bool ProcessorVisualOdometry::voteForKeyFrame() const
{

    // If the last capture was repopulated in preProcess, it means that the number of tracks fell
    // below a threshold in the current incoming track and that, as a consequence, last capture keypoints
    // was repopulated. In this case, the processor needs to create a new Keyframe whatever happens.
    CaptureImagePtr capture_image_incoming = std::dynamic_pointer_cast<CaptureImage>(incoming_ptr_);
    bool vote = capture_image_incoming->getLastWasRepopulated();

    // simple vote based on frame count, should be changed to something that takes into account number of tracks alive, parallax, etc.
    // vote = vote || ((frame_count_ % 5) == 0);

    vote = vote || incoming_ptr_->getFeatureList().size() < params_visual_odometry_->min_features_for_keyframe;

    return vote;
}


void ProcessorVisualOdometry::advanceDerived()
{
    // reinitilize the bookeeping to communicate info from processKnown to processNew
    tracks_map_li_matched_.clear();
}

void ProcessorVisualOdometry::resetDerived()
{
    // reinitilize the bookeeping to communicate info from processKnown to processNew
    tracks_map_li_matched_.clear();
}

TracksMap ProcessorVisualOdometry::kltTrack(const cv::Mat _img_prev, const cv::Mat _img_curr, const KeyPointsMap &_mwkps_prev, KeyPointsMap &_mwkps_curr)
{
    if (_mwkps_prev.empty()) return TracksMap();

    TracksMap tracks_prev_curr;

    // Create cv point list for tracking, we initialize optical flow with previous keypoints
    // We also need a list of indices to build the track map
    std::vector<cv::Point2f> p2f_prev;
    std::vector<size_t> indices_prev;
    for (auto & wkp : _mwkps_prev){
        p2f_prev.push_back(wkp.second.getCvKeyPoint().pt);
        indices_prev.push_back(wkp.first);
    }
    std::vector<cv::Point2f> p2f_curr = p2f_prev;

    // Configure and process KLT optical flow research
    std::vector<uchar> status;
    std::vector<float> err;



    // Process one way: previous->current with current init with previous
    ParamsProcessorVisualOdometry::KltParams prms = params_visual_odometry_->klt;
    cv::calcOpticalFlowPyrLK(
            _img_prev,
            _img_curr, 
            p2f_prev,
            p2f_curr,
            status, err,
            {prms.patch_width, prms.patch_height},
            prms.nlevels_pyramids,
            prms.criteria,
            (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));
    
    // Process the other way: current->previous
    std::vector<uchar> status_back;
    std::vector<float> err_back;
    cv::calcOpticalFlowPyrLK(
            _img_curr,
            _img_prev,
            p2f_curr,
            p2f_prev,
            status_back, err_back,
            {prms.patch_width, prms.patch_height},
            prms.nlevels_pyramids,
            prms.criteria,
            (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));

    // Delete point if KLT failed
    for (size_t j = 0; j < status.size(); j++) {

        if(!status_back.at(j)  ||  (err_back.at(j) > prms.max_err) ||
           !status.at(j)  ||  (err.at(j) > prms.max_err)) {
            continue;
        }

        // We keep the initial point and add the tracked point
        WKeyPoint wkp(cv::KeyPoint(p2f_curr.at(j), 1));
        _mwkps_curr[wkp.getId()] = wkp;

        // Update the map
        tracks_prev_curr[indices_prev.at(j)] = wkp.getId();

        // Other checks? Distance between track points?
    }

    return tracks_prev_curr;
}

bool ProcessorVisualOdometry::filterWithEssential(const KeyPointsMap _mwkps_prev, 
                                                  const KeyPointsMap _mwkps_curr, 
                                                  TracksMap &_tracks_prev_curr, 
                                                  cv::Mat &_E, 
                                                  VectorComposite &_pose_prev_curr)
{
    // TODO: Check new RANSAC methods introduced in opencv 4.5.0  
    // https://opencv.org/evaluating-opencvs-new-ransacs/

    // We need at least five tracks
    // TODO: this case is NOT handled by the rest of the algorithm currently
    if (_tracks_prev_curr.size() < 5) return false;

    ParamsProcessorVisualOdometry::RansacParams prms = params_visual_odometry_->ransac;

    // We need to build lists of pt2d for openCV function
    std::vector<cv::Point2d> p2d_prev, p2d_curr;
    std::vector<size_t> all_indices;
    for (auto & track : _tracks_prev_curr){
        all_indices.push_back(track.first);

        // ////////////////////////
        // // We may want to use rays instead of pixels
        // Eigen::Vector2d ray_prev = pinhole::depixellizePoint(sen_cam_->getPinholeModel(), _mwkps_prev.at(track.first).getEigenKeyPoint());
        // Eigen::Vector2d ray_curr = pinhole::depixellizePoint(sen_cam_->getPinholeModel(), _mwkps_curr.at(track.second).getEigenKeyPoint());
        // p2d_prev.push_back(cv::Point2d(ray_prev.x(), ray_prev.y()));
        // p2d_curr.push_back(cv::Point2d(ray_curr.x(), ray_curr.y()));
        // ////////////////////////

        // use pixels 
        p2d_prev.push_back(_mwkps_prev.at(track.first).getCvPoint());
        p2d_curr.push_back(_mwkps_curr.at(track.second).getCvPoint());
    }

    cv::Mat cvMask;

    // ////////////////////////
    // // If we use rays then the camera matrix to pass is the identity 
    // cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    // double focal = (sen_cam_->getIntrinsicMatrix()(0,0) +
    //                 sen_cam_->getIntrinsicMatrix()(1,1)) / 2;

    // // If using rays, thresh divided by the average focal
    // _E = cv::findEssentialMat(p2d_prev, 
    //                           p2d_curr, 
    //                           K, 
    //                           cv::RANSAC,
    //                           prms.prob,
    //                           prms.thresh / focal,
    //                           cvMask);
    // ////////////////////////
    
    // Essential matrix from pixels
    _E = cv::findEssentialMat(p2d_prev, 
                              p2d_curr, 
                              Kcv_, 
                              cv::RANSAC,
                              prms.prob,
                              prms.thresh,
                              cvMask);

    // recover the pose relative pose if asked
    if (_pose_prev_curr.includesStructure("PO")){
        // clone the mask not to change it. Otherwise it seems that the mask is reduced to all tracks being outliers...
        cv::Mat cvMask_dummy = cvMask.clone();
        _pose_prev_curr = pose_from_essential_matrix(_E, p2d_prev, p2d_curr, Kcv_, cvMask_dummy);
    }

    // Let's remove outliers from tracksMap
    for (size_t k=0; k<all_indices.size(); k++){
        if (cvMask.at<bool>(k) == 0){
            _tracks_prev_curr.erase(all_indices.at(k));
        }
    }

    return true;
}


VectorComposite ProcessorVisualOdometry::pose_from_essential_matrix(const cv::Mat& _E, 
                                                                    const std::vector<cv::Point2d>& _p2d_prev, 
                                                                    const std::vector<cv::Point2d>& _p2d_curr, 
                                                                    const cv::Mat& _K,
                                                                    cv::Mat& _cvMask)
{
    cv::Mat R_out, t_out;
    cv::recoverPose(_E, _p2d_prev, _p2d_curr, _K, R_out, t_out, _cvMask);

    // translation into eigen objects
    Eigen::Matrix3d R_eig;
    Eigen::Vector3d t_eig;
    cv::cv2eigen(R_out, R_eig);
    cv::cv2eigen(t_out, t_eig);

    VectorComposite pose_prev_curr("PO", {3,4});
    pose_prev_curr['P'] = t_eig;
    pose_prev_curr['O'] = Quaterniond(R_eig).coeffs();
    return pose_prev_curr;
}


void ProcessorVisualOdometry::retainBest(std::vector<cv::KeyPoint> &_keypoints, int n)
{
    if (_keypoints.size() > n) {
        if (n == 0) {
            _keypoints.clear();
            return;
        }
        std::nth_element(_keypoints.begin(), _keypoints.begin() + n, _keypoints.end(),
            [](cv::KeyPoint& a, cv::KeyPoint& b) { return a.response > b.response; });
        _keypoints.resize(n);
    }
}


void ProcessorVisualOdometry::equalize_img(cv::Mat &img_incoming)
{
    switch (params_visual_odometry_->equalization.method)
    {
        case 0:
            break;
        case 1:
        {
            // average to central brightness
            auto img_avg = (cv::mean(img_incoming)).val[0];
            img_incoming += cv::Scalar(round(params_visual_odometry_->equalization.average.median - img_avg) );
            break;
        }
        case 2:
        {
            cv::equalizeHist( img_incoming, img_incoming );
            break;
        }
        case 3:
        {
            // Contrast Limited Adaptive Histogram Equalization  CLAHE
            // -> more continuous lighting and higher contrast images
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(params_visual_odometry_->equalization.clahe.clip_limit,
                                                       params_visual_odometry_->equalization.clahe.tile_grid_size);
            clahe->apply(img_incoming, img_incoming);
            break;
        }
    }
}

void ProcessorVisualOdometry::detect_keypoints_empty_grid(cv::Mat img_incoming, CaptureImagePtr capture_image_incoming)
{
    // detect one FAST keypoint in each cell of the grid
    cv::Rect rect_roi;
    Eigen::Vector2i cell_index;
    std::vector<cv::KeyPoint> kps_roi;
    for (int i=1; i < params_visual_odometry_->grid.nbr_cells_h-1; i++){
        for (int j=1; j < params_visual_odometry_->grid.nbr_cells_v-1; j++){
            cell_index << i,j;
            cell_grid_.cell2roi(cell_index, rect_roi);

            cv::Mat img_roi(img_incoming, rect_roi);  // no data copy -> no overhead
            detector_->detect(img_roi, kps_roi);

            if (kps_roi.size() > 0){
                // retain only the best image in each region of interest
                retainBest(kps_roi, 1);
                // Keypoints are detected in the local coordinates of the region of interest
                // -> translate to the full image corner coordinate system
                kps_roi.at(0).pt.x = kps_roi.at(0).pt.x + rect_roi.x;
                kps_roi.at(0).pt.y = kps_roi.at(0).pt.y + rect_roi.y;
                capture_image_incoming->addKeyPoint(kps_roi.at(0));
            }
        }
    }
}


void ProcessorVisualOdometry::filter_last_incoming_tracks_from_ransac_result(const TracksMap& tracks_last_incoming, const KeyPointsMap& mwkps_incoming, const TracksMap& tracks_origin_incoming,
                                                                             TracksMap& tracks_last_incoming_filtered, KeyPointsMap& mwkps_incoming_filtered)
{
    /*
    Filter last -> incoming track (and tracked keypoint map in incoming)
                  L -------> I
    based of the tracks alive after RANSAC check on origin -> incoming
    O ---------------------> I


    This ugly double loop is due to the fact that features ids in Incoming are in the "values" of both maps, which requires linear search.
    Ideally, this may be solved a Boost.Bimap.
    **/

    for (auto & track_origin_incoming : tracks_origin_incoming){
        for (auto & track_last_incoming : tracks_last_incoming){
            if (track_origin_incoming.second == track_last_incoming.second){
                tracks_last_incoming_filtered[track_last_incoming.first] = track_last_incoming.second;
                mwkps_incoming_filtered[track_last_incoming.second] = mwkps_incoming.at(track_last_incoming.second);
                continue;
            }
        }
    }                                                    
}


void ProcessorVisualOdometry::detect_keypoints_in_empty_grid_cells(cv::Mat img_last, const TracksMap& tracks_last_incoming_filtered, const KeyPointsMap& mwkps_last, 
                                                                   std::vector<cv::KeyPoint>& kps_last_new, KeyPointsMap& mwkps_last_filtered)
{
    // Erase all keypoints previously added to the cell grid
    cell_grid_.renew();

    // Add last Keypoints that still form valid tracks between last and incoming
    // And create a filtered map for last keypoints
    for (auto track: tracks_last_incoming_filtered){
        mwkps_last_filtered[track.first] = mwkps_last.at(track.first);
        size_t last_kp_id = track.first;
        cell_grid_.hitCell(capture_image_last_->getKeyPoints().at(last_kp_id).getCvKeyPoint());
    }

    // Use the grid to detect new keypoints in empty cells
    // We try a bunch of times to add keypoints to randomly selected empty regions of interest
    for (int i=0; i < params_visual_odometry_->max_new_features; i++){
        cv::Rect rect_roi;

        bool is_empty = cell_grid_.pickRoi(rect_roi);
        if (!is_empty) // no empty cells!
        {
            break;
        }
        cv::Mat img_roi(img_last, rect_roi);  // no data copy -> no overhead
        std::vector<cv::KeyPoint> kps_roi;
        detector_->detect(img_roi, kps_roi);
        if (kps_roi.size() > 0){
            // retain only the best keypoint in each region of interest
            retainBest(kps_roi, 1);

            // Keypoints are detected in the local coordinates of the region of interest
            // -> translate to the full image corner coordinate system
            kps_roi.at(0).pt.x = kps_roi.at(0).pt.x + rect_roi.x;
            kps_roi.at(0).pt.y = kps_roi.at(0).pt.y + rect_roi.y;
            kps_last_new.push_back(kps_roi.at(0));

            // update grid with this detection
            cell_grid_.hitCell(kps_roi.at(0));
        }
        else
        {
            // block this grid's cell so that it is not reused for detection
            cell_grid_.blockCell(rect_roi);
        }
    }
}







/////////////////////////
// MotionProvider methods
/////////////////////////

VectorComposite ProcessorVisualOdometry::getState( const StateStructure& _structure ) const
{
    return getState(getTimeStamp(), _structure);
}

TimeStamp ProcessorVisualOdometry::getTimeStamp() const
{
    if ( last_ptr_ == nullptr )
        return TimeStamp::Invalid();
    else
        return last_ptr_->getTimeStamp();
}

VectorComposite ProcessorVisualOdometry::getState(const TimeStamp& _ts, const StateStructure& _structure) const
{
    // valid last...
    if (last_ptr_ != origin_ptr_)
    {

        std::shared_ptr<VectorComposite> pose_cam_ol_ptr = buffer_pose_cam_ol_.selectFirstBefore(_ts, getTimeTolerance());

        if( !pose_cam_ol_ptr )
        {
            WOLF_WARN(getName(), ": Requested state with time stamp out of tolerance. Returning empty VectorComposite");
            return VectorComposite();
        }
        else
        {
            return *pose_cam_ol_ptr;
        }
    }
    // return state at origin if possible
    else
    {
        if (origin_ptr_ == nullptr || fabs(_ts - origin_ptr_->getTimeStamp()) > params_->time_tolerance)
        {
            WOLF_WARN(getName(), ": Requested state with time stamp out of tolerance. Returning empty VectorComposite");
            return VectorComposite();
        }
        else
            return origin_ptr_->getFrame()->getState("PO");
    }
}





VectorComposite ProcessorVisualOdometry::getStateFromRelativeOriginLast(VectorComposite co_pose_cl) const
{
    if (origin_ptr_ == nullptr or
        origin_ptr_->isRemoving() or
        last_ptr_ == nullptr or
        last_ptr_->getFrame() == nullptr) // We do not have any info of where to find a valid state
                                          // Further checking here for origin_ptr is redundant: if last=null, then origin=null too.
    {
        WOLF_WARN("Processor has no state. Returning an empty VectorComposite");
        return VectorComposite(); // return empty state
    }
    
    VectorComposite r_pose_c = getSensor()->getState();
    VectorComposite c_pose_r = inverse(r_pose_c);

    // composte poses to obtain the relative robot transformation
    VectorComposite ro_pose_rl = SE3::compose(r_pose_c, SE3::compose(co_pose_cl, c_pose_r));

    // Pose at origin
    VectorComposite w_pose_ro = getOrigin()->getFrame()->getState();

    // Pose at last using composition
    VectorComposite w_pose_rl = SE3::compose(w_pose_ro, ro_pose_rl);

    WOLF_INFO("I WAS CALLED!!!")

    return w_pose_rl;
}




} //namespace wolf

// Register in the FactoryProcessor
#include "core/processor/factory_processor.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR(ProcessorVisualOdometry)
WOLF_REGISTER_PROCESSOR_AUTO(ProcessorVisualOdometry)
} // namespace wolf

