#include "vision/processor/processor_visual_odometry.h"
#include "vision/processor/vo_utils.h"

namespace wolf{

ProcessorVisualOdometry::ProcessorVisualOdometry(ParamsProcessorVisualOdometryPtr _params_vo) :
                ProcessorTracker("ProcessorVisualOdometry", "PO", 3, _params_vo),
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
    Eigen::Matrix3d K = sen_cam_->getIntrinsicMatrix();
    
    Kcv_ = (cv::Mat_<float>(3,3) << K(0,0), 0, K(0,2),
               0, K(1,1), K(1,2),
               0, 0, 1);

    Eigen::MatrixXd d = sen_cam_->getDistortionVector();
    dcv_ = (cv::Mat_<float>(4, 1) << d(0, 0), d(1, 0), 0.0, 0.0);
    
    // Tessalation of the image
    cell_grid_ = ActiveSearchGrid(sen_cam_->getImgWidth(), sen_cam_->getImgHeight(),
                                  params_visual_odometry_->grid.nbr_cells_h,
                                  params_visual_odometry_->grid.nbr_cells_v,
                                  params_visual_odometry_->grid.margin,
                                  params_visual_odometry_->grid.separation);
}


void ProcessorVisualOdometry::processCapture(CaptureBasePtr _incoming_ptr)
{
    using std::abs;

    if (_incoming_ptr == nullptr)
    {
        WOLF_ERROR("Received capture is nullptr.");
        return;
    }

    incoming_ptr_ = _incoming_ptr;

    computeProcessingStep();

    preProcess();

    bool is_first_KF_created = (origin_ptr_ == nullptr && last_ptr_ == nullptr) ? false : true;
    if (!is_first_KF_created)   // Check if the first KF (for initialization) exists
    {
        // Goal: Create first keyframe with enough number of feature points
        
        // populate feature points in the incoming image
        size_t num_detected_features = populateFeatures();
        // WOLF_DEBUG("num_detected_features: ", num_detected_features);
        
        // if the number of feature points exceeds the threshold, add it to the first keyframe
        if (num_detected_features > 2*params_visual_odometry_->min_features_for_keyframe)
        {
            FrameBasePtr keyframe_ptr = addKF(kf_status_);
            
            // update pointers
            origin_ptr_     = incoming_ptr_;
            last_ptr_       = incoming_ptr_;
            last_frame_ptr_ = keyframe_ptr;
            incoming_ptr_   = nullptr;
        }

        return;
    }
    else if (!is_initialized)   // Check if the second KF exists and intialization is performed
    {
        // Goal: Find second keyframe and perform initialzation, the process establishing map points and camera poses
        
        // track 2D feature points (origin->last->incoming)
        size_t num_tracked_features = trackFeatures();
        WOLF_INFO("num_tracked_features: ", num_tracked_features)

        // TODO: if num_tracked_features goes below threshold, re-create the first KF for different initialization

        // check out condition ready to initialize (e.g., enough parallax)
        // BEWARE: This operation is performed between "origin" (first keyframe) and "incoming" 
        // (incoming capture; being inspected and decided whether or not to be a keyframe).
        double parallax = vo_utils::getParallax(sen_cam_->getPinholeModel(), 
                                                capture_image_origin_->getKeyPoints(), capture_image_incoming_->getKeyPoints(),
                                                capture_image_incoming_->getTracksOrigin());

        WOLF_DEBUG("parallax: ", parallax)

        if (parallax > 3.0) { // if enough parallax, create a keyframe and perform initialization
            // We create a keyframe
            FrameBasePtr keyframe_ptr = addKF(kf_status_);

            // Perform initialization of 3D map points and add factors associated with the initialized map points and camera poses.
            establishFactors();

            // Update pointers
            origin_ptr_ = incoming_ptr_;
            last_frame_ptr_ = keyframe_ptr;

            // set is_initialized to be true
            is_initialized = true;

            // add debug function for inspecting reprojection error "before optimization"
            // WOLF_DEBUG("reprojection error before optimization: ")

            // std::vector<Eigen::Vector3d> pts3d;
            // std::vector<Eigen::Vector2d> pts2d;

            // FeatureBasePtrList features_base = track_matrix_.snapshotAsList(capture_image_incoming_);
            // for (const auto& feature_base : features_base) 
            // {
            //     FeaturePointImagePtr feature = std::dynamic_pointer_cast<FeaturePointImage>(feature_base);
            //     LandmarkHpPtr landmark = std::dynamic_pointer_cast<LandmarkHp>(getProblem()->getMap()->getLandmark(feature->trackId()));
                
            //     assert(feature != nullptr && landmark != nullptr);

            //     pts2d.emplace_back(feature->getMeasurement());
            //     pts3d.emplace_back(landmark->point());

            // }
            
            // vo_utils::evalReprojError(pts3d, pts2d, 
            //                           sen_cam_->getPinholeModel(), sen_cam_->getDistortionVector(), 
            //                           capture_image_incoming_->getImage(), true);
        }

        last_ptr_       = incoming_ptr_;
        incoming_ptr_   = nullptr;

        return;
    }
    else    // otherwise (is_first_KF_created && is_initialized), normal running condition
    {
        // normal running condition
        // Goal: perform 

        // add debug function for inspecting reprojection error "after optimization"
        // WOLF_DEBUG("reprojection error after optimization: ")

        // std::vector<Eigen::Vector3d> pts3d;
        // std::vector<Eigen::Vector2d> pts2d;

        // FeatureBasePtrList features_base = track_matrix_.snapshotAsList(capture_image_last_);
        // for (const auto& feature_base : features_base) 
        // {
        //     FeaturePointImagePtr feature = std::dynamic_pointer_cast<FeaturePointImage>(feature_base);
        //     LandmarkHpPtr landmark = std::dynamic_pointer_cast<LandmarkHp>(getProblem()->getMap()->getLandmark(feature->trackId()));
            
        //     assert(feature != nullptr && landmark != nullptr);

        //     pts2d.emplace_back(feature->getMeasurement());
        //     pts3d.emplace_back(landmark->point());

        // }

        // vo_utils::evalReprojError(pts3d, pts2d, 
        //                           sen_cam_->getPinholeModel(), sen_cam_->getDistortionVector(), 
        //                           capture_image_last_->getImage(), true);


        exit(-1);

        if (voteForKeyFrame() && permittedKeyFrame())
        {
            // do things with keyframe creation
        }

        // do else
        
        return;
    }
    
}


void ProcessorVisualOdometry::preProcess()
{
    // Update captures
    capture_image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);

    cv::Mat img_incoming_raw = capture_image_incoming_->getImage();
    cv::Mat img_incoming;
    // cv::undistort(img_incoming_raw, img_incoming, Kcv_, dcv_);
    img_incoming = img_incoming_raw.clone();


    /* Equalize image for better detection and tracking
     * available methods:
     *      0. none
     *      1. average
     *      2. opencv: histogram_equalization
     *      3. opencv: CLAHE
     */
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

    capture_image_incoming_->setImage(img_incoming);

    // Update the other captures
    if (!(running_step_ == FIRST_TIME)) {
        capture_image_origin_   = std::static_pointer_cast<CaptureImage>(origin_ptr_);
        capture_image_last_     = std::static_pointer_cast<CaptureImage>(last_ptr_);
    }

    return;
}


bool ProcessorVisualOdometry::voteForKeyFrame() const
{
    bool vote = false;
    // Simple vote based on the number of features being extracted and tracked util the incoming capture
    // Other rules may take into account number of tracks alive, parallax, etc.
    vote = vote || incoming_ptr_->getFeatureList().size() < params_visual_odometry_->min_features_for_keyframe;

    return vote;
}


FrameBasePtr ProcessorVisualOdometry::addKF(int _kf_status) 
{   
    WOLF_DEBUG("Adding Capture ", incoming_ptr_->id(), " to be KF ...")

    if (_kf_status) {   // WITH_KF
        FrameBasePtr keyframe_from_callback = buffer_frame_.select( incoming_ptr_->getTimeStamp(), params_visual_odometry_->time_tolerance);
        buffer_frame_.removeUpTo( keyframe_from_callback->getTimeStamp() );

        // WOLF_DEBUG( "PT ", getName(), " KF_INSERTION_WITH_KF_FROM_OTHER_PROCESSOR: KF" , keyframe_from_callback->id() , " callback unpacked with ts= " , keyframe_from_callback->getTimeStamp() );

        // check if the callback keyframe has a capture of this sensor
        auto capture_from_callback = keyframe_from_callback->getCaptureOf(this->getSensor());

        if (incoming_ptr_ == capture_from_callback)
        {
            // If captures match, then frames must match too
            assert(incoming_ptr_->getFrame() != nullptr 
                    and incoming_ptr_->getFrame() == keyframe_from_callback 
                    and "The keyframe has a Capture from this sensor, but this capture is not this!");
            // WOLF_DEBUG("PT ", getName(), " This capture has been processed previously by another processor!")
        }
        else
        {
            // WOLF_DEBUG("PT ", getName(), " This capture had not been processed by any other processor!")
            
            // Join KF
            incoming_ptr_->link(keyframe_from_callback);
        }
        
        return keyframe_from_callback;
    }
    else { // WITHOUT_KF
        // WOLF_DEBUG( "PT ", getName(), " KF_INSERTION_WITHOUT_KF_FROM_OTHER_PROCESSOR" );

        // Check if this capture has already a Frame
        auto frame = incoming_ptr_->getFrame();
        assert(frame == nullptr and " This capture has been processed and linked by another processor, but no keyframe callback was received!");

        // WOLF_DEBUG("PT ", getName(), " This capture has not been processed by another processor!")

        // make a new KF at this capture
        FrameBasePtr keyframe = FrameBase::emplace<FrameBase>(getProblem()->getTrajectory(),
                                                            incoming_ptr_->getTimeStamp(),
                                                            getProblem()->getFrameStructure(),
                                                            getProblem()->getState(incoming_ptr_->getTimeStamp()));
        // Append this capture to KF
        incoming_ptr_->link(keyframe);

        // Issue KF callback with new KF
        getProblem()->keyFrameCallback(keyframe, shared_from_this());

        return keyframe;
    }
}


void ProcessorVisualOdometry::establishFactors()
{
    // Function only called when KF is created using incoming
    // Loop over the snapshot in corresponding to incoming capture. Does 2 things:
    //     1) for tracks already associated to a landmark, create a KF-Lmk factor between the incoming KF and the landmark.
    //     2) if the feature track is not associated to a landmark yet and is long enough, create a new landmark
    //        using triangulation as a prior, using previous KF current estimates. Create a KF-Lmk factor for all these KFs. 
    //        For bookkeeping, define the landmark id as the track id.

    std::list<FeatureBasePtr> features = track_matrix_.snapshotAsList(incoming_ptr_);

    if(features.empty())
    {
        WOLF_WARN("Trying to establish factors but no features exist in incoming Capture!");
        return;
    }

    for (auto feature_base: features)
    {
        FeaturePointImagePtr feature = std::static_pointer_cast<FeaturePointImage>(feature_base);

        // get a landmark associated to the track of the feature being inspected
        LandmarkBasePtr lmk_base = getProblem()->getMap()->getLandmark(feature->trackId());

        if (lmk_base) 
        {
            // 1) create a factor between new KF and assocatiated track landmark
            //    HYP: assuming the trackid are the same as the landmark ID -> BAD if other types of landmarks involved
            LandmarkHpPtr lmk = std::dynamic_pointer_cast<LandmarkHp>(lmk_base);
            FactorBase::emplace<FactorPixelHp>(feature,
                                               feature,
                                               lmk,
                                               shared_from_this(),
                                               params_visual_odometry_->apply_loss_function);
        }
        else if(track_matrix_.trackSize(feature->trackId()) >= params_visual_odometry_->min_track_length_for_landmark)
        {
            // 2) create landmark if track is not associated with one and has enough length
            lmk_base = emplaceLandmark(feature_base);

            // Add factors from all KFs of this track to the new landmark
            Track track_over_KFs = track_matrix_.trackAtKeyframes(feature->trackId());
            for (auto track_at_KF: track_over_KFs)
            {
                LandmarkHpPtr lmk = std::dynamic_pointer_cast<LandmarkHp>(lmk_base);
                FactorBase::emplace<FactorPixelHp>(track_at_KF.second,
                                                   track_at_KF.second,
                                                   lmk, 
                                                   shared_from_this(),
                                                   params_visual_odometry_->apply_loss_function);
            }
        }
    }

    return;
}


LandmarkBasePtr ProcessorVisualOdometry::emplaceLandmark(FeatureBasePtr _feature)
{
    // Taken from processor_bundle_adjustment
    // Initialize the landmark in its ray (based on pixel meas) and using a arbitrary distance

    FeaturePointImagePtr feat_pi = std::static_pointer_cast<FeaturePointImage>(_feature);
    Eigen::Vector2d point2d = _feature->getMeasurement();

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
        = Translation<double,3>(feat_pi->getFrame()->getP()->getState())
        * Quaterniond(_feature->getFrame()->getO()->getState().data());
    Transform<double,3,Isometry> T_r_c
		= Translation<double,3>(_feature->getCapture()->getSensorP()->getState())
        * Quaterniond(_feature->getCapture()->getSensorO()->getState().data());
    Eigen::Matrix<double, 4, 1> vec_homogeneous_w = T_w_r
                                           * T_r_c
                                           * vec_homogeneous_c;

    // normalize to make equivalent to a unit quaternion
    vec_homogeneous_w.normalize();

    auto lmk_hp_ptr = LandmarkBase::emplace<LandmarkHp>(getProblem()->getMap(), 
                                                        vec_homogeneous_w, 
                                                        feat_pi->getKeyPoint().getDescriptor());

    // Set all IDs equal to track ID
    size_t track_id = _feature->trackId();
    lmk_hp_ptr->setTrackId(track_id);
    _feature->setLandmarkId(lmk_hp_ptr->id());

    return lmk_hp_ptr;
}


size_t ProcessorVisualOdometry::populateFeatures() 
{
    // detect one FAST keypoint in each cell of the grid
    cv::Rect rect_roi;
    Eigen::Vector2i cell_index;
    std::vector<cv::KeyPoint> kps_roi;
    for (int i=1; i < params_visual_odometry_->grid.nbr_cells_h-1; i++) {
        for (int j=1; j < params_visual_odometry_->grid.nbr_cells_v-1; j++) {
            cell_index << i,j;
            cell_grid_.cell2roi(cell_index, rect_roi);

            cv::Mat img_roi(capture_image_incoming_->getImage(), rect_roi);  // no data copy -> no overhead
            detector_->detect(img_roi, kps_roi);

            if (kps_roi.size() > 0) {
                // retain only the best image in each region of interest
                vo_utils::retainBest(kps_roi, 1);
                // Keypoints are detected in the local coordinates of the region of interest
                // -> translate to the full image corner coordinate system
                kps_roi.at(0).pt.x = kps_roi.at(0).pt.x + rect_roi.x;
                kps_roi.at(0).pt.y = kps_roi.at(0).pt.y + rect_roi.y;
                capture_image_incoming_->addKeyPoint(kps_roi.at(0));
            }
        }
    }
    WOLF_DEBUG( "Initially detected " , capture_image_incoming_->getKeyPoints().size(), " keypoints in incoming" );

    // Initialize the tracks data structure with a "dummy track" where the keypoint is pointing to itself
    TracksMap tracks_init;
    for (auto mwkp : capture_image_incoming_->getKeyPoints()) {
        tracks_init[mwkp.first] = mwkp.first;
    }
    capture_image_incoming_->setTracksOrigin(tracks_init);
    capture_image_incoming_->setTracksPrev(tracks_init);

    return capture_image_incoming_->getKeyPoints().size();

    // TODO: add case for re-population
}


size_t ProcessorVisualOdometry::trackFeatures() 
{
    ////////////////////////////////
    // 2D-2D FEATURE TRACKING
    // Update capture Incoming data
    //   - Track keypoints last->incoming
    //   - Merge tracks origin->last with last->incoming to get origin->incoming
    ////////////////////////////////
    
    KeyPointsMap mwkps_origin   = capture_image_origin_->getKeyPoints();
    KeyPointsMap mwkps_last     = capture_image_last_->getKeyPoints();
    KeyPointsMap mwkps_incoming;  // init incoming

    WOLF_DEBUG("Tracking from KF ", last_frame_ptr_->id(), " (Capture ", capture_image_origin_->id(), ") to Capture ", capture_image_incoming_->id())

    // Create TracksMap between last and incoming
    TracksMap tracks_last_incoming = vo_utils::kltTrack(params_visual_odometry_,
                                                        capture_image_last_->getImage(), capture_image_incoming_->getImage(), 
                                                        mwkps_last, mwkps_incoming);

    // Load TracksMap between origin and last
    TracksMap tracks_origin_last = capture_image_last_->getTracksOrigin();

    // Merge tracks to get TracksMap between origin and incoming
    TracksMap tracks_origin_incoming = vo_utils::mergeTracks(tracks_origin_last, tracks_last_incoming);

    // Update captures
    capture_image_incoming_->addKeyPoints(mwkps_incoming);
    capture_image_incoming_->setTracksPrev(tracks_last_incoming);
    capture_image_incoming_->setTracksOrigin(tracks_origin_incoming);
    
    // Update track matrix
    updateTrackMatrix();

    return tracks_origin_incoming.size();
}


void ProcessorVisualOdometry::updateTrackMatrix()
{
    // step 1: update track matrix for the "existing" track of origin(->last)->incoming
    
    // load features associated with the last
    std::list<FeatureBasePtr> features_last = track_matrix_.snapshotAsList(last_ptr_);

    // get last->incoming track
    TracksMap tracks_last_incoming = capture_image_incoming_->getTracksPrev();

    // bookkeeper if the feature is being tracked origin->last->incoming (beware: different from capture_image_incoming_->getTracksOrigin())
    TracksMap tracks_feature_last_incoming;

    // loop over all features associated with the last
    for (auto e: features_last)
    {
        FeaturePointImagePtr feature_last = std::dynamic_pointer_cast<FeaturePointImage>(e);
        size_t id_feature_last = feature_last->getKeyPoint().getId();

        // inspect if the feature associated with the last exist in last->incoming
        if (tracks_last_incoming.count(id_feature_last))
        {
            // if true, create a feature associated with the incoming and update track_maatrix
            auto track_last_incoming = tracks_last_incoming.find(id_feature_last);
            FeatureBasePtr feature_incoming = FeatureBase::emplace<FeaturePointImage>(
                                                    capture_image_incoming_, 
                                                    capture_image_incoming_->getKeyPoints().at(track_last_incoming->second), 
                                                    pixel_cov_);
            track_matrix_.add(feature_last->trackId(), feature_incoming);
            
            // indicate that the feature is being tracked from 
            auto track_feature_last_incoming = std::pair<size_t, size_t>(track_last_incoming->first, track_last_incoming->second);
            tracks_feature_last_incoming.insert(track_feature_last_incoming);
        }
    }
    
    WOLF_DEBUG("# of 'continued' feature tracks origin->last->incoming: ", tracks_feature_last_incoming.size())
    
    // step 2: update track matrix for the "new" track of last->incoming
    size_t cnt_new_tracks = 0;
    for (std::pair<size_t,size_t> track_last_incoming: tracks_last_incoming)
    {
        // if the current track being inspected does not exist in tracks_feature_last_incoming, i.e., "existing" track of origin(->last)->incoming, 
        // generate features in last and incoming and add them to the track matrix
        if (!tracks_feature_last_incoming.count(track_last_incoming.first))
        {
            WKeyPoint keypoint_last = capture_image_last_->getKeyPoints().at(track_last_incoming.first);
            WKeyPoint keypoint_incoming = capture_image_incoming_->getKeyPoints().at(track_last_incoming.second);

            FeaturePointImagePtr feature_last = FeatureBase::emplace<FeaturePointImage>(capture_image_last_, keypoint_last, pixel_cov_);
            FeaturePointImagePtr feature_incoming = FeatureBase::emplace<FeaturePointImage>(capture_image_incoming_, keypoint_incoming, pixel_cov_);
            
            track_matrix_.newTrack(feature_last);
            track_matrix_.add(feature_last->trackId(), feature_incoming);

            cnt_new_tracks ++;
        }
    }
    
    WOLF_DEBUG("# of 'new' feature tracks last->incoming: ", cnt_new_tracks)
    
    return;
}


// override functions unused yet
void ProcessorVisualOdometry::postProcess() {};
unsigned int ProcessorVisualOdometry::processKnown() { return 0; };
unsigned int ProcessorVisualOdometry::processNew(const int& _max_features) { return 0; };
void ProcessorVisualOdometry::advanceDerived() {};
void ProcessorVisualOdometry::resetDerived() {};

} //namespace wolf

// Register in the FactoryProcessor
#include "core/processor/factory_processor.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR(ProcessorVisualOdometry)
WOLF_REGISTER_PROCESSOR_AUTO(ProcessorVisualOdometry)
} // namespace wolf

