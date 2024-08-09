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
    // Check if the current processor (ProcessorVisualOdometry) is the only processor associated with the sensor (camera)
    // This implies that the odometry results are up-to-scale, so set the flag to true
    // FIXME: Is this the best way of initializing the flag?
    if (sen_cam_->getProcessorList().size() == 1)
    {
        WOLF_DEBUG("ProcessorVisualOdometry is the standalone processor associated with ", sen_cam_->getName(), ". Hence things are up-to-scale...");
        is_up_to_scale = true;
    }

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
            FrameBasePtr new_kf = addKF(kf_status_);
            
            // update pointers
            origin_ptr_     = incoming_ptr_;
            last_ptr_       = incoming_ptr_;
            last_frame_ptr_ = new_kf;
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
                                                capture_origin_->getKeyPoints(), capture_incoming_->getKeyPoints(),
                                                capture_incoming_->getTracksOrigin());

        WOLF_DEBUG("parallax: ", parallax)

        if (parallax > 3.0) { // if enough parallax, create a keyframe and perform initialization
            // We create a keyframe
            FrameBasePtr kf_curr = addKF(kf_status_);

            // Perform initialization of 3D map points and add factors associated with the initialized map points and camera poses.
            method_init_frame_pose_ = EM;
            establishFactors();

            // Update pointers
            origin_ptr_ = incoming_ptr_;
            last_frame_ptr_ = kf_curr;

            // set is_initialized to be true
            is_initialized = true;
        }

        last_ptr_       = incoming_ptr_;
        incoming_ptr_   = nullptr;

        return;
    }
    else    // otherwise (is_first_KF_created && is_initialized), normal running condition
    {
        // normal running condition

        // track 2D feature points (origin->last->incoming)
        size_t num_tracked_features = trackFeatures();
        WOLF_INFO("num_tracked_features: ", num_tracked_features)

        num_captures_elapsed ++;
        
        if (voteForKeyFrame() && permittedKeyFrame())
        {
            // We create a keyframe
            FrameBasePtr kf_curr = addKF(kf_status_);
            // Retrieve the previous frame
            FrameBasePtr kf_prev = kf_curr->getPreviousFrame();

            // Apply esssential matrix based outlier filtering
            filterOutliersByEssentialMatrix(kf_prev, kf_curr, sen_cam_, track_matrix_);

            // DEBUG: Compare different frame pose initialization methods: none (i.e., copying the pose of the last frame) and PnP
            CaptureImagePtr cap_kf_curr = std::static_pointer_cast<CaptureImage>(kf_curr->getCaptureOf(sen_cam_));
            std::vector<Eigen::Vector3d> pts3d;
            std::vector<Eigen::Vector2d> pts2d;

            // Case 1: Take the pose of the last keyframe as the initial guess of the new keyframe
            method_init_frame_pose_ = NONE;
            establishFactors();
            extractPointsFromCaptureImage(cap_kf_curr, pts3d, pts2d);
            vo_utils::evalReprojError(pts3d, pts2d, 
                                      sen_cam_->getPinholeModel(), sen_cam_->getDistortionVector(), 
                                      cap_kf_curr->getImage(), true, "/home/jlee/KF3_before.png");

            // Case 2: Estimate the initial guess of the new keyframe by applying PnP
            method_init_frame_pose_ = EM;
            establishFactors();
            extractPointsFromCaptureImage(cap_kf_curr, pts3d, pts2d);
            vo_utils::evalReprojError(pts3d, pts2d, 
                                      sen_cam_->getPinholeModel(), sen_cam_->getDistortionVector(), 
                                      cap_kf_curr->getImage(), true, "/home/jlee/KF3_before_byEM.png");

            // Case 3: Estimate the initial guess of the new keyframe by applying PnP
            method_init_frame_pose_ = PNP;
            establishFactors();
            extractPointsFromCaptureImage(cap_kf_curr, pts3d, pts2d);
            vo_utils::evalReprojError(pts3d, pts2d, 
                                      sen_cam_->getPinholeModel(), sen_cam_->getDistortionVector(), 
                                      cap_kf_curr->getImage(), true, "/home/jlee/KF3_before_byPnP.png");

            // Update pointers
            origin_ptr_ = incoming_ptr_;
            last_frame_ptr_ = kf_curr;
        }
        
        // DEBUG: If the last frame is associated to the last capture, print out optimization results for debugging
        // if (last_frame_ptr_->getCaptureOf(sen_cam_) == last_ptr_ && getProblem()->getTrajectory()->size() == 3) 
        if (num_captures_elapsed > 3)
        {
            FrameBasePtr kf_curr = last_frame_ptr_;

            CaptureImagePtr cap_kf_curr = std::static_pointer_cast<CaptureImage>(kf_curr->getCaptureOf(sen_cam_));
            std::vector<Eigen::Vector3d> pts3d;
            std::vector<Eigen::Vector2d> pts2d;
            
            extractPointsFromCaptureImage(cap_kf_curr, pts3d, pts2d);
            vo_utils::evalReprojError(pts3d, pts2d, 
                                      sen_cam_->getPinholeModel(), sen_cam_->getDistortionVector(), 
                                      cap_kf_curr->getImage(), true, "/home/jlee/KF3_after.png");

            exit(-1);
        }
        
        last_ptr_       = incoming_ptr_;
        incoming_ptr_   = nullptr;

        return;
    }
    
}


void ProcessorVisualOdometry::preProcess()
{
    // Update captures
    capture_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);

    cv::Mat img_incoming_raw = capture_incoming_->getImage();
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

    capture_incoming_->setImage(img_incoming);

    // Update the other captures
    if (!(running_step_ == FIRST_TIME)) {
        capture_origin_   = std::static_pointer_cast<CaptureImage>(origin_ptr_);
        capture_last_     = std::static_pointer_cast<CaptureImage>(last_ptr_);
    }

    return;
}


bool ProcessorVisualOdometry::voteForKeyFrame() const
{
    bool vote = false;
    // Simple vote based on the number of features being extracted and tracked util the incoming capture
    // Other rules may take into account number of tracks alive, parallax, etc.
    // vote = vote || incoming_ptr_->getFeatureList().size() < params_visual_odometry_->min_features_for_keyframe;
    vote = vote || num_captures_elapsed == 3;

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
    // Function is only called when a keyframe (KF) is created using the incoming capture.
    // Loop over the snapshot corresponding to the incoming capture. This function performs two main tasks:
    //     1) For tracks already associated with a landmark, create a KF-Lmk factor between the incoming KF and the landmark.
    //     2) If the feature track is not associated with a landmark yet and is long enough, create a new landmark
    //        using triangulation between the current and previous KFs as a prior. Establish KF-Lmk factors for all KFs in this case.
    //        For bookkeeping, define the landmark ID as the track ID.

    std::list<FeatureBasePtr> features = track_matrix_.snapshotAsList(incoming_ptr_);

    if(features.empty())
    {
        WOLF_WARN("Trying to establish factors but no features exist in incoming Capture!");
        return;
    }

    // Retrieve the current frame 
    FrameBasePtr frame_curr = incoming_ptr_->getFrame();
    // Retrieve the previous frame
    FrameBasePtr frame_prev = frame_curr->getPreviousFrame();

    // 0) Initialize the current camera pose 
    switch (method_init_frame_pose_)
    {
    // by essential matrix estimation.
    case EM:
    {
        WOLF_DEBUG("Essential Matrix")
        estimatePosebyEM(frame_prev, frame_curr);

        break;
    }
    // by PnP.
    case PNP:
    {
        WOLF_DEBUG("PnP")
        estimatePosebyPnP(frame_curr);

        break;
    }
    case NONE:
    {
        WOLF_DEBUG("None")
        // Get the transformation from the world coordinate frame to the robot coordinate frame at frame_prev
        Eigen::Isometry3d T_inW_ofB_prev = vo_utils::getTinW(frame_prev);

        // Set the transformation from the world coordinate frame to the robot coordinate frame at frame_curr
        vo_utils::setTinW(T_inW_ofB_prev, frame_curr);

        break;
    }
    default:
    {
        exit(-1);

        break;
    }
    }
    
    std::list<FeatureBasePtr> features_to_triangulate;

    for (auto feature_base: features)
    {
        FeaturePointImagePtr feature = std::static_pointer_cast<FeaturePointImage>(feature_base);

        // Get the landmark associated with the track of the current feature
        LandmarkBasePtr landmark_base = getProblem()->getMap()->getLandmark(feature->trackId());

        if (landmark_base) 
        {
            // 1) For tracks already associated with a landmark, create a KF-Lmk factor between the incoming KF and the landmark.
            // Note: Assuming the track ID is the same as the landmark ID, which may not hold if other types of landmarks are involved
            LandmarkHpPtr landmark = std::dynamic_pointer_cast<LandmarkHp>(landmark_base);
            FactorBase::emplace<FactorPixelHp>(feature,
                                               feature,
                                               landmark,
                                               shared_from_this(),
                                               params_visual_odometry_->apply_loss_function);
        }
        else if(track_matrix_.trackSize(feature->trackId()) >= params_visual_odometry_->min_track_length_for_landmark)
        {
            // 2) Bookmark the current feature to create a landmark if the track is not associated with one and has sufficient length
            features_to_triangulate.push_back(feature_base);
        }
    }

    WOLF_DEBUG("# of features_to_triangulate: ", features_to_triangulate.size());

    // 2) Create landmarks by performing triangulation
    if (features_to_triangulate.size() > 0)
    {
        std::list<LandmarkHpPtr> landmarks = emplaceLandmarks(frame_prev, frame_curr, 
                                                            features_to_triangulate);

        // Add factors from all KFs of this track to the new landmark
        for (const auto& landmark : landmarks)
        {
            // Get the track of the landmark at keyframes
            Track track_over_KFs = track_matrix_.trackAtKeyframes(landmark->trackId());
            for (auto track_at_KF: track_over_KFs)
            {
                FactorBase::emplace<FactorPixelHp>(track_at_KF.second,
                                                track_at_KF.second,
                                                landmark, 
                                                shared_from_this(),
                                                params_visual_odometry_->apply_loss_function);
            }
        }
    }

    return;
}


std::list<LandmarkHpPtr> ProcessorVisualOdometry::emplaceLandmarks(const FrameBasePtr frame_prev, 
                                                                   const FrameBasePtr frame_curr, 
                                                                   std::list<FeatureBasePtr> features_curr)
{
    /* Emplace a landmark by performing triangulation between the input feature (associated with the current KF) 
    and the corresponding last feature (associated with the last KF) */

    // Define the transformation from the robot coordinate frame to the camera coordinate frame
    Eigen::Isometry3d T_inB_ofC = Eigen::Translation3d(frame_curr->getCaptureOf(sen_cam_)->getSensorP()->getState()) *
                                  Eigen::Quaterniond(frame_curr->getCaptureOf(sen_cam_)->getSensorO()->getState().data());

    // Retrieve 2D-2D feature matching pairs in between the frames
    std::vector<cv::Point2f> pts_prev, pts_curr;
    std::vector<size_t> track_ids;
    vo_utils::getFeaturePairs(frame_prev, frame_curr, 
                              track_matrix_, sen_cam_, 
                              features_curr,
                              pts_prev, pts_curr,
                              track_ids);

    // Retrieve the transformation from the world coordinate frame to the robot coordinate frame at frame_prev
    Eigen::Isometry3d T_inW_ofB_prev = vo_utils::getTinW(frame_prev);

    // Retrieve the transformation from the world coordinate frame to the robot coordinate frame at frame_curr
    Eigen::Isometry3d T_inW_ofB_curr = vo_utils::getTinW(frame_curr);

    // Retrieve the transformation from the world coordinate frame to the camera coordinate frames
    Eigen::Isometry3d T_inW_ofC_prev = T_inW_ofB_prev * T_inB_ofC;
    Eigen::Isometry3d T_inW_ofC_curr = T_inW_ofB_curr * T_inB_ofC;

    // Get projection matrices of each frame; i.e., matrices mapping from 3D points in the world coordinate to the 2D points in the image coordinate
    cv::Mat P_prev = vo_utils::getCameraProjectionMatrix(Kcv_, T_inW_ofC_prev.inverse());
    cv::Mat P_curr = vo_utils::getCameraProjectionMatrix(Kcv_, T_inW_ofC_curr.inverse());

    // Perform triangulation of the pair of 2D image points associated with the respective camera poses
    cv::Mat pts_inW_cv;
    cv::triangulatePoints(P_prev, P_curr, pts_prev, pts_curr, pts_inW_cv);

    // Ensure pts_inW_cv has the expected dimensions
    assert(pts_inW_cv.rows == 4);

    std::list<LandmarkHpPtr> landmarks;
    int i = 0;
    for (const auto& feature_base : features_curr) 
    {
        FeaturePointImagePtr feature = std::dynamic_pointer_cast<FeaturePointImage>(feature_base);

        double x = static_cast<double>(pts_inW_cv.at<float>(0, i));
        double y = static_cast<double>(pts_inW_cv.at<float>(1, i));
        double z = static_cast<double>(pts_inW_cv.at<float>(2, i));
        double w = static_cast<double>(pts_inW_cv.at<float>(3, i));
        
        // Avoid division by zero
        if (w != 0) {
            // Convert homogeneous coordinates to 3D
            Eigen::Vector3d p_inW(x / w, y / w, z / w);
            
            // Create a 4D homogeneous point in world coordinates
            Eigen::Vector4d ph_inW(p_inW(0), p_inW(1), p_inW(2), 1.0);

            // Normalize the homogeneous point (this is the way in which WOLF handles it for some reason)
            ph_inW.normalize();

            // Emplace the landmark in the map with the 3D position and descriptor
            LandmarkBasePtr landmark_base = LandmarkBase::emplace<LandmarkHp>(getProblem()->getMap(), ph_inW, 
                                                                              feature->getKeyPoint().getDescriptor());
            LandmarkHpPtr landmark = std::dynamic_pointer_cast<LandmarkHp>(landmark_base);

            // Set the IDs for the landmark and feature
            landmark->setTrackId(feature->trackId());
            feature->setLandmarkId(landmark->id());

            landmarks.push_back(landmark);
        } else {
            // Handle the case where w is zero to avoid invalid landmarks
            std::cerr << "Warning: Homogeneous coordinate w is zero, skipping this point." << std::endl;
        }

        i++;
    }

    return landmarks;
}


/*
LandmarkHpPtr ProcessorVisualOdometry::emplaceLandmark(FeaturePointImagePtr feature)
{
    // Taken from processor_bundle_adjustment
    // Initialize the landmark in its ray (based on pixel meas) and using a arbitrary distance (default: 1)

    Eigen::Vector2d pt2d = feature->getMeasurement();

    Eigen::Vector3d pt3d;
    pt3d = pinhole::backprojectPoint(
            getSensor()->getIntrinsic()->getState(),
            (std::static_pointer_cast<SensorCamera>(getSensor()))->getCorrectionVector(),
            pt2d);

    // double dist = params_bundle_adjustment_->distance; // arbitrary value
    double dist = 1;
    Eigen::Vector4d p_inC;
    p_inC = {pt3d(0),pt3d(1),pt3d(2),pt3d.norm()/dist};

    // lmk from camera to world coordinate frame.
    Transform<double,3,Isometry> T_inW_ofB  // world -> robot
        = Translation<double,3>(feature->getFrame()->getP()->getState())
        * Quaterniond(feature->getFrame()->getO()->getState().data());
    Transform<double,3,Isometry> T_inB_ofC  // robot -> camera
		= Translation<double,3>(feature->getCapture()->getSensorP()->getState())
        * Quaterniond(feature->getCapture()->getSensorO()->getState().data());
    Eigen::Vector4d p_inW = T_inW_ofB * T_inB_ofC * p_inC;

    // normalize to make equivalent to a unit quaternion
    p_inW.normalize();

    LandmarkBasePtr landmark_base = LandmarkBase::emplace<LandmarkHp>(getProblem()->getMap(), 
                                                                      p_inW, 
                                                                      feature->getKeyPoint().getDescriptor());

    LandmarkHpPtr landmark = std::dynamic_pointer_cast<LandmarkHp>(landmark_base);

    // Set all IDs equal to track ID
    size_t track_id = feature->trackId();
    landmark->setTrackId(track_id);
    feature->setLandmarkId(landmark->id());

    return landmark;
}
*/

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

            cv::Mat img_roi(capture_incoming_->getImage(), rect_roi);  // no data copy -> no overhead
            detector_->detect(img_roi, kps_roi);

            if (kps_roi.size() > 0) {
                // retain only the best image in each region of interest
                vo_utils::retainBest(kps_roi, 1);
                // Keypoints are detected in the local coordinates of the region of interest
                // -> translate to the full image corner coordinate system
                kps_roi.at(0).pt.x = kps_roi.at(0).pt.x + rect_roi.x;
                kps_roi.at(0).pt.y = kps_roi.at(0).pt.y + rect_roi.y;
                capture_incoming_->addKeyPoint(kps_roi.at(0));
            }
        }
    }
    WOLF_DEBUG( "Initially detected " , capture_incoming_->getKeyPoints().size(), " keypoints in incoming" );

    // Initialize the tracks data structure with a "dummy track" where the keypoint is pointing to itself
    TracksMap tracks_init;
    for (auto mwkp : capture_incoming_->getKeyPoints()) {
        tracks_init[mwkp.first] = mwkp.first;
    }
    capture_incoming_->setTracksOrigin(tracks_init);
    capture_incoming_->setTracksPrev(tracks_init);

    return capture_incoming_->getKeyPoints().size();

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
    
    KeyPointsMap mwkps_origin   = capture_origin_->getKeyPoints();
    KeyPointsMap mwkps_last     = capture_last_->getKeyPoints();
    KeyPointsMap mwkps_incoming;  // init incoming

    WOLF_DEBUG("Tracking from KF ", last_frame_ptr_->id(), " (Capture ", capture_origin_->id(), ") to Capture ", capture_incoming_->id())

    // Create TracksMap between last and incoming
    TracksMap tracks_last_incoming = vo_utils::kltTrack(params_visual_odometry_,
                                                        capture_last_->getImage(), capture_incoming_->getImage(), 
                                                        mwkps_last, mwkps_incoming);

    // Load TracksMap between origin and last
    TracksMap tracks_origin_last = capture_last_->getTracksOrigin();

    // Merge tracks to get TracksMap between origin and incoming
    TracksMap tracks_origin_incoming = vo_utils::mergeTracks(tracks_origin_last, tracks_last_incoming);

    // Update captures
    capture_incoming_->addKeyPoints(mwkps_incoming);
    capture_incoming_->setTracksPrev(tracks_last_incoming);
    capture_incoming_->setTracksOrigin(tracks_origin_incoming);
    
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
    TracksMap tracks_last_incoming = capture_incoming_->getTracksPrev();

    // bookkeeper if the feature is being tracked origin->last->incoming (beware: different from capture_incoming_->getTracksOrigin())
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
                                                    capture_incoming_, 
                                                    capture_incoming_->getKeyPoints().at(track_last_incoming->second), 
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
            WKeyPoint keypoint_last = capture_last_->getKeyPoints().at(track_last_incoming.first);
            WKeyPoint keypoint_incoming = capture_incoming_->getKeyPoints().at(track_last_incoming.second);

            FeaturePointImagePtr feature_last = FeatureBase::emplace<FeaturePointImage>(capture_last_, keypoint_last, pixel_cov_);
            FeaturePointImagePtr feature_incoming = FeatureBase::emplace<FeaturePointImage>(capture_incoming_, keypoint_incoming, pixel_cov_);
            
            track_matrix_.newTrack(feature_last);
            track_matrix_.add(feature_last->trackId(), feature_incoming);

            cnt_new_tracks ++;
        }
    }
    
    WOLF_DEBUG("# of 'new' feature tracks last->incoming: ", cnt_new_tracks)
    
    return;
}

/**
 * @brief Extracts 2D feature points and corresponding transformed 3D landmark points from a given capture image.
 * 
 * This function takes a capture image and extracts the associated 2D feature points and 3D landmark points. 
 * The 3D points are transformed from world coordinates to camera coordinates. The resulting 2D and 3D points 
 * are stored in the provided vectors.
 * 
 * @param capture A constant pointer to the capture image containing the features and landmarks.
 * @param pts3d A reference to a vector of Eigen::Vector3d to store the transformed 3D landmark points in camera coordinates.
 * @param pts2d A reference to a vector of Eigen::Vector2d to store the 2D feature points.
 */
void ProcessorVisualOdometry::extractPointsFromCaptureImage(const CaptureImagePtr capture,
                                                            std::vector<Eigen::Vector3d>& pts3d,
                                                            std::vector<Eigen::Vector2d>& pts2d)
{
    // Clear the output vectors to ensure they start empty
    pts3d.clear();
    pts2d.clear();

    // Retrieve the frame associated with the capture image
    FrameBasePtr frame = capture->getFrame();

    // Define the transformation from the world coordinate frame to the robot coordinate frame
    Eigen::Isometry3d T_inW_ofB = Eigen::Translation3d(frame->getP()->getState()) *
                                  Eigen::Quaterniond(frame->getO()->getState().data());

    // Define the transformation from the robot coordinate frame to the camera coordinate frame
    Eigen::Isometry3d T_inB_ofC = Eigen::Translation3d(frame->getCaptureOf(sen_cam_)->getSensorP()->getState()) *
                                  Eigen::Quaterniond(frame->getCaptureOf(sen_cam_)->getSensorO()->getState().data());

    // Take a snapshot of the features in the current capture image
    FeatureBasePtrList features_base = track_matrix_.snapshotAsList(capture);

    // Iterate through the features
    for (const auto& feature_base : features_base) 
    {
        // Cast the feature base pointer to a feature point image pointer
        FeaturePointImagePtr feature = std::dynamic_pointer_cast<FeaturePointImage>(feature_base);
        
        // Cast the corresponding landmark pointer
        LandmarkHpPtr landmark = std::dynamic_pointer_cast<LandmarkHp>(getProblem()->getMap()->getLandmark(feature->trackId()));

        // Ensure the feature and landmark pointers are valid
        if (feature == nullptr || landmark == nullptr) continue;
        // assert(feature != nullptr && landmark != nullptr);

        // Add the 2D feature point to the pts2d vector
        pts2d.emplace_back(feature->getMeasurement());

        // Transform the 3D landmark point from world coordinates to camera coordinates
        Eigen::Vector3d p_inW = landmark->point();
        Eigen::Vector4d ph_inW(p_inW(0), p_inW(1), p_inW(2), 1.0);
        Eigen::Vector4d ph_inC = T_inB_ofC.inverse() * T_inW_ofB.inverse() * ph_inW;
        Eigen::Vector3d p_inC = ph_inC.head<3>() / ph_inC(3);

        // Add the transformed 3D point to the pts3d vector
        pts3d.emplace_back(p_inC);
    }

    // Return from the function
    return;
}


/**
 * @brief Filters out outliers in feature tracking between two frames using the Essential Matrix and RANSAC.
 * 
 * This function retrieves the 2D feature points associated with the current frame and matches them with 
 * the corresponding points in the previous frame. It then computes the Essential Matrix using RANSAC to 
 * identify and filter out the outliers from the tracked features.
 * 
 * @param frame_prev A pointer to the previous frame containing the tracked features.
 * @param frame_curr A pointer to the current frame containing the tracked features.
 * @param sen_cam A pointer to the sensor camera associated with the frames.
 * @param track_matrix A reference to the track matrix that maintains the tracking information of features across frames.
 */
void ProcessorVisualOdometry::filterOutliersByEssentialMatrix(const FrameBasePtr frame_prev, const FrameBasePtr frame_curr, 
                                                              const SensorCameraPtr sen_cam, TrackMatrix& track_matrix)
{
    // Log debug message
    WOLF_DEBUG("remove outliers in feature tracking ...")

    // Retrieve the list of features associated with the current frame
    std::list<FeatureBasePtr> features = track_matrix.snapshotAsList(frame_curr->getCaptureOf(sen_cam));

    // Retrieve 2D-2D feature matching pairs between the frames
    std::vector<cv::Point2f> pts_prev, pts_curr;
    std::vector<size_t> track_ids;
    vo_utils::getFeaturePairs(frame_prev, frame_curr, 
                              track_matrix, sen_cam, 
                              features,
                              pts_prev, pts_curr,
                              track_ids);

    // Compute the Essential Matrix using RANSAC
    cv::Mat inlierMask;
    cv::Mat essentialMat = cv::findEssentialMat(pts_prev, pts_curr, Kcv_, cv::RANSAC, 0.999, 1.0, inlierMask);

    // Count the number of inliers
    int numInliers = cv::countNonZero(inlierMask);
    WOLF_DEBUG("num. of inliers: ", numInliers, " (total: ", inlierMask.rows, ")")

    // Filter out the outliers
    for (size_t i = 0; i < pts_curr.size(); i++)
    {
        // If the point is marked as an outlier in the inlierMask
        if (!inlierMask.at<uchar>(i))
        {
            // Get the corresponding track ID and remove it from the track matrix
            size_t track_id = track_ids.at(i);
            track_matrix.remove(track_id);
        }
    }
}


void ProcessorVisualOdometry::estimatePosebyPnP(const FrameBasePtr frame)
{
    // Take a snapshot of the features associated with the current frame
    FeatureBasePtrList features_base = track_matrix_.snapshotAsList(frame->getCaptureOf(sen_cam_));

    std::vector<Eigen::Vector2d> imgPts2D; // Vector to store 2D feature points in image coordinates (pixels)
    std::vector<Eigen::Vector3d> mapPts3D; // Vector to store corresponding 3D map points in world coordinates

    // Iterate through the features in the current frame
    for (const auto& feature_base : features_base) 
    {
        // Cast the feature base pointer to a FeaturePointImage pointer
        FeaturePointImagePtr feature = std::dynamic_pointer_cast<FeaturePointImage>(feature_base);
        
        // Retrieve the corresponding landmark from the map using the track ID of the feature
        LandmarkHpPtr landmark = std::dynamic_pointer_cast<LandmarkHp>(getProblem()->getMap()->getLandmark(feature->trackId()));

        // Ensure both the feature and landmark pointers are valid
        if (feature == nullptr || landmark == nullptr) continue;

        // Add the 2D feature point to the imgPts2D vector
        imgPts2D.emplace_back(feature->getMeasurement());

        // Retrieve the 3D position of the landmark in world coordinates
        Eigen::Vector3d p_inW = landmark->point();

        // Add the 3D map point to the mapPts3D vector
        mapPts3D.emplace_back(p_inW);
    }
    
    // Convert Eigen vectors to OpenCV types for use in solvePnPRansac
    std::vector<cv::Point2f> imgPts2D_cv = vo_utils::convertToCvPoint2f(imgPts2D);
    std::vector<cv::Point3f> mapPts3D_cv = vo_utils::convertToCvPoint3f(mapPts3D);
    
    // Solve the Perspective-n-Point (PnP) problem using RANSAC to estimate the camera pose
    cv::Mat rvec, tvec; // Rotation and translation vectors
    std::vector<int> inliers; // Vector to store the indices of inlier points
    cv::solvePnPRansac(mapPts3D_cv, imgPts2D_cv, Kcv_, dcv_, rvec, tvec, false, 100, 10.0, 0.99, inliers);

    WOLF_DEBUG(inliers.size(), " out of ", imgPts2D.size(), " inliers detected for PnP.")

    // Convert the rotation vector (rvec) to a rotation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Convert the rotation matrix from OpenCV (cv::Mat) to Eigen::Matrix3d
    Eigen::Matrix3d eigenR = vo_utils::cvMatToEigen(R);

    // Convert the Eigen::Matrix3d to Eigen::Quaterniond for rotation representation
    Eigen::Quaterniond R_inC_ofW(eigenR);

    // Convert the translation vector from OpenCV (cv::Mat) to Eigen::Translation3d
    Eigen::Translation3d t_inC_ofW(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    // Combine the translation and rotation into a single transformation (Isometry) representing the transformation from the camera coordinate frame to the world frame
    Eigen::Isometry3d T_inC_ofW_estimated = t_inC_ofW * R_inC_ofW;
    
    // Define the transformation from the robot's body coordinate frame to the camera coordinate frame
    Eigen::Isometry3d T_inB_ofC = Eigen::Translation3d(frame->getCaptureOf(sen_cam_)->getSensorP()->getState()) *
                                  Eigen::Quaterniond(frame->getCaptureOf(sen_cam_)->getSensorO()->getState().data());

    // Calculate the transformation from the world frame to the robot's body frame
    Eigen::Isometry3d T_inW_ofB_estimated = T_inC_ofW_estimated.inverse() * T_inB_ofC.inverse();

    // Set the transformation from the world coordinate frame to the robot coordinate frame
    vo_utils::setTinW(T_inW_ofB_estimated, frame);
}


void ProcessorVisualOdometry::estimatePosebyEM(const FrameBasePtr frame_prev, const FrameBasePtr frame_curr) 
{
    // Retrieve the list of tracked features
    std::list<FeatureBasePtr> features = track_matrix_.snapshotAsList(frame_curr->getCaptureOf(sen_cam_));

    // Retrieve 2D-2D feature matching pairs between the frames
    std::vector<cv::Point2f> pts_prev, pts_curr;
    std::vector<size_t> track_ids;
    vo_utils::getFeaturePairs(frame_prev, frame_curr, 
                              track_matrix_, sen_cam_, 
                              features,
                              pts_prev, pts_curr,
                              track_ids);
    
    // Estimate the relative pose using epipolar geometry
    Eigen::Isometry3d T_inC_curr_ofC_prev = vo_utils::getRelativePoseByEpipolarGeometry(pts_prev, pts_curr, Kcv_);

    // Get the transformation from the world coordinate frame to the previous robot coordinate frame
    Eigen::Isometry3d T_inW_ofB_prev = vo_utils::getTinW(frame_prev);

    // Define the transformation from the robot coordinate frame to the camera coordinate frame
    Eigen::Isometry3d T_inB_ofC = Eigen::Translation3d(frame_curr->getCaptureOf(sen_cam_)->getSensorP()->getState()) *
                                    Eigen::Quaterniond(frame_curr->getCaptureOf(sen_cam_)->getSensorO()->getState().data());

    // Estimate the transformation from the world coordinate frame to the current robot coordinate frame
    Eigen::Isometry3d T_inW_ofB_curr_estimated = T_inW_ofB_prev * T_inB_ofC * T_inC_curr_ofC_prev.inverse() * T_inB_ofC.inverse();

    // Set the transformation from the world coordinate frame to the current robot coordinate frame
    vo_utils::setTinW(T_inW_ofB_curr_estimated, frame_curr);
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

