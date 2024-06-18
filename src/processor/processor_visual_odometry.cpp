#include "vision/processor/processor_visual_odometry.h"

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <ctime>

namespace wolf{

ProcessorVisualOdometry::ProcessorVisualOdometry(ParamsProcessorVisualOdometryPtr _params_vo) :
                ProcessorTracker("ProcessorVisualOdometry", "PO", 3, _params_vo),
                params_visual_odometry_(_params_vo)
{
    // Processor stuff
    // Set pixel noise covariance
    Eigen::Vector2d std_pix; std_pix << params_visual_odometry_->std_pix, params_visual_odometry_->std_pix;
    pixel_cov_ = std_pix.array().square().matrix().asDiagonal();

    // [TODO] add things below (if needed)
}


void ProcessorVisualOdometry::configure(SensorBasePtr _sensor)
{
	//Initialize camera sensor pointer
	sen_cam_ = std::static_pointer_cast<SensorCamera>(_sensor);
    Eigen::Matrix3d K = sen_cam_->getIntrinsicMatrix();
    
    Kcv_ = (cv::Mat_<float>(3,3) << K(0,0), 0, K(0,2),
               0, K(1,1), K(1,2),
               0, 0, 1);

    // [TODO] add things below (if needed)

    // Tessalation of the image
    // cell_grid_ = ActiveSearchGrid(sen_cam_->getImgWidth(), sen_cam_->getImgHeight(),
    //                               params_visual_odometry_->grid.nbr_cells_h,
    //                               params_visual_odometry_->grid.nbr_cells_v,
    //                               params_visual_odometry_->grid.margin,
    //                               params_visual_odometry_->grid.separation);
}


TracksMap ProcessorVisualOdometry::mergeTracks(const TracksMap& tracks_prev_curr, const TracksMap& tracks_curr_next){
    TracksMap tracks_prev_next;
    for (auto &match : tracks_prev_curr){
        if (tracks_curr_next.count(match.second)){
            tracks_prev_next[match.first] = tracks_curr_next.at(match.second);
        }
    }
    return tracks_prev_next;
}


void ProcessorVisualOdometry::preProcess()
{
    // Get Capture
    capture_image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_);
    cv::Mat img_incoming = capture_image_incoming_->getImage();

    // [TODO] STEP A. Do CLAHE / tesselation

    // [TODO] STEP B. Do initialization (if needed)

    return;
}


unsigned int ProcessorVisualOdometry::processKnown()
{
    // [TODO] STEP A. Do 3D (map) - 2D (image) feature matching
    
    // [TODO] STEP B. Do current capture pose initialization by PnP
    
    // [TODO] STEP C. Do outlier removal of tracked 2D features


    // return the number of successful tracks until incoming
    return tracks_map_li_matched_.size();
}


unsigned int ProcessorVisualOdometry::processNew(const int& _max_features)
{
    // [TODO] Populate tracker with new features
    

    return counter_new_tracks;
}


void ProcessorVisualOdometry::establishFactors()
{
    // [TODO] STEP A. Triangulate tracked 2D features
    
    // [TODO] STEP B. Add factors associated with the triangulated map points
    
    
    return;
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

    // double distance = params_bundle_adjustment_->distance; // arbitrary value
    double distance = 1;
    Eigen::Vector4d vec_homogeneous_c;
    vec_homogeneous_c = {point3d(0),point3d(1),point3d(2),point3d.norm()/distance};

    // lmk from camera to world coordinate frame.
    Transform<double,3,Isometry> T_w_r
        = Translation<double,3>(feat_pi->getFrame()->getP()->getState())
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
                                                        feat_pi->getKeyPoint().getDescriptor());

    // Set all IDs equal to track ID
    size_t track_id = _feat->trackId();
    lmk_hp_ptr->setTrackId(track_id);
    _feat->setLandmarkId(lmk_hp_ptr->id());

    return lmk_hp_ptr;
}


void ProcessorVisualOdometry::postProcess()
{
    // Delete tracks with no keyframes
    for (const auto& track_id : track_matrix_.trackIds())
    {
        if (track_matrix_.trackAtKeyframes(track_id).empty())
            track_matrix_.remove(track_id);
    }

    // print a bar with the number of active features in incoming
    unsigned int n = capture_image_incoming_->getKeyPoints().size();
    std::string s(n/2, '#');
    // WOLF_INFO("FEATRS/2: ", n, " ", s);

    // print a bar with the number of active tracks
    n = track_matrix_.trackIds().size();
    s = std::string(n/4, 'o');
    // WOLF_INFO("TRACKS/4: ", n, " ", s);

    // print a bar with the number of landmarks
    n = getProblem()->getMap()->getLandmarkList().size();
    s = std::string(n/2, '-');
    // WOLF_INFO("LMARKS/2: ", n, " ", s);
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


} //namespace wolf

// Register in the FactoryProcessor
#include "core/processor/factory_processor.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR(ProcessorVisualOdometry)
WOLF_REGISTER_PROCESSOR_AUTO(ProcessorVisualOdometry)
} // namespace wolf

