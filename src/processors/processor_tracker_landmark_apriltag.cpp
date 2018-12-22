#include "processor_tracker_landmark_apriltag.h"

#include "capture_image.h"
#include "sensor_camera.h"
#include "rotations.h"
#include "features/feature_apriltag.h"
#include "constraints/constraint_autodiff_apriltag.h"
#include "landmark_apriltag.h"
#include "state_quaternion.h"

// April tags
#include "common/homography.h"
#include "common/zarray.h"

#include <tag36h11.h>
#include <tag36h10.h>
#include <tag36artoolkit.h>
#include <tag25h9.h>
#include <tag25h7.h>

// #include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>

namespace wolf {


// Constructor
ProcessorTrackerLandmarkApriltag::ProcessorTrackerLandmarkApriltag( ProcessorParamsTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag) :
        ProcessorTrackerLandmark("TRACKER LANDMARK APRILTAG",  _params_tracker_landmark_apriltag ),
        tag_widths_(_params_tracker_landmark_apriltag->tag_widths_),
        tag_width_default_(_params_tracker_landmark_apriltag->tag_width_default_),
        std_xy_ (_params_tracker_landmark_apriltag->std_xy_ ),
        std_z_  (_params_tracker_landmark_apriltag->std_z_  ),
        std_rpy_(_params_tracker_landmark_apriltag->std_rpy_),
        min_time_vote_(_params_tracker_landmark_apriltag->min_time_vote),
        min_features_for_keyframe_(_params_tracker_landmark_apriltag->min_features_for_keyframe)
{
    // Constant transformation from apriltag camera frame (RUB: Z axis looking away from the tag)
    // to wolf camera frame (RDF: Z axis looking at the tag)
    c_M_ac_.matrix() = (Eigen::Vector4s() << 1, -1, -1, 1).finished().asDiagonal();

    // configure apriltag detector
    std::string famname(_params_tracker_landmark_apriltag->tag_family_);
    if (famname == "tag36h11")
        tag_family_ = *tag36h11_create();
    else if (famname == "tag36h10")
        tag_family_ = *tag36h10_create();
    else if (famname == "tag36artoolkit")
        tag_family_ = *tag36artoolkit_create();
    else if (famname == "tag25h9")
        tag_family_ = *tag25h9_create();
    else if (famname == "tag25h7")
        tag_family_ = *tag25h7_create();
    else {
        WOLF_ERROR("Unrecognized tag family name. Use e.g. \"tag36h11\".");
        exit(-1);
    }

    tag_family_.black_border     = _params_tracker_landmark_apriltag->tag_black_border_;

    detector_ = *apriltag_detector_create();
    apriltag_detector_add_family(&detector_, &tag_family_);

    detector_.quad_decimate     = _params_tracker_landmark_apriltag->quad_decimate_;
    detector_.quad_sigma        = _params_tracker_landmark_apriltag->quad_sigma_;
    detector_.nthreads          = _params_tracker_landmark_apriltag->nthreads_;
    detector_.debug             = _params_tracker_landmark_apriltag->debug_;
    detector_.refine_edges      = _params_tracker_landmark_apriltag->refine_edges_;
    detector_.refine_decode     = _params_tracker_landmark_apriltag->refine_decode_;
    detector_.refine_pose       = _params_tracker_landmark_apriltag->refine_pose_;
}

// Destructor
ProcessorTrackerLandmarkApriltag::~ProcessorTrackerLandmarkApriltag()
{
    // destroy raw pointers in detector_
    //apriltag_detector_destroy(&detector_); cannot be used because it is trying to free() the detector_ itself that is not implemented as a raw pointer in our case
    timeprofile_destroy(detector_.tp);
    apriltag_detector_clear_families(&detector_);
    zarray_destroy(detector_.tag_families);
    workerpool_destroy(detector_.wp);

    //free raw pointers in tag_family_
    free(tag_family_.name);
    free(tag_family_.codes);
}


ProcessorBasePtr ProcessorTrackerLandmarkApriltag::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    std::shared_ptr<ProcessorParamsTrackerLandmarkApriltag> prc_apriltag_params;
    if (_params)
        prc_apriltag_params = std::static_pointer_cast<ProcessorParamsTrackerLandmarkApriltag>(_params);
    else
        prc_apriltag_params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();

    ProcessorTrackerLandmarkApriltagPtr prc_ptr = std::make_shared<ProcessorTrackerLandmarkApriltag>(prc_apriltag_params);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

void ProcessorTrackerLandmarkApriltag::preProcess()
{
    //clear wolf detections so that new ones will be stored inside
    detections_incoming_.clear();

    // first, convert image to grayscale
    cv::cvtColor(std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage(),
                 grayscale_image_,
                 cv::COLOR_BGR2GRAY);

    //detect tags in incoming image
    // Make an image_u8_t header for the Mat data
    image_u8_t im = {   .width  = grayscale_image_.cols,
                        .height = grayscale_image_.rows,
                        .stride = grayscale_image_.cols,
                        .buf    = grayscale_image_.data
                    };

    // get camera intrinsic parameters as required by Apriltag homography-to-pose routine below
    Eigen::Vector4s camera_intrinsics(getSensorPtr()->getIntrinsicPtr()->getState()); //[cx cy fx fy]
    double cx( camera_intrinsics(0));
    double cy( camera_intrinsics(1));
    double fx(-camera_intrinsics(2));  // !! Negative sign advised by apriltag library commentary
    double fy( camera_intrinsics(3));

    // run Apriltag detector
    zarray_t *detections = apriltag_detector_detect(&detector_, &im);

    // loop all detections
    for (int i = 0; i < zarray_size(detections); i++) {


        // get raw Apriltag pose from homography
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        matd_t *pose_matrix = homography_to_pose(det->H, fx, fy, cx, cy);

        // write it in Eigen form
        Eigen::Affine3ds M_april_raw;
        for(int r=0; r<4; r++)
            for(int c=0; c<4; c++)
                M_april_raw.matrix()(r,c) = matd_get(pose_matrix,r,c);

        // write tag corners
        std::vector<cv::Point2d> corners(4);
        for (int c = 0; c < 4; c++)
        {
            corners[i].x = det->p[i][0];
            corners[i].y = det->p[i][1];
        }

        // we identify the raw april with the tag-to-aprilCamera transform (to be revised if needed)
        Eigen::Affine3ds ac_M_t = M_april_raw;

        // compose with aprilCamera-to-camera transform, get tag-to-camera transform
        Eigen::Affine3ds c_M_t ( c_M_ac_ * ac_M_t );

        // Set the scale of the translation vector from the relation: metric_width / units_width
        Eigen::Vector3s translation ( c_M_t.translation() ); // translation vector in apriltag units (tag width in units is 2 units)
        int    tag_id     = det->id;
        Scalar tag_width  = getTagWidth(tag_id);   // tag width in metric
        Scalar scale      = tag_width / 2.0;       // (tag width in units is 2 units)
        translation       = scale * translation;

        // set the measured pose vector
        Eigen::Vector7s pose;
        pose << translation, R2q(c_M_t.linear()).coeffs();

        // compute the covariance
        Eigen::Matrix6s cov = getVarVec().asDiagonal() ;

        // add to detected features list
        detections_incoming_.push_back(std::make_shared<FeatureApriltag>(pose, cov, tag_id, *det));
    }

    apriltag_detections_destroy(detections);
}

void ProcessorTrackerLandmarkApriltag::postProcess()
{

}

ConstraintBasePtr ProcessorTrackerLandmarkApriltag::createConstraint(FeatureBasePtr _feature_ptr,
                                                                     LandmarkBasePtr _landmark_ptr)
{
    ConstraintAutodiffApriltagPtr constraint = std::make_shared<ConstraintAutodiffApriltag>(
            getSensorPtr(),
            getLastPtr()->getFramePtr(),
            std::static_pointer_cast<LandmarkApriltag>(_landmark_ptr),
            std::static_pointer_cast<FeatureApriltag> (_feature_ptr ),
            false,
            CTR_ACTIVE
    );
    return constraint;
}

LandmarkBasePtr ProcessorTrackerLandmarkApriltag::createLandmark(FeatureBasePtr _feature_ptr)
{

    // world to rob
    Vector3s pos = getLastPtr()->getFramePtr()->getPPtr()->getState();
    Quaternions quat (getLastPtr()->getFramePtr()->getOPtr()->getState().data());
    Eigen::Affine3ds w_M_r = Eigen::Translation<Scalar,3>(pos.head(3)) * quat;

    // rob to camera
    pos = getSensorPtr()->getPPtr()->getState();
    quat.coeffs() = getSensorPtr()->getOPtr()->getState();
    Eigen::Affine3ds r_M_c = Eigen::Translation<Scalar,3>(pos.head(3)) * quat;

    // camera to lmk (tag)
    pos = _feature_ptr->getMeasurement().head(3);
    quat.coeffs() = _feature_ptr->getMeasurement().tail(4);
    Eigen::Affine3ds c_M_t   = Eigen::Translation<Scalar,3>(pos) * quat;

    // world to lmk (tag)
    Eigen::Affine3ds w_M_t = w_M_r * r_M_c * c_M_t;

    // make 7-vector for lmk (tag) pose
    pos  = w_M_t.translation();
    quat = w_M_t.linear();
    Vector7s w_pose_t;
    w_pose_t << pos, quat.coeffs();

    FeatureApriltagPtr feat_april = std::static_pointer_cast<FeatureApriltag>(_feature_ptr);
    int tag_id = feat_april->getTagId();

    LandmarkApriltagPtr new_landmark = std::make_shared<LandmarkApriltag>(w_pose_t, tag_id, getTagWidth(tag_id));

    return new_landmark;
}

unsigned int ProcessorTrackerLandmarkApriltag::detectNewFeatures(const unsigned int& _max_features, FeatureBaseList& _feature_list_out)
{
    for (auto feature_in_image : detections_last_)
    {
        bool feature_already_found(false);
        // features and landmarks must be tested with their ID !!
        // list of landmarks in the map
        LandmarkBaseList& landmark_list = getProblem()->getMapPtr()->getLandmarkList();

        //is the feature already associated to a landmark in the map ?
        for(auto it = landmark_list.begin(); it != landmark_list.end(); ++it)
            if(std::static_pointer_cast<LandmarkApriltag>(*it)->getTagId() == std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId())
                feature_already_found = true;

        //if the feature is not yet associated to a landmark in the map then we continue
        if (!feature_already_found)
        {
            // TODO: make detections more robust to decoding errors !!!
            // the detection can be wrong and may decode 2 different tags with the same id.
            // We need to do something if this happens and we need to do it here
            for (FeatureBaseList::iterator it=_feature_list_out.begin(); it != _feature_list_out.end(); ++it)
                if (std::static_pointer_cast<FeatureApriltag>(*it)->getTagId() == std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId())
                {
                    //we have a detection with the same id as the currently processed one. We remove the previous feature from the list for now
                    _feature_list_out.erase(it);
                    break; //it should not be possible two detection with the same id before getting there so we can stop here.
                }

            _feature_list_out.push_back(feature_in_image); // If the feature is not in the map and not in the list of newly detected features yet then we add it.
        } //otherwise we check the next feature
    }

    return _feature_list_out.size();
}

bool ProcessorTrackerLandmarkApriltag::voteForKeyFrame()
{
    Scalar dt_incoming_origin = getIncomingPtr()->getTimeStamp().get() - getOriginPtr()->getTimeStamp().get();
    if (dt_incoming_origin > min_time_vote_){
        bool more_in_last = getLastPtr()->getFeatureList().size() >= min_features_for_keyframe_;
        bool less_in_incoming = getIncomingPtr()->getFeatureList().size() <  min_features_for_keyframe_;
        return more_in_last;
        return more_in_last && less_in_incoming;
//        return getLastPtr()->getFeatureList().size() >= min_features_for_keyframe_
//        && getIncomingPtr()->getFeatureList().size() <  min_features_for_keyframe_;
    }
    return false;
}

unsigned int ProcessorTrackerLandmarkApriltag::findLandmarks(const LandmarkBaseList& _landmark_list_in,
                                                             FeatureBaseList& _feature_list_out,
                                                             LandmarkMatchMap& _feature_landmark_correspondences)
{   
    for (auto feature_in_image : detections_incoming_)
    {
        int tag_id(std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId());

        for (auto landmark_in_ptr : _landmark_list_in)
        {
            if(std::static_pointer_cast<LandmarkApriltag>(landmark_in_ptr)->getTagId() == tag_id)
            {
                _feature_list_out.push_back(feature_in_image);
                Scalar score(1.0);
                LandmarkMatchPtr matched_landmark = std::make_shared<LandmarkMatch>(landmark_in_ptr, score); //TODO: smarter score
                _feature_landmark_correspondences.emplace ( feature_in_image, matched_landmark );
                break;
            }
        }
    }

    return _feature_list_out.size();
}

wolf::Scalar ProcessorTrackerLandmarkApriltag::getTagWidth(int _id) const
{
    if (tag_widths_.find(_id) != tag_widths_.end())
        return tag_widths_.at(_id);
    else
        return tag_width_default_;
}

Eigen::Vector6s ProcessorTrackerLandmarkApriltag::getVarVec()
{
    Eigen::Vector6s var_vec;
    var_vec << std_xy_*std_xy_, std_xy_*std_xy_, std_z_*std_z_, std_rpy_*std_rpy_, std_rpy_*std_rpy_, std_rpy_*std_rpy_;

    return var_vec;
}

FeatureBaseList ProcessorTrackerLandmarkApriltag::getIncomingDetections() const
{
    return detections_incoming_;
}

FeatureBaseList ProcessorTrackerLandmarkApriltag::getLastDetections() const
{
    return detections_last_;
}

void ProcessorTrackerLandmarkApriltag::configure(SensorBasePtr _sensor)
{
    // set processor params from camera params if required

    // SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(_sensor);
    // [...] add more code if needed
}

void ProcessorTrackerLandmarkApriltag::advanceDerived()
{
    ProcessorTrackerLandmark::advanceDerived();
    detections_last_ = std::move(detections_incoming_);
}

void ProcessorTrackerLandmarkApriltag::resetDerived()
{
    ProcessorTrackerLandmark::resetDerived();
    detections_last_ = std::move(detections_incoming_);
}

} // namespace wolf

// Register in the SensorFactory
#include "processor_factory.h"

namespace wolf
{
WOLF_REGISTER_PROCESSOR("TRACKER LANDMARK APRILTAG", ProcessorTrackerLandmarkApriltag)
}
