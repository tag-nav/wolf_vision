#include "processor_tracker_landmark_apriltag.h"
 
namespace wolf {

// Constructor
// TODO Modify this default API to suit your class needs
ProcessorTrackerLandmarkApriltag::ProcessorTrackerLandmarkApriltag( ProcessorParamsTrackerLandmarkPtr _params_tracker_landmark) :
        ProcessorTrackerLandmark("TRACKER_LANDMARK_APRILTAG",  _params_tracker_landmark )
{
    // TODO: use parameters from constructor argument
    apriltag_family_t *tf = NULL;
    std::string famname("tag36h11");
    if (famname == "tag36h11")
        tf = tag36h11_create();
    else if (famname == "tag36h10")
        tf = tag36h10_create();
    else if (famname == "tag36artoolkit")
        tf = tag36artoolkit_create();
    else if (famname == "tag25h9")
        tf = tag25h9_create();
    else if (famname == "tag25h7")
        tf = tag25h7_create();
    else {
        std::cout<< "Unrecognized tag family name. Use e.g. \"tag36h11\"." <<std::endl ;
        exit(-1);
    }

    tf->black_border = 1; //getopt_get_int(getopt, "border"); 

    *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = 1.0; //getopt_get_double(getopt, "decimate");
    td->quad_sigma = 0.0; //getopt_get_double(getopt, "blur");
    td->nthreads = 1; //getopt_get_int(getopt, "threads");
    td->debug = 0; //getopt_get_bool(getopt, "debug");
    td->refine_edges = 1; //getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = 0; //getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = 0; //getopt_get_bool(getopt, "refine-pose");
}

// Destructor
ProcessorTrackerLandmarkApriltag::~ProcessorTrackerLandmarkApriltag()
{
}

void ProcessorTrackerLandmarkApriltag::preProcess()
{
    //std::cout << "PreProcess: " << std::endl;
    // first, convert image to grayscale
    cvtColor(std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage(), grayscale_image_, COLOR_BGR2GRAY);

    //detect tags in incoming image
    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = grayscale_image_.cols,
      .height = grayscale_image_.rows,
      .stride = grayscale_image_.cols,
      .buf = grayscale_image_.data
    };

    //defined camera parameters
    //TODO: get parameters from capture
    double fx(809.135074);
    double fy(809.410030);
    double cx(335.684471);
    double cy(257.352121);

    zarray_t *detections_ = apriltag_detector_detect(td, &im);
    cout << zarray_size(detections) << " tags detected" << endl;

    detections_incoming_->resize(zarray_size(detections));
    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        matd_t *pose_matrix = homography_to_pose(det->H, fx, fy, cx, cy);
        Eigen::Affine3ds t_M_c;

        for(int r=0; r<4; r++)
        {
            for(int c=0; c<4; c++)
            {
                t_M_c.matrix()(r,c) = matd_get(pose_matrix,r,c);
            }
        }

        Eigen::Affine3ds c_M_t(t_M_c.inverse());
        Eigen::Vector7s pose;
        pose << c_M_t.translation(), R2q(c_M_t.linear()).data(); //TODO: quaternion order ?

        Eigen::Matrix6s cov(Eigen::Matrix6s::Identity());
        cov.topLeftCorner(3,3) *= 1e-2;
        cov.bottomRightCorner(3,3) *= 1e-3;
        //get Pose matrix and covarince from det
        detections_incoming_.push_back(std::make_shared<FeatureApriltag>(pose, cov, *det)); //warning: pointer ?
    }

    //destroy pointers
    apriltag_detections_destroy(detections);
}

ConstraintBasePtr ProcessorTrackerLandmarkApriltag::createConstraint(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr)
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::createConstraint is empty." << std::endl;
  ConstraintBasePtr return_var{}; //TODO: fill this variable
  return return_var;
}

LandmarkBasePtr ProcessorTrackerLandmarkApriltag::createLandmark(FeatureBasePtr _feature_ptr)
{
    //get parameters from sensor
    double fx(809.135074);
    double fy(809.410030);
    double cx(335.684471);
    double cy(257.352121);

    matd_t *pose_matrix = homography_to_pose(std::static_pointer_cast<FeatureApriltag>(_feature_ptr)->det->H, fx, fy, cx, cy);
    Eigen::Affine3ds t_M_c;

    for(int r=0; r<4; r++)
    {
        for(int c=0; c<4; c++)
        {
            t_M_c.matrix()(r,c) = matd_get(pose_matrix,r,c);
        }
    }

    Eigen::Affine3ds c_M_t(t_M_c.inverse());
    Eigen::Vector7s pose;
    pose << c_M_t.translation(), R2q(c_M_t.linear()).data(); //TODO: quaternion order ?

    LandmarkApriltagPtr new_landmark = std::make_shared<LandmarkApriltag>(std::make_shared<StateBlock>(pose.head<3>()), std::make_shared<StateQuaternion>(pose.tail<4>()), std::static_pointer_cast<FeatureApriltag>(_feature_ptr)->det->id); //TODO: last parameter is width

    return LandmarkApriltagPtr;
}

unsigned int ProcessorTrackerLandmarkApriltag::detectNewFeatures(const unsigned int& _max_features, FeatureBaseList& _feature_list_out)
{
  for (auto feature_in_image : detections_incoming_)
    {
        int tag_id(std::static_pointer_cast<FeatureApriltag>(feature_in_image)->det->id);
        auto search = matches_landmark_from_incoming_.find(feature_in_image);

        if (search == matches_landmark_from_incoming_.end())
            _feature_list_out.push_back(feature_in_image);
    }

    return _feature_list_out.size();
}

bool ProcessorTrackerLandmarkApriltag::voteForKeyFrame()
{
  std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::voteForKeyFrame is empty." << std::endl;
  bool return_var{}; //TODO: fill this variable
  return return_var;
}

unsigned int findLandmarks(const LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences)
{   
    for (auto feature_in_image : detections_incoming_)
    {
        int tag_id(std::static_pointer_cast<FeatureApriltag>(feature_in_image)->det->id);

        for (auto landmark_in_ptr : _landmark_list_in)
        {
            if(std::static_pointer_cast<LandmarkApriltag>->getTagId() == tag_id)
            {
                _feature_list_out.push_back(feature_in_image);
                LandmarkMatchPtr matched_landmark(landmark_in_ptr, 1.0); //TODO: smarter score
                _feature_landmark_correspondences.insert ( std::pair<FeatureBasePtr, LandmarkMatchPtr>(feature_in_image,matched_landmark) );
                break;
            }
        }
    }

    return _feature_list_out.size();
}

// LandmarkMatchMap& ProcessorTrackerLandmarkApriltag::_feature_landmark_correspondences)(LandmarkMatchMap& _feature_landmark_correspondences)
// {
//   std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerLandmarkApriltag::_feature_landmark_correspondences) is empty." << std::endl;
//   LandmarkMatchMap& return_var{}; //TODO: fill this variable
//   return return_var;
// }

} // namespace wolf
