#include "processor_image_landmark.h"

#include "landmark_corner_2D.h"
#include "landmark_point_3d.h"
#include "constraint_corner_2D.h"
#include "constraint_image.h"
#include "sensor_camera.h"
#include "pinholeTools.h"

#include <Eigen/Geometry>

namespace wolf
{

ProcessorImageLandmark::ProcessorImageLandmark(ProcessorImageParameters _params) :
    ProcessorTrackerLandmark(PRC_TRACKER_DUMMY, _params.algorithm.max_new_features), n_feature_(0), landmark_idx_non_visible_(0),
    matcher_ptr_(nullptr), detector_descriptor_ptr_(nullptr), params_(_params),
    active_search_grid_()
{
    setType("IMAGE");
    // 1. detector-descriptor params
    DetectorDescriptorParamsBase* _dd_params = _params.detector_descriptor_params_ptr;
    switch (_dd_params->type){
        case DD_BRISK:
            {
            DetectorDescriptorParamsBrisk* params_brisk = (DetectorDescriptorParamsBrisk*) _dd_params;
            detector_descriptor_ptr_ = new cv::BRISK(params_brisk->threshold, //
                                                     params_brisk->octaves, //
                                                     params_brisk->pattern_scale);

            detector_descriptor_params_.pattern_radius_ = std::max((unsigned int)((_dd_params->nominal_pattern_radius)*pow(2,params_brisk->octaves)),
                                                                   (unsigned int)((_dd_params->nominal_pattern_radius)*params_brisk->pattern_scale));

            detector_descriptor_params_.size_bits_ = detector_descriptor_ptr_->descriptorSize() * 8;

            break;
            }
        case DD_ORB:
            {
            DetectorDescriptorParamsOrb* params_orb = (DetectorDescriptorParamsOrb*) _dd_params;
            detector_descriptor_ptr_ = new cv::ORB(params_orb->nfeatures, //
                                                   params_orb->scaleFactor, //
                                                   params_orb->nlevels, //
                                                   params_orb->edgeThreshold, //
                                                   params_orb->firstLevel, //
                                                   params_orb->WTA_K, //
                                                   params_orb->scoreType, //
                                                   params_orb->patchSize);

            detector_descriptor_params_.pattern_radius_ =
                    (unsigned int)( (_dd_params->nominal_pattern_radius) * pow(params_orb->scaleFactor, params_orb->nlevels-1) );

            detector_descriptor_params_.size_bits_ = detector_descriptor_ptr_->descriptorSize() * 8;

            break;
            }
        default:
            throw std::runtime_error("Unknown detector-descriptor type");
    }

    // 2. active search params
    active_search_grid_.setParameters(_params.image.width, _params.image.height,
            _params.active_search.grid_width, _params.active_search.grid_height,
            detector_descriptor_params_.pattern_radius_,
            _params.active_search.separation);

    // 3. matcher params
    matcher_ptr_ = new cv::BFMatcher(_params.matcher.similarity_norm);

    // 4. pinhole params

    /* TO USE WHILE TESTING */
    k_parameters_ = {872.791604, 883.154343, 407.599166, 270.343971};
    distortion_ = {-0.284384, -0.030014};
    pinhole::computeCorrectionModel(k_parameters_,distortion_,correction_);



    //k_parameters_= this->getSensorPtr()->getIntrinsicPtr()->getVector();

//    k_parameters_ = _params.pinhole_params.k_parameters;
//    distortion_ = _params.pinhole_params.distortion;
//    pinhole::computeCorrectionModel(k_parameters_,distortion_,correction_);


    //k_parameters_ = this->getSensorPtr()->getIntrinsicPtr()->getVector();

    //SensorCamera* sensor_camera = (SensorCamera*)(this->getSensorPtr());
    // TO DO: The problem is that "sensor_camera" is void. It doesn't have anything.

    //std::cout << "distortion: " << sensor_camera->getDistortionVector().transpose() << std::endl;

//    distortion_ = sensor_camera->getDistortionVector();
//    correction_ = sensor_camera->getCorrectionVector();

//    //k_parameters_ = sensor_camera->getIntrinsicPtr()->getVector();
//    k_parameters_ = sensor_camera->getPinholeModel().transpose();

}

ProcessorImageLandmark::~ProcessorImageLandmark()
{
    //
}

void ProcessorImageLandmark::preProcess()
{
    image_incoming_ = ((CaptureImage*)incoming_ptr_)->getImage();

    if (last_ptr_ == nullptr) // do this just one time!
    {
        params_.image.width = image_incoming_.cols;
        params_.image.height = image_incoming_.rows;
        active_search_grid_.resizeImage(image_incoming_.cols, image_incoming_.rows);
        std::cout << "resized active-search image size!" << std::endl;
    }

    active_search_grid_.renew();
}

void ProcessorImageLandmark::postProcess()
{
    if (last_ptr_!=nullptr)
        drawFeatures(last_ptr_);
}

unsigned int ProcessorImageLandmark::findLandmarks(const LandmarkBaseList& _landmark_list_in,
                                                          FeatureBaseList& _feature_list_out,
                                                          LandmarkMatchMap& _feature_landmark_correspondences)
{
    /* tracker with project */

    unsigned int roi_width = params_.matcher.roi_width;
    unsigned int roi_heigth = params_.matcher.roi_height;
    unsigned int roi_x;
    unsigned int roi_y;
    std::vector<cv::KeyPoint> candidate_keypoints;
    cv::Mat candidate_descriptors;
    std::vector<cv::DMatch> cv_matches;

    std::cout << "Number of features to track: " << _landmark_list_in.size() << std::endl;

    Eigen::Vector3s wc_translation;
    Eigen::Vector4s wc_orientation;
    referenceWorldToCamera(wc_translation,wc_orientation);

    //FeatureBaseList features_from_landmark;
    for (auto landmark_in_ptr : _landmark_list_in)//_feature_list_in)
    {
        /* project */
        LandmarkPoint3D* landmark_ptr = (LandmarkPoint3D*)landmark_in_ptr;
        Eigen::Vector3s point3D = landmark_ptr->getPosition();//landmark_ptr->getPPtr()->getVector();

        frameTransformation(wc_translation,wc_orientation,point3D);



        Eigen::Vector2s point2D;
        point2D = pinhole::projectPoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),
                                        ((SensorCamera*)(this->getSensorPtr()))->getDistortionVector(),point3D);

//        std::cout << "k_params 1: " << k_parameters_(0) << "\t2: " << k_parameters_(1)
//                  << "\t3: " << k_parameters_(2) << "\t4: " << k_parameters_(3) << std::endl;
//        std::cout << "distortion 1: " << distortion_(0) << "\t2: " << distortion_(1) << std::endl;
//        std::cout << "\nPOINT 2D\nx: " << point2D(0) << "\ty: " << point2D(1) << std::endl;

        if(pinhole::isInImage(point2D,params_.image.width,params_.image.height))
        {
            std::cout << "is in image\n" << std::endl;
            /* tracking */

            roi_x = (point2D[0]) - (roi_heigth / 2);
            roi_y = (point2D[1]) - (roi_width / 2);
            cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

            active_search_grid_.hitCell(point2D);  //TODO: Mirar el hitcell en este punto
            //active_search_grid_.blockCell(roi);

            cv::Mat target_descriptor = landmark_ptr->getDescriptor();


            if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
            {
                //the matcher is now inside the match function
                Scalar normalized_score = match(target_descriptor,candidate_descriptors,candidate_keypoints,cv_matches);

                if (normalized_score > params_.matcher.min_normalized_score)
                {
                    //std::cout << "\t <--TRACKED" << std::endl;
                    FeaturePointImage* incoming_point_ptr = new FeaturePointImage(
                                candidate_keypoints[cv_matches[0].trainIdx], (candidate_descriptors.row(cv_matches[0].trainIdx)),
                            Eigen::Matrix2s::Identity());
                    _feature_list_out.push_back(incoming_point_ptr);

                    incoming_point_ptr->setTrackId(incoming_point_ptr->id());

                    _feature_landmark_correspondences[_feature_list_out.back()] = LandmarkMatch({landmark_in_ptr, normalized_score});
                }
                else
                {
                    //std::cout << "\t <--NOT TRACKED" << std::endl;
                }
                //            for (unsigned int i = 0; i < candidate_keypoints.size(); i++)
                //            {
                //                tracker_candidates_.push_back(candidate_keypoints[i].pt);

                //            }
            }
        }
    }
    std::cout << "Number of Features tracked: " << _feature_list_out.size() << std::endl;
    landmarks_in_image_ = _feature_list_out.size();
    return _feature_list_out.size();
}

bool ProcessorImageLandmark::voteForKeyFrame()
{
    //TODO: Keep the number of landmarks that are in the image and compare it to a fixed number
    return landmarks_in_image_ < 10;
}

unsigned int ProcessorImageLandmark::detectNewFeatures(const unsigned int& _max_features)
{
    std::cout << "\n---------------- detectNewFeatures -------------" << std::endl;
    cv::Rect roi;
    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;
    cv::KeyPointsFilter keypoint_filter;
    unsigned int n_new_features = 0;

    for (unsigned int n_iterations = 0; _max_features == 0 || n_iterations < _max_features; n_iterations++)
    {
        if (active_search_grid_.pickRoi(roi))
        {
            detector_roi_.push_back(roi);
            if (detect(image_last_, roi, new_keypoints, new_descriptors))
            {
                keypoint_filter.retainBest(new_keypoints,1);
                FeaturePointImage* point_ptr = new FeaturePointImage(new_keypoints[0], new_descriptors.row(0), false);
                point_ptr->setTrackId(point_ptr->id());
                addNewFeatureLast(point_ptr);
                active_search_grid_.hitCell(new_keypoints[0]);
                active_search_grid_.blockCell(roi);

                //std::cout << "Added point " << point_ptr->trackId() << " at: " << new_keypoints[0].pt << std::endl;

                n_new_features++;

            }
            else
            {
                active_search_grid_.blockCell(roi);
            }
        }
        else
        {
            break;
        }
    }

    std::cout << "Number of new features detected: " << n_new_features << std::endl;

    return n_new_features;
}

LandmarkBase* ProcessorImageLandmark::createLandmark(FeatureBase* _feature_ptr)
{
    FeaturePointImage* feat_point_image_ptr = (FeaturePointImage*) _feature_ptr;

    Eigen::Vector2s point2D;
    point2D[0] = feat_point_image_ptr->getKeypoint().pt.x;
    point2D[1] = feat_point_image_ptr->getKeypoint().pt.y;

    Scalar depth = 10; // arbitrary value

    Eigen::Vector3s point3D;
    point3D = pinhole::backprojectPoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),
                                        ((SensorCamera*)(this->getSensorPtr()))->getCorrectionVector(),point2D,depth);

    std::cout << "point3D BEFORE CHANGE REF x: " << point3D(0) << "; y: " << point3D(1) << "; z: " << point3D(2) << std::endl;

    Eigen::Vector3s cw_translation; Eigen::Vector4s cw_orientation;
    referenceCameraToWorld(cw_translation, cw_orientation, point3D);

    //cv::waitKey(0);

    return new LandmarkPoint3D(new StateBlock(point3D), new StateBlock(3),point3D,feat_point_image_ptr->getDescriptor());
}

ConstraintBase* ProcessorImageLandmark::createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr)
{
    std::cout << "\tProcessorImageLandmark::createConstraint" << std::endl;
    std::cout << "\t\tFeature: " << ((FeaturePointImage*)_feature_ptr)->getMeasurement()[0]
              << "\t" << ((FeaturePointImage*)_feature_ptr)->getMeasurement()[1] << std::endl;
    std::cout << "\t\tLandmark: "<< ((LandmarkPoint3D*)_landmark_ptr)->getPosition()[0]
              << "\t" << ((LandmarkPoint3D*)_landmark_ptr)->getPosition()[1]
              << "\t" << ((LandmarkPoint3D*)_landmark_ptr)->getPosition()[2] << std::endl;

    Eigen::Vector3s point3D = ((LandmarkPoint3D*)_landmark_ptr)->getPosition();
    Eigen::Vector2s point2D;
    point2D = pinhole::projectPoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),
                                    ((SensorCamera*)(this->getSensorPtr()))->getDistortionVector(),point3D);

    std::cout << "\t\tProjection: "<< point2D[0] << "\t" << point2D[1] << std::endl;

    Eigen::VectorXs intrinsic_values =  this->getSensorPtr()->getIntrinsicPtr()->getVector();
    Eigen::VectorXs distortion = ((SensorCamera*)(this->getSensorPtr()))->getDistortionVector();
    // TO DO: CHANGE THE K_PARAMETERS AND THE OTHERS TO THE APROPRIATE VARIABLE (IF NEEDED)
    return new ConstraintImage(_feature_ptr, getProblem()->getTrajectoryPtr()->getLastFramePtr(), _landmark_ptr, intrinsic_values,distortion);
}


// ==================================================================== My own functions

void ProcessorImageLandmark::frameTransformation(Eigen::Vector3s _wc_translation, Eigen::Vector4s _wc_orientation, Eigen::Vector3s& _point3D)
{

//    Eigen::Vector3s camera_pose = {30,0,0};
//    Eigen::Vector4s camera_orientation = {0,0,0,1};

    Eigen::Matrix3s rot_mat;
    rotationMatrix(rot_mat, _wc_orientation);

    _point3D = rot_mat.transpose()*(_wc_translation - _point3D);

    std::cout << "point3D x: " << _point3D(0) << "; y: " << _point3D(1) << "; z: " << _point3D(2) << std::endl;
}

void ProcessorImageLandmark::rotationMatrix(Eigen::Matrix3s& _rotation_matrix, Eigen::Vector4s _orientation)
{

    _rotation_matrix(0,0) = pow(_orientation(3),2) + pow(_orientation(0),2)
                                - pow(_orientation(1),2) - pow(_orientation(2),2);
    _rotation_matrix(0,1) = 2*(_orientation(0)*_orientation(1) + _orientation(3)*_orientation(2));
    _rotation_matrix(0,2) = 2*(_orientation(0)*_orientation(2) - _orientation(3)*_orientation(1));;
    _rotation_matrix(1,0) = 2*(_orientation(0)*_orientation(1) - _orientation(3)*_orientation(2));;
    _rotation_matrix(1,1) = pow(_orientation(3),2) - pow(_orientation(0),2)
                                + pow(_orientation(1),2) - pow(_orientation(2),2);
    _rotation_matrix(1,2) = 2*(_orientation(1)*_orientation(2) + _orientation(3)*_orientation(0));
    _rotation_matrix(2,0) = 2*(_orientation(0)*_orientation(2) + _orientation(3)*_orientation(1));
    _rotation_matrix(2,1) = 2*(_orientation(1)*_orientation(2) - _orientation(3)*_orientation(0));
    _rotation_matrix(2,2) = pow(_orientation(3),2) - pow(_orientation(0),2)
                                - pow(_orientation(1),2) + pow(_orientation(2),2);
}

void ProcessorImageLandmark::referenceCameraToWorld(Eigen::Vector3s& _cw_translation, Eigen::Vector4s& _cw_orientation, Eigen::Vector3s& _point3D)
{

    Eigen::Vector3s cw_translation;
    Eigen::Vector4s cw_orientation;


    Eigen::Vector3s camera_pose = {0,0,0};
    Eigen::Vector4s camera_orientation = {0,0,0,1};

    Eigen::Vector3s robot_pose = getProblem()->getTrajectoryPtr()->getLastFramePtr()->getPPtr()->getVector();
    Eigen::Vector4s robot_orientation = getProblem()->getTrajectoryPtr()->getLastFramePtr()->getOPtr()->getVector();
    //Eigen::Vector3s robot_pose = {0,0,0};
    //Eigen::Vector4s robot_orientation = {0,0,0,1};

//    std::cout << "rob_pos x: " << robot_pose(0) << "\trob_pos y: " << robot_pose(1) << "\trob_pos z: " << robot_pose(2) << std::endl;
//    std::cout << "rob_orien x: " << robot_orientation(0) << "\trob_orien y: " << robot_orientation(1)
//              << "\trob_orien z: " << robot_orientation(2) << "\trob_orien w: " << robot_orientation(3) << std::endl;


    Eigen::Matrix3s rot_mat;
    rotationMatrix(rot_mat,robot_orientation);

    cw_translation = robot_pose + (rot_mat*camera_pose);
    std::cout << "trans x: " << cw_translation(0) << "; trans y: " << cw_translation(1) << "; trans z: " << cw_translation(2) << std::endl;


    //is this qw?
    cw_orientation(3) = robot_orientation(3)*camera_orientation(3) - robot_orientation(0)*camera_orientation(0)
                    - robot_orientation(1)*camera_orientation(1) - robot_orientation(2)*camera_orientation(2);
    cw_orientation(0) = robot_orientation(3)*camera_orientation(0) + robot_orientation(0)*camera_orientation(3)
                    + robot_orientation(1)*camera_orientation(2) - robot_orientation(2)*camera_orientation(1);
    cw_orientation(1) = robot_orientation(3)*camera_orientation(1) - robot_orientation(0)*camera_orientation(2)
                    + robot_orientation(1)*camera_orientation(3) + robot_orientation(2)*camera_orientation(0);
    //is this qz?
    cw_orientation(2) = robot_orientation(3)*camera_orientation(2) + robot_orientation(0)*camera_orientation(1)
                    - robot_orientation(1)*camera_orientation(0) + robot_orientation(2)*camera_orientation(3);

    std::cout << "orien x: " << cw_orientation(0) << "; orien y: " << cw_orientation(1)
              << "; orien z: " << cw_orientation(2) << "; orien w: " << cw_orientation(3) << std::endl;





    Eigen::Matrix3s rot_mat2;
    rotationMatrix(rot_mat2, cw_orientation);

    _point3D = rot_mat2.transpose()*_point3D + cw_translation;

    std::cout << "point3D USED IN BACKPROJECT x: " << _point3D(0) << "; y: " << _point3D(1) << "; z: " << _point3D(2) << std::endl;

}

void ProcessorImageLandmark::referenceWorldToCamera(Eigen::Vector3s& _wc_translation, Eigen::Vector4s& _wc_orientation)
{
    /* not working */
    //Eigen::Vector3s camera_pose = getProblem()->getSensorPtr("narrow_stereo")->getPPtr()->getVector();
    //Eigen::Vector4s camera_orientation = getProblem()->getSensorPtr("narrow_stereo")->getOPtr()->getVector();

    //Eigen::VectorXs test_k_params = getProblem()->getSensorPtr("narrow_stereo")->getIntrinsicPtr()->getVector();
    /* not working */

    //Eigen::VectorXs test_k_params = this->getSensorPtr()->getIntrinsicPtr()->getVector();

    Eigen::Vector3s camera_pose = {0,0,0};
    Eigen::Vector4s camera_orientation = {0,0,0,1};

    Eigen::Vector3s robot_pose = getProblem()->getTrajectoryPtr()->getLastFramePtr()->getPPtr()->getVector();
    Eigen::Vector4s robot_orientation = getProblem()->getTrajectoryPtr()->getLastFramePtr()->getOPtr()->getVector();
    //Eigen::Vector3s robot_pose = {0,0,0};
    //Eigen::Vector4s robot_orientation = {0,0,0,1};

//    std::cout << "rob_pos x: " << robot_pose(0) << "\trob_pos y: " << robot_pose(1) << "\trob_pos z: " << robot_pose(2) << std::endl;
//    std::cout << "rob_orien x: " << robot_orientation(0) << "\trob_orien y: " << robot_orientation(1)
//              << "\trob_orien z: " << robot_orientation(2) << "\trob_orien w: " << robot_orientation(3) << std::endl;


    Eigen::Matrix3s rot_mat;
    rotationMatrix(rot_mat,robot_orientation);

    _wc_translation = robot_pose + (rot_mat*camera_pose);
    std::cout << "trans x: " << _wc_translation(0) << "; trans y: " << _wc_translation(1) << "; trans z: " << _wc_translation(2) << std::endl;


    //is this qw?
    _wc_orientation(3) = robot_orientation(3)*camera_orientation(3) - robot_orientation(0)*camera_orientation(0)
                    - robot_orientation(1)*camera_orientation(1) - robot_orientation(2)*camera_orientation(2);
    _wc_orientation(0) = robot_orientation(3)*camera_orientation(0) + robot_orientation(0)*camera_orientation(3)
                    + robot_orientation(1)*camera_orientation(2) - robot_orientation(2)*camera_orientation(1);
    _wc_orientation(1) = robot_orientation(3)*camera_orientation(1) - robot_orientation(0)*camera_orientation(2)
                    + robot_orientation(1)*camera_orientation(3) + robot_orientation(2)*camera_orientation(0);
    //is this qz?
    _wc_orientation(2) = robot_orientation(3)*camera_orientation(2) + robot_orientation(0)*camera_orientation(1)
                    - robot_orientation(1)*camera_orientation(0) + robot_orientation(2)*camera_orientation(3);

    std::cout << "orien x: " << _wc_orientation(0) << "; orien y: " << _wc_orientation(1)
              << "; orien z: " << _wc_orientation(2) << "; orien w: " << _wc_orientation(3) << std::endl;

}

Scalar ProcessorImageLandmark::match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors,
                             std::vector<cv::KeyPoint> _candidate_keypoints, std::vector<cv::DMatch>& _cv_matches)
{
    //std::cout << " --> " << _candidate_keypoints.size() << " candidates";

    matcher_ptr_->match(_target_descriptor, _candidate_descriptors, _cv_matches);

    //std::cout << "\n\tBest is: [" << _cv_matches[0].trainIdx << "]:" << _cv_matches[0].distance;

    //std::cout << " | at: " << _candidate_keypoints[_cv_matches[0].trainIdx].pt;

    Scalar normalized_score = 1 - (Scalar)(_cv_matches[0].distance)/detector_descriptor_params_.size_bits_;

    //std::cout << " | score: " << normalized_score;

    return normalized_score;
}

unsigned int ProcessorImageLandmark::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& new_descriptors)
{
    cv::Mat _image_roi;

    adaptRoi(_image_roi, _image, _roi);

    detector_descriptor_ptr_->detect(_image_roi, _new_keypoints);
    detector_descriptor_ptr_->compute(_image_roi, _new_keypoints, new_descriptors);

    for (unsigned int i = 0; i < _new_keypoints.size(); i++)
    {
        _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
        _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;
    }
    return _new_keypoints.size();
}

void ProcessorImageLandmark::trimRoi(cv::Rect& _roi)
{
    if(_roi.x < 0)
    {
        int diff_x = -_roi.x;
        _roi.x = 0;
        _roi.width = _roi.width - diff_x;
    }
    if(_roi.y < 0)
    {
        int diff_y = -_roi.y;
        _roi.y = 0;
        _roi.height = _roi.height - diff_y;
    }
    if((unsigned int)(_roi.x + _roi.width) > params_.image.width)
    {
        int diff_width = params_.image.width - (_roi.x + _roi.width);
        _roi.width = _roi.width+diff_width;
    }
    if((unsigned int)(_roi.y + _roi.height) > params_.image.height)
    {
        int diff_height = params_.image.height - (_roi.y + _roi.height);
        _roi.height = _roi.height+diff_height;
    }
}

void ProcessorImageLandmark::inflateRoi(cv::Rect& _roi)
{
    // now both the detector and descriptor patter_radius is the same, but when not, shouldn't the method have that as input parameter?
    int inflation_rate = detector_descriptor_params_.pattern_radius_;

    _roi.x = _roi.x - inflation_rate;
    _roi.y = _roi.y - inflation_rate;
    _roi.width = _roi.width + 2*inflation_rate;
    _roi.height = _roi.height + 2*inflation_rate;
}

void ProcessorImageLandmark::adaptRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi)
{

    inflateRoi(_roi);
    trimRoi(_roi);

    tracker_roi_inflated_.push_back(_roi);

    _image_roi = _image(_roi);
}

void ProcessorImageLandmark::drawFeatures(CaptureBase* const _last_ptr)
{
    unsigned int counter = 1;
    LandmarkBaseList* last_landmark_list = getProblem()->getMapPtr()->getLandmarkListPtr();
    for (auto landmark_base_ptr : *last_landmark_list)
    {
        LandmarkPoint3D* landmark_ptr = (LandmarkPoint3D*)landmark_base_ptr;
        Eigen::Vector3s point3D = landmark_ptr->getPosition();//landmark_ptr->getPPtr()->getVector();

        Eigen::Vector2s point2D;
        point2D = pinhole::projectPoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),
                                        ((SensorCamera*)(this->getSensorPtr()))->getDistortionVector(),point3D);

//        std::cout << "Landmark " << counter << std::endl;
//        std::cout << "x: " << point2D[0] << "; y: " << point2D[1] << std::endl;
//        std::cout << "is in the image?: "
//                  << pinhole::isInImage(point2D,params_.image.width,params_.image.height) << std::endl;

        if(pinhole::isInImage(point2D,params_.image.width,params_.image.height))
        {

            cv::Point point;
            point.x = point2D[0];
            point.y = point2D[1];

            cv::circle(image_last_, point, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
            cv::putText(image_last_, std::to_string(counter), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
        counter++;
        }
        //counter++;
    }
    cv::Point label_for_landmark_point;
    label_for_landmark_point.x = 3;
    label_for_landmark_point.y = 10;
    cv::putText(image_last_, std::to_string(landmarks_in_image_), label_for_landmark_point,
                cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 0.0, 255.0));

    cv::Point label_for_landmark_point2;
    label_for_landmark_point2.x = 3;
    label_for_landmark_point2.y = 20;
    cv::putText(image_last_, std::to_string(counter), label_for_landmark_point2,
                cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 0.0, 255.0));

    cv::imshow("Feature tracker", image_last_);
}


//namespace wolf{

ProcessorBase* ProcessorImageLandmark::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorImageLandmark* prc_ptr = new ProcessorImageLandmark(*((ProcessorImageParameters*)_params));
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}
//} // namespace wolf
}

// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_image = ProcessorFactory::get().registerCreator("IMAGE LANDMARK", ProcessorImageLandmark::create);
}
} // namespace wolf

//}
