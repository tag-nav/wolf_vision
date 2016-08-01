#include "processor_image_landmark.h"

#include "landmark_corner_2D.h"
#include "landmark_AHP.h"
#include "constraint_corner_2D.h"
#include "constraint_image.h"
#include "constraint_image_new_landmark.h"
#include "sensor_camera.h"
#include "pinholeTools.h"

#include <Eigen/Geometry>

namespace wolf
{

ProcessorImageLandmark::ProcessorImageLandmark(ProcessorParamsImage _params) :
    ProcessorTrackerLandmark(PRC_TRACKER_DUMMY, "IMAGE LANDMARK", _params.algorithm.max_new_features), n_feature_(0), landmark_idx_non_visible_(0),
    matcher_ptr_(nullptr), detector_descriptor_ptr_(nullptr), params_(_params),
    active_search_grid_()
{
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

            std::cout << "nominal pattern radius: " << _dd_params->nominal_pattern_radius << std::endl;
            std::cout << "scale factor: " << params_orb->scaleFactor << std::endl;
            std::cout << "nlevels: " << params_orb->nlevels << std::endl;

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


    tracker_roi_.clear();
    detector_roi_.clear();
    tracker_candidates_.clear();
}

void ProcessorImageLandmark::postProcess()
{
    if (last_ptr_!=nullptr)
    {
        cv::Mat image;
        drawFeatures(image);
        drawRoi(image, tracker_roi_, cv::Scalar(255.0, 0.0, 255.0));
        drawRoi(image, detector_roi_, cv::Scalar(0.0,255.0, 255.0));
        drawTrackingFeatures(image,tracker_candidates_,tracker_candidates_);
    }
    if (origin_ptr_!=nullptr)
    {
        for(auto feature_ptr : *origin_ptr_->getFeatureListPtr())
        {
            for(auto constraint_ptr : *feature_ptr->getConstraintListPtr())
            {

//                Eigen::Vector2s residuals;
//                Eigen::Vector3s robot_p = origin_ptr_->getFramePtr()->getPPtr()->getVector();
//                Eigen::Vector4s robot_o = origin_ptr_->getFramePtr()->getOPtr()->getVector();
//                Eigen::Vector4s landmark = constraint_ptr->getLandmarkOtherPtr()->getPPtr()->getVector();
//                (*((ConstraintImage*) constraint_ptr))(robot_p.data(), robot_o.data(), landmark.data(), residuals.data());
            }
        }

    }
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

    //std::cout << "Number of features to track: " << _landmark_list_in.size() << std::endl;

    //referenceWorldToCamera(world2cam_translation_,world2cam_orientation_);

    Eigen::Vector3s translation;
    Eigen::Matrix3s rotation;

//    changeOfReferenceFrame(translation,rotation);

    //FeatureBaseList features_from_landmark;
    for (auto landmark_in_ptr : _landmark_list_in)//_feature_list_in)
    {
        /* project */
        LandmarkAHP* landmark_ptr = (LandmarkAHP*)landmark_in_ptr;

        changeOfReferenceFrame(landmark_ptr,translation,rotation);

        Eigen::Vector3s point3D;

        getLandmarkInReference(landmark_ptr,translation,rotation,point3D);


        Eigen::Vector2s point2D;
//        point2D = pinhole::projectPoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),
//                                        ((SensorCamera*)(this->getSensorPtr()))->getDistortionVector(),point3D);
        point2D = pinhole::projectPointToNormalizedPlane(point3D);
        point2D = pinhole::pixellizePoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);

        std::cout << "point2D x: " << point2D(0) << "\ty: " << point2D(1) << std::endl;

        if(pinhole::isInImage(point2D,params_.image.width,params_.image.height))
        {
            std::cout << "is in image" << std::endl;
            /* tracking */

            roi_x = (point2D[0]) - (roi_heigth / 2);
            roi_y = (point2D[1]) - (roi_width / 2);
            cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

            active_search_grid_.hitCell(point2D);  //TODO: Mirar el hitcell en este punto
            active_search_grid_.blockCell(roi);

            cv::Mat target_descriptor = landmark_ptr->getCvDescriptor();


            //lists used to debug
            tracker_roi_.push_back(roi);

            if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
            {
                //the matcher is now inside the match function
                Scalar normalized_score = match(target_descriptor,candidate_descriptors,candidate_keypoints,cv_matches);

                if (normalized_score > params_.matcher.min_normalized_score)
                {
                    std::cout << "TRACKED\n\n" << std::endl;
                    FeaturePointImage* incoming_point_ptr = new FeaturePointImage(
                                candidate_keypoints[cv_matches[0].trainIdx], (candidate_descriptors.row(cv_matches[0].trainIdx)),
                            Eigen::Matrix2s::Identity());
                    _feature_list_out.push_back(incoming_point_ptr);

                    incoming_point_ptr->setTrackId(incoming_point_ptr->id());

                    _feature_landmark_correspondences[_feature_list_out.back()] = new LandmarkMatch({landmark_in_ptr, normalized_score});
                }
                else
                {
                    std::cout << "NOT TRACKED" << std::endl;
                }
                for (unsigned int i = 0; i < candidate_keypoints.size(); i++)
                {
                    tracker_candidates_.push_back(candidate_keypoints[i].pt);

                }
            }
            else
            {
                //this one means that the detector/descriptor searched the roi, but didn't find ANYTHING at all. So, NOT tracked.
                std::cout << "not detected / NOT TRACKED\n\n" << std::endl;
            }
        }
        else
            std::cout << "is NOT in image" << std::endl;
    }
    //std::cout << "Number of Features tracked: " << _feature_list_out.size() << std::endl;
    landmarks_tracked_ = _feature_list_out.size();
    return _feature_list_out.size();
}

bool ProcessorImageLandmark::voteForKeyFrame()
{
    //TODO: Keep the number of landmarks that are in the image and compare it to a fixed number
    return landmarks_tracked_ < 10;
}

unsigned int ProcessorImageLandmark::detectNewFeatures(const unsigned int& _max_features)
{
    //std::cout << "\n---------------- detectNewFeatures -------------" << std::endl;
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

    //std::cout << "Number of new features detected: " << n_new_features << std::endl;

    return n_new_features;
}

LandmarkBase* ProcessorImageLandmark::createLandmark(FeatureBase* _feature_ptr)
{
    FeaturePointImage* feat_point_image_ptr = (FeaturePointImage*) _feature_ptr;

    Eigen::Vector3s point2D;
    point2D[0] = feat_point_image_ptr->getKeypoint().pt.x;
    point2D[1] = feat_point_image_ptr->getKeypoint().pt.y;
    point2D[2] = 1;

//    std::cout << "point2D x: " << point2D(0) << "; y: " << point2D(1) << "; z: " << point2D(2) << std::endl;
    Scalar depth = 2; // arbitrary value

//    std::cout << "K: " << this->getSensorPtr()->getIntrinsicPtr()->getVector().transpose() << std::endl;
//    std::cout << "distort: " << ((SensorCamera*)(this->getSensorPtr()))->getDistortionVector().transpose() << std::endl;
//    std::cout << "correct: " << ((SensorCamera*)(this->getSensorPtr()))->getCorrectionVector().transpose() << std::endl;

    Eigen::Vector4s k_params = this->getSensorPtr()->getIntrinsicPtr()->getVector();
    Eigen::Matrix3s K;
    K(0,0) = k_params(2);
    K(0,1) = 0;
    K(0,2) = k_params(0);
    K(1,0) = 0;
    K(1,1) = k_params(3);
    K(1,2) = k_params(1);
    K(2,0) = 0;
    K(2,1) = 0;
    K(2,2) = 1;

    Eigen::Vector3s unitary_vector;
    unitary_vector = K.inverse() * point2D;
    unitary_vector.normalize();
//    std::cout << "unitary_vector: " << unitary_vector(0) << "\t" << unitary_vector(1) << "\t" << unitary_vector(2) << std::endl;

    FrameBase* anchor_frame = getProblem()->getTrajectoryPtr()->getLastFramePtr();

    // TODO: Poner el anchor del punto (ahora mismo está en el 0 del world, pero no hay código por si cambia)


    Eigen::Vector4s vec_homogeneous = {unitary_vector(0),unitary_vector(1),unitary_vector(2),1/depth};
//    std::cout << "unitary_vec x: " << unitary_vec(0) << "; y: " << unitary_vec(1) << "; z: " << unitary_vec(2) << std::endl;

    return new LandmarkAHP(vec_homogeneous, anchor_frame, getSensorPtr(), feat_point_image_ptr->getDescriptor());
}

ConstraintBase* ProcessorImageLandmark::createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr)
{
//    std::cout << "\nProcessorImageLandmark::createConstraint" << std::endl;
    if (((LandmarkAHP*)_landmark_ptr)->getAnchorFrame() == last_ptr_->getFramePtr())
    {
        return new ConstraintImageNewLandmark(_feature_ptr, last_ptr_->getFramePtr(),(LandmarkAHP*)_landmark_ptr);
        //std::cout << "DO NOTHING" << std::endl;
    }
    else
        return new ConstraintImage(_feature_ptr, last_ptr_->getFramePtr(),(LandmarkAHP*)_landmark_ptr);
}


// ==================================================================== My own functions

void ProcessorImageLandmark::getLandmarkInReference(LandmarkAHP* _landmark, Eigen::Vector3s _translation, Eigen::Matrix3s _rotation, Eigen::Vector3s& _point3D)
{
    //point3D
    Eigen::Vector4s landmark_point = _landmark->getPPtr()->getVector();
    Eigen::Vector3s m;
    m(0) = landmark_point(0);
    m(1) = landmark_point(1);
    m(2) = landmark_point(2);

    Eigen::Vector3s v;
    v = (_rotation * m) + (_translation * landmark_point(3));

    _point3D(0) = v(0) / landmark_point(3);
    _point3D(1) = v(1) / landmark_point(3);
    _point3D(2) = v(2) / landmark_point(3);
}

void ProcessorImageLandmark::changeOfReferenceFrame(LandmarkAHP* _landmark,Eigen::Vector3s& _translation, Eigen::Matrix3s& _rotation)
{

    //THIS IS OK
    Eigen::Vector3s translation_robot2camera = this->getSensorPtr()->getPPtr()->getVector();
    Eigen::Vector4s quaternion_robot2camera = this->getSensorPtr()->getOPtr()->getVector();
    Eigen::Matrix3s rotation_robot2camera;
    rotationMatrix(rotation_robot2camera, quaternion_robot2camera);

    Eigen::Vector3s translation_F1_world2robot = getProblem()->getTrajectoryPtr()->getLastFramePtr()->getPPtr()->getVector();
    Eigen::Vector4s quaternion_F1_world2robot = getProblem()->getTrajectoryPtr()->getLastFramePtr()->getOPtr()->getVector();
    Eigen::Matrix3s rotation_F1_world2robot;
    rotationMatrix(rotation_F1_world2robot,quaternion_F1_world2robot);

    Eigen::Vector3s translation_F0_world2robot = _landmark->getAnchorFrame()->getPPtr()->getVector(); // is the robot pose
    Eigen::Vector4s quaternion_F0_world2robot = _landmark->getAnchorFrame()->getOPtr()->getVector(); // is the robot orientation
    Eigen::Matrix3s rotation_F0_world2robot;
    rotationMatrix(rotation_F0_world2robot, quaternion_F0_world2robot);


    // camera0 to world
    Eigen::Matrix3s rotation_world2camera;
    rotation_world2camera = rotation_F0_world2robot*rotation_robot2camera;

    Eigen::Vector3s translation_world2camera;
    translation_world2camera = (rotation_F0_world2robot*translation_robot2camera) + translation_F0_world2robot;


    // world to camera1 (inversion)
    Eigen::Matrix3s rotation_camera1_2world;
    rotation_camera1_2world = rotation_robot2camera.transpose()*rotation_F1_world2robot.transpose();

    Eigen::Vector3s translation_camera1_2world;
    translation_camera1_2world = (rotation_robot2camera.transpose()*(-rotation_F1_world2robot.transpose()*translation_F1_world2robot)) +
            (-rotation_robot2camera.transpose()*translation_robot2camera);


    //camera0 to camera1
    Eigen::Matrix3s rotation_camera1_2camera;
    rotation_camera1_2camera = rotation_camera1_2world * rotation_world2camera;

    Eigen::Vector3s translation_camera1_2camera;
    translation_camera1_2camera = (rotation_camera1_2world*translation_world2camera) + translation_camera1_2world;

    _translation = translation_camera1_2camera;
    _rotation = rotation_camera1_2camera;
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

Scalar ProcessorImageLandmark::match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors,
                             std::vector<cv::KeyPoint> _candidate_keypoints, std::vector<cv::DMatch>& _cv_matches)
{
    matcher_ptr_->match(_target_descriptor, _candidate_descriptors, _cv_matches);
    Scalar normalized_score = 1 - (Scalar)(_cv_matches[0].distance)/detector_descriptor_params_.size_bits_;

    return normalized_score;
}

unsigned int ProcessorImageLandmark::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& new_descriptors)
{
    cv::Mat _image_roi;

    adaptRoi(_image_roi, _image, _roi);

    std::cout << "ROI: " << _image_roi.cols << "; " << _image_roi.rows << std::endl;

    detector_descriptor_ptr_->detect(_image_roi, _new_keypoints);
    detector_descriptor_ptr_->compute(_image_roi, _new_keypoints, new_descriptors);

    for (unsigned int i = 0; i < _new_keypoints.size(); i++)
    {
        _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
        _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;
    }
    std::cout << "Features detected: " << _new_keypoints.size() << std::endl;
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

void ProcessorImageLandmark::drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color)
{
    for (auto roi : _roi_list)
    {
        cv::rectangle(_image, roi, _color, 1, 8, 0);
    }
    cv::imshow("Feature tracker", _image);
}

void ProcessorImageLandmark::drawTrackingFeatures(cv::Mat _image, std::list<cv::Point> _target_list, std::list<cv::Point> _candidates_list)
{
    // These "tracking features" are the feature to be used in tracking as well as its candidates

    for(auto target_point : _target_list)
    {
        //target
        //cv::circle(_image, target_point, 2, cv::Scalar(0.0, 255.0, 255.0), -1, 8, 0);
    }
    for(auto candidate_point : _candidates_list)
    {
        //candidate - cyan
        cv::circle(_image, candidate_point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);
    }

    cv::imshow("Feature tracker", _image);

}

void ProcessorImageLandmark::drawFeatures(cv::Mat& _image)
{
    unsigned int counter = 0;
    cv::Mat image = image_incoming_.clone();
    LandmarkBaseList* last_landmark_list = getProblem()->getMapPtr()->getLandmarkListPtr();

    Eigen::Vector3s translation;
    Eigen::Matrix3s rotation;

    for (auto landmark_base_ptr : *last_landmark_list)
    {
        LandmarkAHP* landmark_ptr = (LandmarkAHP*)landmark_base_ptr;

        changeOfReferenceFrame(landmark_ptr,translation,rotation);

        Eigen::Vector3s point3D;

        getLandmarkInReference(landmark_ptr,translation,rotation,point3D);

        Eigen::Vector2s point2D;
//        point2D = pinhole::projectPoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),
//                                        ((SensorCamera*)(this->getSensorPtr()))->getDistortionVector(),point3D);
        point2D = pinhole::projectPointToNormalizedPlane(point3D);
        point2D = pinhole::pixellizePoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);

        std::cout << "Landmark " << counter << std::endl;
        std::cout << "x: " << point2D[0] << "; y: " << point2D[1] << std::endl;
        std::cout << "is in the image?: "
                  << pinhole::isInImage(point2D,params_.image.width,params_.image.height) << std::endl;

        if(pinhole::isInImage(point2D,params_.image.width,params_.image.height))
        {

            cv::Point point;
            point.x = point2D[0];
            point.y = point2D[1];

            cv::circle(image, point, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
            cv::putText(image, std::to_string(counter), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
            counter++;
        }
    }
    cv::Point label_for_landmark_point;
    label_for_landmark_point.x = 3;
    label_for_landmark_point.y = 10;
    cv::putText(image, std::to_string(landmarks_tracked_), label_for_landmark_point,
                cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 0.0, 255.0));

    cv::Point label_for_landmark_point2;
    label_for_landmark_point2.x = 3;
    label_for_landmark_point2.y = 20;
    cv::putText(image, std::to_string(counter), label_for_landmark_point2,
                cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 0.0, 255.0));

    std::cout << "\n\n\n\nlandmarks tracked: " << landmarks_tracked_ << "\ttotal landmarks: " << counter << std::endl;

    cv::imshow("Feature tracker", image);
    _image = image;
}


//namespace wolf{

ProcessorBase* ProcessorImageLandmark::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorImageLandmark* prc_ptr = new ProcessorImageLandmark(*((ProcessorParamsImage*)_params));
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
