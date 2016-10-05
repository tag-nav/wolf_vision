#include "processor_image_landmark.h"

#include "landmark_corner_2D.h"
#include "landmark_AHP.h"
#include "constraint_corner_2D.h"
#include "constraint_AHP.h"
#include "sensor_camera.h"
#include "pinholeTools.h"

#include <Eigen/Geometry>

namespace wolf
{

ProcessorImageLandmark::ProcessorImageLandmark(ProcessorParamsImage _params) :
    ProcessorTrackerLandmark(PRC_TRACKER_DUMMY, "IMAGE LANDMARK", _params.algorithm.max_new_features),
    matcher_ptr_(nullptr),
    detector_descriptor_ptr_(nullptr),
    params_(_params),
    active_search_grid_(),
    n_feature_(0),
    landmark_idx_non_visible_(0)
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
    feat_lmk_found_.clear();
    landmark_found_number_.clear();
}

void ProcessorImageLandmark::postProcess()
{
    if (last_ptr_!=nullptr)
    {
        cv::Mat image = image_incoming_.clone();
        drawRoi(image, tracker_roi_, cv::Scalar(255.0, 0.0, 255.0));
        drawRoi(image, detector_roi_, cv::Scalar(0.0,255.0, 255.0));
        drawFeatures(image);
        //drawTrackingFeatures(image,tracker_candidates_,tracker_candidates_);
        drawFeaturesFromLandmarks(image);

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

//    std::cout << "Number of features to track: " << _landmark_list_in.size() << std::endl;

    unsigned int lmk_nbr = 0;
    //FeatureBaseList features_from_landmark;
    for (auto landmark_in_ptr : _landmark_list_in)//_feature_list_in)
    {
        std::cout << "\nLandmark number [" << lmk_nbr << "] in ";

        /* project */
        LandmarkAHP* landmark_ptr = (LandmarkAHP*)landmark_in_ptr;
        Eigen::Vector4s point3D_hmg;
        Eigen::Vector3s point2D_hmg;
        Eigen::Vector2s point2D;

        LandmarkInCurrentCamera(landmark_ptr,point3D_hmg);

        point2D_hmg = point3D_hmg.head(3);
        point2D = point2D_hmg.head(2)/point2D_hmg(2);
        point2D = pinhole::distortPoint(((SensorCamera*)(this->getSensorPtr()))->getDistortionVector(),point2D);
        point2D = pinhole::pixellizePoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);

        if(pinhole::isInImage(point2D,params_.image.width,params_.image.height))
        {
            /* tracking */

            roi_x = (point2D[0]) - (roi_heigth / 2);
            roi_y = (point2D[1]) - (roi_width / 2);
            cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

            active_search_grid_.hitCell(point2D);  //TODO: Mirar el hitcell en este punto
            active_search_grid_.blockCell(roi);

            cv::Mat target_descriptor = landmark_ptr->getCvDescriptor();

            std::cout << "pixel: " << point2D.transpose() << std::endl;
            std::cout << "target_descriptor[" << lmk_nbr << "]:\n" << target_descriptor.row(0) << std::endl;

            //lists used to debug
            tracker_roi_.push_back(roi);

            if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
            {
                //the matcher is now inside the match function
                Scalar normalized_score = match(target_descriptor,candidate_descriptors,cv_matches);

                std::cout << "candidate keypoints size: " << candidate_keypoints.size() << std::endl;

                if (normalized_score > params_.matcher.min_normalized_score)
                {
                    std::cout << "FOUND // normalized score: " << normalized_score << std::endl;
                    std::cout << "Feature in pixel: " << candidate_keypoints[cv_matches[0].trainIdx].pt << std::endl;
                    std::cout << "Feature descriptor:\n" << candidate_descriptors.row(cv_matches[0].trainIdx) << std::endl;


                    FeaturePointImage* incoming_point_ptr = new FeaturePointImage(candidate_keypoints[cv_matches[0].trainIdx],
                            (candidate_descriptors.row(cv_matches[0].trainIdx)), Eigen::Matrix2s::Identity());
                    incoming_point_ptr->setTrackId(incoming_point_ptr->id());
                    _feature_list_out.push_back(incoming_point_ptr);

                    _feature_landmark_correspondences[_feature_list_out.back()] = new LandmarkMatch({landmark_in_ptr, normalized_score});

                    feat_lmk_found_.push_back(incoming_point_ptr);
                    landmark_found_number_.push_back(lmk_nbr);
//                    std::cout << "list: " << landmark_found_number_.back() << std::endl;
//                    std::cout << "LMK " << lmk_nbr << "; FEATURE IN POINT X: " << incoming_point_ptr->getKeypoint().pt.x
//                              << "\tY: " << incoming_point_ptr->getKeypoint().pt.y << std::endl;

                }
                else
                    std::cout << "NOT FOUND" << std::endl;

                for (unsigned int i = 0; i < candidate_keypoints.size(); i++)
                {
                    tracker_candidates_.push_back(candidate_keypoints[i].pt);
                }
            }
            else
            {
                //this one means that the detector/descriptor searched the roi, but didn't find ANYTHING at all. So, NOT tracked.
                std::cout << "NOT DETECTED/FOUND" << std::endl;
            }
        }
        else
        {
            std::cout << "NOT in the image" << std::endl;
        }

        lmk_nbr++;
    }
    std::cout << "\n\n\tNumber of Features tracked: " << _feature_list_out.size() << std::endl;
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
                std::vector<cv::KeyPoint> list_keypoints = new_keypoints;
                unsigned int index = 0;
                keypoint_filter.retainBest(new_keypoints,1);
                for( int i = 0; i < list_keypoints.size(); i++)
                {
                    if(list_keypoints[i].pt == new_keypoints[0].pt)
                        index = i;
                }

                std::cout << "response: " << new_keypoints[0].response << std::endl;
                if(new_keypoints[0].response > 0.0005)
                {
                    list_response_.push_back(new_keypoints[0].response);
                    FeaturePointImage* point_ptr = new FeaturePointImage(new_keypoints[0], new_descriptors.row(index), false);
                    point_ptr->setTrackId(point_ptr->id());
                    addNewFeatureLast(point_ptr);
                    active_search_grid_.hitCell(new_keypoints[0]);
                    active_search_grid_.blockCell(roi);

                    //std::cout << "Added point " << point_ptr->trackId() << " at: " << new_keypoints[0].pt << std::endl;

                    n_new_features++;
                }

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
//    FrameBase* anchor_frame = getProblem()->getTrajectoryPtr()->getLastFramePtr();
    FrameBase* anchor_frame = getLastPtr()->getFramePtr();

    Eigen::Vector2s point2D;
    point2D[0] = feat_point_image_ptr->getKeypoint().pt.x;
    point2D[1] = feat_point_image_ptr->getKeypoint().pt.y;

    Scalar distance = 2; // arbitrary value
    Eigen::Vector4s vec_homogeneous;

    point2D = pinhole::depixellizePoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);
    point2D = pinhole::undistortPoint(((SensorCamera*)(this->getSensorPtr()))->getCorrectionVector(),point2D);

    Eigen::Vector3s point3D;
    point3D.head(2) = point2D;
    point3D(2) = 1;

    point3D.normalize();


    vec_homogeneous = {point3D(0),point3D(1),point3D(2),1/distance};
//    std::cout << "vec_homogeneous_2 x: " << vec_homogeneous(0) << "; y: " << vec_homogeneous(1) << "; z: " << vec_homogeneous(2)
//              << "; inv_dist: " << vec_homogeneous(3) << std::endl;

    return new LandmarkAHP(vec_homogeneous, anchor_frame, getSensorPtr(), feat_point_image_ptr->getDescriptor());
}

ConstraintBase* ProcessorImageLandmark::createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr)
{

    //    std::cout << "\nProcessorImageLandmark::createConstraint" << std::endl;
    if (((LandmarkAHP*)_landmark_ptr)->getAnchorFrame() == last_ptr_->getFramePtr())
    {
        //std::cout << "Are equal" << std::endl;
        return nullptr;
    }
    else// (((LandmarkAHP*)_landmark_ptr)->getAnchorFrame() != last_ptr_->getFramePtr())
    {
        return new ConstraintAHP(_feature_ptr, last_ptr_->getFramePtr(),(LandmarkAHP*)_landmark_ptr);
    }
}


// ==================================================================== My own functions

void ProcessorImageLandmark::LandmarkInCurrentCamera(LandmarkAHP* _landmark,Eigen::Vector4s& _point3D_hmg)
{
    Eigen::Vector3s pwr1 = getLastPtr()->getFramePtr()->getPPtr()->getVector();
//            getProblem()->getTrajectoryPtr()->getLastFramePtr()->getPPtr()->getVector();
    Eigen::Vector3s pwr0 = _landmark->getAnchorFrame()->getPPtr()->getVector();
    Eigen::Vector3s prc = this->getSensorPtr()->getPPtr()->getVector();

    Eigen::Quaternion<Scalar> qwr1, qwr0, qrc;
    Eigen::Vector4s quaternion_anchor = _landmark->getAnchorFrame()->getOPtr()->getVector();
    Eigen::Vector4s quaternion_current_frame = getLastPtr()->getFramePtr()->getOPtr()->getVector();
    Eigen::Vector4s quaternion_sensor = this->getSensorPtr()->getOPtr()->getVector();
    qwr0 = quaternion_anchor;
    qwr1 = quaternion_current_frame;
    qrc = quaternion_sensor;

//            Eigen::Matrix<T,3,1>  phc, phw ;
    Eigen::Vector4s _lmk_hmg = _landmark->getPPtr()->getVector();


    Eigen::Matrix4s M_R0_C0, M_W_R0, M_R1_W, M_C1_R1;



    // FROM CAMERA0 TO WORLD

    M_R0_C0.block(0,0,3,3) << qrc.matrix();
    M_R0_C0.col(3).head(3) << prc;
    M_R0_C0.row(3) << 0, 0, 0, 1;

    M_W_R0.block(0,0,3,3) << qwr0.matrix();
    M_W_R0.col(3).head(3) << pwr0;
    M_W_R0.row(3) << 0, 0, 0, 1;

    Eigen::Vector4s test;
    test = M_W_R0*M_R0_C0*_lmk_hmg;



    // FROM WORLD TO CAMERA1

    M_R1_W.block(0,0,3,3) << qwr1.matrix().transpose();
    M_R1_W.col(3).head(3) << (-qwr1.matrix().transpose()*pwr1);
    M_R1_W.row(3) << 0, 0,0, 1;

    M_C1_R1.block(0,0,3,3) << qrc.matrix().transpose();
    M_C1_R1.col(3).head(3) << (-qrc.matrix().transpose()*prc);
    M_C1_R1.row(3) << 0, 0, 0, 1;

    Eigen::Vector4s test2;
    test2 = M_C1_R1*M_R1_W*test;


    _point3D_hmg(0) = test2(0);
    _point3D_hmg(1) = test2(1);
    _point3D_hmg(2) = test2(2);
    _point3D_hmg(3) = test2(3);
}

Scalar ProcessorImageLandmark::match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors, std::vector<cv::DMatch>& _cv_matches)
{
    matcher_ptr_->match(_target_descriptor, _candidate_descriptors, _cv_matches);
    Scalar normalized_score = 1 - (Scalar)(_cv_matches[0].distance)/detector_descriptor_params_.size_bits_;
    //std::cout << "normalized score: " << normalized_score << std::endl;
    return normalized_score;
}

unsigned int ProcessorImageLandmark::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& new_descriptors)
{
    cv::Mat _image_roi;

    adaptRoi(_image_roi, _image, _roi);

//    std::cout << "ROI: " << _image_roi.cols << "; " << _image_roi.rows << std::endl;

    detector_descriptor_ptr_->detect(_image_roi, _new_keypoints);
    detector_descriptor_ptr_->compute(_image_roi, _new_keypoints, new_descriptors);

    for (unsigned int i = 0; i < _new_keypoints.size(); i++)
    {
        _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
        _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;
    }
//    std::cout << "Features detected: " << _new_keypoints.size() << std::endl;
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

    //inflateRoi(_roi);
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

    for(auto candidate_point : _candidates_list)
    {
        //candidate - cyan
        cv::circle(_image, candidate_point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);
    }

    cv::imshow("Feature tracker", _image);

}

void ProcessorImageLandmark::drawFeaturesFromLandmarks(cv::Mat _image)
{

    Eigen::VectorXs v(landmark_found_number_.size());
    unsigned int j =0;
    for (auto lmk_found_nbr_ : landmark_found_number_)
    {
        v(j) = lmk_found_nbr_;
        j++;
    }

    unsigned int i = 0;
    for(auto feature_point : feat_lmk_found_)
    {
        //candidate - cyan
        cv::Point2f point = ((FeaturePointImage*)feature_point)->getKeypoint().pt;
        cv::circle(_image, point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);
        std::cout << "feature number [" << (unsigned int)v(i)+1 << "] in: " << point << std::endl;
        cv::Point2f point2 = point;
        point2.x = point2.x - 16;
        cv::putText(_image, std::to_string((unsigned int)v(i)+1), point2,
                    cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100.0, 100.0, 255.0));
        i++;
    }

    cv::imshow("Feature tracker", _image);
}

void ProcessorImageLandmark::drawFeatures(cv::Mat _image)
{
    unsigned int counter = 0;
//    cv::Mat image = image_incoming_.clone();
    LandmarkBaseList* last_landmark_list = getProblem()->getMapPtr()->getLandmarkListPtr();

    unsigned int response_counter = 0;
    Eigen::VectorXs response_vector(last_landmark_list->size());
    for (auto response : list_response_)
    {
        response_vector(response_counter) = response;
        response_counter++;
    }

    for (auto landmark_base_ptr : *last_landmark_list)
    {
        LandmarkAHP* landmark_ptr = (LandmarkAHP*)landmark_base_ptr;
        Eigen::Vector4s point3D_hmg;
        Eigen::Vector3s point2D_hmg;
        Eigen::Vector2s point2D;

        LandmarkInCurrentCamera(landmark_ptr,point3D_hmg);

        point2D_hmg = point3D_hmg.head(3);
        point2D = point2D_hmg.head(2)/point2D_hmg(2);
        point2D = pinhole::distortPoint(((SensorCamera*)(this->getSensorPtr()))->getDistortionVector(),point2D);
        point2D = pinhole::pixellizePoint(this->getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);

        if(pinhole::isInImage(point2D,params_.image.width,params_.image.height))
        {

            cv::Point2f point;
            point.x = point2D[0];
            point.y = point2D[1];

            std::cout << "landmark proj number [" << landmark_ptr->id() << "] in: " << point << std::endl;

            cv::circle(_image, point, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
            cv::putText(_image, std::to_string(landmark_ptr->id()), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
//            cv::putText(image, std::to_string(response_vector[counter]), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
            counter++;
        }
    }
    cv::Point label_for_landmark_point;
    label_for_landmark_point.x = 3;
    label_for_landmark_point.y = 10;
    cv::putText(_image, std::to_string(landmarks_tracked_), label_for_landmark_point,
                cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 0.0, 255.0));

    cv::Point label_for_landmark_point2;
    label_for_landmark_point2.x = 3;
    label_for_landmark_point2.y = 20;
    cv::putText(_image, std::to_string(counter), label_for_landmark_point2,
                cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 0.0, 255.0));

    std::cout << "\t\tTotal landmarks: " << counter << std::endl;

    cv::imshow("Feature tracker", _image);
    //_image = image;
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
