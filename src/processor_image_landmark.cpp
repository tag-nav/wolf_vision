#include "processor_image_landmark.h"

#include "landmark_corner_2D.h"
#include "landmark_point_3d.h"
#include "constraint_corner_2D.h"
#include "constraint_image.h"
#include "pinholeTools.h"

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
    /* problem here */
//    std::string name;
//    for(SensorBaseList::iterator sb = getProblem()->getHardwarePtr()->getSensorListPtr()->begin();
//        sb != getProblem()->getHardwarePtr()->getSensorListPtr()->end(); ++sb)
//                               {
//                                    std::cout << "NAMES" << std::endl;
//                                    SensorBase* sen_base = (SensorBase*)*sb;
//                                    name = sen_base->getName();
//                                    std::cout << name << std::endl;
//                               };
//    Eigen::VectorXs test_k_params = getProblem()->getSensorPtr(name)->getIntrinsicPtr()->getVector();


    k_parameters_ = _params.pinhole_params.k_parameters;
    distortion_ = _params.pinhole_params.distortion;
    pinhole::computeCorrectionModel(k_parameters_,distortion_,correction_);

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

    //FeatureBaseList features_from_landmark;
    for (auto landmark_in_ptr : _landmark_list_in)//_feature_list_in)
    {
        /* project */
        LandmarkPoint3D* landmark_ptr = (LandmarkPoint3D*)landmark_in_ptr;
        Eigen::Vector3s point3D = landmark_ptr->getPosition();//landmark_ptr->getPPtr()->getVector();

        Eigen::Vector2s point2D;
        point2D = pinhole::projectPoint(k_parameters_,distortion_,point3D);

        if(pinhole::isInImage(point2D,params_.image.width,params_.image.height))
        {

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
    point3D = pinhole::backprojectPoint(k_parameters_,correction_,point2D,depth);

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
    point2D = pinhole::projectPoint(k_parameters_,distortion_,point3D);

    std::cout << "\t\tProjection: "<< point2D[0] << "\t" << point2D[1] << std::endl;

    return new ConstraintImage(_feature_ptr, getProblem()->getTrajectoryPtr()->getLastFramePtr() , _landmark_ptr,
                               params_.pinhole_params.k_parameters,params_.pinhole_params.distortion);
}


// ==================================================================== My own functions

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
//    for (auto feature_ptr : *(last_ptr_->getFeatureListPtr()))
//    {
//        FeaturePointImage* point_ptr = (FeaturePointImage*)feature_ptr;
//        if (point_ptr->isKnown())
//        {
//            cv::circle(image_last_, point_ptr->getKeypoint().pt, 7, cv::Scalar(51.0, 255.0, 51.0), 1, 3, 0);
//        }
//        else
//        {
//            cv::circle(image_last_, point_ptr->getKeypoint().pt, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
//        }
//        cv::putText(image_last_, std::to_string(feature_ptr->trackId()), point_ptr->getKeypoint().pt, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
//    }
//    cv::imshow("Feature tracker", image_last_);

    unsigned int counter = 1;
    LandmarkBaseList* last_landmark_list = getProblem()->getMapPtr()->getLandmarkListPtr();
    for (auto landmark_base_ptr : *last_landmark_list)
    {
        LandmarkPoint3D* landmark_ptr = (LandmarkPoint3D*)landmark_base_ptr;
        Eigen::Vector3s point3D = landmark_ptr->getPosition();//landmark_ptr->getPPtr()->getVector();

        Eigen::Vector2s point2D;
        point2D = pinhole::projectPoint(k_parameters_,distortion_,point3D);

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
        }
        counter++;
    }
    cv::imshow("Feature tracker", image_last_);
}


}
