// Wolf includes
#include "processor_image.h"

// OpenCV includes

// other includes
#include <bitset>
#include <algorithm>

namespace wolf
{

ProcessorImage::ProcessorImage(ProcessorImageParameters _params) :
    ProcessorTrackerFeature(PRC_TRACKER_IMAGE, _params.algorithm.max_new_features),
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

}

//Destructor
ProcessorImage::~ProcessorImage()
{

}

void ProcessorImage::preProcess()
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

    //The visualization part is only for debugging. The casts above are necessary.

    if(last_ptr_ != nullptr)
        resetVisualizationFlag(*(last_ptr_->getFeatureListPtr()));

    // Clear of the lists used to debug
    tracker_roi_.clear();
    tracker_roi_inflated_.clear();
    detector_roi_.clear();
    tracker_target_.clear();
    tracker_candidates_.clear();
}

void ProcessorImage::postProcess()
{
    if (last_ptr_!=nullptr)
        drawFeatures(last_ptr_);
//    drawRoi(image_last_,detector_roi_,cv::Scalar(88.0, 70.0, 255.0));   //detector roi(now only shown when it collides with the the image)
//    drawRoi(image_last_,tracker_roi_, cv::Scalar(88.0, 70.0, 255.0));   //tracker roi
//    drawRoi(image_last_,tracker_roi_inflated_,cv::Scalar(225.0, 0.0, 255.0));   //inflated roi(now only shown when it collides with the the image)
//    drawTrackingFeatures(image_last_,tracker_target_,tracker_candidates_);
}

bool ProcessorImage::correctFeatureDrift(const FeatureBase* _origin_feature, const FeatureBase* _last_feature, FeatureBase* _incoming_feature)
{
    std::vector<cv::DMatch> matches_mat;
    FeaturePointImage* feat_incoming_ptr = (FeaturePointImage*)_incoming_feature;
    FeaturePointImage* feat_origin_ptr = (FeaturePointImage*)_origin_feature;

    std::cout << "=============== DRIFT =================" << std::endl;

    cv::Mat origin_descriptor = feat_origin_ptr->getDescriptor();
    cv::Mat incoming_descriptor = feat_incoming_ptr->getDescriptor();

    std::vector<cv::KeyPoint> origin_keypoint;
    origin_keypoint.push_back(feat_origin_ptr->getKeypoint());
    //the matcher is now inside the match function
    Scalar normalized_score = match(origin_descriptor,incoming_descriptor,origin_keypoint,matches_mat);

    std::cout << "\n=============== END DRIFT =================" << std::endl;

    if(normalized_score > 0.8)
    {
        return true;
    }
    else
    {
        /* CORRECT */

        std::cout << "=============== CORRECTION =================" << std::endl;

        unsigned int roi_width = params_.matcher.roi_width;
        unsigned int roi_heigth = params_.matcher.roi_height;
        unsigned int roi_x;
        unsigned int roi_y;

        std::vector<cv::KeyPoint> correction_keypoints;
        cv::Mat correction_descriptors;
        std::vector<cv::DMatch> correction_matches;

        FeaturePointImage* feat_last_ptr = (FeaturePointImage*)_last_feature;
        active_search_grid_.hitCell(feat_last_ptr->getKeypoint());

        std::cout << "Search feature last: " << feat_last_ptr->trackId() << " at: " << feat_last_ptr->getKeypoint().pt;

        roi_x = (feat_last_ptr->getKeypoint().pt.x) - (roi_heigth / 2);
        roi_y = (feat_last_ptr->getKeypoint().pt.y) - (roi_width / 2);
        cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

        if (detect(image_incoming_, roi, correction_keypoints, correction_descriptors))
        {

            //the matcher is now inside the match function
            Scalar normalized_score_correction = match(origin_descriptor,correction_descriptors,correction_keypoints,correction_matches);


            //TODO: Mirar sobre los features que parecen ser el mismo.
            //TODO: Cuando haces el detect sobre el origin, asegurate que eso no es un track ya existente. Puede estar relacionado con el de arriba.
            //



            if(normalized_score_correction > 0.8)
            {
//                FeaturePointImage* incoming_point_ptr = new FeaturePointImage(
//                        correction_keypoints[correction_matches[0].trainIdx], (correction_descriptors.row(correction_matches[0].trainIdx)),
//                        feat_origin_ptr->isKnown());

//                incoming_point_ptr->setTrackId(feat_incoming_ptr->trackId());

                feat_incoming_ptr->setKeypoint(correction_keypoints[correction_matches[0].trainIdx]);
                feat_incoming_ptr->setDescriptor(correction_descriptors.row(correction_matches[0].trainIdx));

                //feat_incoming_ptr->destruct();
                //feat_incoming_ptr = incoming_point_ptr;
                //delete incoming_point_ptr;

                //_feature_matches[incoming_point_ptr] = FeatureMatch({feature_base_ptr, normalized_score});

                std::cout << "\n=============== END CORRECTION true =================" << std::endl;
                return true;
            }
            else
            {
                std::cout << "\n=============== END CORRECTION false =================" << std::endl;
                return false;
            }


        }
        std::cout << "\n=============== No CORRECTION -> false =================" << std::endl;
        return false;
    }
}

unsigned int ProcessorImage::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
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

unsigned int ProcessorImage::detectNewFeatures(const unsigned int& _max_new_features)
{
    std::cout << "\n---------------- detectNewFeatures -------------" << std::endl;
    cv::Rect roi;
    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;
    cv::KeyPointsFilter keypoint_filter;
    unsigned int n_new_features = 0;

    for (unsigned int n_iterations = 0; _max_new_features == 0 || n_iterations < _max_new_features; n_iterations++)
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

void ProcessorImage::resetVisualizationFlag(FeatureBaseList& _feature_list_last)
{
    for (auto feature_base_last_ptr : _feature_list_last)
    {
        FeaturePointImage* feature_last_ptr = (FeaturePointImage*)feature_base_last_ptr;
        feature_last_ptr->setIsKnown(true);
    }
}

void ProcessorImage::filterFeatureLists(FeatureBaseList _original_list, FeatureBaseList& _filtered_list)
{
    unsigned int counter_repeated_features = 0;
    unsigned int counter_total_repeated_features = 0;

    for (std::list<FeatureBase*>::const_iterator first_feature_iterator = _original_list.begin();
                                                 first_feature_iterator != _original_list.end();
                                                 first_feature_iterator++)
    {
        FeaturePointImage* tested_feature_ptr = (FeaturePointImage*)*first_feature_iterator;
        bool repeated_feature = false;

        for (std::list<FeatureBase*>::const_iterator second_feature_iterator = first_feature_iterator;
                                                     second_feature_iterator != _original_list.begin();
                                                     second_feature_iterator--)
        {
           FeaturePointImage* secondary_feature_ptr = (FeaturePointImage*)*second_feature_iterator;

           if(tested_feature_ptr->getKeypoint().pt == secondary_feature_ptr->getKeypoint().pt && tested_feature_ptr->id() != secondary_feature_ptr->id())
           {
               std::cout << "feature 1 track id: " << (int)tested_feature_ptr->trackId() << std::endl;
               std::cout << "feature 1 point: " << tested_feature_ptr->getKeypoint().pt << std::endl;
               std::cout << "feature 2 track id: " << (int)secondary_feature_ptr->trackId() << std::endl;
               std::cout << "feature 2 point: " << secondary_feature_ptr->getKeypoint().pt << std::endl;
               std::cout << "\t\t\t\tRepeated feature" << std::endl;
               counter_total_repeated_features++;
               repeated_feature = true;
           }
        }
        if(!repeated_feature)
        {
            std::cout << "ADDED feature track id: " << (int)tested_feature_ptr->trackId() << std::endl;
            std::cout << "ADDED feature point: " << tested_feature_ptr->getKeypoint().pt << std::endl;
            std::cout << "\t\tAdded feature" << std::endl;
            _filtered_list.push_back(tested_feature_ptr);
        }
        else
        {
            std::cout << "REPEATED feature track id: " << (int)tested_feature_ptr->trackId() << std::endl;
            std::cout << "REPEATED feature point: " << tested_feature_ptr->getKeypoint().pt << std::endl;
            std::cout << "\t\tDiscarted feature" << std::endl;
            counter_repeated_features++;
        }
    }
    std::cout << "feature_list_in SIZE: " << _original_list.size() << std::endl;
    std::cout << "counter of repeated features: " << counter_repeated_features << std::endl;
    std::cout << "counter of TOTAL repeated features: " << counter_total_repeated_features << std::endl;
    std::cout << "filtered_list SIZE: " << _filtered_list.size() << std::endl;

}

unsigned int ProcessorImage::trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out,
                                           FeatureMatchMap& _feature_matches)
{
    std::cout << "\n---------------- trackFeatures ----------------" << std::endl;

    unsigned int roi_width = params_.matcher.roi_width;
    unsigned int roi_heigth = params_.matcher.roi_height;
    unsigned int roi_x;
    unsigned int roi_y;
    std::vector<cv::KeyPoint> candidate_keypoints;
    cv::Mat candidate_descriptors;
    std::vector<cv::DMatch> cv_matches;

    std::cout << "Number of features to track: " << _feature_list_in.size() << std::endl;

    //Asegurarse que el target no está repetido (que dos targets esten muy cerca el uno del otro)
    FeatureBaseList filtered_list;
    filterFeatureLists(_feature_list_in,filtered_list);


    for (auto feature_base_ptr : filtered_list)//_feature_list_in)
    {
        FeaturePointImage* feature_ptr = (FeaturePointImage*)feature_base_ptr;

        //std::cout << "\nSearch feature: " << feature_ptr->trackId() << " at: " << feature_ptr->getKeypoint().pt;

        roi_x = (feature_ptr->getKeypoint().pt.x) - (roi_heigth / 2);
        roi_y = (feature_ptr->getKeypoint().pt.y) - (roi_width / 2);
        cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

        active_search_grid_.hitCell(feature_ptr->getKeypoint());  //TODO: Mirar el hitcell en este punto
        //active_search_grid_.blockCell(roi);

        cv::Mat target_descriptor = feature_ptr->getDescriptor();

        //lists used to debug
        tracker_target_.push_back(feature_ptr->getKeypoint().pt);
        tracker_roi_.push_back(roi);

        if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
        {
            //the matcher is now inside the match function
            Scalar normalized_score = match(target_descriptor,candidate_descriptors,candidate_keypoints,cv_matches);

            if (normalized_score > params_.matcher.min_normalized_score)
            {
                //std::cout << "\t <--TRACKED" << std::endl;
                FeaturePointImage* incoming_point_ptr = new FeaturePointImage(
                        candidate_keypoints[cv_matches[0].trainIdx], (candidate_descriptors.row(cv_matches[0].trainIdx)),
                        feature_ptr->isKnown());
                _feature_list_out.push_back(incoming_point_ptr);

                incoming_point_ptr->setTrackId(feature_ptr->trackId());

                _feature_matches[incoming_point_ptr] = FeatureMatch({feature_base_ptr,
                                                            normalized_score}); //FIXME: 512 is the maximum HAMMING distance
            }
            else
            {
                //std::cout << "\t <--NOT TRACKED" << std::endl;
            }
            for (unsigned int i = 0; i < candidate_keypoints.size(); i++)
            {
                tracker_candidates_.push_back(candidate_keypoints[i].pt);

            }
        }
        //else
            //std::cout << "\t <--NOT FOUND" << std::endl;
    }
    std::cout << "Number of Features tracked: " << _feature_list_out.size() << std::endl;
    return _feature_list_out.size();
}

Scalar ProcessorImage::match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors,
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


void ProcessorImage::trimRoi(cv::Rect& _roi)
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

void ProcessorImage::inflateRoi(cv::Rect& _roi)
{
    // now both the detector and descriptor patter_radius is the same, but when not, shouldn't the method have that as input parameter?
    int inflation_rate = detector_descriptor_params_.pattern_radius_;

    _roi.x = _roi.x - inflation_rate;
    _roi.y = _roi.y - inflation_rate;
    _roi.width = _roi.width + 2*inflation_rate;
    _roi.height = _roi.height + 2*inflation_rate;
}

void ProcessorImage::adaptRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi)
{

    inflateRoi(_roi);
    trimRoi(_roi);

    tracker_roi_inflated_.push_back(_roi);

    _image_roi = _image(_roi);
}

// draw functions ===================================================================

void ProcessorImage::drawTrackingFeatures(cv::Mat _image, std::list<cv::Point> _target_list, std::list<cv::Point> _candidates_list)
{
    // These "tracking features" are the feature to be used in tracking as well as its candidates

    for(auto target_point : _target_list)
    {
        //target
        cv::circle(_image, target_point, 2, cv::Scalar(0.0, 255.0, 255.0), -1, 8, 0);
    }
    for(auto candidate_point : _candidates_list)
    {
        //candidate - cyan
        cv::circle(_image, candidate_point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);
    }

    cv::imshow("Feature tracker", _image);

}

void ProcessorImage::drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color)
{
    for (auto roi : _roi_list)
    {
        cv::rectangle(_image, roi, _color, 1, 8, 0);
    }
    cv::imshow("Feature tracker", _image);
}

void ProcessorImage::drawFeatures(CaptureBase* const _last_ptr)
{
    for (auto feature_ptr : *(last_ptr_->getFeatureListPtr()))
    {
        FeaturePointImage* point_ptr = (FeaturePointImage*)feature_ptr;
        if (point_ptr->isKnown())
        {
            cv::circle(image_last_, point_ptr->getKeypoint().pt, 7, cv::Scalar(51.0, 255.0, 51.0), 1, 3, 0);
        }
        else
        {
            cv::circle(image_last_, point_ptr->getKeypoint().pt, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
        }
        cv::putText(image_last_, std::to_string(feature_ptr->trackId()), point_ptr->getKeypoint().pt, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
    }
    cv::imshow("Feature tracker", image_last_);
}


ProcessorBase* ProcessorImage::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorImage* prc_ptr = new ProcessorImage(*((ProcessorImageParameters*)_params));
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}


} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_image = ProcessorFactory::get().registerCreator("IMAGE", ProcessorImage::create);
}
} // namespace wolf

