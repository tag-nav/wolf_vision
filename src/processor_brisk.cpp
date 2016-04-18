// Wolf includes
#include "processor_brisk.h"

// OpenCV includes

// other includes
#include <bitset>

namespace wolf
{

//Constructor
ProcessorBrisk::ProcessorBrisk(unsigned int _image_rows, unsigned int _image_cols, unsigned int _grid_width,
                               unsigned int _grid_height, unsigned int _separation, unsigned int _max_new_features, unsigned int _min_features_th,
                               int _threshold, int _octaves, float _pattern_scales, unsigned int _adjust) :
        ProcessorTrackerFeature(PRC_TRACKER_BRISK),
        detector_(_threshold, _octaves, _pattern_scales),
        descriptor_(_threshold, _octaves, _pattern_scales),
        matcher_(cv::NORM_HAMMING),
        act_search_grid_(_image_rows, _image_cols, _grid_width, _grid_height, _adjust-_separation+1, _separation),
        min_features_th_(_min_features_th)
{
    ProcessorTrackerFeature::setMaxNewFeatures(_max_new_features);
    //    detector_.create("BRISK");    // These do not seem to be necessary
    //    descriptor_.create("BRISK");  // These do not seem to be necessary
    //    matcher_.train();  // These do not seem to be necessary
    img_width_ = _image_cols;
    img_height_ = _image_rows;
    detector_separation_ = 21*_pattern_scales;
}

//Destructor
ProcessorBrisk::~ProcessorBrisk()
{

}

void ProcessorBrisk::preProcess()
{
    image_incoming_ = ((CaptureImage*)incoming_ptr_)->getImage();

    if (last_ptr_ == nullptr)
        image_last_ = ((CaptureImage*)incoming_ptr_)->getImage();
    else
        image_last_ = ((CaptureImage*)last_ptr_)->getImage();

    act_search_grid_.renew();


    tracker_roi_.clear();
    tracker_target_.clear();
    tracker_candidates_.clear();
}

void ProcessorBrisk::postProcess()
{
    drawFeatures(last_ptr_);
    drawRoiLastFrame(image_incoming_, tracker_roi_);
    drawTrackingFeatures(image_incoming_,tracker_target_,tracker_candidates_);
    cv::waitKey(0);
}

bool ProcessorBrisk::correctFeatureDrift(const FeatureBase* _last_feature, FeatureBase* _incoming_feature)
{
    return true;
}

unsigned int ProcessorBrisk::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& new_descriptors)
{
    //cv::Mat _image_roi = _image(_roi);
    cv::Mat _image_roi;

    adaptRoi(_image_roi, _image, _roi);

    detector_.detect(_image_roi, _new_keypoints);
    descriptor_.compute(_image_roi, _new_keypoints, new_descriptors);
    for (unsigned int i = 0; i < _new_keypoints.size(); i++)
    {
        _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
        _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;
    }
    return _new_keypoints.size();
}

unsigned int ProcessorBrisk::detectNewFeatures(const unsigned int& _max_new_features)
{
    std::cout << std::endl << "\n================= detectNewFeatures =============" << std::endl << std::endl;
    resetVisualizationFlag(*(last_ptr_->getFeatureListPtr()), *(incoming_ptr_->getFeatureListPtr()));
    cv::Rect roi;
    ///START OF THE SEARCH

    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;
    unsigned int n_new_features = 0;

    for (unsigned int n_iterations = 0; n_iterations < _max_new_features * 1.25; n_iterations++)
    {
        if (act_search_grid_.pickRoi(roi))
        {
            if (detect(image_last_, roi, new_keypoints, new_descriptors))
            {
                //Escoger uno de los features encontrados -> el 0 o primero.
                std::cout << new_keypoints.size() << " new features detected in active search roi. Picking only the first one." << std::endl;

                FeaturePointImage* point_ptr = new FeaturePointImage(new_keypoints[0], new_descriptors.row(0), false);
                addNewFeatureLast(point_ptr);
                act_search_grid_.hitCell(new_keypoints[0]);

                std::cout << "Added point " << point_ptr->nodeId() << " at " << new_keypoints[0].pt << std::endl;

                n_new_features++;
                if (n_new_features >= _max_new_features)
                    break;
            }
            else
            {
                act_search_grid_.blockCell(roi);
            }
        }
        else
        {
            break;
        }
    }

    return n_new_features;
}

void ProcessorBrisk::resetVisualizationFlag(FeatureBaseList& _feature_list_last,
                                            FeatureBaseList& _feature_list_incoming)
{
    for (auto feature_base_last_ptr : _feature_list_last)
    {
        FeaturePointImage* feature_last_ptr = (FeaturePointImage*)feature_base_last_ptr;
        feature_last_ptr->setIsKnown(true);
    }

    for (auto feature_base_incoming_ptr : _feature_list_incoming)
    {
        FeaturePointImage* feature_incoming_ptr = (FeaturePointImage*)feature_base_incoming_ptr;
        feature_incoming_ptr->setIsKnown(true);
    }

}

unsigned int ProcessorBrisk::trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out,
                                           FeatureMatchMap& _feature_matches)
{
    std::cout << std::endl << "-------------- trackFeatures ----------------" << std::endl << std::endl;

    unsigned int roi_width = 30;
    unsigned int roi_heigth = 30;
    unsigned int roi_x;
    unsigned int roi_y;
    std::vector<cv::KeyPoint> candidate_keypoints;
    cv::Mat candidate_descriptors;
    std::vector<cv::DMatch> cv_matches;

    std::cout << "Number of features to track: " << _feature_list_in.size() << std::endl << std::endl;
    std::cout << "last*: " << last_ptr_ << " -- incoming*: " << incoming_ptr_ << std::endl;

    Scalar diff = cv::norm(image_incoming_, image_last_, cv::NORM_L1);
    std::cout << "Distance between last and incoming images: " << diff << std::endl;

    for (auto feature_base_ptr : _feature_list_in)
    {
        FeaturePointImage* feature_ptr = (FeaturePointImage*)(((feature_base_ptr)));
        act_search_grid_.hitCell(feature_ptr->getKeypoint());

        std::cout << "Searching for feature ID: " << feature_ptr->nodeId() << " at: " << feature_ptr->getKeypoint().pt << std::endl;

        roi_x = (feature_ptr->getKeypoint().pt.x) - (roi_heigth / 2);
        roi_y = (feature_ptr->getKeypoint().pt.y) - (roi_width / 2);
        cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

        tracker_target_.push_back(feature_ptr->getKeypoint().pt);
        tracker_roi_.push_back(roi);

        if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
        {
            //POSIBLE PROBLEMA: Brisk deja una distancia a la hora de detectar. Si es muy pequeño el roi puede que no detecte nada

            matcher_.match(feature_ptr->getDescriptor(), candidate_descriptors, cv_matches);

            for(auto iterat : cv_matches)
            {
                cv::DMatch match = iterat;
                std::cout << "index of the candidate selected (trainIdx): " << match.trainIdx << std::endl;
                std::cout << "distance: " << match.distance << std::endl;
            }

            unsigned int lowestHD = 600;
            unsigned int row = 0;
            for(unsigned int i = 0; i < candidate_descriptors.rows; i++)
            {
                double dist = cv::norm( feature_ptr->getDescriptor(), candidate_descriptors.row(i), cv::NORM_HAMMING);
                std::cout << "====================================================dist[" << i << "]: " << dist << std::endl;
                if(dist < lowestHD)
                {
                    lowestHD = dist;
                    row = i;
                }
            }

            std::cout << "\tFound at: " << candidate_keypoints[cv_matches[0].trainIdx].pt << " --Hamming: " << cv_matches[0].distance << std::endl;

            if (cv_matches[0].distance < 200)
            {
                FeaturePointImage* incoming_point_ptr = new FeaturePointImage(
                        candidate_keypoints[cv_matches[0].trainIdx], (candidate_descriptors.row(cv_matches[0].trainIdx)),
                        feature_ptr->isKnown());
                _feature_list_out.push_back(incoming_point_ptr);

                _feature_matches[incoming_point_ptr] = FeatureMatch(feature_base_ptr,
                                                                            1 - (Scalar)(cv_matches[0].distance)/512); //FIXME: 512 is the maximum HAMMING distance

            }
            for (unsigned int i = 0; i < candidate_keypoints.size(); i++) // TODO Arreglar todos los <= y -1 por < y nada.
            {
                //if (i != cv_matches[0].trainIdx)
                //{
                    tracker_candidates_.push_back(candidate_keypoints[i].pt);
                //}

            }
        }
        else
            std::cout << "\tNot found" << std::endl;
    }
    std::cout << "\nNumber of Features found: " << _feature_list_out.size() << std::endl << std::endl;
    return _feature_list_out.size();
}


void ProcessorBrisk::trimRoi(cv::Rect& _roi)
{
    if(_roi.x < 0)
    {
        _roi.x = 0;
    }
    if(_roi.y < 0)
    {
        _roi.y = 0;
    }
    if((_roi.x + _roi.width) > img_width_)
    {
        int diff_width = img_width_ - (_roi.x + _roi.width);
        _roi.width = _roi.width+diff_width;
    }
    if((_roi.y + _roi.height) > img_height_)
    {
        int diff_height = img_height_ - (_roi.y + _roi.height);
        _roi.height = _roi.height+diff_height;
    }
}

void ProcessorBrisk::inflateRoi(cv::Rect& _roi)
{
    int inflation_rate = detector_separation_;

    std::cout << "detector separation: " << detector_separation_ << std::endl;

    _roi.x = _roi.x - inflation_rate;
    _roi.y = _roi.y - inflation_rate;
    _roi.width = _roi.width + 2*inflation_rate;
    _roi.height = _roi.height + 2*inflation_rate;
}

void ProcessorBrisk::adaptRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi)
{
    inflateRoi(_roi);

    std::cout << "INFL roi - X: " << _roi.x << std::endl;
    std::cout << "INFL roi - Y: " << _roi.y << std::endl;
    std::cout << "INFL roi - W: " << _roi.width << std::endl;
    std::cout << "INFL roi - H: " << _roi.height << std::endl;

    trimRoi(_roi);

    _image_roi = _image(_roi);
}

// draw functions ===================================================================

void ProcessorBrisk::drawTrackingFeatures(cv::Mat _image, std::list<cv::Point> _target_list, std::list<cv::Point> _candidates_list)
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

    cv::imshow("Incoming", _image);

}

void ProcessorBrisk::drawRoiLastFrame(cv::Mat _image, std::list<cv::Rect> _roi_list)
{
    for (auto roi : _roi_list)
    {
        cv::Rect s = roi;
        //std::cout << "roi furthest position: " << s.y + s.width << std::endl;
        cv::rectangle(_image, roi, cv::Scalar(88.0, 70.0, 254.0), 1, 8, 0);
    }
    cv::imshow("Incoming", _image);
}

void ProcessorBrisk::drawFeatures(CaptureBase* const _last_ptr)
{
    for (auto feature_ptr : *(last_ptr_->getFeatureListPtr()))
    {
        FeaturePointImage* point_ptr = (FeaturePointImage*)feature_ptr;
        if (point_ptr->isKnown())
        {
            cv::circle(image_last_, point_ptr->getKeypoint().pt, 7, cv::Scalar(50.0, 250.0, 54.0), 1, 3, 0);
        }
        else
        {
            cv::circle(image_last_, point_ptr->getKeypoint().pt, 4, cv::Scalar(80.0, 80.0, 254.0), -1, 3, 0);
        }
    }
    cv::imshow("Last", image_last_);
}

} // namespace wolf

