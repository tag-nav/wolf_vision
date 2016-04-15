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
    tracker_features_.clear();
}

void ProcessorBrisk::postProcess()
{
    drawFeatures(last_ptr_);
    drawRoiLastFrame(image_incoming_, tracker_roi_);
    drawTrackingFeatures(image_incoming_,tracker_features_);
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

    inflateRoi(_image_roi, _image, _roi);

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

    WolfScalar diff = cv::norm(image_incoming_, image_last_, cv::NORM_L1);
    std::cout << "Distance between last and incoming images: " << diff << std::endl;

    for (auto feature_base_ptr : _feature_list_in)
    {
        FeaturePointImage* feature_ptr = (FeaturePointImage*)(((feature_base_ptr)));
        act_search_grid_.hitCell(feature_ptr->getKeypoint());

        std::cout << "Searching for feature ID: " << feature_ptr->nodeId() << " at: " << feature_ptr->getKeypoint().pt << std::endl;

        roi_x = (feature_ptr->getKeypoint().pt.x) - (roi_heigth / 2);
        roi_y = (feature_ptr->getKeypoint().pt.y) - (roi_width / 2);
        cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

        //assureRoi(roi);

        std::cout << "roi - X: " << roi.x << std::endl;
        std::cout << "roi - Y: " << roi.y << std::endl;
        std::cout << "roi - W: " << roi.width << std::endl;
        std::cout << "roi - H: " << roi.height << std::endl;

        //IT SHOULD NOT BE LIKE THIS, JUST STORE THE KEYPOINT, BUT I WANT TO SEE THE DESCRIPTOR FOR TESTING
        FeaturePointImage* target_feature_ptr = new FeaturePointImage(
                feature_ptr->getKeypoint(), feature_ptr->getDescriptor(),false);
        tracker_features_.push_back(target_feature_ptr);

        tracker_roi_.push_back(roi);

        if (detect(image_incoming_, roi, candidate_keypoints, candidate_descriptors))
        {
            //POSIBLE PROBLEMA: Brisk deja una distancia a la hora de detectar. Si es muy pequeño el roi puede que no detecte nada

            matcher_.match(feature_ptr->getDescriptor(), candidate_descriptors, cv_matches);

            std::cout << "\tFound at: " << candidate_keypoints[cv_matches[0].trainIdx].pt << " --Hamming: " << cv_matches[0].distance << std::endl;

            if (cv_matches[0].distance < 200)
            {
                FeaturePointImage* incoming_point_ptr = new FeaturePointImage(
                        candidate_keypoints[cv_matches[0].trainIdx], (candidate_descriptors.row(cv_matches[0].trainIdx)),
                        feature_ptr->isKnown());
                _feature_list_out.push_back(incoming_point_ptr);

                _feature_matches[incoming_point_ptr] = FeatureMatch(feature_base_ptr,
                                                                            1 - (WolfScalar)(cv_matches[0].distance)/512); //FIXME: 512 is the maximum HAMMING distance

            }
            for (unsigned int i = 0; i < candidate_keypoints.size(); i++) // TODO Arreglar todos los <= y -1 por < y nada.
            {
                if (i != cv_matches[0].trainIdx)
                {
                    //IT SHOULD NOT BE LIKE THIS, JUST STORE THE KEYPOINT, BUT I WANT TO SEE THE DESCRIPTOR FOR TESTING
                    FeaturePointImage* candidate_feature_ptr = new FeaturePointImage(
                            candidate_keypoints[i], candidate_descriptors.row(i),true);
                    tracker_features_.push_back(candidate_feature_ptr);
                }

            }
        }
        else
            std::cout << "\tNot found" << std::endl;
    }
    std::cout << "\nNumber of Features found: " << _feature_list_out.size() << std::endl << std::endl;
    return _feature_list_out.size();
}


void ProcessorBrisk::assureRoi(cv::Rect& _roi)
{
    //unsigned int x = _roi.x
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

void ProcessorBrisk::inflateRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi)
{
    int inflation_rate = 5;

    //cv::Rect _inflated_roi;
    _roi.x = _roi.x - inflation_rate;
    _roi.y = _roi.y - inflation_rate;
    _roi.width = _roi.width + 2*inflation_rate;
    _roi.height = _roi.height + 2*inflation_rate;

    std::cout << "INFL roi - X: " << _roi.x << std::endl;
    std::cout << "INFL roi - Y: " << _roi.y << std::endl;
    std::cout << "INFL roi - W: " << _roi.width << std::endl;
    std::cout << "INFL roi - H: " << _roi.height << std::endl;

    assureRoi(_roi);

    _image_roi = _image(_roi);
}

// draw functions ===================================================================

void ProcessorBrisk::drawTrackingFeatures(cv::Mat _image, std::list<FeaturePointImage*> _features_list)
{
    // These "tracking features" are the feature to be used in tracking as well as its candidates

    cv::Mat test_image;
    std::vector<cv::KeyPoint> test_keypoint_vector1;
    std::vector<cv::KeyPoint> test_keypoint_vector2;
    for(auto tracker_feat : _features_list)
    {
        if (tracker_feat->isKnown())
        {
            //candidate - cyan
            //cv::circle(_image, tracker_feat->getKeypoint().pt, 2, cv::Scalar(250.0, 250.0, 250.0), -1, 8, 0);
            test_keypoint_vector1.push_back(tracker_feat->getKeypoint());
        }
        else
        {
            //target -
            //cv::circle(_image, tracker_feat->getKeypoint().pt, 2, cv::Scalar(250.0, 180.0, 70.0), -1, 8, 0);
            test_keypoint_vector2.push_back(tracker_feat->getKeypoint());
        }
    }
    //cv::imshow("Keypoint drawing", _image);
    cv::drawKeypoints(image_incoming_,test_keypoint_vector1,test_image,cv::Scalar(255.0, 255.0, 0.0));
    cv::drawKeypoints(test_image,test_keypoint_vector2,test_image,cv::Scalar(0.0, 0.0, 255.0));
    cv::imshow("Incoming", test_image);
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
            cv::circle(image_last_, point_ptr->getKeypoint().pt, 3, cv::Scalar(80.0, 80.0, 254.0), -1, 3, 0);
        }
    }
    cv::imshow("Last", image_last_);
}

} // namespace wolf

