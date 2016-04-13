// Wolf includes
#include "processor_brisk.h"

// OpenCV includes

// other includes
#include <bitset>

namespace wolf
{

/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(unsigned int _image_rows, unsigned int _image_cols, unsigned int _grid_width,
                               unsigned int _grid_height, unsigned int _separation, unsigned int _max_new_features, unsigned int _min_features_th,
                               int _threshold, int _octaves, float _pattern_scales, unsigned int _adjust) :
        ProcessorTrackerFeature(PRC_TRACKER_BRISK),
        detector_(_threshold, _octaves, _pattern_scales),
        matcher_(cv::NORM_HAMMING),
        act_search_grid_(_image_rows, _image_cols, _grid_width, _grid_height, _adjust-_separation+1, _separation),
        min_features_th_(_min_features_th)
{
    ProcessorTrackerFeature::setMaxNewFeatures(_max_new_features);
    detector_.create("Feature2D.BRISK");
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
    //drawFeatures(last_ptr_);
    drawRoiLastFrame(image_incoming_, tracker_roi_);
    //tracker_roi_.clear();
    drawTrackingFeatures(image_incoming_,tracker_features_);
    //tracker_features_.clear();
    cv::waitKey(200);
    //cv::waitKey(0);
}

bool ProcessorBrisk::correctFeatureDrift(const FeatureBase* _last_feature, FeatureBase* _incoming_feature)
{
    return true;
}

unsigned int ProcessorBrisk::detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& new_descriptors)
{
    cv::Mat _image_roi = _image(_roi);
    //Brisk Algorithm
    detector_.detect(_image_roi, _new_keypoints);
    detector_.compute(_image_roi, _new_keypoints, new_descriptors);
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
                std::cout << "# of NEW features detected: " << new_keypoints.size() << std::endl;
                //Escoger uno de los features encontrados -> el 0 o primero.
                FeaturePointImage* point_ptr = new FeaturePointImage(new_keypoints[0], new_descriptors.row(0), false);
                std::cout << "Adding point " << point_ptr->nodeId() << " at " << new_keypoints[0].pt << std::endl;
                addNewFeatureLast(point_ptr);
                act_search_grid_.hitCell(new_keypoints[0]);
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

//    while ((n_features < 1) && (n_iterations < 20))
//    {
//        bool roi_exists = act_search_grid_.pickRoi(roi);
//        if (roi_exists == false)
//        {
//            break;
//        }
//        std::vector<cv::KeyPoint> new_keypoints;
//        cv::Mat new_descriptors;
//        n_features = detect(image_last_, roi, new_keypoints, new_descriptors);
//        if (n_features == 0)
//        {
//            act_search_grid_.blockCell(roi);
//        }
//        else
//        {
//            std::cout << "# of NEW features detected: " << n_features << std::endl;
//            //Escoger uno de los features encontrados -> el 0 o primero.
//            FeaturePointImage* point_ptr = new FeaturePointImage(new_keypoints[0], new_descriptors.row(0), false);
//            std::cout << "Adding point ID " << point_ptr->nodeId() << "; coords " << new_keypoints[0].pt << std::endl;
//            addNewFeatureLast(point_ptr);
//            act_search_grid_.hitCell(new_keypoints[0]);
//        }
//        n_iterations++;
//    }
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
                                           FeatureMatchMap& _feature_correspondences)
{
    std::cout << std::endl << "-------------- trackFeatures ----------------" << std::endl << std::endl;
    ///PROJECTION OF LANDMARKS (features in this case)
    unsigned int feature_roi_width = 30;
    unsigned int feature_roi_heigth = 30;
    unsigned int feature_roi_x = 0;
    unsigned int feature_roi_y = 0;
    unsigned int n_candidates = 0;
    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;
    std::cout << "Number of features to track: " << _feature_list_in.size() << std::endl << std::endl;
    for (auto feature_base_ptr : _feature_list_in)
    {
        FeaturePointImage* feature_ptr = (FeaturePointImage*)(((feature_base_ptr)));
        act_search_grid_.hitCell(feature_ptr->getKeypoint());
        std::cout << "Searching for feature ID: " << feature_ptr->nodeId() << std::endl;
        feature_roi_x = (feature_ptr->getKeypoint().pt.x) - (feature_roi_heigth/2);
        feature_roi_y = (feature_ptr->getKeypoint().pt.y) - (feature_roi_width/2);
        cv::Rect roi_for_matching(feature_roi_x, feature_roi_y, feature_roi_width, feature_roi_heigth);

        //IT SHOULD NOT BE LIKE THIS, JUST STORE THE KEYPOINT, BUT I WANT TO SEE THE DESCRIPTOR FOR TESTING
        FeaturePointImage* target_feature_ptr = new FeaturePointImage(
                feature_ptr->getKeypoint(), feature_ptr->getDescriptor(),false);
        tracker_features_.push_back(target_feature_ptr);


        tracker_roi_.push_back(roi_for_matching);

        n_candidates = detect(image_incoming_, roi_for_matching, new_keypoints, new_descriptors);
        std::cout << "Number of detected candidates: " << n_candidates << std::endl;
        if (n_candidates != 0)
        {
            //POSIBLE PROBLEMA: Brisk deja una distancia a la hora de detectar. Si es muy pequeño el roi puede que no detecte nada
            std::vector<cv::DMatch> matches;
            cv::Mat descriptor = feature_ptr->getDescriptor();
            matcher_.match(descriptor, new_descriptors, matches);
            std::cout << "Number of matches:      " << matches.size() << std::endl;
            std::cout << "First Hamming distance: " << matches[0].distance << std::endl;
            if (matches[0].distance < 200)
            {
                FeaturePointImage* incoming_point_ptr = new FeaturePointImage(
                        new_keypoints[matches[0].trainIdx], (new_descriptors.row(matches[0].trainIdx)),
                        feature_ptr->isKnown());
                _feature_list_out.push_back(incoming_point_ptr);
                _feature_correspondences[incoming_point_ptr] = FeatureMatch(feature_base_ptr,
                                                                            (WolfScalar)(((matches[0].distance))));
            }
            for (unsigned int i = 0; i < new_keypoints.size(); i++) // TODO Arreglar todos los <= y -1 por < y nada.
            {
                if (i != matches[0].trainIdx)
                {
                    //IT SHOULD NOT BE LIKE THIS, JUST STORE THE KEYPOINT, BUT I WANT TO SEE THE DESCRIPTOR FOR TESTING
                    FeaturePointImage* candidate_feature_ptr = new FeaturePointImage(
                            new_keypoints[i], new_descriptors.row(i),true);
                    tracker_features_.push_back(candidate_feature_ptr);
                }
            }
        }
    }
    std::cout << "\nNumber of Features found: " << _feature_list_out.size() << std::endl;
    return _feature_list_out.size();
}

// draw functions ===================================================================

void ProcessorBrisk::drawTrackingFeatures(cv::Mat _image, std::list<FeaturePointImage*> _features_list)
{
    // These "tracking features" are the feature to be used in tracking as well as its candidates
    for(auto tracker_feat : _features_list)
    {
        if (tracker_feat->isKnown())
        {
            //candidate
            cv::circle(_image, tracker_feat->getKeypoint().pt, 2, cv::Scalar(250.0, 250.0, 250.0), -1, 8, 0);
        }
        else
        {
            //target
            cv::circle(_image, tracker_feat->getKeypoint().pt, 2, cv::Scalar(250.0, 180.0, 70.0), -1, 8, 0);
        }
    }
    cv::imshow("Keypoint drawing", _image);
}

void ProcessorBrisk::drawRoiLastFrame(cv::Mat _image, std::list<cv::Rect> _roi_list)
{
    for (auto roi : _roi_list)
    {
        cv::rectangle(_image, roi, cv::Scalar(88.0, 70.0, 254.0), 1, 8, 0);
    }
    cv::imshow("Keypoint drawing", _image);
}

void ProcessorBrisk::drawFeatures(CaptureBase* const _last_ptr)
{
    cv::KeyPoint _kp;
    cv::Mat image = image_last_;

//    for (std::list<FeatureBase*>::iterator feature_iter=last_ptr_->getFeatureListPtr()->begin();feature_iter != last_ptr_->getFeatureListPtr()->end(); ++feature_iter)
    for (auto feature_ptr : *(last_ptr_->getFeatureListPtr()))
    {
        FeaturePointImage* point_ptr = (FeaturePointImage*)feature_ptr;
        if (point_ptr->isKnown())
        {
            cv::circle(image, point_ptr->getKeypoint().pt, 2, cv::Scalar(50.0, 250.0, 54.0), -1, 3, 0);
        }
        else
        {
            cv::circle(image, point_ptr->getKeypoint().pt, 2, cv::Scalar(250.0, 80.0, 254.0), -1, 3, 0);
        }
//        std::cout << "keypoint: " << feature_ptr->getKeypoint().pt << std::endl;
    }
    cv::imshow("Keypoint drawing", image);

}

} // namespace wolf

