// Wolf includes
#include "processor_brisk.h"

// OpenCV includes

// other includes
#include <bitset>

/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(unsigned int _image_rows, unsigned int _image_cols,
                               unsigned int _grid_width, unsigned int _grid_height, unsigned int _min_features_th,
                               int _threshold, int _octaves, float _pattern_scales) :
    ProcessorTrackerFeature(PRC_TRACKER_BRISK),
    brisk_(_threshold, _octaves, _pattern_scales),
    act_search_grid_(_image_rows,_image_cols,_grid_width, _grid_height),
    min_features_th_(_min_features_th)
{
    brisk_.create("Feature2D.BRISK");
}

//Destructor
ProcessorBrisk::~ProcessorBrisk()
{

}


// Tracker function. Returns the number of successful tracks.
unsigned int ProcessorBrisk::processKnown()
{

    std::cout << std::endl << "<---- processKnownFeatures ---->" << std::endl << std::endl;

    act_search_grid_.renew();

    //////// Poner aquí lo de que se asignen los features de last en la lista para dibujar en ROS

    unsigned int n_tracks = 0;
    std::cout << "Nbr of features in incoming: " << incoming_ptr_->getFeatureListPtr()->size() << std::endl;
    FeatureCorrespondenceMap new_incoming_2_new_last;
    n_tracks = trackFeatures(*(last_ptr_->getFeatureListPtr()), *(incoming_ptr_->getFeatureListPtr()), new_incoming_2_new_last);
    std::cout << "Nbr of features in incoming: " << incoming_ptr_->getFeatureListPtr()->size() << std::endl;

    return n_tracks;
}

void ProcessorBrisk::drawTrackingFeatures(cv::Mat _image, Eigen::Vector2i _feature_point, bool _is_candidate)
{
    // These "tracking features" are the feature to be used in tracking as well as its candidates

    cv::Point point;
    point.x = _feature_point[0];
    point.y = _feature_point[1];
    if(_is_candidate)
    {
        cv::circle(_image,point,2,cv::Scalar(250.0,250.0,250.0),-1,8,0);
    }
    else
    {
        cv::circle(_image,point,2,cv::Scalar(250.0,180.0,70.0),-1,8,0);
    }
    cv::imshow("Keypoint drawing",_image);
}

void ProcessorBrisk::drawRoiLastFrame(cv::Mat _image,cv::Rect _roi)
{
    cv::rectangle(_image,_roi,cv::Scalar(88.0,70.0,254.0),1,8,0);
    cv::imshow("Keypoint drawing",_image);
}


void ProcessorBrisk::drawFeatures(CaptureBase* const _last_ptr)
{
    cv::KeyPoint _kp;
    cv::Mat image = image_last_;

    std::cout << "size: " << last_ptr_->getFeatureListPtr()->size() << std::endl;
    for (std::list<FeatureBase*>::iterator feature_iter=last_ptr_->getFeatureListPtr()->begin();feature_iter != last_ptr_->getFeatureListPtr()->end(); ++feature_iter)
    {

        FeaturePointImage* feature_ptr = (FeaturePointImage*)*feature_iter;
        //_kp = ((FeaturePointImage*)*feature_iter)->getKeypoint();
        cv::Point point;
        point.x = feature_ptr->getKeypoint().pt.x;
        point.y = feature_ptr->getKeypoint().pt.y;
        if(feature_ptr->getIsKnown())
        {
            cv::circle(image,point,2,cv::Scalar(50.0,250.0,54.0),-1,3,0);
        }
        else
        {
            cv::circle(image,point,2,cv::Scalar(250.0,80.0,254.0),-1,3,0);
        }
        std::cout << "keypoint: " << feature_ptr->getKeypoint().pt << std::endl;
    }
    cv::imshow("Keypoint drawing",image);

}


unsigned int ProcessorBrisk::briskDetect(cv::Mat _image, cv::Rect &_roi, std::vector<cv::KeyPoint> &_new_keypoints, cv::Mat & new_descriptors)
{
    std::cout << std::endl << "<---- briskImplementation ---->" << std::endl << std::endl;
    std::cout << "0" << std::endl;
    cv::Mat _image_roi = _image(_roi);

    std::cout << "1" << std::endl;

    //Brisk Algorithm
    brisk_.detect(_image_roi, _new_keypoints);
    std::cout << "2" << std::endl;
    brisk_.compute(_image_roi, _new_keypoints,new_descriptors);
    std::cout << "3" << std::endl;

    if(_new_keypoints.size()!=0)
    {
            for(unsigned int i = 0; i <= (_new_keypoints.size()-1);i++)
            {
                _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
                _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;
            }

            return new_descriptors.rows;  //number of features
    }
    else
    {
        return 0;
    }
}

void ProcessorBrisk::addNewFeaturesInCapture(cv::KeyPoint _new_keypoints, cv::Mat _new_descriptors) //std::vector<cv::KeyPoint>
{
//    for(unsigned int i = 0; i <= (_new_keypoints.size()-1);i++)
//    {
//        FeaturePointImage* point_ptr = new FeaturePointImage(_new_keypoints[i],_new_descriptors.row(i),false);
//        addNewFeature(point_ptr);
//    }
//    std::cout << "size of new list: " << getNewFeaturesList().size() << std::endl;

      FeaturePointImage* point_ptr = new FeaturePointImage(_new_keypoints,_new_descriptors,false);
      addNewFeature(point_ptr);
}

void ProcessorBrisk::resetVisualizationFlag(FeatureBaseList& _feature_list_last, FeatureBaseList& _feature_list_incoming)
{
    for(auto feature_base_last_ptr : _feature_list_last)
    {
        FeaturePointImage* feature_last_ptr = (FeaturePointImage*)feature_base_last_ptr;
        feature_last_ptr->setIsKnown(true);
        std::cout << "last_is_known: " << feature_last_ptr->getIsKnown() << std::endl;
    }

    for(auto feature_base_incoming_ptr : _feature_list_incoming)
    {
        FeaturePointImage* feature_incoming_ptr = (FeaturePointImage*)feature_base_incoming_ptr;
        feature_incoming_ptr->setIsKnown(true);
        std::cout << "incoming_is_known: " << feature_incoming_ptr->getIsKnown() << std::endl;
    }

}


// This is intended to create Features that are not among the Features already known in the Map. Returns the number of detected Features.
unsigned int ProcessorBrisk::detectNewFeatures()
{
    std::cout << std::endl << "<---- detectNewFeatures ---->" << std::endl << std::endl;

    resetVisualizationFlag(*(last_ptr_->getFeatureListPtr()),*(incoming_ptr_->getFeatureListPtr()));

    cv::Rect roi;

    ///START OF THE SEARCH
    unsigned int n_features = 0;
    unsigned int n_iterations = 0;
    while((n_features < 5) && (n_iterations <20))
    {
        bool roi_exists = act_search_grid_.pickRoi(roi);

        if(roi_exists==false)
        {
            break;
        }

        std::vector<cv::KeyPoint> new_keypoints;
        cv::Mat new_descriptors;

        n_features = briskDetect(image_last_, roi, new_keypoints, new_descriptors);

        if (n_features == 0)
        {
            act_search_grid_.blockCell(roi);
        }
        else
        {
            std::cout << "================================= NEW features dected: " << n_features << std::endl;
            //Escoger uno de los features encontrados
            addNewFeaturesInCapture(new_keypoints[0],new_descriptors.row(0));
            Eigen::Vector2i feature_point = {new_keypoints[0].pt.x,new_keypoints[0].pt.y};
            act_search_grid_.hitCell(feature_point);
        }

        n_iterations++;
    }
    return n_features;
}


//Vote for KeyFrame generation. If a KeyFrame criterion is validated, this function returns true
bool ProcessorBrisk::voteForKeyFrame()
{
    //std::cout << "Thereshold: " << min_features_th_ << std::endl;
    //std::cout << "Feature_list size: " << ((CaptureImage*)incoming_ptr_)->getFeatureListPtr()->size() << std::endl;
    std::cout << "<--------------------------------> voteForKeyFrame?: " << (((CaptureImage*)incoming_ptr_)->getFeatureListPtr()->size() < min_features_th_) << std::endl;
    return (incoming_ptr_->getFeatureListPtr()->size() < min_features_th_);
    //return 0;
}

void ProcessorBrisk::process(CaptureBase* const _incoming_ptr)
{
    std::cout << std::endl << "<---- process ---->" << std::endl << std::endl;
    image_incoming_ = ((CaptureImage*)_incoming_ptr)->getImage();
    image_last_ = ((CaptureImage*)last_ptr_)->getImage();
    ProcessorTrackerFeature::process(_incoming_ptr);
    std::cout << std::endl << "<---- end process ---->" << std::endl << std::endl;
    drawFeatures(last_ptr_);
    cv::waitKey(1000);
}


/** Protected */

//Initialize one landmark
LandmarkBase* ProcessorBrisk::createLandmark(FeatureBase* _feature_ptr)
{
    return new LandmarkBase(LANDMARK_POINT,new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
}

//Create a new constraint
ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feat_or_lmk_ptr)
{
    return nullptr;
}


/** Private */

unsigned int ProcessorBrisk::trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out, FeatureCorrespondenceMap& _feature_correspondences)
{
    std::cout << std::endl << "<---- processFeaturesForMatching ---->" << std::endl << std::endl;

    ///PROJECTION OF LANDMARKS (features in this case)

    unsigned int feature_roi_width = 30;
    unsigned int feature_roi_heigth = 30;

    unsigned int feature_roi_x = 0;
    unsigned int feature_roi_y = 0;

    unsigned int n_candidates = 0;

    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;

    std::cout << "---------------------Number of features to match: " << _feature_list_in.size() << std::endl;
    unsigned int n_last_capture_feat = 0;
    for(auto feature_base_ptr : _feature_list_in)
    {
        n_candidates = 0;

        FeaturePointImage* feature_ptr = (FeaturePointImage*)feature_base_ptr;
        Eigen::Vector2i feature_point = {feature_ptr->getKeypoint().pt.x,feature_ptr->getKeypoint().pt.y};
        act_search_grid_.hitCell(feature_point);
        std::cout << "------------------- Searching for feature : " << feature_ptr->nodeId() << std::endl;

        feature_roi_x = (feature_ptr->getKeypoint().pt.x)-(feature_roi_heigth/2);
        feature_roi_y = (feature_ptr->getKeypoint().pt.y)-(feature_roi_width/2);
        cv::Rect roi_for_matching(feature_roi_x,feature_roi_y,feature_roi_width,feature_roi_heigth);

        //drawTrackingFeatures(image_incoming_,feature_point,false);
        drawRoiLastFrame(image_incoming_,roi_for_matching);


        n_candidates = briskDetect(image_incoming_, roi_for_matching, new_keypoints, new_descriptors);
        std::cout << "------------------ Number of detected candidates: " << n_candidates << std::endl;

        if(n_candidates != 0)
        {
            //std::vector<float> feature_descriptor = feature_ptr->getDescriptor();

            //POSIBLE PROBLEMA: Brisk deja una distancia a la hora de detectar. Si es muy pequeño el roi puede que no detecte nada

                cv::BFMatcher matcher(cv::NORM_HAMMING);
                std::vector<cv::DMatch> matches;
                cv::Mat f_descriptor = feature_ptr->getDescriptor();

                matcher.match(f_descriptor, new_descriptors, matches);

                std::cout << "------------------ Number of matches: " << matches.size() << std::endl;
                std::cout << "------------------ Hamiing distance: " << matches[0].distance << std::endl;

            if(matches.size() >= 1)
            {
                if(matches[0].distance < 200)
                {
                    FeaturePointImage* point_ptr = new FeaturePointImage(new_keypoints[matches[0].trainIdx],
                            (new_descriptors.row(matches[0].trainIdx)),feature_ptr->getIsKnown());
                    _feature_list_out.push_back(point_ptr);
                    _feature_correspondences[point_ptr] = FeatureCorrespondence(feature_base_ptr, (WolfScalar) matches[0].distance);
                    n_last_capture_feat++;
                }
            }

            for(int i = 0; i <= new_keypoints.size()-1; i++)
            {
                if(i != matches[0].trainIdx)
                {
                    Eigen::Vector2i feature_point = {new_keypoints[i].pt.x,new_keypoints[i].pt.y};
                    //drawTrackingFeatures(image_incoming_,feature_point,true);
                }
            }


        }


    }
    std::cout << "n_last_capture_feat: " << n_last_capture_feat << std::endl;






    return n_last_capture_feat;
}
