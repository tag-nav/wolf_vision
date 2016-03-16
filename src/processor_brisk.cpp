// Wolf includes
#include "processor_brisk.h"

// OpenCV includes


/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(unsigned int _image_rows, unsigned int _image_cols, int _threshold, int _octaves, float _pattern_scales,
                               unsigned int _grid_width, unsigned int _grid_height, unsigned int _min_features_th) :
    ProcessorTracker(PRC_TRACKER_BRISK, true, false),
    brisk_(_threshold, _octaves, _pattern_scales),
    act_search_grid_(_image_rows,_image_cols,_grid_width, _grid_height),
    min_features_th_(_min_features_th)
{
}

//Destructor
ProcessorBrisk::~ProcessorBrisk()
{

}


// Tracker function. Returns the number of successful tracks.
unsigned int ProcessorBrisk::processKnownFeatures()
{
    //std::cout << "<---- processKnownFeatures ---->" << std::endl << std::endl;
    cv::Mat image = ((CaptureImage*)getIncomingPtr())->getImage();

    act_search_grid_.renew();

    ///PROJECTION OF LANDMARKS (features in this case)

    unsigned int n_last_capture_feat = 0;
    for(std::list<FeatureBase*>::iterator feat_list_it=getLastPtr()->getFeatureListPtr()->begin();feat_list_it != getLastPtr()->getFeatureListPtr()->end(); ++feat_list_it)
    {
        //std::cout << "inside iterator" << std::endl;
        FeaturePointImage* last_feature = (FeaturePointImage*)*feat_list_it;
        Eigen::Vector2i feature_point = {last_feature->getKeypoint().pt.x,last_feature->getKeypoint().pt.y}; //TODO: Look if this is the right order
        act_search_grid_.hitCell(feature_point);
        //std::cout << "Last feature keypoint: " << last_feature->getKeypoint().pt << std::endl;

        drawFeaturesLastFrame(image,feature_point);

        n_last_capture_feat++;
    }
    //std::cout << "n_last_capture_feat: " << n_last_capture_feat << std::endl;

    cv::Size last_capture_roi_size(10,10); //cols,rows
    //cv::Point last_capture_roi_point(,last_feature->getKeypoint().pt.y);
    //cv::Rect last_capture_roi(last_capture_roi_point,last_capture_roi_size);

    // 640/8 = 80; 360/8 = 45; These are the roi maximum values. You should search in the last capture features spaces with these roi.


    //TODO:
    //You need to block the cell the features of the last capture are in.
    //You need to find the roi (as in the grid (or less)) to search in the area around the last capture feaures
    //Do the matching





//    cv::waitKey(30);
    return 0;
}

void ProcessorBrisk::drawFeaturesLastFrame(cv::Mat _image, Eigen::Vector2i _feature_point_last)
{
    cv::Point point;
    point.x = _feature_point_last[0];
    point.y = _feature_point_last[1];
    cv::circle(_image,point,2,cv::Scalar(250.0,180.0,70.0),-1,8,0);
    cv::imshow("Keypoint drawing",_image);
}


void ProcessorBrisk::drawFeatures(cv::Mat _image, std::vector<cv::KeyPoint> _kp, cv::Rect _roi)
{
    //std::cout << "<---- drawFeatures ---->" << std::endl << std::endl;

    if(_kp.size()!=0)
    {
        for(unsigned int i = 0; i <= (_kp.size()-1);i++)
        {
            cv::Point point;
            point.x = _kp[i].pt.x;// + _roi.x;
            point.y = _kp[i].pt.y;// + _roi.y;
            cv::circle(_image,point,2,cv::Scalar(88.0,250.0,154.0),-1,8,0);
            cv::rectangle(_image,_roi,cv::Scalar(88.0,250.0,154.0),1,8,0);

        }
        cv::imshow("Keypoint drawing",_image);
    }
    else
    {
        cv::rectangle(_image,_roi,cv::Scalar(88.0,250.0,154.0),1,8,0);
        cv::imshow("Keypoint drawing",_image);
    }
}

unsigned int ProcessorBrisk::briskDetect(cv::Mat _image, cv::Rect &_roi, std::vector<cv::KeyPoint> &_new_keypoints, std::vector<float> & new_descriptors)
{
    //std::cout << "<---- briskImplementation ---->" << std::endl << std::endl;

    cv::Mat _image_roi = _image(_roi);
    cv::Mat descriptors;                    // Matrix of descriptors

    //Brisk Algorithm
    brisk_.create("Feature2D.BRISK");  //TODO: look if this can be done in the constructor
    brisk_.detect(_image_roi, _new_keypoints);
    brisk_.compute(_image_roi, _new_keypoints,descriptors);

    if(_new_keypoints.size()!=0)
    {
            for(unsigned int i = 0; i <= (_new_keypoints.size()-1);i++)
            {
                _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
                _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;

                new_descriptors=descriptors(cv::Range(i,i+1),cv::Range(0,descriptors.cols));
                ((CaptureImage*)getIncomingPtr())->addFeature(new FeaturePointImage(_new_keypoints[i],new_descriptors,false));
                std::cout << "Current feature keypoint: " << _new_keypoints[i].pt << std::endl;
            }

            return descriptors.rows;  //number of features
    }
    else
    {
        return 0;
    }
}

// This is intended to create Features that are not among the Features already known in the Map. Returns the number of detected Features.
unsigned int ProcessorBrisk::detectNewFeatures()
{
    //std::cout << "<---- detectNewFeatures ---->" << std::endl << std::endl;

    cv::Mat image = ((CaptureImage*)getIncomingPtr())->getImage();
    cv::Rect roi;

    ///START OF THE SEARCH
    unsigned int n_features = 0;
    unsigned int n_iterations = 0;
    while((n_features < 1) && (n_iterations <5))
    {
        bool roi_exists = act_search_grid_.pickRoi(roi);

//        if (act_search_grid_.pickRoi((roi)))
//        {
//            // do things;
//            else
//            break;
//        }

        if(roi_exists==false)
        {
            break;
        }

        std::vector<cv::KeyPoint> new_keypoints;
        std::vector<float> new_descriptors;

        n_features = briskDetect(image,roi, new_keypoints, new_descriptors);

        if (n_features == 0)
        {
            act_search_grid_.blockCell(roi);
        }

        std::cout << "NFL size: " << getNewFeaturesList().size() << std::endl;
        for (FeatureBase* feature_ptr : getNewFeaturesList())
        {
            std::cout << "newFeaturesList Feature, keypoint: " << ((FeaturePointImage*)feature_ptr)->getKeypoint().pt << std::endl;


        }
        // A method to draw the keypoints. Optional (false for drawing features)
        //drawFeatures(((CaptureImage*)getIncomingPtr())->getImage(),((CaptureImage*)getIncomingPtr())->getKeypoints(), myROI);

        n_iterations++;
    }
    std::cout << "n_features: " << n_features << std::endl;


    cv::waitKey(30);
    return n_features;
}


//Vote for KeyFrame generation. If a KeyFrame criterion is validated, this function returns true
bool ProcessorBrisk::voteForKeyFrame()
{
    std::cout << "<---- voteForKeyFrame ---->" << std::endl << std::endl;
    std::cout << "feature list size: " << ((CaptureImage*)getIncomingPtr())->getFeatureListPtr()->size() << std::endl;
    std::cout << "threshold: " << (((CaptureImage*)getIncomingPtr())->getFeatureListPtr()->size() < min_features_th_) << std::endl;
    return (((CaptureImage*)getIncomingPtr())->getFeatureListPtr()->size() < min_features_th_);
}


/** Protected */

//Initialize one landmark
LandmarkBase* ProcessorBrisk::createLandmark(FeatureBase* _feature_ptr)
{
    return new LandmarkBase(LANDMARK_POINT,new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
}

//Create a new constraint
ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, NodeBase* _feat_or_lmk_ptr)
{
    //FeatureBase f_b = new FeatureBase(Eigen::Vector2s::Zero(),Eigen::Matrix2s::Zero());
    //FrameBase frame_b = frm_ptr = new FrameBase(NON_KEY_FRAME, TimeStamp(),new StateBlock(Eigen::Vector3s::Zero()), new StateQuaternion);
    //LandmarkBase* lndmrk = new LandmarkBase(LANDMARK_POINT,new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
    //return new ConstraintPoint2D(f_b,frame_b,CTR_ACTIVE);
    //return new ConstraintBase(CTR_IMG_PNT_TO_IMG_PNT,_feature_ptr,CTR_ACTIVE);
    return nullptr;
}
