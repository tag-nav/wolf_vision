// Wolf includes
#include "processor_brisk.h"

// OpenCV includes


/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(int _threshold, int _octaves, float _pattern_scales, unsigned int _image_rows, unsigned int _image_cols,
                               unsigned int _grid_width, unsigned int _grid_height) :
    ProcessorTracker(PRC_TRACKER_BRISK, true, false),
    sensor_cam_ptr_(nullptr), capture_img_ptr_(nullptr),
    brisk_(_threshold, _octaves, _pattern_scales),
    act_search_grid_(_image_rows,_image_cols,_grid_width, _grid_height)
{
    //TODO: remove sensor_cam_ptr_, capture_img_ptr_
}

//Destructor
ProcessorBrisk::~ProcessorBrisk()
{

}


// Tracker function. Returns the number of successful tracks.
unsigned int ProcessorBrisk::processKnownFeatures()
{
    std::cout << "<---- processKnownFeatures ---->" << std::endl << std::endl;
    cv::Mat image = ((CaptureImage*)getIncomingPtr())->getImage();

    act_search_grid_.renew();

    ///PROJECTION OF LANDMARKS (features in this case)

    unsigned int n_last_capture_feat = 0;
    for(std::list<FeatureBase*>::iterator feat_list_it=getLastPtr()->getFeatureListPtr()->begin();feat_list_it != getLastPtr()->getFeatureListPtr()->end(); ++feat_list_it)
    {
        std::cout << "inside iterator" << std::endl;
        //FeatureBase* last_feature = *feat_list;
        FeaturePointImage* last_feature = (FeaturePointImage*)*feat_list_it;
        Eigen::Vector2i feature_point = {last_feature->getKeypoint().pt.x,last_feature->getKeypoint().pt.y}; //TODO: Look if this is the right order
        act_search_grid_.hitCell(feature_point);
        std::cout << "Last feature keypoint: " << last_feature->getKeypoint().pt << std::endl;

        drawFeaturesLastFrame(image,feature_point);

        n_last_capture_feat++;
    }
    std::cout << "n_last_capture_feat: " << n_last_capture_feat << std::endl;

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
    std::cout << "<---- drawFeatures ---->" << std::endl << std::endl;

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

unsigned int ProcessorBrisk::briskDetect(CaptureBase* _capture_ptr, cv::Mat _image, cv::Point _point_roi, bool _known_features)
{
    std::cout << "<---- briskImplementation ---->" << std::endl << std::endl;

    cv::Mat treated_image = _image;         // Set the image to analyze
    std::vector<cv::KeyPoint> keypoints;    // Vector of keypoints
    cv::Mat descriptors;                    // Matrix of descriptors
    Eigen::Vector2s keypoint_coordinates;   // TODO: When "measurement" is erased, this variable should be erased to, all over the code
    std::vector<float> descript_vector;     // Vector to store the descriptor for each keypoint


    std::cout << "type: " << treated_image.type() <<
                 " depth: " << treated_image.depth() <<
                 " channels: " << treated_image.channels()<< std::endl;
    //Brisk Algorithm
    brisk_.create("Feature2D.BRISK");
    brisk_.detect(treated_image, keypoints);
    brisk_.compute(treated_image, keypoints,descriptors);

    if(keypoints.size()!=0)
    {
        if(_known_features==false) //called when you want to detect new features
        {
            // Add the features in the capture
            for(unsigned int i = 0; i <= (keypoints.size()-1);i++)
            {
                keypoint_coordinates(0) = keypoints[i].pt.x + _point_roi.x;    //TODO: This four lines should be modified/erased when "measurement" is gone
                keypoint_coordinates(1) = keypoints[i].pt.y + _point_roi.y;
                keypoints[i].pt.x = keypoint_coordinates(0);
                keypoints[i].pt.y = keypoint_coordinates(1);

                descript_vector=descriptors(cv::Range(i,i+1),cv::Range(0,descriptors.cols));
                ((CaptureImage*)_capture_ptr)->addFeature(new FeaturePointImage(keypoint_coordinates,keypoints[i],descript_vector));
                std::cout << "Current feature keypoint: " << keypoints[i].pt << std::endl;
            }

            // Set the vector of keypoints and the matrix of descriptors in its capture
            /** Not sure if I can do this (the way the "set" is done, as it overwrittes) with the ROIs */
            //The keypoints are referenced to the position of the ROI, not the "absolute" position in the whole image

            ((CaptureImage*)_capture_ptr)->setKeypoints(keypoints);
            ((CaptureImage*)_capture_ptr)->setDescriptors(descriptors);

            return descriptors.rows;  //number of features
        }
        else //called with trying to match with the last capture
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

// This is intended to create Features that are not among the Features already known in the Map. Returns the number of detected Features.
unsigned int ProcessorBrisk::detectNewFeatures()
{
    std::cout << "<---- detectNewFeatures ---->" << std::endl << std::endl;

    cv::Mat image = ((CaptureImage*)getIncomingPtr())->getImage();
    cv::Rect roi;

    ///START OF THE SEARCH
    unsigned int n_features = 0;
    unsigned int n_iterations = 0;
    while((n_features < 1) && (n_iterations <3))
    {
        std::cout << "image rows: " << image.rows << ", image cols: " << image.cols << std::endl;
        bool roi_exists = act_search_grid_.pickRoi(roi);
        std::cout << "roi exists: " << roi_exists << std::endl;

        if(roi_exists==false)
        {
            break;
        }

        //x,y,width,height
        cv::Size image_size;    //locateROI will give the complete heigth and width of the image, not of the roi
        cv::Point ini_point_roi;
        cv::Mat image_roi = image(roi);
        image_roi.locateROI(image_size,ini_point_roi); //TODO: See if there is a way of just getting the point
        cv::Size size_roi(image_roi.cols,image_roi.rows);   //this variable HAS the right heigth and width of the roi
        cv::Rect myROI(ini_point_roi,size_roi);
        std::cout << "image rows: " << image_roi.rows << ", image cols: " << image_roi.cols << std::endl;
        n_features = briskDetect(getIncomingPtr(),image_roi,ini_point_roi,false);//cv_image);

        if (n_features == 0)
        {
            act_search_grid_.blockCell(roi);
        }

        // A method to draw the keypoints. Optional (false for drawing features)
        drawFeatures(((CaptureImage*)getIncomingPtr())->getImage(),((CaptureImage*)getIncomingPtr())->getKeypoints(), myROI);

        //use this to return the image to the original size before computing again the new roi
        //image.adjustROI(ini_point_roi.y,(image_size.height-(ini_point_roi.y+image.rows)),ini_point_roi.x,(image_size.width-(ini_point_roi.x+image.cols))); //TODO: Look for a clever way to do this
        //std::cout << "u_s height: " << unused_size.height << "u_s width: " << unused_size.width <<std::endl;
        //std::cout << "image rows: " << image.rows << ", image cols: " << image.cols << std::endl;
        n_iterations++;
    }
    std::cout << "n_features: " << n_features << std::endl;



    cv::waitKey(0);
    return n_features;
}


//Vote for KeyFrame generation. If a KeyFrame criterion is validated, this function returns true
bool ProcessorBrisk::voteForKeyFrame()
{
    std::cout << "<---- voteForKeyFrame ---->" << std::endl << std::endl;
    return true;
}


/** Protected */

//Initialize one landmark
LandmarkBase* ProcessorBrisk::createLandmark(FeatureBase* _feature_ptr)
{
    return new LandmarkBase(LANDMARK_POINT,new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
}

//Create a new constraint
ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, NodeBase* _node_ptr) //LandmarkBase* _lmk_ptr)
{
    //FeatureBase f_b = new FeatureBase(Eigen::Vector2s::Zero(),Eigen::Matrix2s::Zero());
    //FrameBase frame_b = frm_ptr = new FrameBase(NON_KEY_FRAME, TimeStamp(),new StateBlock(Eigen::Vector3s::Zero()), new StateQuaternion);
    //LandmarkBase* lndmrk = new LandmarkBase(LANDMARK_POINT,new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
    //return new ConstraintPoint2D(f_b,frame_b,CTR_ACTIVE);
    //return new ConstraintBase(CTR_IMG_PNT_TO_IMG_PNT,_feature_ptr,CTR_ACTIVE);
}
