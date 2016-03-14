// Wolf includes
#include "processor_brisk.h"
#include "active_search.h"

// OpenCV includes


/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(int _threshold, int _octaves, float _pattern_scales) :
    ProcessorTracker(PRC_TRACKER_BRISK, true, false),
    sensor_cam_ptr_(nullptr), capture_img_ptr_(nullptr), brisk_(_threshold, _octaves, _pattern_scales) //initializes the algoritm
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

    ActiveSearchGrid act_search_grid(image.rows,image.cols,8,8); //20, 20
    act_search_grid.clear();

    ///PROJECTION OF LANDMARKS (features in this case)

    unsigned int n_last_capture_feat = 0;
    for(std::list<FeatureBase*>::iterator feat_list=getLastPtr()->getFeatureListPtr()->begin();feat_list != getLastPtr()->getFeatureListPtr()->end(); ++feat_list)
    {
        std::cout << "inside iterator" << std::endl;
        //FeatureBase* last_feature = *feat_list;
        FeaturePointImage* last_feature = (FeaturePointImage*)*feat_list;
        Eigen::Vector2i feature_point = {last_feature->getKeypoint().pt.x,last_feature->getKeypoint().pt.y}; //TODO: Look if this is the right order
        act_search_grid.hitCell(feature_point);
        std::cout << "Last feature keypoint: " << last_feature->getKeypoint().pt << std::endl;
        n_last_capture_feat++;
    }
    std::cout << "n_last_capture_feat: " << n_last_capture_feat << std::endl;

    // 640/8 = 80; 360/8 = 45; These are the roi maximum values. You should search in the last capture features spaces with these roi.


    //TODO: Add the ActiveSearchGrid in the constructor of the processor, just like the Brisk.

    //TODO:
    //You need to block the cell the features of the las capture are in.
    //You need to find the roi (as in the grid (or less)) to search in the area around the last capture feaures
    //Do the matching







    //(should this be here?)
    ///START OF THE SEARCH
    unsigned int n_features = 0;
    while(n_features < 1)
    {
        std::cout << "image rows: " << image.rows << ", image cols: " << image.cols << std::endl;
        bool roi_exists = act_search_grid.pickRoi(image);
        std::cout << "roi exists: " << roi_exists << std::endl;

        if(roi_exists==false)
        {
            break;
        }

        //x,y,width,height
        cv::Size image_size;    //locateROI will give the complete heigth and width of the image, not of the roi
        cv::Point ini_point_roi;
        image.locateROI(image_size,ini_point_roi); //TODO: See if there is a way of just getting the point
        cv::Size size_roi(image.cols,image.rows);   //this variable HAS the right heigth and width of the roi
        cv::Rect myROI(ini_point_roi,size_roi);
        std::cout << "image rows: " << image.rows << ", image cols: " << image.cols << std::endl;
        n_features = briskImplementation(getIncomingPtr(),image,myROI);//cv_image);

        if (n_features == 0)
        {
            act_search_grid.blockCell(image);
        }

        // A method to draw the keypoints. Optional (false for drawing features)
        drawFeatures(((CaptureImage*)getIncomingPtr())->getImage(),((CaptureImage*)getIncomingPtr())->getKeypoints(), myROI);

        //use this to return the image to the original size before computing again the new roi
        image.adjustROI(ini_point_roi.y,(image_size.height-(ini_point_roi.y+image.rows)),ini_point_roi.x,(image_size.width-(ini_point_roi.x+image.cols))); //TODO: Look for a clever way to do this
        //std::cout << "u_s height: " << unused_size.height << "u_s width: " << unused_size.width <<std::endl;
        //std::cout << "image rows: " << image.rows << ", image cols: " << image.cols << std::endl;

    }
    std::cout << "n_features: " << n_features << std::endl;





    cv::waitKey(30);
    return n_features;
}


void ProcessorBrisk::drawFeatures(cv::Mat _image, std::vector<cv::KeyPoint> _kp, cv::Rect _roi)
{
    std::cout << "<---- drawFeatures ---->" << std::endl << std::endl;

    if(_kp.size()!=0)
    {
        for(unsigned int i = 0; i <= (_kp.size()-1);i++)
        {
            cv::Point point;
            point.x = _kp[i].pt.x + _roi.x;
            point.y = _kp[i].pt.y + _roi.y;
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

unsigned int ProcessorBrisk::briskImplementation(CaptureBase* _capture_ptr, cv::Mat _image, cv::Rect _roi)
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

    // Set the vector of keypoints and the matrix of descriptors in its capture
    /** Not sure if I can do this (the way the "set" is done, as it overwrittes) with the ROIs */
    ((CaptureImage*)_capture_ptr)->setKeypoints(keypoints);
    ((CaptureImage*)_capture_ptr)->setDescriptors(descriptors);

    if(keypoints.size()!=0)
    {
        // Add the features in the capture
        assert(!keypoints.size()==0 && "Keypoints size is 0");
        for(unsigned int i = 0; i <= (keypoints.size()-1);i++)
        {
            keypoint_coordinates(0) = keypoints[i].pt.x;    //TODO: This two lines should be erased when "measurement" is gone
            keypoint_coordinates(1) = keypoints[i].pt.y;

            descript_vector=descriptors(cv::Range(i,i+1),cv::Range(0,descriptors.cols));
            ((CaptureImage*)_capture_ptr)->addFeature(new FeaturePointImage(keypoint_coordinates,keypoints[i],descript_vector));
            std::cout << "Current feature keypoint: " << keypoints[i].pt << std::endl;
        }

//      drawFeatures(((CaptureImage*)_capture_ptr)->getImage(), keypoints, _roi); // A method to draw the keypoints. Optional
        return descriptors.rows;  //number of features
    }
    else
    {
//      drawFeatures(((CaptureImage*)_capture_ptr)->getImage(), keypoints, _roi); // A method to draw the keypoints. Optional
        return 0;
    }
}

// This is intended to create Features that are not among the Features already known in the Map. Returns the number of detected Features.
unsigned int ProcessorBrisk::detectNewFeatures()
{
    std::cout << "<---- detectNewFeatures ---->" << std::endl << std::endl;
    //setIncomingPtr(_capture_ptr);                                    // Set the capture as incoming_ptr_
    //const cv::Mat& cv_image = ((CaptureImage*)_capture_ptr)->getImage();  // Get the image from the capture

    //cv::Rect myROI(100, 100, 100, 100); //x,y,width,height
    //cv::Mat croppedImage = cv_image(myROI);
    //unsigned int h = briskImplementation(_capture_ptr,croppedImage,myROI);//cv_image);
    return 0;
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
ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, NodeBase* _feat_or_lmk_ptr)
{
    //FeatureBase f_b = new FeatureBase(Eigen::Vector2s::Zero(),Eigen::Matrix2s::Zero());
    //FrameBase frame_b = frm_ptr = new FrameBase(NON_KEY_FRAME, TimeStamp(),new StateBlock(Eigen::Vector3s::Zero()), new StateQuaternion);
    //LandmarkBase* lndmrk = new LandmarkBase(LANDMARK_POINT,new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
    //return new ConstraintPoint2D(f_b,frame_b,CTR_ACTIVE);
    //return new ConstraintBase(CTR_IMG_PNT_TO_IMG_PNT,_feature_ptr,CTR_ACTIVE);
    return nullptr;
}
