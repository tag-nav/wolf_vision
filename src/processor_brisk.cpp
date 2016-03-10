// Wolf includes
#include "processor_brisk.h"

// OpenCV includes


/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(int _threshold, int _octaves, float _pattern_scales) :
    ProcessorTracker(true, false),
    sensor_cam_ptr_(nullptr), capture_img_ptr_(nullptr), brisk_(_threshold, _octaves, _pattern_scales) //initializes the algoritm
{
    //TODO: remove sensor_cam_ptr_, capture_img_ptr_
}

//Destructor
ProcessorBrisk::~ProcessorBrisk()
{

}


// Tracker function. Returns the number of successful tracks.
unsigned int ProcessorBrisk::processKnownFeatures(CaptureBase* _incoming_ptr)
{
    incoming_ptr_ = _incoming_ptr;
    return 0;
}


void ProcessorBrisk::drawFeatures(cv::Mat _image, std::vector<cv::KeyPoint> _kp, cv::Rect _roi)
{
    cv::namedWindow("Keypoint drawing");    // Creates a window for display.
    //if(_kp.size!=0)
    //{
        for(unsigned int i = 0; i <= (_kp.size()-1);i++)
        {
            cv::Point point;
            point.x = _kp[i].pt.x + _roi.x;
            point.y = _kp[i].pt.y + _roi.y;
            cv::circle(_image,point,2,cv::Scalar(88.0,250.0,154.0),-1,8,0); //_kp[i].pt
            cv::rectangle(_image,_roi,cv::Scalar(88.0,250.0,154.0),1,8,0);
        }
        cv::imshow("Keypoint drawing",_image);
        cv::waitKey(20);
    //}
    //else
    //{

      //  cv::imshow("Keypoint drawing",_image);
       // cv::waitKey(20);
    //}
}

unsigned int ProcessorBrisk::briskImplementation(CaptureBase* _capture_ptr, cv::Mat _image, cv::Rect _roi)
{
    cv::Mat treated_image = _image;         // Set the image to analyze
    std::vector<cv::KeyPoint> keypoints;    // Vector of keypoints
    cv::Mat descriptors;                    // Matrix of descriptors
    Eigen::Vector2s keypoint_coordinates;   // TODO: When "measurement" is erased, this variable should be erased to, all over the code
    std::vector<float> descript_vector;     // Vector to store the descriptor for each keypoint

    //Brisk Algorithm
    brisk_.create("Feature2D.BRISK");
    brisk_.detect(treated_image, keypoints);
    brisk_.compute(treated_image, keypoints,descriptors);

    // Set the vector of keypoints and the matrix of descriptors in its capture
    /** Not sure if I can do this (the way the "set" is done, as it overwrittes) with the ROIs */
    ((CaptureImage*)_capture_ptr)->setKeypoints(keypoints);
    ((CaptureImage*)_capture_ptr)->setDescriptors(descriptors);

//    if(keypoints.size()!=0)
//    {
        // Add the features in the capture
        assert(!keypoints.size()==0 && "Keypoints size is 0");
        for(unsigned int i = 0; i <= (keypoints.size()-1);i++)
        {
            keypoint_coordinates(0) = keypoints[i].pt.x;    //TODO: This two lines should be erased when "measurement" is gone
            keypoint_coordinates(1) = keypoints[i].pt.y;

            descript_vector=descriptors(cv::Range(i,i+1),cv::Range(0,descriptors.cols));
            ((CaptureImage*)_capture_ptr)->addFeature(new FeaturePointImage(keypoint_coordinates,keypoints[i],descript_vector));
        }

        drawFeatures(((CaptureImage*)_capture_ptr)->getImage(), keypoints, _roi); // A method to draw the keypoints. Optional

        return descriptors.rows;  //number of features
    //}
    //else
    //{
      //  drawFeatures(((CaptureImage*)_capture_ptr)->getImage(), keypoints, _roi); // A method to draw the keypoints. Optional
       // return 0;
    //}
}

// This is intended to create Features that are not among the Features already known in the Map. Returns the number of detected Features.
unsigned int ProcessorBrisk::detectNewFeatures(CaptureBase* _capture_ptr)
{

    //setIncomingPtr(_capture_ptr);                                    // Set the capture as incoming_ptr_
    const cv::Mat& cv_image = ((CaptureImage*)_capture_ptr)->getImage();  // Get the image from the capture

    std::cout << "Mat rows: " << cv_image.rows << "Mat cols: " << cv_image.cols << std::endl;

    cv::Rect myROI(100, 100, 100, 100); //x,y,width,height
    cv::Mat croppedImage = cv_image(myROI);
    unsigned int h = briskImplementation(_capture_ptr,croppedImage,myROI);//cv_image);
    return h;
}


//Vote for KeyFrame generation. If a KeyFrame criterion is validated, this function returns true
bool ProcessorBrisk::voteForKeyFrame()
{
    return true;
}


/** Protected */

//Initialize one landmark
LandmarkBase* ProcessorBrisk::createLandmark(FeatureBase* _feature_ptr)
{

}

//Create a new constraint
ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _lmk_ptr)
{

}
