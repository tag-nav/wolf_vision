// Wolf includes
#include "processor_brisk.h"

// OpenCV includes


/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(int _threshold, int _octaves, float _pattern_scales) :
    sensor_cam_ptr_(nullptr), capture_img_ptr_(nullptr), brisk_(_threshold, _octaves, _pattern_scales) //initializes the algoritm
{

}

//Destructor
ProcessorBrisk::~ProcessorBrisk()
{

}


// Tracker function. Returns the number of successful tracks.
unsigned int ProcessorBrisk::processKnownFeatures(CaptureBase* _incoming_ptr)
{
    return 1;
}


void ProcessorBrisk::drawFeatures(cv::Mat _image, std::vector<cv::KeyPoint> _kp)
{
    cv::namedWindow("Keypoint drawing");    // Creates a window for display.
    for(unsigned int i = 0; i <= (_kp.size()-1);i++)
    {
        cv::circle(_image,_kp[i].pt,2,cv::Scalar(88.0,250.0,154.0),-1,8,0);
    }
    cv::imshow("Keypoint drawing",_image);
    cv::waitKey(20);

}

unsigned int ProcessorBrisk::briskImplementation(CaptureBase* _capture_ptr, cv::Mat _image)
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
    /** Not sure if I can do this with the ROI */
    ((CaptureImage*)getIncomingPtr())->setKeypoints(keypoints);
    ((CaptureImage*)getIncomingPtr())->setDescriptors(descriptors);


    // Add the features in the capture
    assert(!keypoints.size()==0 && "Keypoints size is 0");
    for(unsigned int i = 0; i <= (keypoints.size()-1);i++)
    {
        keypoint_coordinates(0) = keypoints[i].pt.x;    //TODO: This two lines should be erased when "measurement" is gone
        keypoint_coordinates(1) = keypoints[i].pt.y;

        descript_vector=descriptors(cv::Range(i,i+1),cv::Range(0,descriptors.cols));
        ((CaptureImage*)getIncomingPtr())->addFeature(new FeaturePointImage(keypoint_coordinates,keypoints[i],descript_vector));
    }

    drawFeatures(treated_image, keypoints); // A method to draw the keypoints. Optional

    return descriptors.rows;  //number of features
}

// This is intended to create Features that are not among the Features already known in the Map. Returns the number of detected Features.
unsigned int ProcessorBrisk::detectNewFeatures(CaptureBase* _capture_ptr)
{

    //setIncomingPtr(_capture_ptr);                                    // Set the capture as incoming_ptr_
    cv::Mat cv_image = ((CaptureImage*)getIncomingPtr())->getImage();  // Get the image from the capture
    unsigned int h = briskImplementation(_capture_ptr, cv_image);
    return h;
}


//Vote for KeyFrame generation. If a KeyFrame criterion is validated, this function returns true
bool ProcessorBrisk::voteForKeyFrame()
{
    return true;
}


//Full processing of an incoming Capture
void ProcessorBrisk::process(CaptureBase* const _incoming_ptr)
{
    setIncomingPtr(_incoming_ptr);
    /** TODO
     * method: processKnownFeatures
        - Do the landmark projection and select the ROI to be used to do the matching
        - Use brisk in that ROIs to get features and do the matching
        - Create constraints
     * Analyze the number of matches. Decide if the method detectNewFeatures should be applied or not depending on that
     * method: detectNewFeatures
        - Use brisk in ROIs that do not detect features (that can be without features or just not enough of them)
        - Analyze if the detected features are good enough (and, if not, decide if the detection should be done again elsewhere)
        - Do the landmark creation
     * method: voteForKeyFrame (at present I don't know how to implement this, but it should go at the end anyway) (it does not create keyframes)
     */


    int n_features = detectNewFeatures(_incoming_ptr);
    std::cout << "n_features: " << n_features << std::endl;
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
