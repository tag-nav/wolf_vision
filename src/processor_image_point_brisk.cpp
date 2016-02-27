
// Wolf includes
#include "processor_image_point_brisk.h"

// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "unistd.h"


ProcessorImagePointBrisk::ProcessorImagePointBrisk(int _threshold, int _octaves, float _pattern_scales) :
        sensor_cam_ptr_(nullptr), capture_img_ptr_(nullptr), brisk_(_threshold, _octaves, _pattern_scales) //initializes the algoritm
{
    std::cout << "ProcessorImagePointBrisk constructor" << std::endl;
}

ProcessorImagePointBrisk::~ProcessorImagePointBrisk()
{

}

void ProcessorImagePointBrisk::extractFeatures(CaptureBase *_capture_ptr)
{
    /** The features are obtained and introduced in the capture object */
    clock_t t1 = clock();
    capture_img_ptr_ = (CaptureImage*)(_capture_ptr);

    //BRISK TEST
    //std::cout << "Brisk Testing" << std::endl;
    std::cout << "Time1: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //variables needed for Brisk
    cv::Mat treated_image = capture_img_ptr_->getImage();
    std::cout << "Time15: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::cout << "Time16: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //Brisk parameters
    //int Threshl=30;
    //int Octaves=0; //(pyramid layer) from which the keypoint has been extracted
    //float PatternScales=1.0f;

    //Brisk Algorithm
    std::cout << "Time17: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //cv::BRISK BRISKD(Threshl,Octaves,PatternScales); //initializes the algoritm
    std::cout << "Time2: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    brisk_.create("Feature2D.BRISK");
    std::cout << "Time3: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    brisk_.detect(treated_image, keypoints);
    std::cout << "Time4: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    brisk_.compute(treated_image, keypoints,descriptors); //
    std::cout << "Time5: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;

    //Extras needed to draw and store the values
    cv::namedWindow("Keypoint drawing"); // Creates a window for display.
    Eigen::Vector2s keypoint_coordinates;

    //Drawing the features (circles) and storing them in the capture object
    assert(!keypoints.size()==0 && "Keypoints size is 0");
    std::cout << "Time6: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    for(unsigned int i = 0; i <= (keypoints.size()-1);i++)
    {
        cv::circle(treated_image,keypoints[i].pt,2,cv::Scalar(88.0,250.0,154.0),-1,8,0);
        //std::cout << "keypoint[" << i << "]: " << keypoints[i].pt << std::endl;
        keypoint_coordinates(0) = keypoints[i].pt.x;
        keypoint_coordinates(1) = keypoints[i].pt.y;
        //FeaturePoint* feat_point = new FeaturePoint(keypoint_coordinates);
        capture_img_ptr_->addFeature(new FeaturePoint(keypoint_coordinates));
    }
    std::cout << "Time7: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //Visualize the image
    cv::imshow("Keypoint drawing",treated_image);
    std::cout << "Time8: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //cv::waitKey(20); //for video
    //cv::waitKey(0); //for image
    //
    std::cout << "Time9: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    std::cout << "capture_img_ptr_ size: " << capture_img_ptr_->getFeatureListPtr()->size() << std::endl;
    //std::cout << "capture_img_ptr_ empty: " << capture_img_ptr_->getFeatureListPtr()->empty()<< std::endl;

    //std::cout << "Brisk Testing End" << std::endl;
}

void ProcessorImagePointBrisk::establishConstraints(CaptureBase *_capture_ptr)
{

}
