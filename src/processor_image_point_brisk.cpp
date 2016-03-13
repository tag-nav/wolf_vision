
// Wolf includes
#include "processor_image_point_brisk.h"

// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "unistd.h"


ProcessorImagePointBrisk::ProcessorImagePointBrisk(int _threshold, int _octaves, float _pattern_scales) :
        ProcessorTracker(PRC_TRACKER_BRISK), sensor_cam_ptr_(nullptr), capture_img_ptr_(nullptr), brisk_(_threshold, _octaves, _pattern_scales) //initializes the algoritm
{
    std::cout << "ProcessorImagePointBrisk constructor" << std::endl;
}

ProcessorImagePointBrisk::~ProcessorImagePointBrisk()
{

}

void ProcessorImagePointBrisk::process(CaptureBase* _capture_ptr)
{
    extractFeatures(_capture_ptr);
    establishConstraints(_capture_ptr);
}



void ProcessorImagePointBrisk::extractFeatures(CaptureBase *_capture_ptr)
{
    clock_t t1 = clock();
    std::cout << "Time1: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;


    /** The features are obtained and introduced in the capture object */
    capture_img_ptr_ = (CaptureImage*)(_capture_ptr);


    //variables needed for Brisk
    cv::Mat treated_image = capture_img_ptr_->getImage();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::cout << "Time2: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;


    //Brisk Algorithm
    brisk_.create("Feature2D.BRISK");
    std::cout << "Time3: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    brisk_.detect(treated_image, keypoints);
    std::cout << "Time4: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    brisk_.compute(treated_image, keypoints,descriptors); //
    std::cout << "Time5: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;


    //Introducing keypoint vector and descriptor matrix into the capture
    capture_img_ptr_->setKeypoints(keypoints);
    capture_img_ptr_->setDescriptors(descriptors);

//    for (auto keypoint : keypoints){
//        keypoint(0) = keypoint(1);
//    }

//    for (std::vector<cv::KeyPoint>::iterator it = keypoints.rbegin(); it != keypoints.rend(); ++it)
//    {
//        cv::KeyPoint keypoint = *it;
//        keypoint(0) = keypoint(1);

//        std::vector<cv::KeyPoint> a;
//        a(5) = 9;
//        cv::KeyPoint b[10];
//        b[5] = 0;
//        Eigen::VectorXs c;
//        c(9) =3;
//    }

    /**
    //To include the descriptor in the features
    std::cout << "Descriptor info. Cols: " << descriptors.cols << ", Rows: " << descriptors.rows << std::endl;
    std::cout << "Keypoint size: " << keypoints.size() << std::endl;

    for(unsigned int j = 0; j <= (descriptors.rows-1);j++)
    {
        std::cout << "descriptor vector: " << descriptors(cv::Range(j,j+1),cv::Range(0,descriptors.cols)) << std::endl;
        descript_vector=descriptors(cv::Range(j,j+1),cv::Range(0,descriptors.cols));
        std::cout << "desc_v value: " << descript_vector[63] << std::endl;
        std::cout << "desc_v lenght: " << descript_vector.size() << std::endl;
    }*/


    //Extras needed to draw and store the values
    cv::namedWindow("Keypoint drawing");    // Creates a window for display.
    Eigen::Vector2s keypoint_coordinates;   // Variable needed to store the point of the keypoints to send it to the feature
    std::vector<float> descript_vector;     // Vector to store the descriptor for each keypoint

    //Drawing the features (circles) and storing them in the capture object
    assert(!keypoints.size()==0 && "Keypoints size is 0");
    std::cout << "Time6: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    for(unsigned int i = 0; i <= (keypoints.size()-1);i++)
    {
        cv::circle(treated_image,keypoints[i].pt,2,cv::Scalar(88.0,250.0,154.0),-1,8,0); //TODO: Do this somewhere else
        keypoint_coordinates(0) = keypoints[i].pt.x;
        keypoint_coordinates(1) = keypoints[i].pt.y;
        //FeaturePoint* feat_point = new FeaturePoint(keypoint_coordinates);

        descript_vector=descriptors(cv::Range(i,i+1),cv::Range(0,descriptors.cols));

        capture_img_ptr_->addFeature(new FeaturePointImage(keypoint_coordinates,keypoints[i],descript_vector));
    }

    std::cout << "Time10: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;

    //Visualize the image
    cv::imshow("Keypoint drawing",treated_image);
    std::cout << "capture_img_ptr_ size: " << capture_img_ptr_->getFeatureListPtr()->size() << std::endl;
    std::cout << "Time11: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
}

void ProcessorImagePointBrisk::establishConstraints(CaptureBase *_other_capture_ptr)
{

}
