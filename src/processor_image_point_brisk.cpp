
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
    std::cout << "Time2: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::cout << "Time3: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //Brisk parameters
    //int Threshl=30;
    //int Octaves=0; //(pyramid layer) from which the keypoint has been extracted
    //float PatternScales=1.0f;

    //Brisk Algorithm
    std::cout << "Time4: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //cv::BRISK BRISKD(Threshl,Octaves,PatternScales); //initializes the algoritm
    std::cout << "Time5: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    brisk_.create("Feature2D.BRISK");
    std::cout << "Time6: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    brisk_.detect(treated_image, keypoints);
    std::cout << "Time7: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    brisk_.compute(treated_image, keypoints,descriptors); //
    std::cout << "Time8: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;


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

    //std::cout << "descriptor: " << descriptors(cv::Range(0,descriptors.rows),cv::Range(0,descriptors.cols));

    //Extras needed to draw and store the values
    cv::namedWindow("Keypoint drawing"); // Creates a window for display.
    Eigen::Vector2s keypoint_coordinates; // Variable needed to store the point of the keypoints to send it to the feature
    std::vector<float> descript_vector; // Vector to store the descriptor for each keypoint

    //Drawing the features (circles) and storing them in the capture object
    assert(!keypoints.size()==0 && "Keypoints size is 0");
    std::cout << "Time9: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    for(unsigned int i = 0; i <= (keypoints.size()-1);i++)
    {
        cv::circle(treated_image,keypoints[i].pt,2,cv::Scalar(88.0,250.0,154.0),-1,8,0);
        //std::cout << "keypoint[" << i << "]: " << keypoints[i].pt << std::endl;
        keypoint_coordinates(0) = keypoints[i].pt.x;
        keypoint_coordinates(1) = keypoints[i].pt.y;
        //FeaturePoint* feat_point = new FeaturePoint(keypoint_coordinates);

        descript_vector=descriptors(cv::Range(i,i+1),cv::Range(0,descriptors.cols));

        capture_img_ptr_->addFeature(new FeaturePoint(keypoint_coordinates,descript_vector));
    }
    std::cout << "Time10: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //Visualize the image
    cv::imshow("Keypoint drawing",treated_image);
    std::cout << "Time11: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    //cv::waitKey(20); //for video
    //cv::waitKey(0); //for image
    //
    std::cout << "Time12: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    std::cout << "capture_img_ptr_ size: " << capture_img_ptr_->getFeatureListPtr()->size() << std::endl;
    //std::cout << "capture_img_ptr_ empty: " << capture_img_ptr_->getFeatureListPtr()->empty()<< std::endl;

    //std::cout << "Brisk Testing End" << std::endl;
}

void ProcessorImagePointBrisk::establishConstraints(CaptureBase *_other_capture_ptr)
{
    //Compare _other_capture_ptr con la _capture_ptr
    clock_t t1 = clock();
    int position_range = 10;
    int descriptor_deviation = 15;  //64*15 = threshold (?)



    std::cout << "============================= FEATURE LIST ==============================="<< std::endl;
    //std::cout << "capture_img_ptr_ size: " << capture_img_ptr_->getFeatureListPtr()->size() << std::endl;

    std::cout << "TimeC1: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    cv::Mat first_descriptor_matrix((int)capture_img_ptr_->getFeatureListPtr()->size(),64,2);//rows,cols,type//TO DO: type? I just put 2 (1 cuts values in half)
    cv::Mat second_descriptor_matrix((int)_other_capture_ptr->getFeatureListPtr()->size(),64,2);
    //((FeaturePoint*) *capture_img_ptr_->getFeatureListPtr()
    //std::cout << " SIZE: " << capture_img_ptr_->getFeatureListPtr()->size() << std::endl;

    std::cout << "TimeC2: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;

    int t = 0;
    for(std::list<FeatureBase*>::iterator it=capture_img_ptr_->getFeatureListPtr()->begin(); it != capture_img_ptr_->getFeatureListPtr()->end(); it++)
    //for(capture_img_ptr_->getFeatureListPtr()->iterator.operator ++())
    {
        //std::cout << "t: " << t << std::endl;

        std::vector<float> descript_v;
        descript_v = ((FeaturePoint *)(*it))->getDescriptor();
        for(int i = 0; i <= descript_v.size()-1;i++)
        {
            first_descriptor_matrix(cv::Range(t,t+1),cv::Range(i,i+1)) = descript_v[i];
            //std::cout << "COPIED VALUE: " << descript_v[i] << std::endl;
        }
        //std::cout << "COPIED VALUE matrix: " << first_descriptor_matrix(cv::Range(t,t+1),cv::Range(0,descript_v.size())) << std::endl;
        //std::cout << "descriptor vector:" << std::endl;
        /** for(int i=0; i <= descript_v.size()-1;i++)
        {
            std::cout << descript_v[i] << " ";
        }
        std::cout << ";" << std::endl;
        */
        t++;
    }
    std::cout << "TimeC3: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    /**
    std::cout << "first descriptor matrix:" << std::endl;
    for(int i=0; i <= first_descriptor_matrix.rows-1;i++)
    {

        std::cout << first_descriptor_matrix(cv::Range(i,i+1),cv::Range(0,first_descriptor_matrix.cols-1)) << " ";
        std::cout << ";" << std::endl;
    }
    std::cout << ";" << std::endl;
    */



    int g = 0;
    for(std::list<FeatureBase*>::iterator it2=_other_capture_ptr->getFeatureListPtr()->begin(); it2 != _other_capture_ptr->getFeatureListPtr()->end(); it2++)
    //for(capture_img_ptr_->getFeatureListPtr()->iterator.operator ++())
    {
        //std::cout << "g: " << g << std::endl;

        std::vector<float> descript_v2;
        descript_v2 = ((FeaturePoint *)(*it2))->getDescriptor();
        for(int i = 0; i <= descript_v2.size()-1;i++)
        {
            second_descriptor_matrix(cv::Range(g,g+1),cv::Range(i,i+1)) = descript_v2[i];
            //std::cout << "COPIED VALUE: " << descript_v2[i] << std::endl;
        }

        /**
        std::cout << "descriptor vector:" << std::endl;
        for(int i=0; i <= descript_v2.size()-1;i++)
        {
            std::cout << descript_v2[i] << " ";
        }
        std::cout << ";" << std::endl;
        */

        g++;
    }
    std::cout << "TimeC4: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    /**
    std::cout << "second descriptor matrix:" << std::endl;
    for(int i=0; i <= second_descriptor_matrix.rows-1;i++)
    {

        std::cout << second_descriptor_matrix(cv::Range(i,i+1),cv::Range(0,second_descriptor_matrix.cols-1)) << " ";
        std::cout << ";" << std::endl;
    }
    std::cout << ";" << std::endl;
    */

    //Matching
    std::vector<float> matching_vector;
    //std::cout<< "value: " << std::abs((int)first_descriptor_matrix(cv::Range(0,1),cv::Range(k,k+1))
    //                                  -(int)second_descriptor_matrix(cv::Range(i,i+1),cv::Range(k,k+1)))

    /** for(int i=0; i<=((second_descriptor_matrix.rows-1)*(first_descriptor_matrix.rows-1));i++)
    {
        matching_vector[i]=0;
        for(int k=0; k<=second_descriptor_matrix.cols-1;k++)
        {
            matching_vector[i]=matching_vector[i]+std::abs((int)first_descriptor_matrix(cv::Range(i,i+1),cv::Range(k,k+1))-
                                        (int)second_descriptor_matrix(cv::Range(i,i+1),cv::Range(k,k+1)));
        }
    }
    */



}
