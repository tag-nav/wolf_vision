//std includes
#include <iostream>

// Vision utils
#include <vision_utils.h>
// REMOVE
// OpenCV includes
//#if defined (HAVE_OPENCV3)
//#include <opencv2/features2d.hpp>
//#else
//#include <opencv2/features2d/features2d.hpp>
//#endif

#include "processor_image_landmark.h"

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== tracker ORB test ======================" << std::endl;

    // parsing input params
     const char * filename;
     cv::VideoCapture capture;
     if (argc < 2)
     {
         std::cout << "Please use\n\t./test_opencv <arg> "
                 "\nwith "
                 "\n\targ = <path/videoname.mp4> for video input "
                 "\n\targ = 0 for camera input." << std::endl;
         return 0;
     }
     else if (std::string(argv[1]) == "0")
     {
         filename = "0"; // camera
         capture.open(0);
         std::cout << "Input stream from camera " << std::endl;
     }
     else
     {
         filename = argv[1]; // provided through argument
         capture.open(filename);
         std::cout << "Input video file: " << filename << std::endl;
     }
     // Open input stream
     if (!capture.isOpened())  // check if we succeeded
         std::cout << "failed" << std::endl;
     else
         std::cout << "succeded" << std::endl;

     // set and print image properties
     unsigned int img_width = (unsigned int)capture.get(CV_CAP_PROP_FRAME_WIDTH);
     unsigned int img_height = (unsigned int)capture.get(CV_CAP_PROP_FRAME_HEIGHT);
     std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    //=====================================================
    // Environment variable for configuration files
    std::string wolf_root = _WOLF_ROOT_DIR;
    //=====================================================

    //=====================================================
    // Detector, descriptor and matcher
    cv::Ptr<cv::FeatureDetector> detector_descriptor_ptr_;
    cv::Ptr<cv::DescriptorMatcher> matcher_ptr_;

    unsigned int nfeatures = 500;
    float scaleFactor = 2;
    unsigned int nlevels = 8;
    unsigned int edgeThreshold = 16;
    unsigned int firstLevel = 0;
    unsigned int WTA_K = 2;                  //# See: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#a180ae17d3300cf2c619aa240d9b607e5
    unsigned int scoreType = 0;              //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    unsigned int patchSize = 31;

    //unsigned int fastThreshold = 20;

    unsigned int roi_width = 200;
    unsigned int roi_heigth = 200;

    detector_descriptor_ptr_ = cv::ORB::create(nfeatures, //
                                           scaleFactor, //
                                           nlevels, //
                                           edgeThreshold, //
                                           firstLevel, //
                                           WTA_K, //
                                           scoreType, //
                                           patchSize);//

    unsigned int pattern_radius = (unsigned int)(patchSize);

    unsigned int size_bits = detector_descriptor_ptr_->descriptorSize() * 8;

    matcher_ptr_ = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");
    //=====================================================

    unsigned int buffer_size = 20;
    std::vector<cv::Mat> frame(buffer_size);
    unsigned int f  = 1;
    capture >> frame[f % buffer_size];

    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);

    cv::imshow("Feature tracker", frame[f % buffer_size]);
    cv::waitKey(1);

    std::vector<cv::KeyPoint> target_keypoints;
    std::vector<cv::KeyPoint> tracked_keypoints_;
    std::vector<cv::KeyPoint> tracked_keypoints_2;
    std::vector<cv::KeyPoint> current_keypoints;
    cv::Mat target_descriptors;
    cv::Mat tracked_descriptors;
    cv::Mat tracked_descriptors2;
    cv::Mat current_descriptors;
    cv::Mat image_original = frame[f % buffer_size].clone();
    cv::Mat image_graphics;


    unsigned int roi_x;
    unsigned int roi_y;

    int n_first_1 = 0;
    int n_second_1 = 0;


    detector_descriptor_ptr_->detect(image_original, target_keypoints);
    detector_descriptor_ptr_->compute(image_original, target_keypoints, target_descriptors);

    while(!(frame[f % buffer_size].empty()))
    {
        f++;
        capture >> frame[f % buffer_size];

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cv::Mat image = frame[f % buffer_size];
        image_graphics = image.clone();
        std::vector<cv::DMatch> cv_matches;
        bool matched = false;
        n_first_1 = n_second_1 = 0;

        unsigned int tracked_keypoints = 0;

        for(unsigned int j = 0; j < target_keypoints.size(); j++)
        {
            std::cout << "\npixel: " << target_keypoints[j].pt << std::endl;
            std::cout << "target_descriptor[" << j << "]:\n" << target_descriptors.row(j) << std::endl;

            matched = false;
            roi_x = (target_keypoints[j].pt.x) - (roi_heigth / 2);
            roi_y = (target_keypoints[j].pt.y) - (roi_width / 2);
            cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);
            cv::Point2f roi_up_left_corner;
            roi_up_left_corner.x = roi.x;
            roi_up_left_corner.y = roi.y;

            //inflate
            roi.x = roi.x - pattern_radius;
            roi.y = roi.y - pattern_radius;
            roi.width = roi.width + 2*pattern_radius;
            roi.height = roi.height + 2*pattern_radius;

            //trim
            if(roi.x < 0)
            {
                int diff_x = -roi.x;
                roi.x = 0;
                roi.width = roi.width - diff_x;
            }
            if(roi.y < 0)
            {
                int diff_y = -roi.y;
                roi.y = 0;
                roi.height = roi.height - diff_y;
            }
            if((unsigned int)(roi.x + roi.width) > img_width)
            {
                int diff_width = img_width - (roi.x + roi.width);
                roi.width = roi.width + diff_width;
            }
            if((unsigned int)(roi.y + roi.height) > img_height)
            {
                int diff_height = img_height - (roi.y + roi.height);
                roi.height = roi.height+diff_height;
            }


            //assign
            cv::Mat image_roi = image(roi);

            for(unsigned int f = 0; f < 2; f++)
            {

                detector_descriptor_ptr_->detect(image_roi, keypoints);
                detector_descriptor_ptr_->compute(image_roi, keypoints, descriptors);

                cv::Mat target_descriptor; //B(cv::Rect(0,0,vec_length,1));
                target_descriptor = target_descriptors(cv::Rect(0,j,target_descriptors.cols,1));

                if(keypoints.size() != 0)
                {
                    matcher_ptr_->match(target_descriptor, descriptors, cv_matches);
                    Scalar normalized_score = 1 - (Scalar)(cv_matches[0].distance)/size_bits;
                    std::cout << "pixel: " << keypoints[cv_matches[0].trainIdx].pt + roi_up_left_corner << std::endl;
                    std::cout << "normalized score: " << normalized_score << std::endl;
                    if(normalized_score < 0.8)
                    {
                        std::cout << "not tracked" << std::endl;
                    }
                    else
                    {
                        std::cout << "tracked" << std::endl;

                        matched = true;

                        cv::Point2f point,t_point;
                        point.x = keypoints[cv_matches[0].trainIdx].pt.x + roi.x;
                        point.y = keypoints[cv_matches[0].trainIdx].pt.y + roi.y;
                        t_point.x = target_keypoints[j].pt.x;
                        t_point.y = target_keypoints[j].pt.y;

                        cv::circle(image_graphics, t_point, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
                        cv::circle(image_graphics, point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);
                        cv::putText(image_graphics, std::to_string(j), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));

                        //introduce in list - tracked point
                        cv::KeyPoint tracked_kp = keypoints[cv_matches[0].trainIdx];
                        tracked_kp.pt.x = tracked_kp.pt.x + roi.x;
                        tracked_kp.pt.y = tracked_kp.pt.y + roi.y;
                        if(f==0)
                            tracked_keypoints_.push_back(tracked_kp);
                        else
                            tracked_keypoints_2.push_back(tracked_kp);

                        cv::Mat tracked_desc;
                        tracked_desc = descriptors(cv::Rect(0,cv_matches[0].trainIdx,target_descriptors.cols,1));
                        if(f==0)
                            tracked_descriptors.push_back(tracked_desc);
                        else
                            tracked_descriptors2.push_back(tracked_desc);

                        //introduce in list - target point
                        if(f==0)
                        {
                            current_keypoints.push_back(target_keypoints[j]);
                            current_descriptors.push_back(target_descriptor);
                        }

                        if (f == 0 && normalized_score == 1)n_first_1++;
                        if (f == 1 && normalized_score == 1)n_second_1++;
                    }
                }
                else
                    std::cout << "not tracked" << std::endl;

            }
            if (matched) tracked_keypoints++;


        }

        std::cout << "\ntracked keypoints: " << tracked_keypoints << "/" << target_keypoints.size() << std::endl;
        std::cout << "percentage first: " << ((float)((float)tracked_keypoints/(float)target_keypoints.size()))*100 << "%" << std::endl;
        std::cout << "Number of perfect first matches: " << n_first_1 << std::endl;
        std::cout << "Number of perfect second matches: " << n_second_1 << std::endl;

        if(tracked_keypoints == 0)
        {
            detector_descriptor_ptr_->detect(image, target_keypoints);
            detector_descriptor_ptr_->compute(image, target_keypoints, target_descriptors);
            std::cout << "number of new keypoints to be tracked: " << target_keypoints.size() << std::endl;
        }
        else
        {
            std::cout << "\n\nADVANCE" << std::endl;
//            for(unsigned int i = 0; i < target_keypoints.size(); i++)
//            {
//                std::cout << "\ntarget keypoints";
//                std::cout << target_keypoints[i].pt;
//            }
//            for(unsigned int i = 0; i < current_keypoints.size(); i++)
//            {
//                std::cout << "\ncurrent keypoints";
//                std::cout << current_keypoints[i].pt;
//            }
//            for( int i = 0; i < target_descriptors.rows; i++)
//            {
//                std::cout << "\ntarget descriptors";
//                std::cout << target_descriptors.row(i);
//            }
//            for( int i = 0; i < current_descriptors.rows; i++)
//            {
//                std::cout << "\ncurrent descriptors";
//                std::cout << current_descriptors.row(i);
//            }
//            for( int i = 0; i < current_descriptors2.rows; i++)
//            {
//                std::cout << "\ncurrent descriptors2";
//                std::cout << current_descriptors2.row(i);
//            }

            //target_keypoints.clear();
            target_keypoints = current_keypoints;
            current_descriptors.copyTo(target_descriptors);
            current_keypoints.clear();
            current_descriptors.release();


            std::cout << "\nAFTER THE ADVANCE" << std::endl;
//            for(unsigned int i = 0; i < target_keypoints.size(); i++)
//            {
//                std::cout << "target keypoints";
//                std::cout << target_keypoints[i].pt << "\t" ;
//            }
//            for( int i = 0; i < target_descriptors.rows; i++)
//            {
//                std::cout << "\ntarget descriptors";
//                std::cout << target_descriptors.row(i);
//            }

            std::cout << "\nEND OF ADVANCE\n";

        }

        tracked_keypoints = 0;
        cv::imshow("Feature tracker", image_graphics);
        cv::waitKey(1);

//        f++;
//        capture >> frame[f % buffer_size];
    }
}
