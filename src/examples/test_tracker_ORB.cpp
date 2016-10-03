//std includes
#include <iostream>

//opencv includes
#include "opencv2/features2d/features2d.hpp"

#include "processor_image_landmark.h"

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== tracker ORB test ======================" << std::endl;

    cv::VideoCapture capture;
    const char * filename;
    if (argc == 1)
    {
        filename = "/home/jtarraso/Vídeos/gray.mp4";
        capture.open(filename);
    }
    else if (std::string(argv[1]) == "0")
    {
        //camera
        filename = "0";
        capture.open(0);
    }
    else
    {
        filename = argv[1];
        capture.open(filename);
    }
    std::cout << "Input video file: " << filename << std::endl;
    if(!capture.isOpened()) std::cout << "failed" << std::endl; else std::cout << "succeded" << std::endl;
    capture.set(CV_CAP_PROP_POS_MSEC, 3000);

    unsigned int img_width  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int img_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;



    cv::Feature2D* detector_descriptor_ptr_;
    cv::DescriptorMatcher* matcher_ptr_;

    unsigned int nfeatures = 500;
    float scaleFactor = 1.2;
    unsigned int nlevels = 3;
    unsigned int edgeThreshold = 3;
    unsigned int firstLevel = 0;
    unsigned int WTA_K = 2;                  //# See: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#a180ae17d3300cf2c619aa240d9b607e5
    unsigned int scoreType = 0;              //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    unsigned int patchSize = 3;

    detector_descriptor_ptr_ = new cv::ORB(nfeatures, //
                                           scaleFactor, //
                                           nlevels, //
                                           edgeThreshold, //
                                           firstLevel, //
                                           WTA_K, //
                                           scoreType, //
                                           patchSize);

    unsigned int nominal_pattern_radius = 0;
    unsigned int pattern_radius = (unsigned int)( (nominal_pattern_radius) * pow(scaleFactor, nlevels-1));

//    std::cout << "nominal pattern radius: " << _dd_params->nominal_pattern_radius << std::endl;
//    std::cout << "scale factor: " << params_orb->scaleFactor << std::endl;
//    std::cout << "nlevels: " << params_orb->nlevels << std::endl;

    unsigned int size_bits = detector_descriptor_ptr_->descriptorSize() * 8;

    matcher_ptr_ = new cv::BFMatcher(6);



    // CAPTURES
//    SensorCamera* camera_ptr_;
//    CaptureImage* image_ptr;
    TimeStamp t = 1;

    unsigned int buffer_size = 20;
    std::vector<cv::Mat> frame(buffer_size);
    unsigned int f  = 1;
    capture >> frame[f % buffer_size];

    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);

//    image_ptr = new CaptureImage(t, camera_ptr_, frame[f % buffer_size]);

    cv::imshow("Feature tracker", frame[f % buffer_size]);
    cv::waitKey(0);

    std::vector<cv::KeyPoint> target_keypoints;
    std::vector<cv::KeyPoint> current_keypoints;
    std::vector<cv::KeyPoint> current_keypoints2;
    cv::Mat target_descriptors;
    cv::Mat current_descriptors;
    cv::Mat current_descriptors2;
    cv::Mat image_roi_ = frame[f % buffer_size];


    unsigned int roi_width = 30;
    unsigned int roi_heigth = 30;
    unsigned int roi_x;
    unsigned int roi_y;


    detector_descriptor_ptr_->detect(image_roi_, target_keypoints);
    detector_descriptor_ptr_->compute(image_roi_, target_keypoints, target_descriptors);



    while(!(frame[f % buffer_size].empty()))
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cv::Mat image_roi = frame[f % buffer_size];
        std::vector<cv::DMatch> cv_matches;

        unsigned int tracked_keypoints = 0;

        for(unsigned int j = 0; j < target_keypoints.size(); j++)
        {
            std::cout << "target_descriptor[" << j << "]:\n" << target_descriptors.row(j) << std::endl;

            roi_x = (target_keypoints[j].pt.x) - (roi_heigth / 2);
            roi_y = (target_keypoints[j].pt.y) - (roi_width / 2);
            cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);

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
                roi.width = roi.width+diff_width;
            }
            if((unsigned int)(roi.y + roi.height) > img_height)
            {
                int diff_height = img_height - (roi.y + roi.height);
                roi.height = roi.height+diff_height;
            }

            //assign
            cv::Mat test_image = image_roi(roi);

            for(unsigned int f = 0; f < 2; f++)
            {

                detector_descriptor_ptr_->detect(test_image, keypoints);
                detector_descriptor_ptr_->compute(test_image, keypoints, descriptors);



                cv::Mat target_descriptor; //B(cv::Rect(0,0,vec_length,1));
                target_descriptor = target_descriptors(cv::Rect(0,j,target_descriptors.cols,1));

                if(keypoints.size() != 0)
                {
                    matcher_ptr_->match(target_descriptor, descriptors, cv_matches);
                    Scalar normalized_score = 1 - (Scalar)(cv_matches[0].distance)/size_bits;
                    std::cout << "normalized score: " << normalized_score << std::endl;
                    if(normalized_score < 0.9)
                    {
                        std::cout << "not tracked" << std::endl;
                        std::cout << "choosen_descriptor:\n" << descriptors.row(cv_matches[0].trainIdx) << std::endl;
                    }
                    else
                    {
                        std::cout << "tracked" << std::endl;
                        std::cout << "choosen_descriptor:\n" << descriptors.row(cv_matches[0].trainIdx) << std::endl;


                        tracked_keypoints++;
                        cv::Point point,t_point;
                        point.x = keypoints[cv_matches[0].trainIdx].pt.x + roi.x;
                        point.y = keypoints[cv_matches[0].trainIdx].pt.y + roi.y;
                        t_point.x = target_keypoints[j].pt.x;
                        t_point.y = target_keypoints[j].pt.y;

                        cv::circle(image_roi, t_point, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
                        cv::circle(image_roi, point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);
                        cv::putText(image_roi, std::to_string(j), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));

                        //introduce in list - tracked point
                        cv::KeyPoint tracked_kp = keypoints[cv_matches[0].trainIdx];
                        tracked_kp.pt.x = tracked_kp.pt.x + roi.x;
                        tracked_kp.pt.y = tracked_kp.pt.y + roi.y;
                        if(f==0)
                            current_keypoints.push_back(tracked_kp);
                        else
                            current_keypoints2.push_back(tracked_kp);

                        cv::Mat tracked_desc;
                        tracked_desc = descriptors(cv::Rect(0,cv_matches[0].trainIdx,target_descriptors.cols,1));
                        if(f==0)
                            current_descriptors.push_back(tracked_desc);
                        else
                            current_descriptors2.push_back(tracked_desc);

                        //introduce in list - target point
                        //                    current_keypoints.push_back(target_keypoints[j]);
                        //                    current_descriptors.push_back(target_descriptor);
                    }
                }
                else
                    std::cout << "not tracked" << std::endl;

            }

        }

        std::cout << "tracked keypoints: " << tracked_keypoints << "/" << target_keypoints.size() << std::endl;
        std::cout << "percentage: " << ((float)((float)tracked_keypoints/(float)target_keypoints.size()))*100 << "%" << std::endl;


        if(tracked_keypoints == 0)
        {
            detector_descriptor_ptr_->detect(image_roi_, target_keypoints);
            detector_descriptor_ptr_->compute(image_roi_, target_keypoints, target_descriptors);
            std::cout << "numbre of new keypoints to be tracked: " << target_keypoints.size() << std::endl;
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
            for(unsigned int i = 0; i < target_descriptors.rows; i++)
            {
                std::cout << "\ntarget descriptors";
                std::cout << target_descriptors.row(i);
            }
            for(unsigned int i = 0; i < current_descriptors.rows; i++)
            {
                std::cout << "\ncurrent descriptors";
                std::cout << current_descriptors.row(i);
            }
            for(unsigned int i = 0; i < current_descriptors2.rows; i++)
            {
                std::cout << "\ncurrent descriptors2";
                std::cout << current_descriptors2.row(i);
            }

            //target_keypoints.clear();
            target_keypoints = current_keypoints;
            current_descriptors.copyTo(target_descriptors);
            current_keypoints.clear();

            std::cout << "\nAFTER THE ADVANCE" << std::endl;
//            for(unsigned int i = 0; i < target_keypoints.size(); i++)
//            {
//                std::cout << "target keypoints";
//                std::cout << target_keypoints[i].pt << "\t" ;
//            }
            for(unsigned int i = 0; i < target_descriptors.rows; i++)
            {
                std::cout << "\ntarget descriptors";
                std::cout << target_descriptors.row(i);
            }

            std::cout << "\nEND OF ADVANCE\n";

        }

        cv::imshow("Feature tracker", image_roi);
        cv::waitKey(0);

        f++;
        capture >> frame[f % buffer_size];
    }
}
