/*
 * test_opencv.cpp
 *
 *  Created on: Apr 24, 2016
 *      Author: jsola
 */



// Testing things for the 3D image odometry

// general includes
#include "unistd.h"
#include <time.h>
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// General includes
#include <math.h>
#include <complex>      // std::complex, std::norm

//std includes
#include <iostream>


int main(int argc, char** argv)
{
    std::cout << std::endl << " ========= OpenCv test ===========" << std::endl << std::endl;

    const char * filename;
    if (argc == 1)
    {
        filename = "/home/jtarraso/Vídeos/House interior.mp4";
    }
    else
    {
        if (std::string(argv[1]) == "0")
        {
            //camera
            filename = "0";
        }
        else
        {
            filename = argv[1];
        }
    }
    std::cout << "Input video file: " << filename << std::endl;
    cv::VideoCapture capture(filename);
    if(!capture.isOpened())  // check if we succeeded
    {
        std::cout << "failed" << std::endl;
    }
    else
    {
        std::cout << "succeded" << std::endl;
    }
    capture.set(CV_CAP_PROP_POS_MSEC, 0000);
    unsigned int img_width = (unsigned int)capture.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int img_height = (unsigned int)capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    unsigned int buffer_size = 4;
    std::vector<cv::Mat> frame(buffer_size);
    cv::Mat img_1, img_2;

    unsigned int margin = 0;
    cv::Rect roi(margin,margin,img_width-2*margin, img_height-2*margin);

    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);

    cv::BRISK detector;
    cv::BRISK descriptor;
//    cv::FlannBasedMatcher matcher;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2, matched_1, matched_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector< cv::DMatch > matches;

    unsigned int f = 0;
    while(f<800)
    {
        f++;
        std::cout << "\n=============== Frame #: " << f << " in buffer: " << f%buffer_size << " ===============" << std::endl;

        capture >> frame[f % buffer_size];

        if(!frame[f % buffer_size].empty())
        {
            cv::imshow("Feature tracker", frame[f % buffer_size]);
            cv::waitKey(1);


            if (f > 1)
            {
                img_1 = frame[f % buffer_size];
                img_2 = frame[(f-1) % buffer_size];

                // detect and describe in both images
                detector.detect(img_1(roi), keypoints_1);
                detector.detect(img_2(roi), keypoints_2);
                descriptor.compute(img_1(roi), keypoints_1, descriptors_1);
                descriptor.compute(img_2(roi), keypoints_2, descriptors_2);

                // match (try flann later)
                //-- Step 3: Matching descriptor vectors using FLANN matcher
                matcher.match( descriptors_1, descriptors_2, matches );

                double max_dist = 0; double min_dist = 512;

                //-- Quick calculation of max and min distances between keypoints
                for( int i = 0; i < descriptors_1.rows; i++ )
                { double dist = matches[i].distance;
                  if( dist < min_dist ) min_dist = dist;
                  if( dist > max_dist ) max_dist = dist;
                }

                printf("-- Max dist : %f \n", max_dist );
                printf("-- Min dist : %f \n", min_dist );

                //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
                //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
                //-- small)
                //-- PS.- radiusMatch can also be used here.
                std::vector< cv::DMatch > good_matches;

                for( int i = 0; i < descriptors_1.rows; i++ )
                { if( matches[i].distance <= 50)//std::max(2*min_dist, 0.02) )
                  { good_matches.push_back( matches[i]); }
                }

                //-- Draw only "good" matches
                cv::Mat img_matches;
                cv::drawMatches( img_1(roi), keypoints_1, img_2(roi), keypoints_2,
                             good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                             std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                //-- Show detected matches
                imshow( "Good Matches", img_matches );

//                // find fundamental matrix using RANSAC
//                cv::Mat F = findFundamentalMat(keypoints_1, keypoints_2, cv::FM_RANSAC);
//
//                std::cout << F << std::endl;
            }

        }
        else
        {
            cv::waitKey(2000);
        }
    }
}





