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
#include "opencv2/imgproc/imgproc.hpp"

// General includes
#include <math.h>
#include <complex>      // std::complex, std::norm

//std includes
#include <iostream>

int main(int argc, char** argv)
{
    std::cout << std::endl << " ========= OpenCv test ===========" << std::endl << std::endl;

    // parsing input params
    const char * filename;
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
        std::cout << "Input stream from camera " << std::endl;
    }
    else
    {
        filename = argv[1]; // provided through argument
        std::cout << "Input video file: " << filename << std::endl;
    }

    // Open input stream
    cv::VideoCapture capture(filename);
    if (!capture.isOpened())  // check if we succeeded
        std::cout << "failed" << std::endl;
    else
        std::cout << "succeded" << std::endl;

    // set and print image properties
    unsigned int img_width = (unsigned int)capture.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int img_height = (unsigned int)capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    // set the image buffer
    unsigned int buffer_size = 5;
    std::vector<cv::Mat> image_buffer(buffer_size);
    cv::Mat img_1, img_2;

    // user interface
    cv::namedWindow("Feature tracker"); // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);

    // set image processors
    cv::BRISK detector(30, 0, 1.0);
    cv::BRISK descriptor(30, 0, 1.0);
//    cv::FlannBasedMatcher matcher;
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    // declare all variables
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches, inlier_matches;
    std::vector<cv::Point2f> matched_1, matched_2;
    cv::Mat inliers_bool;
    std::vector<cv::Point2f> inliers_1, inliers_2;
    cv::Mat img_matches;
    cv::Mat img_scaled;
    double scale = 1;//0.875;

    unsigned int f = 0;
    capture >> image_buffer[f % buffer_size];
    while (!(image_buffer[f % buffer_size].empty()))
    {
        f++;
        std::cout << "\n=============== Frame #: " << f << " in buffer: " << f % buffer_size << " ==============="
                << std::endl;

        capture >> image_buffer[f % buffer_size];

        matched_1.clear();
        matched_2.clear();
        good_matches.clear();
        inlier_matches.clear();
        inliers_1.clear();
        inliers_2.clear();

        if (!image_buffer[f % buffer_size].empty())
        {
            if (f > buffer_size)
            {
                // use two temporaries for readability
                img_1 = image_buffer[f % buffer_size];
                img_2 = image_buffer[(f - buffer_size + 1) % buffer_size];

                // detect and describe in both images
                detector.detect(img_1, keypoints_1);
                detector.detect(img_2, keypoints_2);
                descriptor.compute(img_1, keypoints_1, descriptors_1);
                descriptor.compute(img_2, keypoints_2, descriptors_2);

                unsigned int max_dist = 0;
                unsigned int min_dist = 512;

                if (keypoints_1.size() * keypoints_2.size() != 0)
                {
                    // match (try flann later)
                    //-- Step 3: Matching descriptor vectors using FLANN matcher
                    matcher.match(descriptors_1, descriptors_2, matches);

                    //-- Quick calculation of max and min distances between keypoints
                    for (int i = 0; i < descriptors_1.rows; i++)
                    {
                        unsigned int dist = matches[i].distance;
                        if (dist < min_dist)
                            min_dist = dist;
                        if (dist > max_dist)
                            max_dist = dist;
                    }
                }
                else
                {
                    min_dist = 999;
                    max_dist = 999;
                }

                printf("-- Max dist : %d \n", max_dist);
                printf("-- Min dist : %d \n", min_dist);

                //-- Select only "good" matches (i.e. those at a distance which is small)
                for (unsigned int i = 0; i < matches.size(); i++)
                {
                    if (matches[i].distance <= 100) //std::max(2*min_dist, 0.02) )
                    {
                        good_matches.push_back(matches[i]);
                        matched_1.push_back(keypoints_1[matches[i].trainIdx].pt);
                        matched_2.push_back(keypoints_1[matches[i].queryIdx].pt);
                    }
                }

                // Compute the Fundamental matrix F and discard outliers through ransac in one go
                if (good_matches.size() > 20) // Minimum is 8 points, but we need more for robustness
                {
                    // find fundamental matrix using RANSAC, return set of inliers as bool vector
                    cv::Mat F = findFundamentalMat(matched_1, matched_2, cv::FM_RANSAC, 3.0, 0.99, inliers_bool);

                    std::cout << "F = " << F << std::endl;

                    // Recover the inlier matches
                    for (unsigned int i = 0; i < good_matches.size(); i++)
                        if (inliers_bool.at<char>(i) == 1)
                            inlier_matches.push_back(good_matches[i]);
                }

                std::cout << "Matches: initial " << matches.size() << " | good " << good_matches.size() << " | inliers "
                        << inlier_matches.size() << std::endl;

                // Draw RANSAC inliers
                cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, inlier_matches, img_matches,
                                cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), 0); //cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                //-- Show detected matches
//                resize(img_matches, img_scaled, cv::Size(), scale, scale, cv::INTER_NEAREST);
//                imshow("Feature tracker", img_1);
//                imshow("Feature tracker", img_2);
                imshow("Feature tracker", img_matches);

                // pause every X images and wait for key
                if (f % 100)
                    cv::waitKey(1); // continue
                else
                    cv::waitKey(0); // pause
            }

        }
        else
        {
            cv::waitKey(2000);
        }
        capture >> image_buffer[f % buffer_size];
    }
}

