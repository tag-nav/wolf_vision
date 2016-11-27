// wolf includes
#include "wolf.h"
#include "logging.h"

//std includes
#include <iostream>

//opencv includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

int main(int argc, char** argv)
{

    std::cout << std::endl << "==================== tracker ORB test ======================" << std::endl;

    cv::VideoCapture capture;
    const char * filename;
    if (argc == 1)
    {
        filename = "/home/jtarraso/Imágenes/Test_ORB.png";
        capture.open(filename);
    }
    else
    {
        filename = argv[1];
        capture.open(filename);
    }
    std::cout << "Input image file: " << filename << std::endl;
    if(!capture.isOpened()) std::cout << "failed" << std::endl; else std::cout << "succeded" << std::endl;
    capture.set(CV_CAP_PROP_POS_MSEC, 3000);

    unsigned int img_width  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int img_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    cv::Feature2D* detector_descriptor_ptr_;

    unsigned int nfeatures = 20;
    float scaleFactor = 1.2;
    unsigned int nlevels = 8;
    unsigned int edgeThreshold = 16;
    unsigned int firstLevel = 0;
    unsigned int WTA_K = 2;                  //# See: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#a180ae17d3300cf2c619aa240d9b607e5
    unsigned int scoreType = 0;              //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    unsigned int patchSize = 31;

    unsigned int roi_width = 50;
    unsigned int roi_heigth = 50;

    detector_descriptor_ptr_ = new cv::ORB(nfeatures, //
                                           scaleFactor, //
                                           nlevels, //
                                           edgeThreshold, //
                                           firstLevel, //
                                           WTA_K, //
                                           scoreType, //
                                           patchSize);//,


    std::vector<cv::KeyPoint> target_keypoints;
    cv::Mat target_descriptors;
    cv::Mat image_original;
    capture >> image_original;
    cv::Mat image;
    cv::Mat image_graphics = image_original;
    cv::KeyPointsFilter keypoint_filter;


    unsigned int roi_x;
    unsigned int roi_y;

    roi_y = (102) - (roi_width / 2);

    for(unsigned int i = 0; i < 2; i++)
    {
        if(i == 0)
            roi_y = (102) - (roi_width / 2);
        else
            roi_y = (476) - (roi_width / 2);
        for(roi_x = 0; roi_x < img_width; roi_x++)
        {

            cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);
            cv::Rect roi_inflate = roi;

            roi_inflate.x = roi.x - edgeThreshold;
            roi_inflate.y = roi.y - edgeThreshold;
            roi_inflate.width = roi.width + 2*edgeThreshold;
            roi_inflate.height = roi.height + 2*edgeThreshold;

            if(roi_inflate.x < 0)
            {
                int diff_x = -roi_inflate.x;
                roi_inflate.x = 0;
                roi_inflate.width = roi_inflate.width - diff_x;
            }
            if(roi_inflate.y < 0)
            {
                int diff_y = -roi_inflate.y;
                roi_inflate.y = 0;
                roi_inflate.height = roi_inflate.height - diff_y;
            }
            if((unsigned int)(roi_inflate.x + roi_inflate.width) > img_width)
            {
                int diff_width = img_width - (roi_inflate.x + roi_inflate.width);
                roi_inflate.width = roi_inflate.width + diff_width;
            }
            if((unsigned int)(roi_inflate.y + roi_inflate.height) > img_height)
            {
                int diff_height = img_height - (roi_inflate.y + roi_inflate.height);
                roi_inflate.height = roi_inflate.height+diff_height;
            }

            image = image_original.clone();

            WOLF_TRACE("img size: ", image.cols , "x", image.rows);
            WOLF_TRACE("roi     : (", roi_inflate.x , ",", roi_inflate.x , ") ", roi_inflate.width, "x", roi_inflate.height);
            cv::Mat image_roi = image(roi_inflate);

            WOLF_TRACE("");
            detector_descriptor_ptr_->detect(image_roi, target_keypoints);

            WOLF_TRACE("");
            detector_descriptor_ptr_->compute(image_roi, target_keypoints, target_descriptors);
            WOLF_TRACE("");

            keypoint_filter.retainBest(target_keypoints,1);
            std::cout << "number of keypoints found: " << target_keypoints.size() << std::endl;
            for(unsigned int i = 0; i < target_keypoints.size(); i++)
            {
                target_keypoints[i].pt.x += roi_inflate.x;
                target_keypoints[i].pt.y += roi_inflate.y;
            }
            image_graphics = image_original.clone();
            cv::drawKeypoints(image_graphics,target_keypoints,image_graphics);
            cv::rectangle(image_graphics, roi, cv::Scalar(255.0, 0.0, 255.0), 1, 8, 0);
            //cv::circle(image_graphics, point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);

            cv::imshow("test",image_graphics);
            cv::waitKey(0);

        }
    }
}

