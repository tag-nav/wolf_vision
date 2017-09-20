//std includes
#include <iostream>

// OpenCV includes
#if defined (HAVE_OPENCV3)
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#else
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#endif

int main(int argc, char** argv)
{

    std::cout << std::endl << "==================== tracker ORB test ======================" << std::endl;

    cv::Mat image;
    const char * filename;
    if (argc == 1)
    {
        std::cout <<" Usage: ./test_ROI_ORB image_file_name" << std::endl;
        return -1;
    }
    else
    {
        filename = argv[1];
        image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    }
    std::cout << "Input image file: " << filename << std::endl;
    if(!image.data)
    {
        std::cout << "failed" << std::endl;
        return -1;
    }
    else std::cout << "succeded" << std::endl;
    unsigned int img_width = image.cols;
    unsigned int img_height = image.rows;
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    cv::Ptr<cv::FeatureDetector> detector_descriptor_ptr_;

    unsigned int nfeatures = 20;
    float scaleFactor = 1.2;
    unsigned int nlevels = 8;
    unsigned int edgeThreshold = 16;
    unsigned int firstLevel = 0;
    unsigned int WTA_K = 2;                  //# See: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#a180ae17d3300cf2c619aa240d9b607e5
    unsigned int scoreType = 0;              //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    unsigned int patchSize = 31;

#if defined (HAVE_OPENCV3)
    detector_descriptor_ptr_ = cv::ORB::create(nfeatures, //
                                           scaleFactor, //
                                           nlevels, //
                                           edgeThreshold, //
                                           firstLevel, //
                                           WTA_K, //
                                           scoreType, //
                                           patchSize);//,
#else
    detector_descriptor_ptr_ = cv::ORB(nfeatures, //
                                       scaleFactor, //
                                       nlevels, //
                                       edgeThreshold, //
                                       firstLevel, //
                                       WTA_K, //
                                       scoreType, //
                                       patchSize);
#endif

    std::vector<cv::KeyPoint> target_keypoints;
    cv::KeyPointsFilter keypoint_filter;


    unsigned int roi_x;
    unsigned int roi_y;
    unsigned int roi_width = 50;
    unsigned int roi_heigth = 50;

    for(unsigned int i = 0; i < 3; i++)
    {
        if(i == 0)
            roi_y = 102 - (roi_width / 2);
        else if(i == 1)
            roi_y = 250 - (roi_width / 2);
        else
            roi_y = 476 - (roi_width / 2);
        for(roi_x = 0; roi_x < img_width; roi_x += 20)
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

            cv::Mat image_roi = image(roi_inflate);
            detector_descriptor_ptr_->detect(image_roi, target_keypoints);

            if (!target_keypoints.empty())
            {
                std::cout << "Keypoints detected: " << target_keypoints.size();
                keypoint_filter.retainBest(target_keypoints,1);
                std::cout << " - retained: " << target_keypoints.size();
                std::cout << "  at: ";
                for(unsigned int i = 0; i < target_keypoints.size(); i++)
                {
                    target_keypoints[i].pt.x += roi_inflate.x;
                    target_keypoints[i].pt.y += roi_inflate.y;
                    std::cout << "[ " << target_keypoints[i].pt.x << " , " << target_keypoints[i].pt.y << " ] ";
                }
                std::cout << std::endl;
            }

            cv::Mat image_graphics = image.clone();
            cv::drawKeypoints(image_graphics,target_keypoints,image_graphics);
            cv::rectangle(image_graphics, roi, cv::Scalar(255.0, 0.0, 255.0), 1, 8, 0);
            cv::rectangle(image_graphics, roi_inflate, cv::Scalar(255.0, 255.0, 0.0), 1, 8, 0);

            cv::imshow("test",image_graphics);
            cv::waitKey(5);

        }
    }
}

