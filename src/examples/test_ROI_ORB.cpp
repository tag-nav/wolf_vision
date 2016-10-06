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
//        filename = "/home/jtarraso/Vídeos/gray.mp4";
        filename = "/home/jtarraso/Imágenes/Test_ORB.png";
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

    unsigned int nfeatures = 20;
    float scaleFactor = 1.2;
    unsigned int nlevels = 8;
    unsigned int edgeThreshold = 16;
    unsigned int firstLevel = 0;
    unsigned int WTA_K = 2;                  //# See: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#a180ae17d3300cf2c619aa240d9b607e5
    unsigned int scoreType = 0;              //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    unsigned int patchSize = 31;

    unsigned int fastThreshold = 20;

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
//                                           fastThreshold);

    unsigned int nominal_pattern_radius = 0;
    unsigned int pattern_radius = (unsigned int)( (nominal_pattern_radius) * pow(scaleFactor, nlevels-1));

//    std::cout << "nominal pattern radius: " << _dd_params->nominal_pattern_radius << std::endl;
//    std::cout << "scale factor: " << params_orb->scaleFactor << std::endl;
//    std::cout << "nlevels: " << params_orb->nlevels << std::endl;

    unsigned int size_bits = detector_descriptor_ptr_->descriptorSize() * 8;

    matcher_ptr_ = new cv::BFMatcher(6);




    unsigned int buffer_size = 20;
    std::vector<cv::Mat> frame(buffer_size);
    unsigned int f  = 1;
    capture >> frame[f % buffer_size];


//    image_ptr = new CaptureImage(t, camera_ptr_, frame[f % buffer_size]);


    std::vector<cv::KeyPoint> target_keypoints;
//    std::vector<cv::KeyPoint> tracked_keypoints_;
//    std::vector<cv::KeyPoint> tracked_keypoints_2;
//    std::vector<cv::KeyPoint> current_keypoints;
    cv::Mat target_descriptors;
//    cv::Mat tracked_descriptors;
//    cv::Mat tracked_descriptors2;
//    cv::Mat current_descriptors;
    cv::Mat image_original = frame[f % buffer_size].clone();
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

            cv::Mat image_roi = image(roi_inflate);
            detector_descriptor_ptr_->detect(image_roi, target_keypoints);
            detector_descriptor_ptr_->compute(image_roi, target_keypoints, target_descriptors);

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

