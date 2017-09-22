//std includes
#include <iostream>

// Vision utils
#include <vision_utils/vision_utils.h>
#include <vision_utils/detectors.h>

//Wolf
#include "wolf.h"

int main(int argc, char** argv)
{
    //=====================================================
    // Environment variable for configuration files
    std::string wolf_root = _WOLF_ROOT_DIR;
    //=====================================================

    std::cout << std::endl << "==================== tracker ORB test ======================" << std::endl;

    //=====================================================
    // Read Image
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
    //=====================================================

    //=====================================================
    // Define detector
    std::string yaml_file_params = wolf_root + "/src/examples/yaml/roi_orb.yaml";
    std::string det_name = vision_utils::readYamlType(yaml_file_params, "detector");
    vision_utils::DetectorBasePtr det_ptr = vision_utils::setupDetector(det_name, det_name + " detector", yaml_file_params);
    det_ptr = std::static_pointer_cast<vision_utils::DetectorORB>(det_ptr);
    //=====================================================

    // Loop variables
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
            // Set a candidate ROI
            cv::Rect roi = cv::Rect(roi_x, roi_y, roi_width, roi_heigth);
            cv::Rect roi_inflate = roi;

            // Detect features in ROI
            target_keypoints = det_ptr->detect(image, roi_inflate);

            // Keep only one KP in ROI
            if (!target_keypoints.empty())
            {
                std::cout << "Keypoints detected: " << target_keypoints.size();
                keypoint_filter.retainBest(target_keypoints,1);
                std::cout << " - retained: " << target_keypoints.size();
                std::cout << "  at: ";
                for(unsigned int i = 0; i < target_keypoints.size(); i++)
                    std::cout << "[ " << target_keypoints[i].pt.x << " , " << target_keypoints[i].pt.y << " ] ";
                std::cout << std::endl;
            }

            cv::Mat image_graphics = image.clone();
            cv::drawKeypoints(image_graphics,target_keypoints,image_graphics);
            cv::rectangle(image_graphics, roi, cv::Scalar(255.0, 0.0, 255.0), 1, 8, 0);
            cv::rectangle(image_graphics, roi_inflate, cv::Scalar(255.0, 255.0, 0.0), 1, 8, 0);
            cv::imshow("test_roi_orb",image_graphics);
            cv::waitKey(1);
        }
    }
}

