// Testing things for the 3D image odometry

//Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "feature_point_image.h"
#include "processor_brisk.h"
#include "state_block.h"
#include "state_quaternion.h"

// general includes
#include "unistd.h"
#include <time.h>
#include "opencv2/calib3d/calib3d.hpp"

//std includes
#include <iostream>


int main(int argc, char** argv)
{
    using namespace wolf;

    /**
    if (argc != 2 || atoi(argv[1]) < 0 || atoi(argv[1]) > 1)
    {
        std::cout << "Please call me with: [./test_image IS_VIDEO], where:" << std::endl;
        std::cout << "    IS_VIDEO is 0 for image and 1 for video" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    // auxiliar variables
    bool image_or_video = (atoi(argv[1]) == 1);
    */

    //ProcessorBrisk test
    std::cout << std::endl << " ========= ProcessorBrisk test ===========" << std::endl << std::endl;

    unsigned int f = 0;
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
    cv::Mat last_frame;



    TimeStamp t = 1;

    Eigen::Vector4s k = {320,240,320,320};
    StateBlock* intr = new StateBlock(k,false);
    SensorCamera* sen_cam_ = new SensorCamera(new StateBlock(Eigen::Vector3s::Zero()), new StateBlock(Eigen::Vector3s::Zero()), intr,img_width,img_height);


    Problem* wolf_problem_ = new Problem(FRM_PO_3D);
    wolf_problem_->getHardwarePtr()->addSensor(sen_cam_);

    ProcessorImageParameters tracker_params;
    tracker_params.image = {img_width,  img_height};
    tracker_params.detector.threshold = 30;
    tracker_params.detector.threshold_new_features = 70;
    tracker_params.detector.octaves = 0;
    tracker_params.detector.nominal_pattern_radius = 4;
    tracker_params.descriptor.size = 512;
    tracker_params.descriptor.pattern_scale = 1.0f;
    tracker_params.descriptor.nominal_pattern_radius = 18;
    tracker_params.matcher.min_normalized_score = 0.80;
    tracker_params.matcher.similarity_norm = cv::NORM_HAMMING;
    tracker_params.matcher.roi_width = 30;
    tracker_params.matcher.roi_height = 30;
    tracker_params.active_search.grid_width = 15;
    tracker_params.active_search.grid_height = 10;
    tracker_params.active_search.separation = 1;
    tracker_params.algorithm.max_new_features = 25;
    tracker_params.algorithm.min_features_for_keyframe = 20;

    ProcessorBrisk* p_brisk = new ProcessorBrisk(tracker_params);
    sen_cam_->addProcessor(p_brisk);

    CaptureImage* capture_brisk_ptr;

    cv::namedWindow("Last");    // Creates a window for display.
    cv::moveWindow("Last", 0, 0);

    while(f<800)
    {
        f++;
        std::cout << "\n=============== Frame #: " << f << " in buffer: " << f%buffer_size << " ===============" << std::endl;

        capture >> frame[f % buffer_size];

        if(!frame[f % buffer_size].empty())
        {

            if (f>1){ // check if consecutive images are different
                Scalar diff = cv::norm(frame[f % buffer_size], last_frame, cv::NORM_L1);
                std::cout << "\nincoming data: " << (unsigned long int)(frame[f % buffer_size].data) << " last data: " << (unsigned long int)(last_frame.data) << std::endl;
                std::cout << "Image difference: " << diff << std::endl;
            }

            capture_brisk_ptr = new CaptureImage(t,sen_cam_,frame[f % buffer_size],img_width,img_height);

            //        clock_t t1 = clock();
            p_brisk->process(capture_brisk_ptr);
            //        std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;

            last_frame = frame[f % buffer_size];
        }
        else
        {
            cv::waitKey(2000);
        }
    }

    wolf_problem_->destruct();
}
