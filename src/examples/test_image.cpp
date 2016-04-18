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

    //Welcome message
    std::cout << std::endl << " ========= WOLF IMAGE test ===========" << std::endl << std::endl;


    std::cout << "0" << std::endl;
    //std::vector<float> points3D = {2,3,5};
    cv::Point3f points3D;
    points3D.x = 2.0;
    points3D.y = 5.0;
    points3D.z = 6.0;
    std::vector<cv::Point3f> point_in_3D;
    point_in_3D.push_back(points3D);
    points3D.x = 4.0;
    points3D.y = 2.0;
    points3D.z = 1.0;
    point_in_3D.push_back(points3D);

    std::cout << "1" << std::endl;
    std::vector<float> rot_mat = {0,0,0};
    std::vector<float> trans_mat = {1,1,1};

    std::cout << "2" << std::endl;
    cv::Mat cam_mat(3,3,CV_32F);
    cam_mat.row(0).col(0).setTo(1);
    cam_mat.row(0).col(1).setTo(0);
    cam_mat.row(0).col(2).setTo(2);
    cam_mat.row(1).col(0).setTo(0);
    cam_mat.row(1).col(1).setTo(1);
    cam_mat.row(1).col(2).setTo(2);
    cam_mat.row(2).col(0).setTo(0);
    cam_mat.row(2).col(1).setTo(0);
    cam_mat.row(2).col(2).setTo(1);

    std::cout << "cam_mat[1,2]: " << cam_mat.row(1).col(0) << std::endl;

    std::cout << "3" << std::endl;
    std::vector<float> dist_coef = {0,0,0,0,0};
    //std::vector<float> points2D;
    std::vector<cv::Point2f> points2D;

    std::cout << "4" << std::endl;
    cv::projectPoints(point_in_3D,rot_mat,trans_mat,cam_mat,dist_coef,points2D);
    std::cout << "5" << std::endl;

    for (auto it : points2D)
    {
        std::cout << "points2D- X: " << it.x << "; Y: " << it.y << std::endl;
    }

    //ProcessorBrisk test
    std::cout << std::endl << " ========= ProcessorBrisk test ===========" << std::endl << std::endl;

    unsigned int img_width;
    unsigned int img_height;

    unsigned int f = 0;
    const char * filename;
    if (argc == 1)
    {
        filename = "/home/jtarraso/Vídeos/House interior.mp4";
        img_width = 640;
        img_height = 360;
    }
    else
    {
        if (std::string(argv[1]) == "0")
        {
            // camera
            //img_width = 640;
            //img_height = 480;
            filename = "/home/jtarraso/Escritorio/Test Brisk 1 - 40 40 - 1 punto.jpg";
            img_width = 640;
            img_height = 360;
        }
        else
        {
            filename = argv[1];
            img_width = 640;
            img_height = 360;
        }
    }
    std::cout << "Input video file: " << filename << std::endl;

    TimeStamp t = 1;

    Eigen::Vector4s k = {320,240,320,320};
    StateBlock* intr = new StateBlock(k,false);
    SensorCamera* sen_cam_ = new SensorCamera(new StateBlock(Eigen::Vector3s::Zero()), new StateBlock(Eigen::Vector3s::Zero()), intr,img_width,img_height);


    WolfProblem* wolf_problem_ = new WolfProblem(FRM_PO_3D);
    wolf_problem_->getHardwarePtr()->addSensor(sen_cam_);

//    ImageTrackerParameters tracker_params;
//    tracker_params.image = {img_width,  img_height};
//    tracker_params.detector = {30, 0, 1.0f, 5};
//    tracker_params.descriptor = {512, 5};
//    tracker_params.matcher.max_similarity_distance = 0.3;
//    tracker_params.active_search = {8, 8};
//    tracker_params.algorithm.max_new_features = 20;
//    tracker_params.algorithm.min_features_th = 40;

    ProcessorBrisk* p_brisk = new ProcessorBrisk(img_height,img_width,9,9,4,20,30,30,0,1.0f,10);
    sen_cam_->addProcessor(p_brisk);

    cv::VideoCapture capture(filename);
    if(!capture.isOpened())  // check if we succeeded
    {
        std::cout << "failed" << std::endl;
    }
    else
    {
        std::cout << "succeded" << std::endl;
    }

    cv::Mat frame;

    capture.set(CV_CAP_PROP_POS_MSEC, 3000);

    CaptureImage* capture_brisk_ptr;

    cv::Mat last_frame;

    cv::namedWindow("Last");    // Creates a window for display.
    cv::moveWindow("Last", 0, 0);
    cv::namedWindow("Incoming");    // Creates a window for display.
    cv::moveWindow("Incoming", 0, 500);

    while(f<800)
    {
        f++;
        std::cout << "Frame #: " << f << std::endl;

        capture >> frame;

        if(!frame.empty())
        {

        if (f>1){ // check if consecutive images are different
            WolfScalar diff = cv::norm(frame, last_frame, cv::NORM_L1);
            std::cout << "test_image: Image increment: " << diff << std::endl;
        }

        capture_brisk_ptr = new CaptureImage(t,sen_cam_,frame,img_width,img_height);

//        clock_t t1 = clock();
        p_brisk->process(capture_brisk_ptr);
//        std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;

        last_frame = frame.clone();
        }
        else
        {
            cv::waitKey(2000);
        }
    }

    wolf_problem_->destruct();
}
