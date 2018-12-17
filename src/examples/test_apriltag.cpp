/**
 * \file test_apriltag.cpp
 *
 *  Created on: Dec 14, 2018
 *      \author: Dinesh Atchtuhan
 */

//std
#include <iostream>
#include <stdlib.h>

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

//Wolf
#include "wolf.h"
#include "ceres_wrapper/ceres_manager.h"
#include "problem.h"
#include "sensor_camera.h"
#include "state_block.h"
#include "processors/processor_tracker_landmark_apriltag.h"
#include "capture_image.h"

//#define IMAGE_OUTPUT

bool str_ends_with (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char *argv[])
{
    /*
     * HOW TO USE ?
     * For now, just call the executable and append the list of images to be processed
     * For example, if you want to process only one image located at wolf/bin/images/frame1.jpg
     * and if wolf_root is correctly set. then just run (from wolf/bin)
     * ./test_apriltag /bin/images/frame1.jpg
     */

    using namespace wolf;

#ifdef IMAGE_OUTPUT
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
#endif

    WOLF_INFO( "==================== processor apriltag test ======================" )

    std::string wolf_root = _WOLF_ROOT_DIR;
    // Wolf problem
    ProblemPtr problem = Problem::create("PO 3D");
    SensorBasePtr sen = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_apriltag_params.yaml");
    SensorCameraPtr sen_cam = std::static_pointer_cast<SensorCamera>(sen);
    ProcessorBasePtr prc = problem->installProcessor("TRACKER LANDMARK APRILTAG", "apriltags", "camera", wolf_root + "/src/examples/processor_tracker_landmark_apriltag.yaml");
    // set prior
    FrameBasePtr F1 = problem->setPrior((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 0.0, 0.1);
    ceres::Solver::Options options;
    CeresManagerPtr ceres_manager = std::make_shared<CeresManager>(problem, options);

    WOLF_INFO( "====================       Problem set      ======================" )

    // first argument is the name of the program.
    // following arguments are path to image (from wolf_root)
    const int inputs = argc -1;
    WOLF_DEBUG("nb of images: ", inputs);
    cv::Mat frame;
    Scalar ts(0);

    WOLF_INFO( "====================        Main loop       ======================" )
    Scalar dt = 1;
    for (int input = 1; input <= inputs; input++) {
        ts += dt;
        std::string path = wolf_root + argv[input];
        WOLF_DEBUG("path to image ", path);
        frame = cv::imread(path, CV_LOAD_IMAGE_COLOR);

        if( frame.data ) //if imread succeeded
        {
#ifdef IMAGE_OUTPUT
            sleep(500); //wait for 0.5 s
            imshow( "Display window", frame );  // display original image.
#endif
            CaptureImagePtr cap = std::make_shared<CaptureImage>(ts, sen_cam, frame);
            WOLF_DEBUG("Processing image...");
            sen->process(cap);
            WOLF_DEBUG("Image processed...");
        }
        else
            WOLF_WARN("could not load image ", path)
    }

    problem->print(4,1,1,1);
    WOLF_INFO( "====================    Solving problem    ======================" )
    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
    problem->print(4,1,1,1);

    return 0;

}
