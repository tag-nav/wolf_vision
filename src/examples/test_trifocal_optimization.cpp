// Testing creating wolf tree from imported .graph file

//C includes for sleep, time and main args
#include "unistd.h"

//std includes
#include <iostream>

// Vision utils
#include "vision_utils.h"

//Wolf includes
#include "../processors/processor_tracker_feature_trifocal.h"
#include "../capture_image.h"
#include "../sensor_camera.h"

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    using Eigen::Vector2s;

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr problem = Problem::create("PO 3D");

    // Install tracker (sensor and processor)
    Eigen::Vector7s cam_ext; cam_ext << 0.0,0.0,0.0, 0.0,0.0,0.0,1.0;
    std::string cam_intr_yaml = wolf_root + "/src/examples/camera_params.yaml";
    problem->installSensor("CAMERA","camera",cam_ext,cam_intr_yaml);

    std::string proc_trifocal_params_yaml = wolf_root + "/src/examples/processor_tracker_feature_trifocal.yaml";
    problem->installProcessor("TRACKER FEATURE TRIFOCAL","trifocal","camera", proc_trifocal_params_yaml);

    // Set problem PRIOR
    Scalar dt = 0.01;
    TimeStamp   t(0.0);
    Vector7s    x; x << 0.0,0.0,0.0, 0.0,0.0,0.0,1.0;
    Matrix6s    P = Matrix6s::Identity() * 0.1;
//KF1
    problem->setPrior(x, P, t, dt/2);


    // read image
    std::string img_path = wolf_root + "/src/examples/Test_ORB.png";
    cv::Mat image;
    image = cv::imread(img_path, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
    if(! image.data )                              // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image );                   // Show our image inside it.

    cv::waitKey(0);                                          // Wait for a keystroke in the window

    return 0;
}
