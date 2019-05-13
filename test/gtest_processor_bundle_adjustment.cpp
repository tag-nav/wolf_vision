
#include "utils_gtest.h"

#include "vision/sensor/sensor_camera.h"
#include "core/sensor/sensor_factory.h"

using namespace wolf;

// Use the following in case you want to initialize tests with predefines variables or methods.
//class ProcessorBundleAdjustment_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

TEST(ProcessorBundleAdjustment, installCamera)
{
    Eigen::VectorXs extrinsics(7); extrinsics << 0,0,0, 0,0,0,1;
    IntrinsicsCameraPtr params = std::make_shared<IntrinsicsCamera>();
    params->width  = 640;
    params->height = 480;
    params->pinhole_model_raw       << 321, 241, 321, 321;
    params->pinhole_model_rectified << 320, 240, 320, 320;
    params->distortion = Eigen::Vector3s( 0, 0, 0 );

    // Wolf problem
	ProblemPtr problem = Problem::create("PO", 3);

	// Install camera
	auto sen = problem->installSensor("CAMERA", "camera", extrinsics, params);

    ASSERT_NE(sen, nullptr);

    SensorCameraPtr cam = std::static_pointer_cast<SensorCamera>(sen);

    ASSERT_NE(cam, nullptr);
    ASSERT_EQ(cam->getImgWidth(), 640);
}

/*#include "vision/processor/processor_bundle_adjustment.h"
#include "vision/capture/capture_image.h"

#include "vision_utils.h"

TEST(ProcessorBundleAdjustment, installProcessor)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);

    // Install camera
    IntrinsicsCameraPtr intr = std::make_shared<IntrinsicsCamera>(); // TODO init params or read from YAML
    intr->width  = 640;
    intr->height = 480;
    auto sens_cam = problem->installSensor("CAMERA", "camera", (Eigen::Vector7s() << 0,0,0,  0,0,0,1).finished(), intr);

    // Install processor
//    ProcessorParamsBundleAdjustmentPtr params = make_shared<ProcessorParamsBundleAdjustment>();
//    params->delete_ambiguities = true;
//    params->yaml_file_params_vision_utils = wolf_root + "/src/examples/processor_bundle_adjustment_vision_utils.yaml";
//    params->pixel_noise_std                = 1.0;
//    params->min_track_length_for_factor = 3;
//    params->voting_active = true;
//    params->max_new_features = 5;
//    auto proc = problem->installProcessor("BUNDLE ADJUSTMENT", "processor", sens_cam, params);
//
//    std::cout << "sensor & processor created and added to wolf problem" << std::endl;
//
//    ASSERT_EQ(proc->getProblem(), problem);
//    ASSERT_TRUE(problem->check());
}*/

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

