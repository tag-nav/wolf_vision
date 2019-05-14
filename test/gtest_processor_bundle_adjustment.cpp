
#include "utils_gtest.h"
#include "vision_utils.h"

#include "vision/sensor/sensor_camera.h"
#include "core/sensor/sensor_factory.h"
#include "vision/processor/processor_bundle_adjustment.h"
#include "vision/capture/capture_image.h"


using namespace wolf;

class Dummy : public ProcessorBundleAdjustment
{
	public:

		Dummy(ProcessorParamsBundleAdjustmentPtr& _params_bundle_adjustment):ProcessorBundleAdjustment(_params_bundle_adjustment)
		{
		}

		void setLast(CaptureImagePtr& _last_ptr)
		{
			last_ptr_ = _last_ptr;
			capture_image_last_ = _last_ptr;
		}
		void setInc(CaptureImagePtr& _incoming_ptr)
		{
			incoming_ptr_ = _incoming_ptr;
			capture_image_incoming_ = _incoming_ptr;
		}

		CaptureBasePtr getLast()
		{
			return last_ptr_;
		}
		CaptureBasePtr getInc()
		{
			return capture_image_incoming_;
		}

		void fillCaptureImage(CaptureImagePtr& im)
		{
		    // Detect KeyPoints
		    im->keypoints_ = det_ptr_->detect(im->getImage());
		    // Compute Descriptors
		    im->descriptors_ = des_ptr_->getDescriptor(im->getImage(), im->keypoints_);

		}
};


// Use the following in case you want to initialize tests with predefines variables or methods.
//class ProcessorBundleAdjustment_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

TEST(ProcessorBundleAdjustment, installProcessor)
{
    std::string wolf_root = _WOLF_ROOT_DIR;
    std::cout << wolf_root << std::endl;

    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);

    // Install camera
    IntrinsicsCameraPtr intr = std::make_shared<IntrinsicsCamera>(); // TODO init params or read from YAML
    intr->width  = 640;
    intr->height = 480;
    auto sens_cam = problem->installSensor("CAMERA", "camera", (Eigen::Vector7s() << 0,0,0,  0,0,0,1).finished(), intr);

    // Install processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_root + "/src/examples/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc = problem->installProcessor("BUNDLE ADJUSTMENT", "processor", sens_cam, params);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    ASSERT_EQ(proc->getProblem(), problem);
    ASSERT_TRUE(problem->check());
}

TEST(ProcessorBundleAdjustment, preProcess)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    // Create processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_root + "/src/examples/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc = std::make_shared<ProcessorBundleAdjustment>(params);

    // Fill incoming_ptr with an image
    Dummy init = Dummy(params);
	const Scalar time = 0.0;
	TimeStamp ts(time);
    CaptureImagePtr im = std::make_shared<CaptureImage>(ts, nullptr, cv::imread(wolf_root + "/src/examples/Test_gazebo_x-20cm_y-20cm.jpg"));
    init.setInc(im);
    proc->preProcess();

    ASSERT_NE(init.getLast(), nullptr);


    // Test detectNewFeatures
    FeatureBasePtrList feat_list = std::list<FeatureBasePtr>();
    unsigned int num = proc->detectNewFeatures(params->max_new_features, feat_list);

    ASSERT_EQ(num, params->max_new_features);
}

TEST(ProcessorBundleAdjustment, detectNewFeatures)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    // Create processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_root + "/src/examples/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc = std::make_shared<ProcessorBundleAdjustment>(params);

    // Fill last_ptr with an image
    Dummy init = Dummy(params);
	const Scalar time = 0.0;
	TimeStamp ts(time);
    CaptureImagePtr im = std::make_shared<CaptureImage>(ts, nullptr, cv::imread(wolf_root + "/src/examples/Test_gazebo_x-20cm_y-20cm.jpg"));
    init.setLast(im);
    std::cout << "Set last_ptr_" << std::endl;  //TODO: Seg fault
    init.fillCaptureImage(im);

    CaptureImagePtr cap = std::static_pointer_cast<CaptureImage>(init.getInc());
    ASSERT_NE(cap->keypoints_.size(), 0);


    // Test detectNewFeatures
    FeatureBasePtrList feat_list = std::list<FeatureBasePtr>();
    unsigned int num = proc->detectNewFeatures(params->max_new_features, feat_list);

    ASSERT_EQ(num, params->max_new_features);
}





int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

