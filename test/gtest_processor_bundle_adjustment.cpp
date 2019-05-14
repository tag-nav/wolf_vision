
#include "utils_gtest.h"
#include "vision_utils.h"

#include "vision/sensor/sensor_camera.h"
#include "core/sensor/sensor_factory.h"
#include "vision/processor/processor_bundle_adjustment.h"
#include "vision/capture/capture_image.h"
#include "vision/internal/config.h"


using namespace wolf;

// std::string wolf_root = "/home/ovendrell/dev/vision";
std::string wolf_root = _WOLF_VISION_ROOT_DIR;

class ProcessorBundleAdjustmentDummy : public ProcessorBundleAdjustment
{
	public:

		ProcessorBundleAdjustmentDummy(ProcessorParamsBundleAdjustmentPtr& _params_bundle_adjustment):ProcessorBundleAdjustment(_params_bundle_adjustment){}

		void setLast(CaptureImagePtr _last_ptr)
		{
			last_ptr_ = _last_ptr;
			capture_image_last_ = _last_ptr;
		}
		void setInc(CaptureImagePtr _incoming_ptr)
		{
			incoming_ptr_ = _incoming_ptr;
			capture_image_incoming_ = _incoming_ptr;
		}

		CaptureImagePtr createCaptureImage(std::string _path, bool detectAndDescript = false)
		{
			const Scalar time = 0.0;
			TimeStamp ts(time);
		    CaptureImagePtr im = std::make_shared<CaptureImage>(ts, nullptr, cv::imread(_path));
		    if (detectAndDescript){
		    	// Detect KeyPoints
		    	im->keypoints_ = det_ptr_->detect(im->getImage());
		    	// Compute Descriptors
		    	im->descriptors_ = des_ptr_->getDescriptor(im->getImage(), im->keypoints_);
		    }
		    return im;
		}
};

// Use the following in case you want to initialize tests with predefines variables or methods.
//class ProcessorBundleAdjustment_class : public testing::demo{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

TEST(ProcessorBundleAdjustment, installProcessor)
{
//    std::string wolf_root = _WOLF_ROOT_DIR;
//    std::cout << wolf_root << std::endl;

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
    params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc = problem->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    ASSERT_EQ(proc->getProblem(), problem);
    ASSERT_TRUE(problem->check());
}

TEST(ProcessorBundleAdjustment, preProcess)
{
//    std::string wolf_root = _WOLF_ROOT_DIR;

    // Create processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;

    auto proc_dummy = std::make_shared<ProcessorBundleAdjustmentDummy>(params);

    // Put an image on incoming_ptr_
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg");
    proc_dummy->setInc(image_inc_ptr);
    // Put an image on last_ptr_
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", true);
    proc_dummy->setLast(image_last_ptr);
    // demo dpreProcess
    proc_dummy->preProcess();
    CaptureImagePtr inc = std::static_pointer_cast<CaptureImage>(proc_dummy->getIncoming());
    CaptureImagePtr last = std::static_pointer_cast<CaptureImage>(proc_dummy->getLast());
    ASSERT_EQ(inc->keypoints_.size(), last->keypoints_.size());
    ASSERT_NE(inc->keypoints_.size(), 0);
}

TEST(ProcessorBundleAdjustment, detectNewFeatures)
{
//    std::string wolf_root = _WOLF_ROOT_DIR;

    // Create processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc_dummy = std::make_shared<ProcessorBundleAdjustmentDummy>(params);

    FeatureBasePtrList feat_list = std::list<FeatureBasePtr>();

    // Put an image on last_ptr_
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", true);
    ASSERT_NE(image_last_ptr->keypoints_.size(), 0);
    proc_dummy->setLast(image_last_ptr);

    // demo detectNewFeatures
    unsigned int num = proc_dummy->detectNewFeatures(params->max_new_features, feat_list);
    ASSERT_EQ(num, 0);

    // Put an image on incoming_ptr_
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg");
    proc_dummy->setInc(image_inc_ptr);

    // demo detectNewFeatures
    unsigned int num2 = proc_dummy->detectNewFeatures(params->max_new_features, feat_list);
    ASSERT_EQ(num2, 0);

    // preProcess Incoming to fill last_ptr_ map_index_to_next_
    proc_dummy->preProcess();
    CaptureImagePtr last = std::static_pointer_cast<CaptureImage>(proc_dummy->getLast());
    ASSERT_NE(last->map_index_to_next_.size(),0);

    // demo detectNewFeatures
    unsigned int num3 = proc_dummy->detectNewFeatures(params->max_new_features, feat_list);
    ASSERT_EQ(num3, params->max_new_features);
}

TEST(ProcessorBundleAdjustment, trackFeatures)
{
//    std::string wolf_root = _WOLF_ROOT_DIR;

    // Create processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc_dummy = std::make_shared<ProcessorBundleAdjustmentDummy>(params);

    //fill feat_last list
    FeatureBasePtrList feat_list = std::list<FeatureBasePtr>();
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", true);
    proc_dummy->setLast(image_last_ptr);
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg");
    proc_dummy->setInc(image_inc_ptr);
    proc_dummy->preProcess();
    CaptureImagePtr last = std::static_pointer_cast<CaptureImage>(proc_dummy->getLast());
    proc_dummy->detectNewFeatures(params->max_new_features, feat_list);
    //demo trackFeatures
    FeatureBasePtrList feat_list_out = std::list<FeatureBasePtr>();
    FeatureMatchMap feat_correspondance = FeatureMatchMap();
    proc_dummy->trackFeatures(feat_list, feat_list_out, feat_correspondance);
    ASSERT_EQ(feat_list.size(), feat_list_out.size());
}

TEST(ProcessorBundleAdjustment, postProcess)
{
//    std::string wolf_root = _WOLF_ROOT_DIR;

	    // Create processor
	    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
	    params->delete_ambiguities = true;
	    params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
	    params->pixel_noise_std                = 1.0;
	    params->min_track_length_for_factor = 3;
	    params->voting_active = true;
	    params->max_new_features = 5;
	    auto proc_dummy = std::make_shared<ProcessorBundleAdjustmentDummy>(params);

	    //fill feat_last list
	    FeatureBasePtrList feat_list = std::list<FeatureBasePtr>();
	    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", true);
	    proc_dummy->setLast(image_last_ptr);
	    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg");
	    proc_dummy->setInc(image_inc_ptr);
	    proc_dummy->preProcess();
	    CaptureImagePtr last = std::static_pointer_cast<CaptureImage>(proc_dummy->getLast());
	    proc_dummy->detectNewFeatures(params->max_new_features, feat_list);
	    //demo trackFeatures
	    FeatureBasePtrList feat_list_out = std::list<FeatureBasePtr>();
	    FeatureMatchMap feat_correspondance = FeatureMatchMap();
	    proc_dummy->trackFeatures(feat_list, feat_list_out, feat_correspondance);

	    proc_dummy->postProcess();

}
/*
TEST(ProcessorBundleAdjustment,fulldemo)
{
	//    std::string wolf_root = _WOLF_ROOT_DIR;
	//    std::cout << wolf_root << std::endl;

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
	params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
	params->pixel_noise_std                = 1.0;
	params->min_track_length_for_factor = 3;
	params->voting_active = true;
	params->max_new_features = 5;
	auto proc = problem->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);

    // initialize
    Scalar dt = 0.01
    TimeStamp   t(0.0);
    Vector7s    x; x << 0,0,0, 0,0,0,1;
    Matrix6s    P = Matrix6s::Identity() * 0.000001;
    problem->setPrior(x, P, t, dt/2);             // KF1

    // Track
    cv::Mat image(intr->height, intr->width, CV_8UC3); // OpenCV cv::Mat(rows, cols)
    image *= 0; // TODO see if we want to use a real image
    SensorCameraPtr sens_trk_cam = std::static_pointer_cast<SensorCamera>(sens_cam);
    CaptureImagePtr capt_trk = make_shared<CaptureImage>(t, sens_trk_cam, image);
    proc->process(capt_trk);

    problem->print(2,0,1,0);

    for (size_t ii=0; ii<8; ii++ )
    {
        // Move
        t = t+dt;
        WOLF_INFO("----------------------- ts: ", t , " --------------------------");

        // Track
        capt_trk = make_shared<CaptureImage>(t, sens_trk_cam, image);
        proc_trk->process(capt_trk);

        problem->print(2,0,1,0);

        // Only odom creating KFs
        ASSERT_TRUE( problem->getLastKeyFrame()->getType().compare("PO 3D")==0 );
    }
}
}*/


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

