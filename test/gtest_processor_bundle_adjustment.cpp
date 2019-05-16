
#include "utils_gtest.h"
#include "vision_utils.h"

#include "vision/sensor/sensor_camera.h"
#include "core/sensor/sensor_factory.h"
#include "vision/processor/processor_bundle_adjustment.h"
#include "vision/capture/capture_image.h"
#include "vision/internal/config.h"

// Vision utils includes
#include <vision_utils.h>
#include <sensors.h>
#include <common_class/buffer.h>
#include <common_class/frame.h>

using namespace wolf;

//std::string wolf_root = "/home/ovendrell/dev/vision";
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

		std::map<size_t, LandmarkBasePtr> getLandmarkMap()
		{
			return lmk_track_map_;
		}

		unsigned int getProcessKnown()
		{
			return this->processKnown();
		}

		unsigned int getProcessNew(const int& _max_new_features)
		{
			return this->processNew(_max_new_features);
		}

		TrackMatrix getTrackMatrix()
		{
			return track_matrix_;
		}

		FeatureBasePtrList getKnownFeaturesIncoming()
		{
			return known_features_incoming_;
		}
		FeatureMatchMap getMatchesLastFromIncoming()
		{
			return matches_last_from_incoming_;
		}

		CaptureImagePtr createCaptureImage(std::string _path, SensorCameraPtr _sensor, bool detectAndDescript = false)
		{
			const Scalar time = 0.0;
			TimeStamp ts(time);
		    CaptureImagePtr im = std::make_shared<CaptureImage>(ts, _sensor, cv::imread(_path));
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
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr);
    proc_dummy->setInc(image_inc_ptr);
    // Put an image on last_ptr_
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr, true);
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
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr, true);
    ASSERT_NE(image_last_ptr->keypoints_.size(), 0);
    proc_dummy->setLast(image_last_ptr);

    // demo detectNewFeatures
    unsigned int num = proc_dummy->detectNewFeatures(params->max_new_features, feat_list);
    ASSERT_EQ(num, 0);

    // Put an image on incoming_ptr_
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr);
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
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr, true);
    proc_dummy->setLast(image_last_ptr);
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr);
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


TEST(ProcessorBundleAdjustment, establishFactors)
{
	std::cout << "EMPTY Test\n";
}

TEST(ProcessorBundleAdjustment, createLandmark)
{
	std::cout << "EMPTY Test\n";
}


TEST(ProcessorBundleAdjustment, process)
{
    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);

    // Install camera
    IntrinsicsCameraPtr intr = std::make_shared<IntrinsicsCamera>(); // TODO init params or read from YAML
    intr->width  = 640;
    intr->height = 480;
    SensorCameraPtr sens_cam = std::static_pointer_cast<SensorCamera>(problem->installSensor("CAMERA", "camera", (Eigen::Vector7s() << 0,0,0,  0,0,0,1).finished(), intr));

    // Install processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc = problem->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);
	auto proc_dummy = std::static_pointer_cast<ProcessorBundleAdjustmentDummy>(proc);

	//1ST TIME
	CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", sens_cam);
	proc_dummy->process(image_inc_ptr);
	ASSERT_EQ(proc_dummy->getTrackMatrix().numTracks(), 0);
	ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 0);

	//2ND TIME
	CaptureImagePtr image_inc_ptr2 = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", sens_cam);
//	proc_dummy->setInc(image_inc_ptr2);
//	proc_dummy->preProcess();
//	proc_dummy->getProcessKnown();
//	proc_dummy->getProcessNew(params->max_new_features);
//	proc_dummy->establishFactors();
	proc_dummy->process(image_inc_ptr2);
	ASSERT_EQ(problem->getMap()->getLandmarkList().size(), params->max_new_features);
	ASSERT_EQ(proc_dummy->getTrackMatrix().numTracks(), params->max_new_features);

	//3RD TIME -- RUNNING
	CaptureImagePtr image_inc_ptr3 = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", sens_cam);
	assert(proc_dummy->getOrigin()!=nullptr);
	assert(proc_dummy->getLast()!= nullptr && proc_dummy->getLast()!=proc_dummy->getOrigin());
	proc_dummy->process(image_inc_ptr3);
	ASSERT_EQ(problem->getMap()->getLandmarkList().size(), params->max_new_features);
	ASSERT_EQ(proc_dummy->getTrackMatrix().numTracks(), params->max_new_features);

	//4TH TIME -- RUNNING
	CaptureImagePtr image_inc_ptr4 = proc_dummy->createCaptureImage(wolf_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", sens_cam);
	proc_dummy->process(image_inc_ptr4);
	ASSERT_EQ(image_inc_ptr4->getFeatureList().size(), params->max_new_features);
}

TEST(ProcessorBundleAdjustment, processVideo)
{
    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);

    // Install camera
    IntrinsicsCameraPtr intr = std::make_shared<IntrinsicsCamera>(); // TODO init params or read from YAML
    intr->width  = 640;
    intr->height = 480;
    SensorCameraPtr sens_cam = std::static_pointer_cast<SensorCamera>(problem->installSensor("CAMERA", "camera", (Eigen::Vector7s() << 0,0,0,  0,0,0,1).finished(), intr));

    // Install processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = false;
    params->yaml_file_params_vision_utils = wolf_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 400;
    params->min_features_for_keyframe = 0;
    auto proc = problem->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);
	auto proc_dummy = std::static_pointer_cast<ProcessorBundleAdjustmentDummy>(proc);

	//==================================vision_utils ============================================
	vision_utils::SensorCameraPtr sen_ptr = std::make_shared<vision_utils::SensorCamera>();
	sen_ptr->open("/home/ovendrell/eclipse-workspace/ddm_triad/data/video/VID4b.mp4");
				//"/home/ovendrell/dev/vision_utils/src/test/data/test_usb_cam.mp4");

    unsigned int buffer_size = 10;
    vision_utils::Buffer<vision_utils::FramePtr> frame_buff(buffer_size);
    frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), 0) );

    unsigned int img_width  = frame_buff.back()->getImage().cols;
    unsigned int img_height = frame_buff.back()->getImage().rows;
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;
    //==================================vision_utils ============================================

    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sens_cam);
    camera->setImgWidth(img_width);
    camera->setImgHeight(img_height);
    ProcessorBundleAdjustmentPtr proc_bundle_adj = std::static_pointer_cast<ProcessorBundleAdjustment>(proc);

    TimeStamp t = 0;
    // main loop
    Scalar dt = 0.04;
    for(int frame_count = 0; frame_count<150; ++frame_count)
    {
    	t += dt;
    	std::cout << "Avis 0" << std::endl;
        // Image ---------------------------------------------
        frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), frame_count) );
        std::cout << "Avis 1" << std::endl;
        CaptureImagePtr image = std::make_shared<CaptureImage>(t, camera, frame_buff.back()->getImage());
        std::cout << "Avis 2" << std::endl;
        /* process */
        camera->process(image);
        std::cout << "FEATURES:" << image->getFeatureList().size() << std::endl;
        std::cout << "MATCHES:" << image->matches_from_precedent_.size() << std::endl;

    }
}


TEST(ProcessorBundleAdjustment, processKnown)
{
	std::cout << "EMPTY Test\n";
}

TEST(ProcessorBundleAdjustment, processNew)
{
	std::cout << "EMPTY Test\n";
}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

