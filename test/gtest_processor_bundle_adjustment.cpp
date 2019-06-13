
// this plugin includes
#include "vision/sensor/sensor_camera.h"
#include "vision/processor/processor_bundle_adjustment.h"
#include "vision/factor/factor_pixelHP.h"
#include "vision/landmark/landmark_HP.h"
#include "vision/capture/capture_image.h"
#include "vision/internal/config.h"


// wolf includes
#include <core/utils/utils_gtest.h>
#include <core/sensor/sensor_factory.h>

// Vision utils includes
#include <vision_utils/vision_utils.h>
#include <vision_utils/sensors.h>
#include <vision_utils/common_class/buffer.h>
#include <vision_utils/common_class/frame.h>

using namespace wolf;

std::string wolf_vision_root = _WOLF_VISION_ROOT_DIR;

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
		        // Create and fill incoming grid
		        im->grid_features_ = std::make_shared<vision_utils::FeatureIdxGrid>(im->getImage().rows, im->getImage().cols, params_bundle_adjustment_->n_cells_v, params_bundle_adjustment_->n_cells_h);
		        im->grid_features_->insert(im->keypoints_);
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
    params->yaml_file_params_vision_utils = wolf_vision_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc = problem->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    ASSERT_EQ(proc->getProblem(), problem);
    ASSERT_TRUE(problem->check(0));
}

TEST(ProcessorBundleAdjustment, preProcess)
{
    // Create processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_vision_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    params->n_cells_h = 50;
    params-> n_cells_v = 50;
    params->min_response_new_feature = 50;

    auto proc_dummy = std::make_shared<ProcessorBundleAdjustmentDummy>(params);

    // Put an image on incoming_ptr_
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr);
    proc_dummy->setInc(image_inc_ptr);
    // Put an image on last_ptr_
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr, true);
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
    params->yaml_file_params_vision_utils = wolf_vision_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    params->n_cells_h = 20;
    params-> n_cells_v = 15;
    params->min_response_new_feature = 0;
    auto proc_dummy = std::make_shared<ProcessorBundleAdjustmentDummy>(params);

    FeatureBasePtrList feat_list = std::list<FeatureBasePtr>();

    // Put an image on last_ptr_
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr, true);
    ASSERT_NE(image_last_ptr->keypoints_.size(), 0);
    proc_dummy->setLast(image_last_ptr);

    // demo detectNewFeatures
    unsigned int num = proc_dummy->detectNewFeatures(params->max_new_features, image_last_ptr, feat_list);
    ASSERT_EQ(num, 0);

    // Put an image on incoming_ptr_
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr);
    proc_dummy->setInc(image_inc_ptr);

    // demo detectNewFeatures
    unsigned int num2 = proc_dummy->detectNewFeatures(params->max_new_features, image_last_ptr, feat_list);
    ASSERT_EQ(num2, 0);

    // preProcess Incoming to fill last_ptr_ map_index_to_next_
    proc_dummy->preProcess();
    CaptureImagePtr last = std::static_pointer_cast<CaptureImage>(proc_dummy->getLast());
    ASSERT_NE(last->map_index_to_next_.size(),0);

    // demo detectNewFeatures
    unsigned int num3 = proc_dummy->detectNewFeatures(params->max_new_features, image_last_ptr, feat_list);
    ASSERT_LE(num3, params->max_new_features);
}

TEST(ProcessorBundleAdjustment, trackFeatures)
{
    // Create processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_vision_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    params->n_cells_h = 50;
    params-> n_cells_v = 50;
    params->min_response_new_feature = 50;
    auto proc_dummy = std::make_shared<ProcessorBundleAdjustmentDummy>(params);

    //fill feat_last list
    FeatureBasePtrList feat_list = std::list<FeatureBasePtr>();
    CaptureImagePtr image_last_ptr = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr, true);
    proc_dummy->setLast(image_last_ptr);
    CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", nullptr);
    proc_dummy->setInc(image_inc_ptr);
    proc_dummy->preProcess();
    CaptureImagePtr last = std::static_pointer_cast<CaptureImage>(proc_dummy->getLast());
    proc_dummy->detectNewFeatures(params->max_new_features, last, feat_list);
    //demo trackFeatures
    FeatureBasePtrList feat_list_out = std::list<FeatureBasePtr>();
    FeatureMatchMap feat_correspondance = FeatureMatchMap();
    proc_dummy->trackFeatures(feat_list, image_inc_ptr, feat_list_out, feat_correspondance);
    ASSERT_EQ(feat_list.size(), feat_list_out.size());
}


TEST(ProcessorBundleAdjustment, establishFactors)
{
	std::cout << "EMPTY Test\n";
}

TEST(ProcessorBundleAdjustment, emplaceLandmark)
{
	//Build problem
	ProblemPtr problem_ptr = Problem::create("PO", 3);

    // Install sensor
    IntrinsicsCameraPtr intr = std::make_shared<IntrinsicsCamera>();
    intr->width  = 640;
    intr->height = 480;
    auto sens_cam = problem_ptr->installSensor("CAMERA", "camera", (Eigen::Vector7s() << 0,0,0,  0,0,0,1).finished(), intr);
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sens_cam);
    // Install processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_vision_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    auto proc = problem_ptr->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);
    ProcessorBundleAdjustmentPtr proc_bundle_adj = std::static_pointer_cast<ProcessorBundleAdjustment>(proc);

    //Frame
	FrameBasePtr frm0 = problem_ptr->emplaceFrame(KEY, problem_ptr->zeroState(), TimeStamp(0));

	// Capture, feature and factor
	auto cap0 = std::static_pointer_cast<CaptureImage>(CaptureImage::emplace<CaptureImage>(frm0, TimeStamp(0), camera, cv::Mat::zeros(480,640, 1)));

	cv::Point2f p = cv::Point2f(240, 320);
	cv::KeyPoint kp = cv::KeyPoint(p, 32.0f);
	cv::Mat des = cv::Mat::zeros(1,8, CV_8U);

	FeaturePointImagePtr fea0 = std::make_shared<FeaturePointImage>(kp, 0, des, Eigen::Matrix2s::Identity()* pow(1, 2));
	fea0->link(cap0);

	ASSERT_TRUE(problem_ptr->check(0));

	LandmarkBasePtr lmk = proc_bundle_adj->emplaceLandmark(fea0);
	LandmarkHPPtr lmk_hp = std::static_pointer_cast<LandmarkHP>(lmk);

	ASSERT_EQ(problem_ptr->getMap()->getLandmarkList().size(), 1);
}


TEST(ProcessorBundleAdjustment, process)
{
    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);

    // Install camera
    IntrinsicsCameraPtr intr = std::make_shared<IntrinsicsCamera>(); // TODO init params or read from YAML
    intr->pinhole_model_raw = Eigen::Vector4s(0,0,1,1);  //TODO: initialize
    intr->width  = 640;
    intr->height = 480;
    SensorCameraPtr sens_cam = std::static_pointer_cast<SensorCamera>(problem->installSensor("CAMERA", "camera", (Eigen::Vector7s() << 0,0,0,  0,0,0,1).finished(), intr));

    // Install processor
    ProcessorParamsBundleAdjustmentPtr params = std::make_shared<ProcessorParamsBundleAdjustment>();
    params->delete_ambiguities = true;
    params->yaml_file_params_vision_utils = wolf_vision_root + "/demos/processor_bundle_adjustment_vision_utils.yaml";
    params->pixel_noise_std                = 1.0;
    params->min_track_length_for_factor = 3;
    params->voting_active = true;
    params->max_new_features = 5;
    params->n_cells_h = 50;
    params-> n_cells_v = 50;
    params->min_response_new_feature = 50;
    auto proc = problem->installProcessor("TRACKER BUNDLE ADJUSTMENT", "processor", sens_cam, params);
	auto proc_dummy = std::static_pointer_cast<ProcessorBundleAdjustmentDummy>(proc);

	//1ST TIME
	CaptureImagePtr image_inc_ptr = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", sens_cam);
	proc_dummy->process(image_inc_ptr);
	ASSERT_EQ(proc_dummy->getTrackMatrix().numTracks(), 0);
	ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 0);

	//2ND TIME
	CaptureImagePtr image_inc_ptr2 = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", sens_cam);
//	proc_dummy->setInc(image_inc_ptr2);
//	proc_dummy->preProcess();
//	proc_dummy->getProcessKnown();
//	proc_dummy->getProcessNew(params->max_new_features);
//	proc_dummy->establishFactors();
	proc_dummy->process(image_inc_ptr2);
	ASSERT_LE(proc_dummy->getTrackMatrix().numTracks(), params->max_new_features);
	ASSERT_NE(problem->getMap(), nullptr);
	ASSERT_EQ(problem->getMap()->getLandmarkList().empty(), true);


	//3RD TIME -- RUNNING
	CaptureImagePtr image_inc_ptr3 = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", sens_cam);
	assert(proc_dummy->getOrigin()!=nullptr);
	assert(proc_dummy->getLast()!= nullptr && proc_dummy->getLast()!=proc_dummy->getOrigin());
	proc_dummy->process(image_inc_ptr3);
	ASSERT_LE(proc_dummy->getTrackMatrix().numTracks(), params->max_new_features);

	//4TH TIME -- RUNNING
	CaptureImagePtr image_inc_ptr4 = proc_dummy->createCaptureImage(wolf_vision_root + "/demos/demo_gazebo_x00cm_y00cm.jpg", sens_cam);
	proc_dummy->process(image_inc_ptr4);
	ASSERT_LE(image_inc_ptr4->getFeatureList().size(), params->max_new_features);

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

