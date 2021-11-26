//--------LICENSE_START--------
//
// Copyright (C) 2020,2021 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------

#include "vision/processor/processor_tracker_feature_trifocal.h"
#include "vision/capture/capture_image.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/internal/config.h"

#include <core/utils/utils_gtest.h>
#include "core/common/wolf.h"
#include "core/utils/logging.h"
#include "core/processor/processor_odom_3d.h"

#include "vision_utils/vision_utils.h"

using namespace Eigen;
using namespace wolf;

// Use the following in case you want to initialize tests with predefines variables or methods.
//class ProcessorTrackerFeatureTrifocal_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

//TEST(ProcessorTrackerFeatureTrifocal, Constructor)
//{
//  std::cout << "\033[1;33m [WARN]:\033[0m gtest for ProcessorTrackerFeatureTrifocal Constructor is empty." << std::endl;
//}

//TEST(ProcessorTrackerFeatureTrifocal, Destructor)
//{
//  std::cout << "\033[1;33m [WARN]:\033[0m gtest for ProcessorTrackerFeatureTrifocal Destructor is empty." << std::endl;
//}

////[Class methods]
//TEST(ProcessorTrackerFeatureTrifocal, trackFeatures)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal trackFeatures is empty." << std::endl;
//}
//
//TEST(ProcessorTrackerFeatureTrifocal, correctFeatureDrift)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal correctFeatureDrift is empty." << std::endl;
//}
//
//TEST(ProcessorTrackerFeatureTrifocal, voteForKeyFrame)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal voteForKeyFrame is empty." << std::endl;
//}
//
//TEST(ProcessorTrackerFeatureTrifocal, detectNewFeatures)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal detectNewFeatures is empty." << std::endl;
//}
//
//TEST(ProcessorTrackerFeatureTrifocal, createFactor)
//{
//  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal createFactor is empty." << std::endl;
//}

TEST(ProcessorTrackerFeatureTrifocal, KeyFrameCallback)
{

    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    using Eigen::Vector2d;

    std::string wolf_root = _WOLF_VISION_ROOT_DIR;

    double dt = 0.01;

    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);

    // Install tracker (sensor and processor)
    ParamsSensorCameraPtr intr = make_shared<ParamsSensorCamera>(); // TODO init params or read from YAML
    intr->width  = 640;
    intr->height = 480;

    auto sens_trk = SensorBase::emplace<SensorCamera>(problem->getHardware(), (Eigen::Vector7d()<<0,0,0, 0,0,0,1).finished(),
                                                      intr);
    ParamsProcessorTrackerFeatureTrifocalPtr params_tracker_feature_trifocal = std::make_shared<ParamsProcessorTrackerFeatureTrifocal>();
    params_tracker_feature_trifocal->pixel_noise_std                = 1.0;
    params_tracker_feature_trifocal->voting_active                  = true;
    params_tracker_feature_trifocal->min_features_for_keyframe      = 5;
    params_tracker_feature_trifocal->time_tolerance                 = dt/2;
    params_tracker_feature_trifocal->min_features_for_keyframe      = 5;
    params_tracker_feature_trifocal->pixel_noise_std                = 1.0;
    params_tracker_feature_trifocal->max_new_features               = 5;
    params_tracker_feature_trifocal->min_response_new_feature       = 25;
    params_tracker_feature_trifocal->n_cells_h                      = 10;
    params_tracker_feature_trifocal->n_cells_v                      = 10;
    params_tracker_feature_trifocal->yaml_file_params_vision_utils  = wolf_root + "/demos/processor_tracker_feature_trifocal_vision_utils.yaml";
    params_tracker_feature_trifocal->debug_view                     = false;

    // ProcessorTrackerFeatureTrifocalPtr proc_trk = make_shared<ProcessorTrackerFeatureTrifocal>(params_tracker_feature_trifocal);
    auto proc_trk = std::static_pointer_cast<ProcessorTrackerFeatureTrifocal>(ProcessorBase::emplace<ProcessorTrackerFeatureTrifocal>(sens_trk, params_tracker_feature_trifocal));
    proc_trk->configure(sens_trk);

    // problem->addSensor(sens_trk);
    // sens_trk->addProcessor(proc_trk);

    // Install odometer (sensor and processor)
    ParamsSensorOdom3dPtr params = std::make_shared<ParamsSensorOdom3d>();
    params->min_disp_var = 0.000001;
    params->min_rot_var  = 0.000001;
    SensorBasePtr sens_odo = problem->installSensor("SensorOdom3d", "odometer", (Vector7d() << 0,0,0,  0,0,0,1).finished(), params);
    ParamsProcessorOdom3dPtr proc_odo_params = make_shared<ParamsProcessorOdom3d>();
    proc_odo_params->voting_active   = true;
    proc_odo_params->time_tolerance  = dt/2;
    proc_odo_params->max_buff_length = 3;
    ProcessorBasePtr proc_odo = problem->installProcessor("ProcessorOdom3d", "odometer", sens_odo, proc_odo_params);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    // Sequence to test KeyFrame creations (callback calls)

    // initialize
    TimeStamp   t(0.0);
    VectorComposite x("PO", {Vector3d::Zero(), Quaterniond::Identity().coeffs()});
    VectorComposite s("PO", {1e-3*Vector3d::Ones(), 1e-3*Vector3d::Ones()});
    auto KF1 = problem->setPriorFactor(x, s, t, dt/2);             // KF1
    std::static_pointer_cast<ProcessorOdom3d>(proc_odo)->setOrigin(KF1);

    MatrixXd P = (s.vector("PO").array() * s.vector("PO").array()).matrix().asDiagonal();
    CaptureOdom3dPtr capt_odo = make_shared<CaptureOdom3d>(t, sens_odo, Vector6d::Zero(), P);

    // Track
    cv::Mat image(intr->height, intr->width, CV_8UC3); // OpenCV cv::Mat(rows, cols)
    image *= 0; // TODO see if we want to use a real image
    SensorCameraPtr sens_trk_cam = std::static_pointer_cast<SensorCamera>(sens_trk);
    CaptureImagePtr capt_trk = make_shared<CaptureImage>(t, sens_trk_cam, image);
    sens_trk_cam->process(capt_trk);

    problem->print(2,0,1,0);

    for (size_t ii=0; ii<8; ii++ )
    {
        // Move
        t = t+dt;
        WOLF_INFO("----------------------- ts: ", t , " --------------------------");

        capt_odo->setTimeStamp(t);
        sens_odo->process(capt_odo);

        // Track
        capt_trk = make_shared<CaptureImage>(t, sens_trk_cam, image);
        sens_trk_cam->process(capt_trk);

        problem->print(2,0,1,0);

        // Only odom creating KFs
//        ASSERT_TRUE( problem->getLastKeyFrame()->getType().compare("PO 3d")==0 );
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

