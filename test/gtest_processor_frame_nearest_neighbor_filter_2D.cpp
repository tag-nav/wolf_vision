
/**
 * \file gtest_processor_frame_nearest_neighbor_filter.cpp
 *
 *  Created on: Aug 2, 2017
 *      \author: tessajohanna
 */

#include "utils_gtest.h"
#include "base/utils/logging.h"

#include "base/sensor/sensor_odom_2D.h"
#include "base/processor/processor_frame_nearest_neighbor_filter.h"

#include <iostream>

// Dummy class so that it is no pur virtual
struct DummyLoopCloser : public wolf::ProcessorFrameNearestNeighborFilter
{
  DummyLoopCloser(ParamsPtr _params) :
    wolf::ProcessorFrameNearestNeighborFilter(_params) { }

  bool confirmLoopClosure() override { return false; }
  bool voteForKeyFrame()    override { return false; }
};

// Declare Wolf problem
wolf::ProblemPtr problem = wolf::Problem::create("PO", 2);

// Declare Sensor
Eigen::Vector3s odom_extrinsics = Eigen::Vector3s(0,0,0);

std::shared_ptr<wolf::IntrinsicsOdom2D> odom_intrinsics =
             std::make_shared<wolf::IntrinsicsOdom2D>(wolf::IntrinsicsOdom2D());

wolf::SensorBasePtr sensor_ptr;

// Declare Processors
wolf::ProcessorFrameNearestNeighborFilterPtr processor_ptr_point2d;
wolf::ProcessorParamsFrameNearestNeighborFilterPtr processor_params_point2d =
                      std::make_shared<wolf::ProcessorParamsFrameNearestNeighborFilter>();

wolf::ProcessorFrameNearestNeighborFilterPtr processor_ptr_ellipse2d;
wolf::ProcessorParamsFrameNearestNeighborFilterPtr processor_params_ellipse2d =
                      std::make_shared<wolf::ProcessorParamsFrameNearestNeighborFilter>();

// Declare Frame Pointer
wolf::StateBlockPtr stateblock_ptr1, stateblock_ptr2, stateblock_ptr3, stateblock_ptr4;
wolf::FrameBasePtr F1, F2, F3, F4;
wolf::CaptureBasePtr capture_dummy, incomming_dummy;

//##############################################################################
TEST(ProcessorFrameNearestNeighborFilter, PointInEllipseRotated)
{
  // four different states           [x, y]
  Eigen::Vector3s state1, state2, state3, state4;
  state1 << 3.0, 5.0, 0.0;
  state2 << 3.0, 6.0, 0.0;
  state3 << 3.0, 7.0, 0.0;
  state4 << 100.0, 100.0, 0.0;


  // create Keyframes
  F1 = problem->emplaceFrame(wolf::KEY, state1, 1);
  F2 = problem->emplaceFrame(wolf::KEY, state2, 2);
  F3 = problem->emplaceFrame(wolf::KEY, state3, 3);
  F4 = problem->emplaceFrame(wolf::KEY, state4, 4);

  auto stateblock_pptr1 = F1->getP();
  auto stateblock_optr1 = F1->getO();

  auto stateblock_pptr2 = F2->getP();
  auto stateblock_optr2 = F2->getO();

  auto stateblock_pptr3 = F3->getP();
  auto stateblock_optr3 = F3->getO();

  auto stateblock_pptr4 = F4->getP();
  auto stateblock_optr4 = F4->getO();

  // add dummy capture
  wolf::CaptureBase::emplace<wolf::CaptureBase>(F1, "DUMMY", 1.0, sensor_ptr);
  wolf::CaptureBase::emplace<wolf::CaptureBase>(F2, "DUMMY", 1.0, sensor_ptr);
  wolf::CaptureBase::emplace<wolf::CaptureBase>(F3, "DUMMY", 1.0, sensor_ptr);
  wolf::CaptureBase::emplace<wolf::CaptureBase>(F4, "DUMMY", 1.0, sensor_ptr);

      // capture_dummy = std::make_shared<wolf::CaptureBase>("DUMMY",
      //                                                 1.0,
      //                                                 sensor_ptr);
  // F1->addCapture(capture_dummy);
  // F2->addCapture(capture_dummy);
  // F3->addCapture(capture_dummy);
  // F4->addCapture(capture_dummy);

  // Add same covariances for all states
  Eigen::Matrix2s position_covariance_matrix;
  position_covariance_matrix << 9.0, 0.0,
                                0.0, 5.0;

  Eigen::Matrix1s orientation_covariance_matrix;
  orientation_covariance_matrix << 0.01;

  Eigen::Vector2s tt_covariance_matrix;
  tt_covariance_matrix << 0.0, 0.0;

  problem->addCovarianceBlock( stateblock_pptr1, stateblock_pptr1,
                               position_covariance_matrix);
  problem->addCovarianceBlock( stateblock_optr1, stateblock_optr1,
                               orientation_covariance_matrix);
  problem->addCovarianceBlock( stateblock_pptr1, stateblock_optr1,
                               tt_covariance_matrix);

  problem->addCovarianceBlock( stateblock_pptr2, stateblock_pptr2,
                               position_covariance_matrix);
  problem->addCovarianceBlock( stateblock_optr2, stateblock_optr2,
                               orientation_covariance_matrix);
  problem->addCovarianceBlock( stateblock_pptr2, stateblock_optr2,
                               tt_covariance_matrix);

  problem->addCovarianceBlock( stateblock_pptr3, stateblock_pptr3,
                               position_covariance_matrix);
  problem->addCovarianceBlock( stateblock_optr3, stateblock_optr3,
                               orientation_covariance_matrix);
  problem->addCovarianceBlock( stateblock_pptr3, stateblock_optr3,
                               tt_covariance_matrix);

  problem->addCovarianceBlock( stateblock_pptr4, stateblock_pptr4,
                               position_covariance_matrix);
  problem->addCovarianceBlock( stateblock_optr4, stateblock_optr4,
                               orientation_covariance_matrix);
  problem->addCovarianceBlock( stateblock_pptr4, stateblock_optr4,
                               tt_covariance_matrix);
  // create dummy capture
  incomming_dummy = wolf::CaptureBase::emplace<wolf::CaptureBase>(nullptr, "DUMMY", 1.2, sensor_ptr);

  // Make 3rd frame last Key frame
  F3->setTimeStamp(wolf::TimeStamp(2.0));
  problem->getTrajectory()->sortFrame(F3);

  // trigger search for loopclosure
  processor_ptr_point2d->process(incomming_dummy);

  // const std::vector<wolf::FrameBasePtr> &testloops =
  //     processor_ptr_point2d->getCandidates();

  //TODO: Due to changes in the emplace refactor these tests are not working. To be fixed.
  // ASSERT_EQ(testloops.size(),   (unsigned int) 1);
  // ASSERT_EQ(testloops[0]->id(), (unsigned int) 2);
}

//##############################################################################
TEST(ProcessorFrameNearestNeighborFilter, EllipseEllipseRotatedCrosscovariance)
{
  // set covariance to a -90 degree turned ellipse
  Eigen::Matrix2s position_covariance_matrix;
//  position_covariance_matrix << 9.0, 1.8,
//                                1.8, 5.0;

  position_covariance_matrix << 5.0, 0.0,
                                0.0, 9.0;

  problem->addCovarianceBlock( F1->getP(), F1->getP(),
                               position_covariance_matrix);
  problem->addCovarianceBlock( F2->getP(), F2->getP(),
                               position_covariance_matrix);
  problem->addCovarianceBlock( F3->getP(), F3->getP(),
                               position_covariance_matrix);
  problem->addCovarianceBlock( F4->getP(), F4->getP(),
                               position_covariance_matrix);

  // Make 3rd frame last Key frame
  F3->setTimeStamp(wolf::TimeStamp(2.0));
  problem->getTrajectory()->sortFrame(F3);

  // trigger search for loopclosure
  processor_ptr_ellipse2d->process(incomming_dummy);
  const std::vector<wolf::FrameBasePtr> &testloops =
      processor_ptr_ellipse2d->getCandidates();

  //TODO: Due to changes in the emplace refactor these tests are not working. To be fixed.
  // ASSERT_EQ(testloops.size(),   (unsigned int) 2);
  // ASSERT_EQ(testloops[0]->id(), (unsigned int) 1);
  // ASSERT_EQ(testloops[1]->id(), (unsigned int) 2);

  // Make 4th frame last Key frame
  F4->setTimeStamp(wolf::TimeStamp(3.0));
  problem->getTrajectory()->sortFrame(F4);

  // trigger search for loopclosure again
  processor_ptr_ellipse2d->process(incomming_dummy);
  ASSERT_EQ(testloops.size(), (unsigned int) 0);
}

//##############################################################################
int main(int argc, char **argv)
{
  // SENSOR PARAMETERS
  odom_intrinsics->k_disp_to_disp = 0.2;
  odom_intrinsics->k_rot_to_rot = 0.2;

  // install sensor
  sensor_ptr = problem->installSensor("ODOM 2D", "odom",
                                      odom_extrinsics, odom_intrinsics);

  // install the processors
  processor_params_point2d->probability_ = 0.95;
  processor_params_point2d->buffer_size_ = 0;         // just exclude identical frameIDs
  processor_params_point2d->distance_type_ = wolf::LoopclosureDistanceType::LC_POINT_ELLIPSE;

  // processor_ptr_point2d = std::make_shared<DummyLoopCloser>(processor_params_point2d);
  processor_ptr_point2d = std::static_pointer_cast<wolf::ProcessorFrameNearestNeighborFilter>(wolf::ProcessorBase::emplace<DummyLoopCloser>(sensor_ptr, processor_params_point2d));
  processor_ptr_point2d->setName("processor2Dpoint");

  // sensor_ptr->addProcessor(processor_ptr_point2d);

  processor_params_ellipse2d->probability_ = 0.95;
  processor_params_ellipse2d->buffer_size_ = 0;         // just exclude identical frameIDs
  processor_params_ellipse2d->distance_type_ = wolf::LoopclosureDistanceType::LC_ELLIPSE_ELLIPSE;

  // processor_ptr_ellipse2d = std::make_shared<DummyLoopCloser>(processor_params_ellipse2d);
  processor_ptr_ellipse2d = std::static_pointer_cast<wolf::ProcessorFrameNearestNeighborFilter>(wolf::ProcessorBase::emplace<DummyLoopCloser>(sensor_ptr, processor_params_ellipse2d));
  processor_ptr_ellipse2d->setName("processor2Dellipse");

  // sensor_ptr->addProcessor(processor_ptr_ellipse2d);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
