/*
 * gtest_problem.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: jsola
 */

#include "utils_gtest.h"
#include "base/utils/logging.h"

#include "base/problem/problem.h"
#include "base/solver/solver_manager.h"
#include "base/sensor/sensor_base.h"
#include "base/sensor/sensor_odom_3D.h"
#include "base/processor/processor_odom_3D.h"
#include "base/processor/processor_tracker_feature_dummy.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;


WOLF_PTR_TYPEDEFS(DummySolverManager);

struct DummySolverManager : public SolverManager
{
  DummySolverManager(const ProblemPtr& _problem)
    : SolverManager(_problem)
  {
    //
  }
  virtual void computeCovariances(const CovarianceBlocksToBeComputed blocks){};
  virtual void computeCovariances(const std::vector<StateBlockPtr>& st_list){};
  virtual bool hasConverged(){return true;};
  virtual SizeStd iterations(){return 0;};
  virtual Scalar initialCost(){return 0;};
  virtual Scalar finalCost(){return 0;};
  virtual std::string solveImpl(const ReportVerbosity report_level){return std::string("");};
  virtual void addFactor(const FactorBasePtr& fac_ptr){};
  virtual void removeFactor(const FactorBasePtr& fac_ptr){};
  virtual void addStateBlock(const StateBlockPtr& state_ptr){};
  virtual void removeStateBlock(const StateBlockPtr& state_ptr){};
  virtual void updateStateBlockStatus(const StateBlockPtr& state_ptr){};
  virtual void updateStateBlockLocalParametrization(const StateBlockPtr& state_ptr){};
};

TEST(Problem, create)
{
    ProblemPtr P = Problem::create("POV 3D");

    // check double pointers to branches
    ASSERT_EQ(P, P->getHardware()->getProblem());
    ASSERT_EQ(P, P->getTrajectory()->getProblem());
    ASSERT_EQ(P, P->getMap()->getProblem());

    // check frame structure through the state size
    ASSERT_EQ(P->getFrameStructureSize(), 10);
}

TEST(Problem, Sensors)
{
    ProblemPtr P = Problem::create("POV 3D");

    // add a dummy sensor
    SensorBasePtr S = std::make_shared<SensorBase>("Dummy", nullptr, nullptr, nullptr, 2, false);
    P->addSensor(S);

    // check pointers
    ASSERT_EQ(P, S->getProblem());
    ASSERT_EQ(P->getHardware(), S->getHardware());

}

TEST(Problem, Processor)
{
    ProblemPtr P = Problem::create("PO 3D");

    // check motion processor is null
    ASSERT_FALSE(P->getProcessorMotion());

    // add a motion sensor and processor
    SensorBasePtr Sm = std::make_shared<SensorOdom3D>((Eigen::Vector7s()<<0,0,0, 0,0,0,1).finished(), IntrinsicsOdom3D()); // with dummy intrinsics
    P->addSensor(Sm);

    // add motion processor
    ProcessorMotionPtr Pm = std::make_shared<ProcessorOdom3D>(std::make_shared<ProcessorParamsOdom3D>());
    Sm->addProcessor(Pm);

    // check motion processor IS NOT set by addSensor <-- using InstallProcessor it should, see test Installers
    ASSERT_FALSE(P->getProcessorMotion());

    // set processor motion
    P->setProcessorMotion(Pm);
    // re-check motion processor IS set by addSensor
    ASSERT_EQ(P->getProcessorMotion(), Pm);
}

TEST(Problem, Installers)
{
    std::string wolf_root = _WOLF_ROOT_DIR;
    ProblemPtr P = Problem::create("PO 3D");
    Eigen::Vector7s xs;

    SensorBasePtr    S = P->installSensor   ("ODOM 3D", "odometer",        xs,         wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // install processor tracker (dummy installation under an Odometry sensor -- it's OK for this test)
    ProcessorParamsTrackerFeaturePtr params = std::make_shared<ProcessorParamsTrackerFeature>();
    params->time_tolerance = 0.1;
    params->max_new_features = 5;
    params->min_features_for_keyframe = 10;
    ProcessorBasePtr pt = std::make_shared<ProcessorTrackerFeatureDummy>(ProcessorTrackerFeatureDummy(params));
    S->addProcessor(pt);

    // check motion processor IS NOT set
    ASSERT_FALSE(P->getProcessorMotion());

    // install processor motion
    ProcessorBasePtr pm = P->installProcessor("ODOM 3D", "odom integrator", "odometer", wolf_root + "/src/examples/processor_odom_3D.yaml");

    // check motion processor is set
    ASSERT_TRUE(P->getProcessorMotion());

    // check motion processor is correct
    ASSERT_EQ(P->getProcessorMotion(), pm);
}

TEST(Problem, SetOrigin_PO_2D)
{
    ProblemPtr P = Problem::create("PO 2D");
    TimeStamp       t0(0);
    Eigen::VectorXs x0(3); x0 << 1,2,3;
    Eigen::MatrixXs P0(3,3); P0.setIdentity(); P0 *= 0.1; // P0 is 0.1*Id

    P->setPrior(x0, P0, t0, 1.0);

    // check that no sensor has been added
    ASSERT_EQ(P->getHardware()->getSensorList().size(), (SizeStd) 0);

    // check that the state is correct
    ASSERT_MATRIX_APPROX(x0, P->getCurrentState(), Constants::EPS_SMALL );

    // check that we have one frame, one capture, one feature, one factor
    TrajectoryBasePtr T = P->getTrajectory();
    ASSERT_EQ(T->getFrameList().size(), (SizeStd) 1);
    FrameBasePtr F = P->getLastFrame();
    ASSERT_EQ(F->getCaptureList().size(), (SizeStd) 1);
    CaptureBasePtr C = F->getCaptureList().front();
    ASSERT_EQ(C->getFeatureList().size(), (SizeStd) 1);
    FeatureBasePtr f = C->getFeatureList().front();
    ASSERT_EQ(f->getFactorList().size(), (SizeStd) 1);

    // check that the factor is absolute (no pointers to other F, f, or L)
    FactorBasePtr c = f->getFactorList().front();
    ASSERT_FALSE(c->getFrameOther());
    ASSERT_FALSE(c->getLandmarkOther());

    // check that the Feature measurement and covariance are the ones provided
    ASSERT_MATRIX_APPROX(x0, f->getMeasurement(), Constants::EPS_SMALL );
    ASSERT_MATRIX_APPROX(P0, f->getMeasurementCovariance(), Constants::EPS_SMALL );

    //    P->print(4,1,1,1);
}

TEST(Problem, SetOrigin_PO_3D)
{
    ProblemPtr P = Problem::create("PO 3D");
    TimeStamp       t0(0);
    Eigen::VectorXs x0(7); x0 << 1,2,3,4,5,6,7;
    Eigen::MatrixXs P0(6,6); P0.setIdentity(); P0 *= 0.1; // P0 is 0.1*Id

    P->setPrior(x0, P0, t0, 1.0);

    // check that no sensor has been added
    ASSERT_EQ(P->getHardware()->getSensorList().size(), (SizeStd) 0);

    // check that the state is correct
    ASSERT_TRUE((x0 - P->getCurrentState()).isMuchSmallerThan(1, Constants::EPS_SMALL));

    // check that we have one frame, one capture, one feature, one factor
    TrajectoryBasePtr T = P->getTrajectory();
    ASSERT_EQ(T->getFrameList().size(), (SizeStd) 1);
    FrameBasePtr F = P->getLastFrame();
    ASSERT_EQ(F->getCaptureList().size(), (SizeStd) 1);
    CaptureBasePtr C = F->getCaptureList().front();
    ASSERT_EQ(C->getFeatureList().size(), (SizeStd) 1);
    FeatureBasePtr f = C->getFeatureList().front();
    ASSERT_EQ(f->getFactorList().size(), (SizeStd) 1);

    // check that the factor is absolute (no pointers to other F, f, or L)
    FactorBasePtr c = f->getFactorList().front();
    ASSERT_FALSE(c->getFrameOther());
    ASSERT_FALSE(c->getLandmarkOther());

    // check that the Feature measurement and covariance are the ones provided
    ASSERT_TRUE((x0 - f->getMeasurement()).isMuchSmallerThan(1, Constants::EPS_SMALL));
    ASSERT_TRUE((P0 - f->getMeasurementCovariance()).isMuchSmallerThan(1, Constants::EPS_SMALL));

    //    P->print(4,1,1,1);
}

TEST(Problem, emplaceFrame_factory)
{
    ProblemPtr P = Problem::create("PO 2D");

    FrameBasePtr f0 = P->emplaceFrame("PO 2D",    KEY, VectorXs(3),  TimeStamp(0.0));
    FrameBasePtr f1 = P->emplaceFrame("PO 3D",    KEY, VectorXs(7),  TimeStamp(1.0));
    FrameBasePtr f2 = P->emplaceFrame("POV 3D",   KEY, VectorXs(10), TimeStamp(2.0));

    //    std::cout << "f0: type PO 2D?    "  << f0->getType() << std::endl;
    //    std::cout << "f1: type PO 3D?    "  << f1->getType() << std::endl;
    //    std::cout << "f2: type POV 3D?   "  << f2->getType() << std::endl;

    ASSERT_EQ(f0->getType().compare("PO 2D"), 0);
    ASSERT_EQ(f1->getType().compare("PO 3D"), 0);
    ASSERT_EQ(f2->getType().compare("POV 3D"), 0);

    // check that all frames are effectively in the trajectory
    ASSERT_EQ(P->getTrajectory()->getFrameList().size(), (SizeStd) 3);

    // check that all frames are linked to Problem
    ASSERT_EQ(f0->getProblem(), P);
    ASSERT_EQ(f1->getProblem(), P);
    ASSERT_EQ(f2->getProblem(), P);
}

TEST(Problem, StateBlocks)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    ProblemPtr P = Problem::create("PO 3D");
    Eigen::Vector7s xs;

    // 2 state blocks, fixed
    SensorBasePtr    Sm = P->installSensor   ("ODOM 3D", "odometer",xs, wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) 2);

    // 3 state blocks, fixed
    SensorBasePtr    St = P->installSensor   ("CAMERA", "camera",   xs, wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) (2 + 3));

    ProcessorParamsTrackerFeaturePtr params = std::make_shared<ProcessorParamsTrackerFeature>();
    params->time_tolerance            = 0.1;
    params->max_new_features          = 5;
    params->min_features_for_keyframe = 10;
    ProcessorBasePtr pt = std::make_shared<ProcessorTrackerFeatureDummy>(ProcessorTrackerFeatureDummy(params));

    St->addProcessor(pt);
    ProcessorBasePtr pm = P->installProcessor("ODOM 3D",            "odom integrator",      "odometer", wolf_root + "/src/examples/processor_odom_3D.yaml");

    // 2 state blocks, estimated
    auto KF = P->emplaceFrame("PO 3D", KEY, xs, 0);
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd)(2 + 3 + 2));

    // Notifications
    Notification notif;
    ASSERT_TRUE(P->getStateBlockNotification(KF->getP(), notif));
    ASSERT_EQ(notif, ADD);
    ASSERT_TRUE(P->getStateBlockNotification(KF->getO(), notif));
    ASSERT_EQ(notif, ADD);

    //    P->print(4,1,1,1);

    // change some SB properties
    St->unfixExtrinsics();
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd)(2 + 3 + 2)); // changes in state_blocks status (fix/state/localparam) does not raise a notification in problem, only ADD/REMOVE

    //    P->print(4,1,1,1);

    // consume notifications
    DummySolverManagerPtr SM = std::make_shared<DummySolverManager>(P);
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd)(2 + 3 + 2));
    SM->update(); // calls P->consumeStateBlockNotificationMap();
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) (0)); // consume empties the notification map

    // remove frame
    auto SB_P = KF->getP();
    auto SB_O = KF->getO();
    KF->remove();
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd)(2));
    ASSERT_TRUE(P->getStateBlockNotification(SB_P, notif));
    ASSERT_EQ(notif, REMOVE);
    ASSERT_TRUE(P->getStateBlockNotification(SB_O, notif));
    ASSERT_EQ(notif, REMOVE);

}

TEST(Problem, Covariances)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    ProblemPtr P = Problem::create("PO 3D");
    Eigen::Vector7s xs;

    SensorBasePtr    Sm = P->installSensor   ("ODOM 3D", "odometer",xs, wolf_root + "/src/examples/sensor_odom_3D.yaml");
    SensorBasePtr    St = P->installSensor   ("CAMERA", "camera",   xs, wolf_root + "/src/examples/camera_params_ueye_sim.yaml");

    ProcessorParamsTrackerFeaturePtr params = std::make_shared<ProcessorParamsTrackerFeature>();
    params->time_tolerance            = 0.1;
    params->max_new_features          = 5;
    params->min_features_for_keyframe = 10;
    ProcessorBasePtr pt = std::make_shared<ProcessorTrackerFeatureDummy>(ProcessorTrackerFeatureDummy(params));

    St->addProcessor(pt);
    ProcessorBasePtr pm = P->installProcessor("ODOM 3D",            "odom integrator",      "odometer", wolf_root + "/src/examples/processor_odom_3D.yaml");

    // 4 state blocks, estimated
    St->unfixExtrinsics();
    FrameBasePtr F = P->emplaceFrame("PO 3D", KEY, xs, 0);

    // set covariance (they are not computed without a solver)
    P->addCovarianceBlock(F->getP(), Eigen::Matrix3s::Identity());
    P->addCovarianceBlock(F->getO(), Eigen::Matrix3s::Identity());
    P->addCovarianceBlock(F->getP(), F->getO(), Eigen::Matrix3s::Zero());

    // get covariance
    Eigen::MatrixXs Cov;
    ASSERT_TRUE(P->getFrameCovariance(F, Cov));

    ASSERT_EQ(Cov.cols() , 6);
    ASSERT_EQ(Cov.rows() , 6);
    ASSERT_MATRIX_APPROX(Cov, Eigen::Matrix6s::Identity(), 1e-12);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

