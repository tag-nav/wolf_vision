//std includes
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//Ceres includes
#include "ceres/ceres.h"
#include "glog/logging.h"

//Wolf includes
#include "wolf.h"
#include "sensor_base.h"
#include "sensor_odom_2D.h"
#include "sensor_gps_fix.h"
#include "feature_base.h"
#include "frame_base.h"
#include "state_point.h"
#include "state_complex_angle.h"
#include "capture_base.h"
#include "capture_relative.h"
#include "capture_odom_2D.h"
#include "capture_gps_fix.h"
#include "capture_laser_2D.h"
#include "state_base.h"
#include "constraint_sparse.h"
#include "constraint_gps_2D.h"
#include "constraint_odom_2D_theta.h"
#include "constraint_odom_2D_complex_angle.h"
#include "trajectory_base.h"
#include "map_base.h"
#include "wolf_problem.h"

class WolfManager
{
  protected:
    bool use_complex_angles_;
    WolfProblem* problem_;
    std::queue<CaptureBase*> new_captures_;
    SensorBase* sensor_prior_;
    unsigned int window_size_;
    FrameBaseIter first_window_frame_;
    CaptureRelative* last_capture_relative_;
    WolfScalar new_frame_elapsed_time_;

  public:
    WolfManager(SensorBase* _sensor_prior, const bool _complex_angle, const unsigned int& _state_length, const Eigen::VectorXs& _init_frame, const WolfScalar& _new_frame_elapsed_time=0.1, const unsigned int& _w_size=10) :
        use_complex_angles_(_complex_angle),
        problem_(new WolfProblem(_state_length)),
        sensor_prior_(_sensor_prior),
        window_size_(_w_size),
        new_frame_elapsed_time_(_new_frame_elapsed_time)
    {
      createFrame(_init_frame, TimeStamp());
      problem_->getTrajectoryPtr()->getFrameListPtr()->back()->fix();
    }

    virtual ~WolfManager()
    {
      delete problem_;
    }

    void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
    {
      // Create frame and add it to the trajectory
      StateBase* new_position = new StatePoint2D(problem_->getNewStatePtr());
      problem_->addState(new_position, _frame_state.head(2));

      StateBase* new_orientation;
      if (use_complex_angles_)
        new_orientation = new StateComplexAngle(problem_->getNewStatePtr());
      else
        new_orientation = new StateTheta(problem_->getNewStatePtr());

      problem_->addState(new_orientation, _frame_state.tail(new_orientation->getStateSize()));

      problem_->getTrajectoryPtr()->addFrame(new FrameBase(_time_stamp, new_position, new_orientation));

      // add a zero odometry capture (in order to integrate)
      problem_->getTrajectoryPtr()->getFrameListPtr()->back()->addCapture(new CaptureOdom2D(_time_stamp,
                                                                          sensor_prior_,
                                                                          Eigen::Vector2s::Zero(),
                                                                          Eigen::Matrix2s::Zero()));
      last_capture_relative_ = (CaptureRelative*)(problem_->getTrajectoryPtr()->getFrameListPtr()->back()->getCaptureListPtr()->front());
    }

    void addCapture(CaptureBase* _capture)
    {
      new_captures_.push(_capture);
      //std::cout << "added new capture: " << _capture->nodeId() << std::endl;
    }

    void update()
    {
      while (!new_captures_.empty())
      {
        // EXTRACT NEW CAPTURE
        CaptureBase* new_capture = new_captures_.front();
        new_captures_.pop();

        // ODOMETRY SENSOR
        if (new_capture->getSensorPtr() == sensor_prior_)
        {
          // INTEGRATING ODOMETRY CAPTURE & COMPUTING PRIOR
          //std::cout << "integrating captures " << previous_relative_capture->nodeId() << " " << new_capture->nodeId() << std::endl;
          last_capture_relative_->integrateCapture((CaptureRelative*)(new_capture));

          // NEW KEY FRAME (if enough time from last frame)
          //std::cout << "new TimeStamp - last Frame TimeStamp = " << new_capture->getTimeStamp().get() - problem_->getTrajectoryPtr()->getFrameListPtr()->back()->getTimeStamp().get() << std::endl;
          if (new_capture->getTimeStamp().get() - problem_->getTrajectoryPtr()->getFrameListPtr()->back()->getTimeStamp().get() > new_frame_elapsed_time_)
          {
            //std::cout << "store prev frame" << std::endl;
            auto previous_frame_ptr = problem_->getTrajectoryPtr()->getFrameListPtr()->back();

            // NEW FRAME
            //std::cout << "new frame" << std::endl;
            createFrame(last_capture_relative_->computePrior(), new_capture->getTimeStamp());

            // COMPUTE PREVIOUS FRAME CAPTURES
            //std::cout << "compute prev frame" << std::endl;
            for (auto capture_it = previous_frame_ptr->getCaptureListPtr()->begin(); capture_it != previous_frame_ptr->getCaptureListPtr()->end(); capture_it++)
              (*capture_it)->processCapture();

            // WINDOW of FRAMES (remove or fix old frames)
            //std::cout << "frame window" << std::endl;
            if (problem_->getTrajectoryPtr()->getFrameListPtr()->size() > window_size_)
            {
              //std::cout << "frame fixing" << std::endl;
              //problem_->getTrajectoryPtr()->removeFrame(problem_->getTrajectoryPtr()->getFrameListPtr()->begin());
              (*first_window_frame_)->fix();
              first_window_frame_++;
            }
            else
              first_window_frame_ = problem_->getTrajectoryPtr()->getFrameListPtr()->begin();
          }
        }
        else
        {
          // ADD CAPTURE TO THE LAST FRAME (or substitute the same sensor previous capture)
          //std::cout << "adding not odometry capture..." << std::endl;
          for (auto capture_it = problem_->getTrajectoryPtr()->getFrameListPtr()->back()->getCaptureListPtr()->begin(); capture_it != problem_->getTrajectoryPtr()->getFrameListPtr()->back()->getCaptureListPtr()->end(); capture_it++)
          {
            if ((*capture_it)->getSensorPtr() == new_capture->getSensorPtr())
            {
              //std::cout << "removing previous capture" << std::endl;
              problem_->getTrajectoryPtr()->getFrameListPtr()->back()->removeCapture(capture_it);
              //std::cout << "removed!" << std::endl;
              break;
            }
          }
          problem_->getTrajectoryPtr()->getFrameListPtr()->back()->addCapture(new_capture);
        }
      }
    }

    Eigen::VectorXs getVehiclePose()
    {
      return last_capture_relative_->computePrior();
    }

    WolfProblem* getProblemPtr()
    {
      return problem_;
    }
};
