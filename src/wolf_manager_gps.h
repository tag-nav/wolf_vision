//Wolf includes
#include "constraint_sparse.h"
#include "constraint_odom_2D.h"
#include "capture_motion.h"
#include "trajectory_base.h"
#include "map_base.h"
#include "wolf.h"
#include "wolf_problem.h"

//Ceres includes
#include "ceres/ceres.h"
#include "glog/logging.h"

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//std includes
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>


class WolfManagerGPS
{
    protected:
        //sets the problem 
        WolfProblem* problem_;

        //pointer to a sensor providing predictions
        SensorBase* sensor_prior_;
        
        //auxiliar/temporary iterators, frames and captures
        FrameBaseIter first_window_frame_;
        FrameBase* current_frame_;
        FrameBase* last_key_frame_;
        CaptureMotion* last_capture_relative_;
        CaptureMotion* second_last_capture_relative_;
        std::queue<CaptureBase*> new_captures_;

        //Manager parameters
        unsigned int trajectory_size_;
        WolfScalar new_frame_elapsed_time_;

    public:
        WolfManagerGPS(const FrameStructure _frame_structure,
                    SensorBase* _sensor_prior_ptr,
                    const Eigen::VectorXs& _prior,
                    const Eigen::MatrixXs& _prior_cov,
                    const unsigned int& _trajectory_size = 10,
                    const WolfScalar& _new_frame_elapsed_time = 0.1);

        virtual ~WolfManagerGPS();

        void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp);

        void createFrame(const TimeStamp& _time_stamp);

        void addSensor(SensorBase* _sensor_ptr);

        void addCapture(CaptureBase* _capture);

        void manageWindow();

        bool checkNewFrame(CaptureBase* new_capture);

        void update();

        Eigen::VectorXs getVehiclePose(const TimeStamp& _now = 0);

        WolfProblem* getProblemPtr();

        void setWindowSize(const unsigned int& _size);

        void setNewFrameElapsedTime(const WolfScalar& _dt);
};
