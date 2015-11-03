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
#include "capture_fix.h"
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
#include "state_quaternion.h"

template <class StatePositionT, class StateOrientationT, class StateVelocityT=NoState, class StateOmegaT=NoState>
class WolfManager
{
    protected:
        //sets the problem 
        WolfProblem* problem_;

        //pointer to a sensor providing predictions
        SensorBase* sensor_prior_;
        
        //auxiliar/temporary iterators, frames and captures
        FrameBaseIter first_window_frame_;
        FrameBase* current_frame_;
        FrameBase* previous_frame_;
        CaptureRelative* last_capture_relative_;
        CaptureRelative* second_last_capture_relative_;
        std::queue<CaptureBase*> new_captures_;

        //Manager parameters
        unsigned int window_size_;
        WolfScalar new_frame_elapsed_time_;

    public:
        WolfManager(const unsigned int& _state_length,
                    SensorBase* _sensor_prior, 
                    const Eigen::VectorXs& _init_frame, 
                    const Eigen::MatrixXs& _init_frame_cov, 
                    const unsigned int& _w_size = 10, 
                    const WolfScalar& _new_frame_elapsed_time = 0.1);

        virtual ~WolfManager();

        void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp);

        void createFrame(const TimeStamp& _time_stamp);

        void addCapture(CaptureBase* _capture);

        void manageWindow();

        bool checkNewFrame(CaptureBase* new_capture);

        void update();

        Eigen::VectorXs getVehiclePose(const TimeStamp& _now = 0);

        WolfProblem* getProblemPtr();

        void setWindowSize(const unsigned int& _size);

        void setNewFrameElapsedTime(const WolfScalar& _dt);
};


//////////////////////////////////////////
//          IMPLEMENTATION
//////////////////////////////////////////
template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::WolfManager(const unsigned int& _state_length,
                                                                                         SensorBase* _sensor_prior,
                                                                                         const Eigen::VectorXs& _init_frame,
                                                                                         const Eigen::MatrixXs& _init_frame_cov,
                                                                                         const unsigned int& _w_size,
                                                                                         const WolfScalar& _new_frame_elapsed_time) :
                    
        problem_(new WolfProblem(_state_length)),
        sensor_prior_(_sensor_prior),
        current_frame_(nullptr),
        previous_frame_(nullptr),
        last_capture_relative_(nullptr),
        window_size_(_w_size),
        new_frame_elapsed_time_(_new_frame_elapsed_time)
{
    assert( _init_frame.size() == StatePositionT::BLOCK_SIZE + StateOrientationT::BLOCK_SIZE &&
            _init_frame_cov.cols() == StatePositionT::BLOCK_SIZE + StateOrientationT::BLOCK_SIZE &&
            _init_frame_cov.rows() == StatePositionT::BLOCK_SIZE + StateOrientationT::BLOCK_SIZE &&
            "Wrong init_frame state vector or covariance matrix size");

    std::cout << "initializing wolfmanager" << std::endl;

    // Initial frame
    createFrame(_init_frame, TimeStamp(0));
    first_window_frame_ = problem_->getTrajectoryPtr()->getFrameListPtr()->begin();

    // Initial covariance
    CaptureFix* initial_covariance = new CaptureFix(TimeStamp(0), _init_frame, _init_frame_cov);
    current_frame_->addCapture(initial_covariance);
    initial_covariance->processCapture();

    // Current robot frame
    createFrame(_init_frame, TimeStamp(0));

    std::cout << " wolfmanager initialized" << std::endl;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::~WolfManager()
{
    delete problem_;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
void WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
{
    //std::cout << "creating new frame..." << std::endl;

    assert(_frame_state.size() == StatePositionT::BLOCK_SIZE + StateOrientationT::BLOCK_SIZE + StateVelocityT::BLOCK_SIZE + StateOmegaT::BLOCK_SIZE && "Wrong frame vector when creating a new frame");

    // Process current frame non-odometry captures
    if (current_frame_ != nullptr)
    {
        //std::cout << "Processing last frame non-odometry captures " << current_frame_->getCaptureListPtr()->size() << std::endl;
        for (auto capture_it = current_frame_->getCaptureListPtr()->begin(); capture_it != current_frame_->getCaptureListPtr()->end(); capture_it++)
            if ((*capture_it)->getSensorPtr() != sensor_prior_)
            {
                //std::cout << "processing capture " << (*capture_it)->nodeId() << std::endl;
                (*capture_it)->processCapture();
            }
    }
    //std::cout << "Last frame non-odometry captures processed" << std::endl;

    // Store last frame
    previous_frame_ = current_frame_;

    // Create frame and add it to the trajectory
    StatePositionT* new_position = new StatePositionT(problem_->getNewStatePtr());
    problem_->addState(new_position, _frame_state.head<StatePositionT::BLOCK_SIZE>());
    //std::cout << "StatePosition created" << std::endl;

    StateOrientationT* new_orientation = new StateOrientationT(problem_->getNewStatePtr());
    problem_->addState(new_orientation, _frame_state.segment<StateOrientationT::BLOCK_SIZE>(new_position->getStateSize()));
    //std::cout << "StateOrientation created" << std::endl;

    StateVelocityT* new_velocity = nullptr;
    if (StateVelocityT::BLOCK_SIZE != 0)
    {
        new_velocity = new StateVelocityT(problem_->getNewStatePtr());
        problem_->addState(new_velocity, _frame_state.segment<StateVelocityT::BLOCK_SIZE>(new_position->getStateSize()+new_orientation->getStateSize()));
    }

    StateOmegaT* new_omega = nullptr;
    if (StateOmegaT::BLOCK_SIZE != 0)
    {
        new_omega = new StateOmegaT(problem_->getNewStatePtr());
        problem_->addState(new_omega, _frame_state.segment<StateOmegaT::BLOCK_SIZE>(new_position->getStateSize()+new_orientation->getStateSize()+new_velocity->getStateSize()));
    }

    problem_->getTrajectoryPtr()->addFrame(new FrameBase(_time_stamp, new_position, new_orientation, new_velocity, new_omega));
    //std::cout << "frame created" << std::endl;

    // Store new current frame
    current_frame_ = problem_->getLastFramePtr();
    //std::cout << "current_frame_" << std::endl;

    // empty last capture relative
    if (previous_frame_ != nullptr)
    {
        CaptureRelative* empty_odom = new CaptureOdom2D(_time_stamp, _time_stamp, sensor_prior_, Eigen::Vector3s::Zero());
        previous_frame_->addCapture(empty_odom);
        empty_odom->processCapture();
        last_capture_relative_ = empty_odom;
    }
    //std::cout << "last_frame_" << std::endl;

    // Fixing or removing old frames
    manageWindow();
    //std::cout << "new frame created" << std::endl;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
void WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::createFrame(const TimeStamp& _time_stamp)
{
    //std::cout << "creating new frame from prior..." << std::endl;
    createFrame(last_capture_relative_->computePrior(_time_stamp), _time_stamp);
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
void WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::addCapture(CaptureBase* _capture)
{
    new_captures_.push(_capture);
    //std::cout << "added new capture: " << _capture->nodeId() << " stamp: ";
    //_capture->getTimeStamp().print();
    //std::cout << std::endl;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
void WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::manageWindow()
{
    //std::cout << "managing window..." << std::endl;
    // WINDOW of FRAMES (remove or fix old frames)
    if (problem_->getTrajectoryPtr()->getFrameListPtr()->size() > window_size_+1)
    {
        //std::cout << "first_window_frame_ " << (*first_window_frame_)->nodeId() << std::endl;
        //problem_->getTrajectoryPtr()->removeFrame(problem_->getTrajectoryPtr()->getFrameListPtr()->begin());
        (*first_window_frame_)->fix();
        first_window_frame_++;
    }
    //std::cout << "window managed" << std::endl;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
bool WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::checkNewFrame(CaptureBase* new_capture)
{
    //std::cout << "checking if new frame..." << std::endl;
    // TODO: not only time, depending on the sensor...
    //std::cout << new_capture->getTimeStamp().get() - last_frame_->getTimeStamp().get() << std::endl;
    return new_capture->getTimeStamp().get() - previous_frame_->getTimeStamp().get() > new_frame_elapsed_time_;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
void WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::update()
{
    //std::cout << "updating..." << std::endl;
    while (!new_captures_.empty())
    {
        // EXTRACT NEW CAPTURE
        CaptureBase* new_capture = new_captures_.front();
        new_captures_.pop();

        // OVERWRITE CURRENT STAMP
        current_frame_->setTimeStamp(new_capture->getTimeStamp());

        // INITIALIZE FIRST FRAME STAMP
        if (previous_frame_->getTimeStamp().get() == 0)
            previous_frame_->setTimeStamp(new_capture->getTimeStamp());

        // ODOMETRY SENSOR
        if (new_capture->getSensorPtr() == sensor_prior_)
        {
            //std::cout << "adding odometry capture..." << new_capture->nodeId() << std::endl;
            // NEW KEY FRAME ?
            if (checkNewFrame(new_capture))
                createFrame(new_capture->getTimeStamp());

            // ADD/INTEGRATE NEW ODOMETRY TO THE LAST FRAME
            last_capture_relative_->integrateCapture((CaptureRelative*) (new_capture));
            current_frame_->setState(last_capture_relative_->computePrior(new_capture->getTimeStamp()));
            current_frame_->setTimeStamp(new_capture->getTimeStamp());
        }
        else
        {
            //std::cout << "adding not odometry capture..." << new_capture->nodeId() << std::endl;
            // NEW KEY FRAME ?
            if (checkNewFrame(new_capture))
                createFrame(new_capture->getTimeStamp());

            // ADD CAPTURE TO THE CURRENT FRAME (or substitute the same sensor previous capture)
            //std::cout << "searching repeated capture..." << new_capture->nodeId() << std::endl;
            CaptureBaseIter repeated_capture_it = current_frame_->hasCaptureOf(new_capture->getSensorPtr());

            if (repeated_capture_it != current_frame_->getCaptureListPtr()->end()) // repeated capture
            {
                //std::cout << "repeated capture, keeping new capture" << new_capture->nodeId() << std::endl;
                current_frame_->removeCapture(repeated_capture_it);
                current_frame_->addCapture(new_capture);
            }
            else
            {
                //std::cout << "not repeated, adding capture..." << new_capture->nodeId() << std::endl;
                current_frame_->addCapture(new_capture);
            }
        }
    }
    //std::cout << "updated" << std::endl;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
Eigen::VectorXs WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::getVehiclePose(const TimeStamp& _now)
{
    if (last_capture_relative_ == nullptr)
        return Eigen::Map<Eigen::Vector3s>(current_frame_->getPPtr()->getPtr());
    else
        return last_capture_relative_->computePrior(_now);
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
WolfProblem* WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::getProblemPtr()
{
    return problem_;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
void WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::setWindowSize(const unsigned int& _size)
{
    window_size_ = _size;
}

template <class StatePositionT, class StateOrientationT, class StateVelocityT, class StateOmegaT>
void WolfManager<StatePositionT, StateOrientationT, StateVelocityT, StateOmegaT>::setNewFrameElapsedTime(const WolfScalar& _dt)
{
    new_frame_elapsed_time_ = _dt;
}
