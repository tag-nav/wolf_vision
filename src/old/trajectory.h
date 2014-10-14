/*
 * trajectory.h
 *
 *  Created on: May 21, 2014
 *      \author: jsola
 */

#ifndef TRAJECTORY_SIMPLE_H_
#define TRAJECTORY_SIMPLE_H_

//std
#include <vector>
#include <memory>
#include <list>

//eigen
#include <eigen3/Eigen/Dense>

//wolf
#include "state_pqv.h"
#include "state_imu.h"
#include "frame.h"


/**
 * \brief Class for trajectories as windows of keyframes
 * \param StateType state type of the mobile platform, one per keyframe
 * \param OtherState state class of other states, shared among all keyframes
 *
 * This class implements a sliding window of keyframes. Each keyframe is an
 * object of class Frame. The sliding window looks like this:
 *
 * \image html KFWindow.png
 *
 * It is implemented as a circular buffer, with the addition of a
 * storage vector to store, contiguously in memory, all the states
 * of all frames in the window that want to be estimated. These states
 * are sometimes keyframe-specific (such as the position and orientation
 * of the mobile platform at the time the keyframe was taken), or
 * trajectory-specific (such as sensor calibration parameters, biases,
 * gravity vectors, or any magnitude that is either constant or has a dynamics which
 * is much slower than the whole duration of the window).
 *
 * This design, including the particularities within the keyframes, is sketched below. We can see the storage
 * vector,
 * a circular buffer of keyframes (implementing the sliding window), and some other states OS which
 * affect the whole trajectory.
 *
 * Each keyframe has a time stamp TS, a list of pointers to observations OB*, and a State VS.
 * The State has a state vector, which is mapped (With Eigen::Map) onto the Trajectory storage vector.
 *
 * \image html TrajectoryStructure.png
 *
 * See the overall WOLF documentation, the Frame class and the StateBase class for more information.
 *
 * The Trajectory class takes care of the circular buffer via the function member addFrame(). This function adds a
 * keyframe to the head of the trajectory. In case the buffer is full at the time of calling addFrame()
 * (which is normally the case, except for the
 * initial steps of the algorithm while the buffer is being filled up) the oldest keyframe is dropped from the window.
 *
 * Trajectory allows the use of a regular Frame at the head of the window, as explained here. An exceptional
 * frame is the foremost frame of the window: in some cases, it
 * might be interesting to just have the last frame received, even if it is not a keyframe, to be optimized
 * jointly with the rest of the keyframes. The window is then formed by a sequence of keyframes and the last
 * Frame. At the arrival of a new Frame, we must do (see the figure):
 *   - If the head Frame is not keyframe, replace it with the new frame.
 *   - If the head Frame is keyframe, push the new Frame (the oldest keyframe is automatically dropped from the window).
 * This is automatically implemented in addFrame(). In order for it to work, you are the responsible for setting
 * the 'keyframe' marker in the Frame class (using Frame::setKey())
 *
 * \image html AddKF.png
 */
template<class StateType, class OtherType>
class Trajectory
{
    private:
        unsigned int capacity_; ///<allocated space (frame sized) in memory
        unsigned int head_; ///<index to newest frame
        unsigned int size_; ///<current number of frames

        Eigen::VectorXs state_storage_; ///<full state vector
        std::vector<Frame> keyframes_; ///<sliding window of (key)frames
        OtherType other_state_; ///<auxiliar parameters to estimate

    public:

        /**
         * Constructor from length
         * \param _length number of frames of the trajectory
         */
        Trajectory(unsigned int _length);

        virtual ~Trajectory();

        /**
         * Current length of the trajectory
         */
        unsigned int size() const;

        /**
         * Add a new frame.
         * \param _fr the frame to add.
         */
        void addFrame(Frame& _fr);

        /**
         * Return the non_circular index of a frame.
         * \param _i the index, counting from the head and back.
         * \return the index of the non-circular structure.
         *
         * This function is used to access the index of the physical storage from the index of the circular buffer.
         */
        unsigned int frameIndex(unsigned int _i);

        /**
         * Get a frame.
         * \param _i the frame index counting from the head of the trajectory
         * \return the frame
         */
        Frame& frame(const unsigned int _i);

        /**
         * Get auxiliary state
         * \return the auxiliary states, shared along the trajectory
         */
        OtherType& otherState();

        /**
         * Get a reference to the storage state vector
         * \return a reference to the storage state vector
         */
        Eigen::VectorXs& x();

        /**
         * Set the storage state vector
         * \param _x the storage state vector
         */
        void x(const Eigen::VectorXs& _x);

    private:
        /**
         * Add a new frame to the head
         */
        void pushToHead(Frame& _fr);

        /**
         * Replace the head frame
         */
        void replaceHeadFrame(Frame& _fr);

        bool isFull();
        bool isEmpty();

    public:
        /**
         * \brief Complete features and correspondences of new frames
         *
         * XXX this is just a test by now
         *
         * This function detects features, makes matchings, and creates links between frames.
         * The result is a complete structure of the frame, with all features and correspondences in place.
         * */
        void completeUnprocessedFrames();

};

/*
 * Constructor from length
 */
template<class StateType, class OtherType>
Trajectory<StateType, OtherType>::Trajectory(unsigned int _length) :
        capacity_(_length), //
        head_(_length - 1), //
        size_(0), //
        state_storage_(_length * StateType::SIZE_ + OtherType::SIZE_), //
        keyframes_(), // ensure states are not mapped at this point
        other_state_(state_storage_, _length * StateType::SIZE_) //
{
    //allocate memory for all keyframes
    keyframes_.reserve(_length);
    //construct & push states
    for (unsigned int it = 0; it < _length; it++)
    {
        StateType st_tmp(state_storage_, it * StateType::SIZE_);
        Frame fr_tmp(0, st_tmp);
        keyframes_.push_back(fr_tmp);
    }
    Frame<StateType>::resetIdFactory(); // Ensures frame Ids start correctly after this point.
}

template<class StateType, class OtherType>
unsigned int Trajectory<StateType, OtherType>::size() const
{
    return size_;
}

template<class StateType, class OtherType>
void Trajectory<StateType, OtherType>::addFrame(Frame<StateType>& _fr)
{
    if (isEmpty() | keyframes_.at(head_).isKey())
    {
        pushToHead(_fr);
    }
    else
    {
        replaceHeadFrame(_fr);
    }
}

template<class StateType, class OtherType>
unsigned int Trajectory<StateType, OtherType>::frameIndex(unsigned int _i)
{
    return (head_ - _i + capacity_) % capacity_;
}

template<class StateType, class OtherType>
Frame& Trajectory<StateType, OtherType>::frame(const unsigned int _i)
{
    assert(_i < size());
    return keyframes_.at(frameIndex(_i));
}

template<class StateType, class OtherType>
OtherType& Trajectory<StateType, OtherType>::otherState()
{
    return other_state_;
}

template<class StateType, class OtherType>
void Trajectory<StateType, OtherType>::x(const Eigen::VectorXs& _x)
{
    assert(_x.size() == state_storage_.size());
    state_storage_ = _x;
}

template<class StateType, class OtherType>
Eigen::VectorXs& Trajectory<StateType, OtherType>::x()
{
    return state_storage_;
}

template<class StateType, class OtherType>
void Trajectory<StateType, OtherType>::pushToHead(Frame& _fr)
{
    if (!isFull())
    {
        size_++; // increase size
    }
    head_ = (head_ + 1) % capacity_;
    keyframes_.at(head_) = _fr;
}

template<class StateType, class OtherType>
void Trajectory<StateType, OtherType>::replaceHeadFrame(Frame& _fr)
{
    keyframes_.at(head_) = _fr;
}

template<class StateType, class OtherType>
bool Trajectory<StateType, OtherType>::isFull()
{
    return (size_ == capacity_);
}

template<class StateType, class OtherType>
Trajectory<StateType, OtherType>::~Trajectory()
{
}

template<class StateType, class OtherType>
bool Trajectory<StateType, OtherType>::isEmpty()
{
    return (size_ == 0);
}

// overloaded <<
template<class StateType, class OtherType>
std::ostream& operator<<(std::ostream & os, Trajectory<StateType, OtherType>& trj)
{
    os << "Head index: " << trj.frameIndex(0);
    os << "  /  Other states: (" << trj.otherState().x().transpose() << ") ";
    os << "  /  Frames (ID:TS): >-";
    for (unsigned int i = 0; i < trj.size(); i++)
    {
        os << trj.frame(trj.size() - 1 - i) << "-";
    }
    os << ">";
    return os;
}

typedef Trajectory<StatePQV, StateIMU> TrajectoryPQVIMU;

/*****************************************************************************
 * This is testing for high level algorithms to be placed here, maybe
 *****************************************************************************/
template<class StateType, class OtherType>
void Trajectory<StateType, OtherType>::completeUnprocessedFrames()
{

//    list<shared_ptr<SensorDataBase> >::iterator SD_i;
//    for (SD_i = frame(0).sensor_data_.begin(); SD_i != frame(0).sensor_data_.end(); SD_i++)
//    {
//        (*SD_i)->detect();
//        for (unsigned int F_j = 1; F_j < size(); F_j++)
//        {
//            list<shared_ptr<SensorDataBase> >::iterator SD_k;
//            for (SD_k = frame(F_j).sensor_data_.begin(); SD_k != frame(F_j).sensor_data_.end(); SD_k++)
//            {
//                if ((*SD_i)->isMatchable(*SD_k))
//                {
//                    (*SD_i)->match(*SD_k);
//                }
//            }
//        }
//    }

}

#endif /* TRAJECTORY_SIMPLE_H_ */
