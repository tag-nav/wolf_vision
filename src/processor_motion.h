/**
 * \file processor_motion.h
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_MOTION_H_
#define PROCESSOR_MOTION_H_

// Wolf
#include "processor_motion.h"
#include "capture_motion2.h"
#include "motion_buffer.h"
#include "time_stamp.h"
#include "wolf.h"

namespace wolf {

/** \brief class for Motion processors
 *
 * This processor integrates motion data into vehicle states.
 *
 * The motion data is provided by the sensor owning this processor.
 * This data is, in the general case, in the reference frame of the sensor, while the integrated motion refers to the robot frame.
 *
 * The reference frame convention used are specified as follows.
 *   - The robot state R is expressed in a global or 'Map' reference frame, named M.
 *   - The sensor frame S is expressed wrt the robot frame R.
 *
 * This processor, therefore, needs to convert the motion data in two ways:
 *   - First, convert the format of this data into a delta-state.
 *     A delta-state is an expression of a state increment that can be treated
 *     algebraically together with states (operations sum (+) for an additive composition,
 *     and substract (-) for the reverse).
 *   - Second, it needs to be converted from the Sensor frame to the Robot frame: R <-- S:
 *
 *       delta_R = fromSensorFrame(delta_S) : this transforms delta_S from frame S to frame R.
 *   - The two operations are performed by the pure virtual method data2delta(). A possible implementation
 *     of data2delta() could be (we use the data member delta_ as the return value):
 * \code
 *     void data2delta(const VectorXs _data)
 *     {
 *          delta_S = format(_data);
 *          delta_R = fromSensorFrame(delta_S);
 *          delta_  = delta_R;
 *     }
 * \endcode
 *     where format() is any code you need to format the data into a delta form,
 *     and fromSensorFrame() is explained below.
 *
 * Only when the motion delta is expressed in the robot frame R, we can integrate it
 * on top of the current Robot frame: R <-- R (+) delta_R
 *
 *     \code    xPlusDelta(R_old, delta_R, R_new) \endcode
 *
 *
 *
 * ### Defining (or not) the fromSensorFrame():
 *
 * In most cases, one will be interested in avoiding the \b fromSensorFrame() issue.
 * This can be trivially done by defining the Robot frame precisely at the Sensor frame,
 * so that S is the identity. In this case, \b fromSensorFrame() does nothing and delta_R = delta_S.
 *
 * Notes:
 *   - This class does not declare any prototype for \b fromSensorFrame().
 *   - In cases where this identification is not possible, or not desired,
 * classes deriving from this class will have to implement fromSensorFrame(),
 * and call it within data2delta(), or write the frame transformation code directly in data2delta().
 */
class ProcessorMotion : public ProcessorBase
{

        // This is the main public interface
    public:
        ProcessorMotion(ProcessorType _tp, Scalar _dt, size_t _state_size, size_t _delta_size, size_t _data_size);
        virtual ~ProcessorMotion();

        // Instructions to the processor:

        virtual void process(CaptureBase* _incoming_ptr);

        // Queries to the processor:

        virtual bool voteForKeyFrame();

        /** \brief Fills a reference to the state integrated so far
         * \param the returned state vector
         */
        const void getState(Eigen::VectorXs& _x);

        /** \brief Gets a constant reference to the state integrated so far
         * \return the state vector
         */
        const Eigen::VectorXs& getState();

        /** \brief Fills the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \param _x the returned state
         */
        void getState(const TimeStamp& _ts, Eigen::VectorXs& _x);

        /** \brief Gets the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \return the state vector
         */
        Eigen::VectorXs& getState(const TimeStamp& _ts);

        /** \brief Provides the motion integrated so far
         * \return a const reference to the integrated delta state
         */
        const Motion& getMotion() const;
        void getMotion(Motion& _motion) const;

        /** \brief Provides the motion integrated until a given timestamp
         * \return a reference to the integrated delta state
         */
        const Motion& getMotion(const TimeStamp& _ts) const;
        void getMotion(const TimeStamp& _ts, Motion& _motion) const;

        /** Composes the deltas in two pre-integrated Captures
         * \param _cap1_ptr pointer to the first Capture
         * \param _cap2_ptr pointer to the second Capture. This is local wrt. the first Capture.
         * \param _delta1_plus_delta2 the concatenation of the deltas of Captures 1 and 2.
         */
        void sumDeltas(CaptureMotion2* _cap1_ptr, CaptureMotion2* _cap2_ptr,
                       Eigen::VectorXs& _delta1_plus_delta2);

        /** Set the origin of all motion for this processor
         * \param _x_origin the state at the origin
         * \param _origin_ptr pointer to a Capture in the origin.
         *        This can be any type of Capture, derived from CaptureBase.
         */
        void setOrigin(const Eigen::VectorXs& _x_origin, CaptureBase* _origin_ptr = nullptr);



        // Helper functions:
    public: // TODO change to protected

        void splitBuffer(const TimeStamp& _t_split, MotionBuffer& _newest_part);

        void reset(CaptureMotion2* _capture_ptr);

        FrameBase* makeFrame(CaptureBase* _capture_ptr, FrameType _type = NON_KEY_FRAME);

        MotionBuffer* getBufferPtr();

        const MotionBuffer* getBufferPtr() const;

    protected:
        void integrate();

        void updateDt();

        // These are the pure virtual functions doing the mathematics
    protected:

         /** \brief convert raw CaptureMotion data to the delta-state format
          *
          * This function accesses the members data_ (as produced by extractData()) and dt_,
          * and computes the value of the delta-state delta_.
          *
          * \param _data the raw motion data
          * \param _dt the time step (not always needed)
          * \param _delta the returned motion delta
          *
          * Rationale:
          *
          * The delta-state format must be compatible for integration using
          * the composition functions doing the math in this class: xPlusDelta(), deltaPlusDelta() and deltaMinusDelta().
          * See the class documentation for some Eigen::VectorXs suggestions.
          *
          * The data format is generally not the same as the delta format,
          * because it is the format of the raw data provided by the Capture,
          * which is unaware of the needs of this processor.
          *
          * Additionally, sometimes the data format is in the form of a
          * velocity, while the delta is in the form of an increment.
          * In such cases, converting from data to delta-state needs integrating
          * the data over the period dt.
          *
          * Two trivial implementations would establish:
          *  - If data_ is an increment: delta_ = data_;
          *  - If data_ is a velocity: delta_ = data_* dt_.
          *
          *  However, other more complicated relations are possible.
          */
         virtual void data2delta(const Eigen::VectorXs& _data, const Scalar _dt, Eigen::VectorXs& _delta) = 0;

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta,
                                Eigen::VectorXs& _x_plus_delta) = 0;

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                    Eigen::VectorXs& _delta1_plus_delta2) = 0;

        /** \brief Delta zero
         * \return a delta state equivalent to the null motion.
         *
         * Hint: you can use a method setZero() in the Eigen::VectorXs class. See ProcessorOdom3d::Odom3dDelta for reference.
         *
         * Examples (see documentation of the the class for info on Eigen::VectorXs):
         *   - 2D odometry: a 3-vector with all zeros, e.g. VectorXs::Zero(3)
         *   - 3D odometry: delta type is a PQ vector: 7-vector with [0,0,0, 0,0,0,1]
         *   - IMU: PQVBB 16-vector with [0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0]
         */
        virtual Eigen::VectorXs deltaZero() const = 0;

    protected:
        // Attributes
        size_t x_size_;    ///< The size of the getState vector
        size_t delta_size_;
        size_t data_size_; ///< the size of the incoming data
        CaptureBase* origin_ptr_;
        CaptureMotion2* last_ptr_;
        CaptureMotion2* incoming_ptr_;

    protected:
        // helpers to avoid allocation
        Scalar dt_; ///< Time step
        Eigen::VectorXs x_; ///< state temporary
        Eigen::VectorXs delta_, delta_integrated_; ///< current delta and integrated deltas
        Eigen::VectorXs data_; ///< current data

    private:
        unsigned int count_;

};


inline ProcessorMotion::ProcessorMotion(ProcessorType _tp, Scalar _dt, size_t _state_size, size_t _delta_size, size_t _data_size) :
        ProcessorBase(_tp), x_size_(_state_size), delta_size_(_delta_size), data_size_(_data_size),
        origin_ptr_(nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr),
        dt_(_dt), x_(_state_size),
        delta_(_delta_size), delta_integrated_(_delta_size),
        data_(_data_size)
{
    //
}


inline ProcessorMotion::~ProcessorMotion()
{
    //
}


inline void ProcessorMotion::process(CaptureBase* _incoming_ptr)
{
    incoming_ptr_ = (CaptureMotion2*)(_incoming_ptr);

    if (last_ptr_ == nullptr)
    {
        // first time
        count_ = 1;
        last_ptr_ = incoming_ptr_;

        // Make keyframe
        if (last_ptr_->getFramePtr() == nullptr)
            makeFrame(last_ptr_);

        delta_integrated_ = deltaZero();
        getBufferPtr()->clear();
        getBufferPtr()->pushBack(_incoming_ptr->getTimeStamp(), delta_integrated_);
        integrate();
        count_++;
    }
    else
    {
        if(count_ == 2)
        {
            // second time only
            last_ptr_ = incoming_ptr_;

            // make non-key frame
            makeFrame(last_ptr_);
        }
        // second and other times
        integrate();
        count_ ++;
}
}

inline void ProcessorMotion::splitBuffer(const TimeStamp& _t_split, MotionBuffer& _newest_part)
{
    last_ptr_->getBufferPtr()->splice(_t_split, _newest_part);
}

inline void ProcessorMotion::reset(CaptureMotion2* _capture_ptr)
{
    // Make a frame with the provided capture, if it did not have one:
    if (_capture_ptr->getFramePtr() == nullptr)
        // make keyframe
        makeFrame(_capture_ptr, KEY_FRAME);
    else
    {
        // set keyframe (maybe it was keyframe already, who cares)
        _capture_ptr->getFramePtr()->setKey();
    }
    // Transfer the old half of the buffer to the new keyframe's Capture
    splitBuffer(_capture_ptr->getTimeStamp(), *(_capture_ptr->getBufferPtr()));
}

inline FrameBase* ProcessorMotion::makeFrame(CaptureBase* _capture_ptr, FrameType _type)
{
    // We need to create the new free Frame to hold what will become the last Capture
    FrameBase* new_frame_ptr = getWolfProblem()->createFrame(_type, _capture_ptr->getTimeStamp());
    new_frame_ptr->addCapture(_capture_ptr); // Add incoming Capture to the new Frame
    return new_frame_ptr;
}

inline bool ProcessorMotion::voteForKeyFrame()
{
    return false;
}


inline Eigen::VectorXs& ProcessorMotion::getState(const TimeStamp& _ts)
{
    getState(_ts, x_);
    return x_;
}


inline void ProcessorMotion::getState(const TimeStamp& _ts, Eigen::VectorXs& _x)
{
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBufferPtr()->getDelta(_ts), _x);
}


inline const Eigen::VectorXs& ProcessorMotion::getState()
{
    getState(x_);
    return x_;
}


inline const void ProcessorMotion::getState(Eigen::VectorXs& _x)
{
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBufferPtr()->getDelta(), _x);
}

inline const Motion& ProcessorMotion::getMotion() const
{
    return getBufferPtr()->getMotion();
}

inline void ProcessorMotion::getMotion(Motion& _motion) const
{
    getBufferPtr()->getMotion(_motion);
}

inline const Motion& ProcessorMotion::getMotion(const TimeStamp& _ts) const
{
    return getBufferPtr()->getMotion(_ts);
}

inline void ProcessorMotion::getMotion(const TimeStamp& _ts, Motion& _motion) const
{
    getBufferPtr()->getMotion(_ts, _motion);
}

inline void ProcessorMotion::sumDeltas(CaptureMotion2* _cap1_ptr,
                                                         CaptureMotion2* _cap2_ptr,
                                                         Eigen::VectorXs& _delta1_plus_delta2)
{
    deltaPlusDelta(_cap1_ptr->getDelta(), _cap2_ptr->getDelta(), _delta1_plus_delta2);
}

inline void ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, CaptureBase* _origin_ptr)
{
    origin_ptr_ = _origin_ptr;
    if (origin_ptr_->getFramePtr() == nullptr)
        makeFrame(origin_ptr_, KEY_FRAME);
    else
        origin_ptr_->getFramePtr()->setKey();
    origin_ptr_->getFramePtr()->setState(_x_origin);
}

inline void ProcessorMotion::integrate()
{
    // Set dt
    updateDt();
    // get data and convert it to delta
    data2delta(incoming_ptr_->getData(), dt_, delta_);
    // then integrate
    deltaPlusDelta(getBufferPtr()->getDelta(), delta_, delta_integrated_);
    // then push it into buffer
    getBufferPtr()->pushBack(incoming_ptr_->getTimeStamp(), delta_integrated_);
}


inline void ProcessorMotion::updateDt()
{
    dt_ = incoming_ptr_->getTimeStamp() - getBufferPtr()->getTimeStamp();
}


inline const MotionBuffer* ProcessorMotion::getBufferPtr() const
{
    return last_ptr_->getBufferPtr();
}


inline MotionBuffer* ProcessorMotion::getBufferPtr()
{
    return last_ptr_->getBufferPtr();
}

} // namespace wolf


#endif /* PROCESSOR_MOTION2_H_ */
