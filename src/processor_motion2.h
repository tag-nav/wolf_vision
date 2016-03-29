/**
 * \file processor_motion2.h
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_MOTION2_H_
#define PROCESSOR_MOTION2_H_

// Wolf
#include "processor_base.h"
#include "capture_motion2.h"
#include "time_stamp.h"
#include "wolf.h"

// STL
#include <deque>

/** \brief class for Motion processors
 *  * \param MotionDeltaType The type of the motion delta and the motion integrated delta. It can be an Eigen::VectorXs (default) or any other construction, most likely a struct.
 *        Generalized Delta types allow for optimized algorithms. For example, in an IMU, the Delta type might be defined as:
 * \code
 *   typedef struct {
 *     Eigen::Vector3s dp;     // Position delta
 *     Eigen::Matrix3s dR;     // Rotation matrix delta
 *     Eigen::Vector3s dv;     // Velocity delta
 *     Eigen::Vector3s dab;    // Acc. bias delta
 *     Eigen::Vector3s dwb;    // Gyro bias delta
 *   } ImuDeltaType;
 * \endcode
 *     Also, using quaternions instead of rotation matrices
 * \code
 *   typedef struct {
 *     Eigen::Vector3s    dp;  // Position delta
 *     Eigen::Quaternions dq;  // Quaternion delta
 *     Eigen::Vector3s    dv;  // Velocity delta
 *     Eigen::Vector3s    dab; // Acc. bias delta
 *     Eigen::Vector3s    dwb; // Gyro bias delta
 *   } ImuDeltaType;
 * \endcode
 */
template <class MotionDeltaType = Eigen::VectorXs>
class ProcessorMotion2 : public ProcessorBase
{

        // This is the main public interface
    public:
        ProcessorMotion2(ProcessorType _tp, WolfScalar _dt, size_t _state_size, size_t _delta_size, size_t _data_size);
        virtual ~ProcessorMotion2();

        // Instructions to the processor:

        virtual void process(CaptureBase* _incoming_ptr);
        void init(CaptureMotion2<MotionDeltaType>* _origin_ptr);
        void update();
        void reset(const TimeStamp& _ts);
        void makeKeyFrame(const TimeStamp& _ts);

        // Queries to the processor:

        /** \brief Fills a reference to the state integrated so far
         * \param the returned state vector
         */
        const void state(Eigen::VectorXs& _x);
        /** \brief Gets a constant reference to the state integrated so far
         * \return the state vector
         */
        const Eigen::VectorXs& state();
        /** \brief Fills the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \param _x the returned state
         */
        void state(const TimeStamp& _ts, Eigen::VectorXs& _x);
        /** \brief Gets the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \return the state vector
         */
        Eigen::VectorXs state(const TimeStamp& _ts);
        /** \brief Provides the delta-state integrated so far
         * \return a reference to the integrated delta state
         */
        const MotionDeltaType& deltaState() const
        {
            return getBufferPtr()->getDelta();
        }
        /** \brief Provides the delta-state between two time-stamps
         * \param _t1 initial time
         * \param _t2 final time
         * \param _Delta the integrated delta-state between _t1 and _t2
         */
        void deltaState(const TimeStamp& _t1, const TimeStamp& _t2, MotionDeltaType& _Delta)
        {
            deltaMinusDelta(getBufferPtr()->getDelta(_t2), getBufferPtr()->getDelta(_t2), _Delta);
        }
        /** Composes the deltas in two pre-integrated Captures
         * \param _cap1_ptr pointer to the first Capture
         * \param _cap2_ptr pointer to the second Capture. This is local wrt. the first Capture.
         * \param _delta1_plus_delta2 the concatenation of the deltas of Captures 1 and 2.
         */
        void sumDeltas(CaptureMotion2<MotionDeltaType>* _cap1_ptr, CaptureMotion2<MotionDeltaType>* _cap2_ptr,
                       MotionDeltaType& _delta1_plus_delta2)
        {
            deltaPlusDelta(_cap1_ptr->getDelta(), _cap2_ptr->getDelta(), _delta1_plus_delta2);
        }

        // Helper functions:
    protected:

        void integrate(CaptureMotion2<MotionDeltaType>* _incoming_ptr);

        typename CaptureMotion2<MotionDeltaType>::MotionBuffer* getBufferPtr();

        const typename CaptureMotion2<MotionDeltaType>::MotionBuffer* getBufferPtr() const;

        // These are the pure virtual functions doing the mathematics
    protected:

         /** \brief convert raw CaptureMotion data to the delta-state format
          *
          * This function accesses the members data_ (as produced by extractData()) and dt_,
          * and computes the value of the delta-state delta_.
          *
          * Rationale:
          *
          * The delta-state format must be compatible for integration using
          * the composition functions doing the math in this class: xPlusDelta(), deltaPlusDelta() and deltaMinusDelta().
          *
          * The data format is not necessarily the same, as it is the
          * format of the raw data provided by the Capture,
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
         virtual void data2delta(const Eigen::VectorXs& _data, MotionDeltaType& _delta) = 0;

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const MotionDeltaType& _delta,
                                MotionDeltaType& _x_plus_delta) = 0;

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */
        virtual void deltaPlusDelta(const MotionDeltaType& _delta1, const MotionDeltaType& _delta2,
                                    MotionDeltaType& _delta1_plus_delta2) = 0;

        /** \brief Computes the delta-state that goes from one delta-state to another
         * \param _delta1 the initial delta
         * \param _delta2 the final delta
         * \param _delta2_minus_delta1 the delta-state. It has the format of a delta-state.
         *
         * This function implements the composition (-) so that _delta2_minus_delta1 = _delta2 (-) _delta1.
         */
        virtual void deltaMinusDelta(const MotionDeltaType& _delta1, const MotionDeltaType& _delta2,
                                     MotionDeltaType& _delta2_minus_delta1) = 0;

    protected:
        // Attributes
        WolfScalar dt_; ///< Time step
        size_t x_size_;    ///< The size of the state vector
        size_t delta_size_;   ///< the size of the integrated delta
        size_t data_size_; ///< the size of the incoming data

        CaptureMotion2<MotionDeltaType>* origin_ptr_;
        CaptureMotion2<MotionDeltaType>* last_ptr_;
        Eigen::VectorXs x_origin_; ///< state at the origin Capture
        Eigen::VectorXs x_last_; ///< state at the last Capture

    protected:
        // helpers to avoid allocation
        Eigen::VectorXs x_t_; ///< current state; state at time t
        Eigen::VectorXs delta_, delta_integrated_; ///< current delta and integrated delta
        Eigen::VectorXs data_; ///< current data

};

template<class MotionDeltaType>
inline ProcessorMotion2<MotionDeltaType>::ProcessorMotion2(ProcessorType _tp, WolfScalar _dt, size_t _state_size,
                                                           size_t _delta_size, size_t _data_size) :
        ProcessorBase(_tp), dt_(_dt), x_size_(_state_size), delta_size_(_delta_size), data_size_(_data_size), origin_ptr_(
                nullptr), last_ptr_(nullptr), x_origin_(_state_size), x_last_(_state_size), x_t_(_state_size), delta_(
                _delta_size), delta_integrated_(_delta_size), data_(_data_size)
{
    //
}

template<class MotionDeltaType>
inline ProcessorMotion2<MotionDeltaType>::~ProcessorMotion2()
{
    //
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::process(CaptureBase* _incoming_ptr)
{
    CaptureMotion2<MotionDeltaType>* incoming_ptr = (CaptureMotion2<MotionDeltaType>*)((((((((_incoming_ptr))))))));
    //    incoming_ptr->getBufferPtr()->setDt(dt_);
    integrate(incoming_ptr);
    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // TODO:
        // Make KeyFrame
        //        makeKeyFrame(incoming_ptr_);
        // Reset the Tracker
        //        reset();
    }
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::init(CaptureMotion2<MotionDeltaType>* _origin_ptr)
{
    //TODO: This fcn needs to change:
    // input: framebase: this is a Keyframe
    // create a new non-key Frame and make it last_
    // first delta in buffer must be zero
    // we do not need to extract data from any Capture
    origin_ptr_ = _origin_ptr;
    last_ptr_ = _origin_ptr;
    x_origin_ = x_last_ = _origin_ptr->getFramePtr()->getState();
    delta_ = delta_integrated_ = Eigen::VectorXs::Zero(delta_size_);
    getBufferPtr()->clear();
    getBufferPtr()->pushBack(_origin_ptr->getTimeStamp(), delta_integrated_);
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::update()
{
    x_origin_ = origin_ptr_->getFramePtr()->getState();
    state(x_last_);
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::reset(const TimeStamp& _ts)
{
    // TODO what to do?
    //cut the buffer in 2 parts at _ts
    // create a new Capture for the future
    // Create a
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::makeKeyFrame(const TimeStamp& _ts)
{
    //TODO: see how to adapt this code from ProcessorTracker::makeKeyFrame(void)
    //    // Create a new non-key Frame in the Trajectory with the incoming Capture
    //    getTop()->createFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
    //    getTop()->getLastFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
    //    // Make the last Capture's Frame a KeyFrame so that it gets into the solver
    //    last_ptr_->getFramePtr()->setKey();
}

template<class MotionDeltaType>
inline Eigen::VectorXs ProcessorMotion2<MotionDeltaType>::state(const TimeStamp& _ts)
{
    state(_ts, x_t_);
    return x_t_;
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::state(const TimeStamp& _ts, Eigen::VectorXs& _x)
{
    xPlusDelta(x_origin_, getBufferPtr()->getDelta(_ts), _x);
}

template<class MotionDeltaType>
inline const Eigen::VectorXs& ProcessorMotion2<MotionDeltaType>::state()
{
    state(x_last_);
    return x_last_;
}

template<class MotionDeltaType>
inline const void ProcessorMotion2<MotionDeltaType>::state(Eigen::VectorXs& _x)
{
    xPlusDelta(x_origin_, getBufferPtr()->getDelta(), _x);
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::integrate(CaptureMotion2<MotionDeltaType>* _incoming_ptr)
{
    // First get data and convert it to delta
    data2delta(_incoming_ptr->getData(), delta_);
    // then integrate
    deltaPlusDelta(getBufferPtr()->getDelta(), delta_, delta_integrated_);
    // then push it into buffer
    getBufferPtr()->pushBack(_incoming_ptr->getTimeStamp(), delta_integrated_);
}

template<class MotionDeltaType>
inline const typename CaptureMotion2<MotionDeltaType>::MotionBuffer* ProcessorMotion2<MotionDeltaType>::getBufferPtr() const
{
    return last_ptr_->getBufferPtr();
}

template<class MotionDeltaType>
inline typename CaptureMotion2<MotionDeltaType>::MotionBuffer* ProcessorMotion2<MotionDeltaType>::getBufferPtr()
{
    return last_ptr_->getBufferPtr();
}

#endif /* PROCESSOR_MOTION2_H_ */
