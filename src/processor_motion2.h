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

class ProcessorMotion2 : public ProcessorBase
{

        // This is the main public interface
    public:
        ProcessorMotion2(ProcessorType _tp, WolfScalar _dt, size_t _state_size, size_t _delta_size, size_t _data_size,
                         size_t _noise_size);
        virtual ~ProcessorMotion2();

        // Instructions to the processor:

        virtual void process(CaptureBase* _incoming_ptr);
        void init(CaptureMotion2* _origin_ptr);
        void update();
        void reset(const TimeStamp& _t);
        void makeKeyFrame(const TimeStamp& _t);

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
        void state(const TimeStamp& _t, Eigen::VectorXs& _x);
        /** \brief Gets the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \return the state vector
         */
        Eigen::VectorXs state(const TimeStamp& _t);
        /** \brief Provides the delta-state integrated so far
         * \return a reference to the integrated delta state
         */
        Eigen::VectorXs& deltaState(){return getBufferPtr()->getDelta();}
        /** \brief Provides the delta-state between two time-stamps
         * \param _t1 initial time
         * \param _t2 final time
         * \param _Delta the integrated delta-state between _t1 and _t2
         */
        void deltaState(const TimeStamp& _t1, const TimeStamp& _t2, Eigen::VectorXs& _Delta);
        /** Composes the deltas in two pre-integrated Captures
         * \param _cap1_ptr pointer to the first Capture
         * \param _cap2_ptr pointer to the second Capture. This is local wrt. the first Capture.
         * \param _delta1_plus_delta2 the concatenation of the deltas of Captures 1 and 2.
         */
        void sumDeltas(CaptureMotion2* _cap1_ptr, CaptureMotion2* _cap2_ptr, Eigen::VectorXs& _delta1_plus_delta2);

        // Helper functions:
    protected:

        void integrate(CaptureMotion2* _incoming_ptr); ///< Integrate the last received IMU data

        CaptureMotion2::MotionBuffer* getBufferPtr();

        /** \brief Extract data from a derived capture.
         * \param _capture_ptr pointer to the Capture we want to extract data from.
         * This function:
         *  - accesses the incoming Capture
         *  - Fills in the ts_ field
         *  - Fills in the data_ field
         *  - Fills in the delta_ field by converting it from the data_ field, using data2delta().
         */
         void extractData(const CaptureMotion2* _capture_ptr);

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
         virtual void data2delta() = 0;

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

        /** \brief Computes the delta-state the goes from one delta-state to another
         * \param _delta1 the initial delta
         * \param _delta2 the final delta
         * \param _delta2_minus_delta1 the delta-state. It has the format of a delta-state.
         *
         * This function implements the composition (-) so that _delta2_minus_delta1 = _delta2 (-) _delta1.
         */
        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                     Eigen::VectorXs& _delta2_minus_delta1) = 0;

    protected:
        // Attributes
        WolfScalar dt_; ///< Time step --- assumed constant
        size_t x_size_;    ///< The size of the state vector
        size_t delta_size_;   ///< the size of the delta integrator
        size_t data_size_; ///< the size of the incoming data
        size_t noise_size_; ///< the size of the noise vector

        CaptureMotion2* origin_ptr_;
        CaptureMotion2* last_ptr_;
        Eigen::VectorXs x_origin_; ///< state at the origin

    protected:
        // helpers to avoid allocation
        Eigen::VectorXs x_, x_t_; ///< current state; state at time t
        TimeStamp ts_; ///< current time stamp
        Eigen::VectorXs delta_, delta_integrated_; ///< current delta and integrated delta
        Eigen::VectorXs data_; ///< current data

    private:
        void updateOriginState();
};

inline void ProcessorMotion2::update()
{
    updateOriginState();
    state(x_);
}

inline void ProcessorMotion2::reset(const TimeStamp& _t)
{
    // TODO what to do?
}

inline void ProcessorMotion2::makeKeyFrame(const TimeStamp& _t)
{
    //TODO
}

inline Eigen::VectorXs ProcessorMotion2::state(const TimeStamp& _t)
{
    state(_t, x_t_);
    return x_t_;
}

inline void ProcessorMotion2::state(const TimeStamp& _t, Eigen::VectorXs& _x)
{
    xPlusDelta(x_origin_, getBufferPtr()->getDelta(_t), _x);
}

inline const Eigen::VectorXs& ProcessorMotion2::state()
{
    state(x_);
    return x_;
}

inline const void ProcessorMotion2::state(Eigen::VectorXs& _x)
{
    xPlusDelta(x_origin_, getBufferPtr()->getDelta(), _x);
}

inline void ProcessorMotion2::deltaState(const TimeStamp& _t1, const TimeStamp& _t2, Eigen::VectorXs& _Delta)
{
    deltaMinusDelta(getBufferPtr()->getDelta(_t2), getBufferPtr()->getDelta(_t2), _Delta);
}

inline void ProcessorMotion2::sumDeltas(CaptureMotion2* _cap1_ptr, CaptureMotion2* _cap2_ptr,
                                        Eigen::VectorXs& _delta1_plus_delta2)
{
    deltaPlusDelta(_cap1_ptr->getDelta(), _cap2_ptr->getDelta(), _delta1_plus_delta2);
}

inline void ProcessorMotion2::integrate(CaptureMotion2* _incoming_ptr)
{
    // First get data and push it into buffer
    extractData(_incoming_ptr);
    deltaPlusDelta(getBufferPtr()->getDelta(), delta_, delta_integrated_);
    getBufferPtr()->pushBack(ts_,delta_integrated_);
}

inline CaptureMotion2::MotionBuffer* ProcessorMotion2::getBufferPtr()
{
    return last_ptr_->getBufferPtr();
}

inline void ProcessorMotion2::extractData(const CaptureMotion2* _capture_ptr)
{
    ts_ = _capture_ptr->getTimeStamp();
    data_ = _capture_ptr->getData();
    data2delta();
}

inline void ProcessorMotion2::updateOriginState()
{
    x_origin_ = origin_ptr_->getFramePtr()->getState();
}


#endif /* PROCESSOR_MOTION2_H_ */
