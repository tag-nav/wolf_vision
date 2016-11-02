/**
 * \file processor_motion.h
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_MOTION_H_
#define PROCESSOR_MOTION_H_

// Wolf
#include "capture_motion.h"
#include "processor_base.h"
#include "time_stamp.h"

// std
#include <iomanip>

namespace wolf
{

/** \brief class for Motion processors
 *
 * This processor integrates motion data into vehicle states.
 *
 * The motion data is provided by the sensor owning this processor.
 * This data is, in the general case, in the reference frame of the sensor, while the integrated motion refers to the robot frame.
 *
 * The reference frame convention used are specified as follows.
 *   - The robot state R is expressed in a global or 'Map' reference frame, named M.
 *   - The sensor frame S is expressed in the robot frame R.
 *   - The motion data data_ is expressed in the sensor frame S.
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
 * Should you need extra functionality for your derived types, you can overload these two methods,
 *
 *   -  preProcess() { }
 *   -  postProcess() { }
 *
 * which are called at the beginning and at the end of process(). See the doc of these functions for more info.
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
    public:
        typedef std::shared_ptr<ProcessorMotion> Ptr;
        typedef std::weak_ptr<ProcessorMotion> WPtr;

        // This is the main public interface
    public:
        ProcessorMotion(ProcessorType _tp, const std::string& _type, Size _state_size, Size _delta_size, Size _delta_cov_size, Size _data_size, const Scalar& _time_tolerance = 0.1);
        virtual ~ProcessorMotion();

        // Instructions to the processor:

        virtual void process(CaptureBasePtr _incoming_ptr);
        virtual void resetDerived();

        // Queries to the processor:

        virtual bool voteForKeyFrame();

        /** \brief Fills a reference to the state integrated so far
         * \param _x the returned state vector
         */
        void getCurrentState(Eigen::VectorXs& _x);

        /** \brief Fills a reference to the state integrated so far and its stamp
         * \param _x the returned state vector
         * \param _ts the returned stamp
         */
        void getCurrentState(Eigen::VectorXs& _x, TimeStamp& _ts);

        /** \brief Gets a constant reference to the state integrated so far
         * \return the state vector
         */
        const Eigen::VectorXs& getCurrentState();

        /** \brief Gets a constant reference to the state integrated so far and its stamp
         * \param _ts the returned stamp
         * return the state vector
         */
        const Eigen::VectorXs& getCurrentState(TimeStamp& _ts);

        /** \brief Fills the state corresponding to the provided time-stamp
         * \param _ts the time stamp
         * \param _x the returned state
         */
        void getState(const TimeStamp& _ts, Eigen::VectorXs& _x);

        /** \brief Gets the state corresponding to the provided time-stamp
         * \param _ts the time stamp
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

        /** \brief Finds the capture that contains the closest previous motion of _ts
         * \return a pointer to the capture (if it exist) or a nullptr (otherwise)
         */
        CaptureMotion::Ptr findCaptureContainingTimeStamp(const TimeStamp& _ts) const;

        /** Composes the deltas in two pre-integrated Captures
         * \param _cap1_ptr pointer to the first Capture
         * \param _cap2_ptr pointer to the second Capture. This is local wrt. the first Capture.
         * \param _delta1_plus_delta2 the concatenation of the deltas of Captures 1 and 2.
         */
        void sumDeltas(CaptureMotion* _cap1_ptr,
                       CaptureMotion* _cap2_ptr,
                       Eigen::VectorXs& _delta1_plus_delta2);

        /** Set the origin of all motion for this processor
         * \param _origin_frame the key frame to be the origin
         */
        void setOrigin(FrameBasePtr _origin_frame);

        /** Set the origin of all motion for this processor
         * \param _x_origin the state at the origin
         * \param _ts_origin origin timestamp.
         */
        void setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin);

        virtual bool keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol);

        MotionBuffer& getBuffer();
        const MotionBuffer& getBuffer() const;

        virtual bool isMotion();

        // Helper functions:
    protected:

        void splitBuffer(const TimeStamp& _t_split, MotionBuffer& _oldest_part);

    protected:
        void updateDt();
        void integrate();
        void reintegrate(CaptureMotion::Ptr _capture_ptr);

        /** Pre-process incoming Capture
         *
         * This is called by process() just after assigning incoming_ptr_ to a valid Capture.
         *
         * Overload this function to prepare stuff on derived classes.
         *
         * Typical uses of prePrecess() are:
         *   - casting base types to derived types
         *   - initializing counters, flags, or any derived variables
         *   - initializing algorithms needed for processing the derived data
         */
        virtual void preProcess() { };

        /** Post-process
         *
         * This is called by process() after finishing the processing algorithm.
         *
         * Overload this function to post-process stuff on derived classes.
         *
         * Typical uses of postPrecess() are:
         *   - resetting and/or clearing variables and/or algorithms at the end of processing
         *   - drawing / printing / logging the results of the processing
         */
        virtual void postProcess() { };


        // These are the pure virtual functions doing the mathematics
    protected:

        /** \brief convert raw CaptureMotion data to the delta-state format
         *
         * This function accepts raw data and time step dt,
         * and computes the value of the delta-state and its covariance. Note that these values are
         * held by the members delta_ and delta_cov_.
         *
         * \param _data the raw motion data
         * \param _data_cov the raw motion data covariance
         * \param _dt the time step (not always needed)
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
         * velocity, or a higher derivative, while the delta is in the form of an increment.
         * In such cases, converting from data to delta-state needs integrating
         * the data over the period dt.
         *
         * Two trivial implementations would establish:
         *  - If _data is an increment:
         *         _delta = _data;
         *         _delta_cov = _data_cov
         *  - If _data is a velocity:
         *         _delta = _data * _dt
         *         _delta_cov = _data_cov * _dt.
         *
         *  However, other more complicated relations are possible.
         */
        virtual void data2delta(const Eigen::VectorXs& _data,
                                const Eigen::MatrixXs& _data_cov,
                                const Scalar _dt) = 0;

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _dt2 the second delta-state's time delta
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2.
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                                    const Eigen::VectorXs& _delta2,
                                    const Scalar _dt2,
                                    Eigen::VectorXs& _delta1_plus_delta2) = 0;

        /** \brief composes a delta-state on top of another delta-state, and computes the Jacobians
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _dt2 the second delta-state's time delta
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         * \param _jacobian1 the jacobian of the composition w.r.t. _delta1.
         * \param _jacobian2 the jacobian of the composition w.r.t. _delta2.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2 and its jacobians.
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                                    const Eigen::VectorXs& _delta2,
                                    const Scalar _dt2,
                                    Eigen::VectorXs& _delta1_plus_delta2,
                                    Eigen::MatrixXs& _jacobian1,
                                    Eigen::MatrixXs& _jacobian2) = 0;

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         * \param _dt the time interval spanned by the Delta
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x,
                                const Eigen::VectorXs& _delta,
                                const Scalar _dt,
                                Eigen::VectorXs& _x_plus_delta) = 0;


        /** \brief Delta zero
         * \return a delta state equivalent to the null motion.
         *
         * Examples (see documentation of the the class for info on Eigen::VectorXs):
         *   - 2D odometry: a 3-vector with all zeros, e.g. VectorXs::Zero(3)
         *   - 3D odometry: delta type is a PQ vector: 7-vector with [0,0,0, 0,0,0,1]
         *   - IMU: PQVBB 10-vector with [0,0,0, 0,0,0,1, 0,0,0] // No biases in the delta !!
         */
        virtual Eigen::VectorXs deltaZero() const = 0;

        virtual Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts) = 0;

        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_motion, FrameBasePtr _frame_origin) = 0;

        Motion motionZero(const TimeStamp& _ts);

    public:
        virtual CaptureBasePtr getOriginPtr();
        virtual CaptureMotion::Ptr getLastPtr();
        virtual CaptureMotion::Ptr getIncomingPtr();


    protected:
        // Attributes
        Size x_size_;           ///< The size of the state vector
        Size delta_size_;       ///< the size of the deltas
        Size delta_cov_size_;   ///< the size of the delta covariances matrix
        Size data_size_;        ///< the size of the incoming data
        CaptureBasePtr origin_ptr_; // TODO check pointer type
        CaptureMotion::Ptr last_ptr_; // TODO check pointer type
        CaptureMotion::Ptr incoming_ptr_; // TODO check pointer type

    protected:
        // helpers to avoid allocation
        Scalar dt_;                             ///< Time step
        Eigen::VectorXs x_;                     ///< current state
        Eigen::VectorXs delta_;                 ///< current delta
        Eigen::MatrixXs delta_cov_;             ///< current delta covariance
        Eigen::VectorXs delta_integrated_;      ///< integrated delta
        Eigen::MatrixXs delta_integrated_cov_;  ///< integrated delta covariance
        Eigen::VectorXs data_;                  ///< current data
        Eigen::MatrixXs jacobian_delta_preint_; ///< jacobian of delta composition w.r.t previous delta integrated
        Eigen::MatrixXs jacobian_delta_;        ///< jacobian of delta composition w.r.t current delta

};

}

#include "frame_base.h"

namespace wolf{

inline void ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin)
{
    // make a new key frame
    FrameBasePtr key_frame_ptr = getProblem()->createFrame(KEY_FRAME, _x_origin, _ts_origin);
    // set the key frame as origin
    setOrigin(key_frame_ptr);
}

inline void ProcessorMotion::splitBuffer(const TimeStamp& _t_split, MotionBuffer& _oldest_part)
{
    last_ptr_->getBuffer().split(_t_split, _oldest_part);
}

inline void ProcessorMotion::resetDerived()
{
    // Blank function, to be implemented in derived classes
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
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBuffer().getDelta(_ts), _ts - origin_ptr_->getTimeStamp(), _x);
}

inline const Eigen::VectorXs& ProcessorMotion::getCurrentState()
{
    getCurrentState(x_);
    return x_;
}

inline const Eigen::VectorXs& ProcessorMotion::getCurrentState(TimeStamp& _ts)
{
    getCurrentState(x_, _ts);
    return x_;
}

inline void ProcessorMotion::getCurrentState(Eigen::VectorXs& _x)
{
    Scalar Dt = getBuffer().get().back().ts_ - origin_ptr_->getTimeStamp();
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBuffer().get().back().delta_integr_, Dt, _x);
}

inline void ProcessorMotion::getCurrentState(Eigen::VectorXs& _x, TimeStamp& _ts)
{
    getCurrentState(_x);
    _ts = getBuffer().get().back().ts_;
}

inline const Motion& ProcessorMotion::getMotion() const
{
    return getBuffer().get().back();
}

inline const Motion& ProcessorMotion::getMotion(const TimeStamp& _ts) const
{
    auto capture_ptr = findCaptureContainingTimeStamp(_ts);
    assert(capture_ptr != nullptr && "ProcessorMotion::getMotion: timestamp older than first motion");

    return capture_ptr->getBuffer().getMotion(_ts);
}

inline void ProcessorMotion::getMotion(Motion& _motion) const
{
    _motion = getBuffer().get().back();
}

inline void ProcessorMotion::getMotion(const TimeStamp& _ts, Motion& _motion) const
{
    auto capture_ptr = findCaptureContainingTimeStamp(_ts);
    assert(capture_ptr != nullptr && "ProcessorMotion::getMotion: timestamp older than first motion");

    capture_ptr->getBuffer().getMotion(_ts, _motion);
}

inline bool ProcessorMotion::isMotion()
{
    return true;
}

inline void ProcessorMotion::updateDt()
{
    dt_ = incoming_ptr_->getTimeStamp() - getBuffer().get().back().ts_;
}

inline const MotionBuffer& ProcessorMotion::getBuffer() const
{
    return last_ptr_->getBuffer();
}

inline MotionBuffer& ProcessorMotion::getBuffer()
{
    return last_ptr_->getBuffer();
}

inline void ProcessorMotion::sumDeltas(CaptureMotion* _cap1_ptr, CaptureMotion* _cap2_ptr,
                                       Eigen::VectorXs& _delta1_plus_delta2)
{
    Scalar dt = _cap2_ptr->getTimeStamp() - _cap1_ptr->getTimeStamp();
    deltaPlusDelta(_cap1_ptr->getDelta(),_cap2_ptr->getDelta(), dt, _delta1_plus_delta2);
}

inline Motion ProcessorMotion::motionZero(const TimeStamp& _ts)
{
    return Motion(
            {_ts,
             deltaZero(),
             deltaZero(),
             Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_),
             Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_)
             });
}

inline CaptureBasePtr ProcessorMotion::getOriginPtr()
{
    return origin_ptr_;
}

inline CaptureMotion::Ptr ProcessorMotion::getLastPtr()
{
    return last_ptr_;
}

inline CaptureMotion::Ptr ProcessorMotion::getIncomingPtr()
{
    return incoming_ptr_;
}


} // namespace wolf

#endif /* PROCESSOR_MOTION2_H_ */
