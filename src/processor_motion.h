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
 *
 * This data is, in the general case, in the reference frame of the sensor:
 *
 *   - Beware of the frame transformations Map to Robot, and Robot to Sensor, so that your produced
 *   motion Constraints are correctly expressed.
 *     - The robot state R is expressed in a global or 'Map' reference frame, named M.
 *     - The sensor frame S is expressed in the robot frame R.
 *     - The motion data data_ is expressed in the sensor frame S.
 *   - You can use three basic methods for this:
 *     - The trivial: make the sensor frame and the robot frame the same frame, that is, S = Id.
 *     - Transform incoming data from sensor frame to robot frame, and then integrate motion in robot frame.
 *     - Integrate motion directly in sensor frame, and transform to robot frame at the time of:
 *       - Publishing the robot state (see getCurrentState() and similar functions)
 *       - Creating Keyframes and Constraints (see emplaceConstraint() ).
 *
 * Should you need extra functionality for your derived types, you can overload these two methods,
 *
 *   -  preProcess() { }
 *   -  postProcess() { }
 *
 * which are called at the beginning and at the end of process(). See the doc of these functions for more info.
 */
 /* // TODO: JS: review these instructions from here onwards:
 *
 * while the integrated motion refers to the robot frame.
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
 *     \code    statePlusDelta(R_old, delta_R, R_new) \endcode
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
 *
 * // TODO: JS: review instructions up to here
 *
 */
class ProcessorMotion : public ProcessorBase
{
    private:
        enum
        {
            IDLE,
            RUNNING
        } status_;

    // This is the main public interface
    public:
        ProcessorMotion(const std::string& _type,
                        Size _state_size,
                        Size _delta_size,
                        Size _delta_cov_size,
                        Size _data_size,
                        Scalar _time_tolerance = 0.1,
                        Size _calib_size = 0);
        virtual ~ProcessorMotion();

        // Instructions to the processor:

        void process(CaptureBasePtr _incoming_ptr);
        virtual void resetDerived();

        // Queries to the processor:
        virtual bool isMotion();

        virtual bool voteForKeyFrame();

        /** \brief Fill a reference to the state integrated so far
         * \param _x the returned state vector
         */
        void getCurrentState(Eigen::VectorXs& _x);

        /** \brief Get a constant reference to the state integrated so far
         * \return the state vector
         */
        Eigen::VectorXs getCurrentState();

        /** \brief Fill a reference to the state integrated so far and its stamp
         * \param _x the returned state vector
         * \param _ts the returned stamp
         */
        void getCurrentStateAndStamp(Eigen::VectorXs& _x, TimeStamp& _ts);

        /** \brief Fill the state corresponding to the provided time-stamp
         * \param _ts the time stamp
         * \param _x the returned state
         */
        void getState(const TimeStamp& _ts, Eigen::VectorXs& _x);

        /** \brief Get the state corresponding to the provided time-stamp
         * \param _ts the time stamp
         * \return the state vector
         */
        Eigen::VectorXs getState(const TimeStamp& _ts);

        /** \brief Gets the delta preintegrated covariance from all integrations so far
         * \return the delta preintegrated covariance matrix
         */
        const Eigen::MatrixXs getCurrentDeltaPreintCov();

        /** \brief Provide the motion integrated so far
         * \return the integrated motion
         */
        Motion getMotion() const;

        /** \brief Provide the motion integrated until a given timestamp
         * \return the integrated motion
         */
        Motion getMotion(const TimeStamp& _ts) const;

        /** \brief Finds the capture that contains the closest previous motion of _ts
         * \return a pointer to the capture (if it exists) or a nullptr (otherwise)
         */
        CaptureMotionPtr findCaptureContainingTimeStamp(const TimeStamp& _ts) const;

        /** Set the origin of all motion for this processor
         * \param _origin_frame the keyframe to be the origin
         */
        void setOrigin(FrameBasePtr _origin_frame);

        /** Set the origin of all motion for this processor
         * \param _x_origin the state at the origin
         * \param _ts_origin origin timestamp.
         */
        FrameBasePtr setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin);

        virtual bool keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol);

        MotionBuffer& getBuffer();
        const MotionBuffer& getBuffer() const;


        // Helper functions:
    protected:

        void updateDt();
        void integrateOneStep();
        void splitBuffer(const TimeStamp& _t_split, MotionBuffer& _oldest_part);
        void reintegrateBuffer(CaptureMotionPtr _capture_ptr);

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

        /**
         * @brief Get the incoming CaptureBasePtr and returns a CaptureMotionPtr out of it.
         * If not overloaded, the base class calls
         * std::static_pointer_cast<CaptureMotion>(_incoming_ptr)
         * @return CaptureMotionPtr.
         */
        virtual CaptureMotionPtr getIncomingCaptureMotion(CaptureBasePtr& _incoming_ptr);


        // These are the pure virtual functions doing the mathematics
    protected:


        /** \brief convert raw CaptureMotion data to the delta-state format
         *
         * This function accepts raw data and time step dt,
         * and computes the value of the delta-state and its covariance. Note that these values are
         * held by the members delta_ and delta_cov_.
         *
         * @param _data measured motion data
         * @param _data_cov covariance
         * @param _dt time step
         * @param _delta computed delta
         * @param _delta_cov covariance
         * @param _calib current state of the calibrated parameters
         * @param _jacobian_calib Jacobian of the delta wrt calib
         *
         * Rationale:
         *
         * The delta-state format must be compatible for integration using
         * the composition functions doing the math in this class: statePlusDelta() and deltaPlusDelta().
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
         *  - If `_data` is an increment:
         *
         *         delta_     = _data;
         *         delta_cov_ = _data_cov
         *  - If `_data` is a velocity:
         *
         *         delta_     = _data * _dt
         *         delta_cov_ = _data_cov * _dt.
         *
         *  However, other more complicated relations are possible.
         *  In general, we'll have a nonlinear
         *  function relating `delta_` to `_data` and `_dt`, as follows:
         *
         *      delta_ = f ( _data, _dt)
         *
         *  The delta covariance is obtained by classical uncertainty propagation of the data covariance,
         *  that is, through the Jacobians of `f()`,
         *
         *      delta_cov_ = F_data * _data_cov * F_data.transpose
         *
         *  where `F_data = d_f/d_data` is the Jacobian of `f()`.
         */
        virtual void computeCurrentDelta(const Eigen::VectorXs& _data,
                                const Eigen::MatrixXs& _data_cov,
                                const Eigen::VectorXs& _calib,
                                const Scalar _dt,
                                Eigen::VectorXs& _delta,
                                Eigen::MatrixXs& _delta_cov,
                                Eigen::MatrixXs& _jacobian_calib) = 0;

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
        virtual void statePlusDelta(const Eigen::VectorXs& _x,
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


        /** \brief Interpolate motion to an intermediate time-stamp
         *
         * @param _ref    The motion reference
         * @param _second The second motion. It is modified by the function (see documentation below).
         * @param _ts     The intermediate time stamp: it must be bounded by  `_ref.ts_ <= _ts <= _second.ts_`.
         * @return        The interpolated motion (see documentation below).
         *
         * This function interpolates a motion between two existing motions.
         */
        virtual Motion interpolate(const Motion& _ref, Motion& _second, TimeStamp& _ts) = 0;

        /** \brief create a CaptureMotion and add it to a Frame
         * \param _ts time stamp
         * \param _sensor Sensor that produced the Capture
         * \param _data a vector of motion data
         * \param _sata_cov covariances matrix of the motion data data
         * \param _frame_own frame owning the Capture to create
         * \param _frame_origin frame acting as the origin of motions for the MorionBuffer of the created MotionCapture
         */
        CaptureMotionPtr emplaceCapture(const TimeStamp& _ts,
                                        const SensorBasePtr& _sensor,
                                        const VectorXs& _data,
                                        const MatrixXs& _data_cov,
                                        const FrameBasePtr& _frame_own,
                                        const FrameBasePtr& _frame_origin);

        virtual CaptureMotionPtr createCapture(const TimeStamp& _ts,
                                               const SensorBasePtr& _sensor,
                                               const VectorXs& _data,
                                               const MatrixXs& _data_cov,
                                               const FrameBasePtr& _frame_origin) = 0;

        /** \brief create a feature corresponding to given capture and add the feature to this capture
         * \param _capture_motion: the parent capture
         */
        FeatureBasePtr emplaceFeature(CaptureMotionPtr _capture_own);
        virtual FeatureBasePtr createFeature(CaptureMotionPtr _capture_own) = 0;

        /** \brief create a constraint and link it in the wolf tree
         * \param _feature_motion: the parent feature
         * \param _frame_origin: the frame constrained by this motion constraint
         */
        virtual ConstraintBasePtr emplaceConstraint(FeatureBasePtr _feature_motion, CaptureBasePtr _capture_origin) = 0;

        Motion motionZero(const TimeStamp& _ts);
        CaptureMotionPtr getCaptureMotionContainingTimeStamp(const TimeStamp& _ts);

    public:
        virtual CaptureBasePtr getOriginPtr();
        virtual CaptureMotionPtr getLastPtr();
        virtual CaptureMotionPtr getIncomingPtr();


    protected:
        // Attributes
        Size x_size_;           ///< The size of the state vector
        Size data_size_;        ///< the size of the incoming data
        Size delta_size_;       ///< the size of the deltas
        Size delta_cov_size_;   ///< the size of the delta covariances matrix
        Size calib_size_;       ///< size of the extra parameters (TBD in derived classes)
        CaptureBasePtr origin_ptr_;
        CaptureMotionPtr last_ptr_;
        CaptureMotionPtr incoming_ptr_;

    protected:
        // helpers to avoid allocation
        Scalar dt_;                             ///< Time step
        Eigen::VectorXs x_;                     ///< current state
        Eigen::VectorXs data_;                  ///< current data
        Eigen::VectorXs delta_;                 ///< current delta
        Eigen::MatrixXs delta_cov_;             ///< current delta covariance
        Eigen::VectorXs delta_integrated_;      ///< integrated delta
        Eigen::MatrixXs delta_integrated_cov_;  ///< integrated delta covariance
        Eigen::VectorXs calib_;                 ///< calibration vector
        Eigen::MatrixXs jacobian_delta_preint_; ///< jacobian of delta composition w.r.t previous delta integrated
        Eigen::MatrixXs jacobian_delta_;        ///< jacobian of delta composition w.r.t current delta
        Eigen::MatrixXs jacobian_calib_;        ///< jacobian of delta preintegration wrt calibration params
        Eigen::MatrixXs jacobian_delta_calib_;  ///< jacobian of delta wrt calib params

    private:
        wolf::TimeStamp getCurrentTimeStamp();
};

}

#include "frame_base.h"

namespace wolf{

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

inline Eigen::VectorXs ProcessorMotion::getState(const TimeStamp& _ts)
{
    getState(_ts, x_);
    return x_;
}

inline void ProcessorMotion::getState(const TimeStamp& _ts, Eigen::VectorXs& _x)
{
    CaptureMotionPtr capture_motion;

    if (_ts >= origin_ptr_->getTimeStamp())
        // timestamp found in the current processor buffer
        capture_motion = last_ptr_;
    else
        // We need to search in previous keyframes for the capture containing a motion buffer with the queried time stamp
        capture_motion = getCaptureMotionContainingTimeStamp(_ts);

    if (capture_motion)
    {
        // We found a CaptureMotion whose buffer contains the time stamp
        VectorXs state_0 = capture_motion->getOriginFramePtr()->getState();
        VectorXs delta   = capture_motion->getDelta(origin_ptr_->getCalibration(), _ts);
        Scalar   dt      = _ts - capture_motion->getBuffer().get().front().ts_;

        statePlusDelta(state_0, delta, dt, _x);
    }
    else
        // We could not find any CaptureMotion for the time stamp requested
        std::runtime_error("Could not find any Capture for the time stamp requested");
}

inline wolf::TimeStamp ProcessorMotion::getCurrentTimeStamp()
{
    return getBuffer().get().back().ts_;
}

inline Eigen::VectorXs ProcessorMotion::getCurrentState()
{
    getCurrentState(x_);
    return x_;
}

inline void ProcessorMotion::getCurrentState(Eigen::VectorXs& _x)
{
    Scalar Dt = getCurrentTimeStamp() - origin_ptr_->getTimeStamp();
    statePlusDelta(origin_ptr_->getFramePtr()->getState(), last_ptr_->getDelta(origin_ptr_->getCalibration()), Dt, _x);
}

inline void ProcessorMotion::getCurrentStateAndStamp(Eigen::VectorXs& _x, TimeStamp& _ts)
{
    getCurrentState(_x);
    _ts = getCurrentTimeStamp();
}

inline const Eigen::MatrixXs ProcessorMotion::getCurrentDeltaPreintCov()
{
    return getBuffer().get().back().delta_integr_cov_;
}

inline Motion ProcessorMotion::getMotion() const
{
    return getBuffer().get().back();
}

inline Motion ProcessorMotion::getMotion(const TimeStamp& _ts) const
{
    auto capture_ptr = findCaptureContainingTimeStamp(_ts);
    assert(capture_ptr != nullptr && "ProcessorMotion::getMotion: timestamp older than first motion");

    return capture_ptr->getBuffer().getMotion(_ts);
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

inline Motion ProcessorMotion::motionZero(const TimeStamp& _ts)
{
    return Motion(_ts,
                  VectorXs::Zero(data_size_), // data
                  Eigen::MatrixXs::Zero(data_size_, data_size_), // Cov data
                  deltaZero(),
                  Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_), // Cov delta
                  deltaZero(),
                  Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_), // Cov delta_integr
                  Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_), // Jac delta
                  Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_), // Jac delta_integr
                  Eigen::MatrixXs::Zero(delta_cov_size_, calib_size_)      // Jac calib
    );
}

inline CaptureBasePtr ProcessorMotion::getOriginPtr()
{
    return origin_ptr_;
}

inline CaptureMotionPtr ProcessorMotion::getLastPtr()
{
    return last_ptr_;
}

inline CaptureMotionPtr ProcessorMotion::getIncomingPtr()
{
    return incoming_ptr_;
}


} // namespace wolf

#endif /* PROCESSOR_MOTION2_H_ */
