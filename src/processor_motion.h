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
        ProcessorMotion(const std::string& _type, Size _state_size, Size _delta_size, Size _delta_cov_size, Size _data_size, const Scalar& _time_tolerance = 0.1);
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
        void sumDeltas(CaptureMotion::Ptr _cap1_ptr,
                       CaptureMotion::Ptr _cap2_ptr,
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
         * the composition functions doing the math in this class: xPlusDelta() and deltaPlusDelta().
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


        /** \brief Interpolate motion to an intermediate time-stamp
         *
         * @param _ref    The motion reference
         * @param _second The second motion. It is modified by the function (see documentation below).
         * @param _ts     The intermediate time stamp: it must be bounded by  `_ref.ts_ <= _ts <= _second.ts_`.
         * @return        The interpolated motion (see documentation below).
         *
         * This function interpolates a motion between two existing motions.
         *
         * In particular, given a reference motion `R=_ref` at time `t_R`,
         * a final motion `F=_second` at time `t_F`, and an interpolation time `t_I=_ts`,
         * we search for the two interpolate motions `I` and `S` such that:
         *
         *   - `I` is the motion between `t_R` and `t_I`
         *   - `S` is the motion between `t_I` and `t_F`
         *
         * ### Rationale
         *
         * Let us name
         *
         * ```
         *     R = _ref      // initial motion where interpolation starts
         *     F = _second   // final motion where interpolation ends
         * ```
         * and let us define
         *
         * ```
         *     t_R            // timestamp at R
         *     t_F            // timestamp at F
         *     t_I = _ts      // time stamp where interpolation is queried.
         * ```
         * We can introduce the results of the interpolation as
         *
         * ```
         *     I = motion_interpolated // from t_R to t_I
         *     S = motion_second       // from t_I to t_F
         * ```
         * The Motion structure in wolf has the following members (among others; see below):
         *
         * ```
         *     ts_           // time stamp
         *     delta_        // relative motion between the previous motion and this one. It might be seen as a local motion.
         *     delta_integr_ // integration of relative deltas, since some origin. It might be seen as a globally defined motion.
         * ```
         * In this documentation, we differentiate these deltas with lower-case d and upper-case D:
         *
         * ```
         *     d = any_motion.delta_            // local delta, from previous to this
         *     D = any_motion.delta_integr_     // global Delta, from origin to this
         * ```
         * so that `D_(i+1) = D_(i) (+) d_(i+1)`, where (i) is in {R, I, S} and (i+1) is in {I, S, F}
         *
         * NOTE: the operator (+) is implemented as `deltaPlusDelta()` in each class deriving from this.
         *
         * This is a schematic sketch of the situation (see more explanations below),
         * before and after calling `interpolate()`:
         *
         *
         *     BEFORE             _ref         _ts       _second        variable names
         *        ------+-----------+-----------+-----------+----->     time scale
         *            origin        R                       F           motion short names
         *           t_origin      t_R         t_I         t_F          time stamps
         *                          0          tau          1           interp. factor
         *              +----D_R----+----------d_F----------+           D_R (+) d_F
         *              +----------------D_F----------------+           D_F = D_R (+) d_F
         *
         *     AFTER              _ref        return     _second        variable names and return value
         *        ------+-----------+-----------+-----------+----->     time scale
         *                          R           I           S           motion short names
         *              +----D_R----+----d_I----+----d_S----+           D_R (+) d_I (+) d_S
         *              +----------D_I----------+----d_S----+           D_I (+) d_S
         *              +----------------D_S----------------+           D_S = D_I (+) d_S = D_R (+) d_I (+) d_S
         *
         * where '`origin`' exists somewhere, but it is irrelevant for the operation of the interpolation.
         * According to the schematic, and assuming a generic composition operator (+), the motion composition satisfies
         *
         * ```
         *   d_I (+) d_S = d_F      (1)
         * ```
         * from where `d_I` and `d_S` are first derived. Then, the integrated deltas satisfy
         *
         * ```
         *   D_I = D_R (+) d_I      (2)
         *   D_S = D_F              (3)
         * ```
         * from where `D_I` and `D_S` can be derived.
         *
         * ### Interpolating `d_I`
         *
         * Equation (1) has two unknowns, `d_I` and `d_S`.
         * To solve, we first need to consider the interpolation time,
         * `t_I`, that determines `d_I`.
         *
         * In general, we do not have information about the particular trajectory
         * taken between `R = _ref` and `F = _second`.
         * Therefore, we consider a linear interpolation.
         * The linear interpolation factor `tau` is defined from the time stamps,
         *
         * ```
         *     tau = (t_I - t_R) / (t_F - t_R)
         * ```
         * such that for `tau=0` we are at `R`, and for `tau=1` we are at `F`.
         *
         * Conceptually, we want an interpolation such that the local motion 'd' takes the fraction,
         *
         * ```
         *   d_I = tau (*) d_F       // the fraction of the local delta
         * ```
         * where again the operator (*) needs to be defined properly.
         *
         * ### Defining the operators (*) and (+)
         *
         * We often break down these 'd' and 'D' deltas into chunks of data, e.g.
         *
         *     dp = delta of position
         *     Dp = delta integrated of position
         *     dq = delta of quaternion
         *     Da = delta integrated of orientation angle
         *     etc...
         *
         * which makes it easier to define the operators (+) and (*).
         * In effect, defining (*) is now easy:
         *
         *   - for linear magnitudes, (*) is the regular product *:
         * ```
         *         dv_I = tau * dv_F
         * ```
         *   - for simple angles, (*) is the regular product:
         * ```
         *         da_I = tau * da_F
         * ```
         *   - for quaternions, we use slerp():
         * ```
         *     dq_I = 1.slerp(tau, dq_F) // '1' is the unit quaternion
         * ```
         * As for the operator (+), we simply make use of `deltaPlusDelta()`, which is implemented in each derived class.
         *
         * ### Computing `d_S`
         *
         * Applying (1), we can define
         *
         *     d_S = d_F (-) d_I
         *
         * where the operator (-) might be implemented explicitly,
         * or through a `deltaMinusDelta()` defined akin to `deltaPlusDelta()`.
         *
         * By now, this `deltaMinusDelta()` is not enforced by this class as an abstract method,
         * and its implementation in derived classes is up to the user.
         *
         * For simple pose increments, we can use a local implementation:
         *
         *   - for 2D
         * ```
         *     dp_S = dR_I.tr * (1-tau)*dp_F      // dR is the rotation matrix of the angle delta 'da'; 'tr' is transposed
         *     da_S = dR_I.tr * (1-tau)*da_F
         * ```
         *   - for 3D
         * ```
         *     dp_S = dq_I.conj * (1-tau)*dp_F    // dq is a quaternion; 'conj' is the conjugate quaternion.
         *     dq_S = dq_I.conj * dq_F
         * ```
         *
         * Please refer to the examples at the end of this documentation for the computation of `d_S`.
         *
         * ### Computing `D_I`
         *
         * Conceptually, the global motion 'D' is interpolated, that is:
         * ```
         *     D_I = (1-tau) (*) D_R (+) tau (*) D_F  // the interpolation of the global Delta
         * ```
         * However, we better make use of (2) and write
         * ```
         *     D_I = D_R (+) d_I
         *         = deltaPlusDelta(D_R, d_I)         // This form provides an easy implementation.
         * ```
         * ### Covariances
         *
         * The Motion structure adds local and global covariances, that we rename as,
         *
         *     dC: delta_cov_
         *     DC: delta_integr_cov_
         *
         * and which are integrated as follows
         * ```
         *     dC_I = tau * dC_F
         *     DC_I = (1-tau) * DC_R + tau * dC_F = DC_R + dC_I
         * ```
         * and
         * ```
         *     dC_S = (1-tau) * dC_F
         *     DC_S = DC_F
         * ```
         * ### Examples
         *
         * #### Example 1: For 2D poses
         *
         * ```
         *     t_I  = _ts                         // time stamp of the interpolated motion
         *     tau = (t_I - t_R) / (t_F - t_R)    // interpolation factor
         *
         *     dp_I = tau*dp_F                    // dp is a 2-vector
         *     da_I = tau*da_F                    // da is an angle, for 2D poses
         *
         *     D_I  = deltaPlusDelta(D_R, d_I)
         *
         *     dp_S = dR_I.tr * (1-tau)*dp_F      // dR.tr is the transposed rotation matrix corresponding to 'da' above
         *     da_S = dR_I.tr * (1-tau)*da_F
         *
         *     D_S  = D_F
         * ```
         * #### Example 2: For 3D poses
         *
         * Orientation is in quaternion form, which is the best for interpolation using `slerp()` :
         * ```
         *     t_I  = _ts                         // time stamp of the interpolated motion
         *     tau = (t_I - t_R) / (t_F - t_R)    // interpolation factor
         *
         *     dp_I = tau*dp_F                    // dp is a 3-vector
         *     dq_I = 1.slerp(tau, dq_F)          // '1' is the identity quaternion; slerp() interpolates 3D rotation.
         *
         *     D_I  = deltaPlusDelta(D_R, d_I)
         *
         *     dp_S = dq_I.conj * (1-tau)*dp_F    // dq is a quaternion; 'conj' is the conjugate quaternion.
         *     dq_S = dq_I.conj * dq_F
         *
         *     D_S  = D_F
         * ```
         */
        virtual Motion interpolate(const Motion& _ref, Motion& _second, TimeStamp& _ts) = 0;

        /** \brief create a constraint and link it in the wolf tree
         * \param _feature_motion: the parent feature
         * \param _frame_origin: the frame constrained by this motion constraint
         */
        virtual ConstraintBasePtr emplaceConstraint(FeatureBasePtr _feature_motion, FrameBasePtr _frame_origin) = 0;

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

    private:
        wolf::TimeStamp getCurrentTimeStamp();
};

}

#include "frame_base.h"

namespace wolf{

inline void ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin)
{
    // make a new key frame
    FrameBasePtr key_frame_ptr = getProblem()->emplaceFrame(KEY_FRAME, _x_origin, _ts_origin);
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

inline wolf::TimeStamp ProcessorMotion::getCurrentTimeStamp()
{
    return getBuffer().get().back().ts_;
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
    _ts = getCurrentTimeStamp();
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

inline void ProcessorMotion::sumDeltas(CaptureMotion::Ptr _cap1_ptr, CaptureMotion::Ptr _cap2_ptr,
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
