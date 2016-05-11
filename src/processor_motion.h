/**
 * \file processor_motion.h
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_MOTION_H_
#define PROCESSOR_MOTION_H_

// Wolf
#include "processor_base.h"
#include "capture_motion2.h"
#include "motion_buffer.h"
#include "time_stamp.h"
#include "wolf.h"

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

        // This is the main public interface
    public:
        ProcessorMotion(ProcessorType _tp, size_t _state_size, size_t _delta_size, size_t _data_size);
        virtual ~ProcessorMotion();

        // Instructions to the processor:

        virtual void process(CaptureBase* _incoming_ptr);

        // Queries to the processor:

        virtual bool voteForKeyFrame();

        /** \brief Fills a reference to the state integrated so far
         * \param _x the returned state vector
         */
        const void getState(Eigen::VectorXs& _x);

        /** \brief Gets a constant reference to the state integrated so far
         * \param _x the state vector
         */
        const Eigen::VectorXs& getState();

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

        /** Composes the deltas in two pre-integrated Captures
         * \param _cap1_ptr pointer to the first Capture
         * \param _cap2_ptr pointer to the second Capture. This is local wrt. the first Capture.
         * \param _delta1_plus_delta2 the concatenation of the deltas of Captures 1 and 2.
         */
        void sumDeltas(CaptureMotion2* _cap1_ptr, CaptureMotion2* _cap2_ptr, Eigen::VectorXs& _delta1_plus_delta2);

        /** Composes two delta covariances
         * \param _delta_cov1 covariance of the first delta
         * \param _delta_cov2 covariance of the delta to be composed
         * \param _jacobian1 jacobian of the composition w.r.t. _delta1
         * \param _jacobian2 jacobian of the composition w.r.t. _delta2
         * \param _delta_cov1_plus_delta_cov2 the covariance of the composition.
         */
        void deltaCovPlusDeltaCov(const Eigen::MatrixXs& _delta_cov1, const Eigen::MatrixXs& _delta_cov2,
                                  const Eigen::MatrixXs& _jacobian1, const Eigen::MatrixXs& _jacobian2,
                                  Eigen::MatrixXs& _delta_cov1_plus_delta_cov2);
        /** Set the origin of all motion for this processor
         * \param _origin_frame the key frame to be the origin
         * \param _ts_origin origin timestamp.
         */
        void setOrigin(FrameBase* _origin_frame, const TimeStamp& _ts_origin);

        /** Set the origin of all motion for this processor
         * \param _x_origin the state at the origin
         * \param _ts_origin origin timestamp.
         */
        void setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin);

        virtual bool keyFrameCallback(FrameBase* _keyframe_ptr, const Scalar& _time_tol);

        // Helper functions:
    public:
        // TODO change to protected

        void splitBuffer(const TimeStamp& _t_split, MotionBuffer& _oldest_part);

        //        void reset(CaptureMotion2* _capture_ptr);

        FrameBase* makeFrame(CaptureBase* _capture_ptr, FrameKeyType _type = NON_KEY_FRAME);
        FrameBase* makeFrame(CaptureBase* _capture_ptr, const Eigen::VectorXs& _state, FrameKeyType _type);

        MotionBuffer* getBufferPtr();

        const MotionBuffer* getBufferPtr() const;

    protected:
        void updateDt();
        void integrate();
        void reintegrate();

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
         * This function accesses the members data_ (as produced by extractData()) and dt_,
         * and computes the value of the delta-state delta_.
         *
         * \param _data the raw motion data
         * \param _data_cov the raw motion data covariance
         * \param _dt the time step (not always needed)
         * \param _delta the returned motion delta
         * \param _delta_cov the returned motion delta covariance
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
         *  - If _data is an increment:
         *         _delta = _data;
         *         _delta_cov = _data_cov
         *  - If _data is a velocity:
         *         _delta = _data * _dt
         *         _delta_cov = _data_cov * _dt.
         *
         *  However, other more complicated relations are possible.
         */
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt,
                                Eigen::VectorXs& _delta, Eigen::MatrixXs& _delta_cov) = 0;

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
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2.
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                    Eigen::VectorXs& _delta1_plus_delta2) = 0;

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         * \param _jacobian1 the jacobian of the composition w.r.t. _delta1.
         * \param _jacobian2 the jacobian of the composition w.r.t. _delta2.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2 and its jacobians.
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                    Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                    Eigen::MatrixXs& _jacobian2) = 0;

        /** \brief Delta zero
         * \return a delta state equivalent to the null motion.
         *
         * Examples (see documentation of the the class for info on Eigen::VectorXs):
         *   - 2D odometry: a 3-vector with all zeros, e.g. VectorXs::Zero(3)
         *   - 3D odometry: delta type is a PQ vector: 7-vector with [0,0,0, 0,0,0,1]
         *   - IMU: PQVBB 16-vector with [0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0]
         */
        virtual Eigen::VectorXs deltaZero() const = 0;

        virtual Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts) = 0;

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin) = 0;

        Motion motionZero(TimeStamp& _ts);

    protected:
        // Attributes
        Size x_size_;    ///< The size of the state vector
        Size delta_size_;    ///< the size of the deltas
        Size data_size_; ///< the size of the incoming data
        CaptureBase* origin_ptr_; //TODO: JV: change by FrameBase* origin_frame_ptr_
        CaptureMotion2* last_ptr_;
        CaptureMotion2* incoming_ptr_;

    protected:
        // helpers to avoid allocation
        Scalar dt_;                             ///< Time step
        Eigen::VectorXs x_;                     ///< current state
        Eigen::VectorXs delta_;                 ///< current delta
        Eigen::MatrixXs delta_cov_;             ///< current delta covariance
        Eigen::VectorXs delta_integrated_;      ///< integrated delta
        Eigen::MatrixXs delta_integrated_cov_;  ///< integrated delta covariance
        Eigen::VectorXs data_;                  ///< current data
        Eigen::MatrixXs jacobian_prev_;         ///< jacobian of delta composition w.r.t previous delta integrated
        Eigen::MatrixXs jacobian_curr_;         ///< jacobian of delta composition w.r.t current delta

};

inline ProcessorMotion::ProcessorMotion(ProcessorType _tp, size_t _state_size, size_t _delta_size, size_t _data_size) :
        ProcessorBase(_tp), x_size_(_state_size), delta_size_(_delta_size), data_size_(_data_size), origin_ptr_(
                nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr), dt_(0.0), x_(_state_size), delta_(_delta_size), delta_cov_(
                delta_size_, delta_size_), delta_integrated_(_delta_size), delta_integrated_cov_(delta_size_,
                                                                                                 delta_size_), data_(
                _data_size), jacobian_prev_(delta_size_, delta_size_), jacobian_curr_(delta_size_, delta_size_)
{
    //
}

inline ProcessorMotion::~ProcessorMotion()
{
    if (incoming_ptr_!= nullptr)
        incoming_ptr_->destruct();
}

inline void ProcessorMotion::deltaCovPlusDeltaCov(const Eigen::MatrixXs& _delta_cov1,
                                                  const Eigen::MatrixXs& _delta_cov2, const Eigen::MatrixXs& _jacobian1,
                                                  const Eigen::MatrixXs& _jacobian2,
                                                  Eigen::MatrixXs& _delta_cov1_plus_delta_cov2)
{
    //std::cout << "delta_1_cov" << std::endl;
    //std::cout << _delta_cov1 << std::endl;
    //std::cout << "delta_2_cov" << std::endl;
    //std::cout << _delta_cov2 << std::endl;
    //std::cout << "_jacobian1" << std::endl;
    //std::cout << _jacobian1 << std::endl;
    //std::cout << "_jacobian2" << std::endl;
    //std::cout << _jacobian2 << std::endl;

    _delta_cov1_plus_delta_cov2 = _jacobian1 * _delta_cov1 * _jacobian1.transpose()
            + _jacobian2 * _delta_cov2 * _jacobian2.transpose();

    //std::cout << "_delta_cov1_plus_delta_cov2" << std::endl;
    //std::cout << _delta_cov1_plus_delta_cov2 << std::endl;
}

inline void ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin)
{
    // make a new key frame
    FrameBase* key_frame_ptr = getProblem()->createFrame(KEY_FRAME, _x_origin, _ts_origin);
    // set the key frame as origin
    setOrigin(key_frame_ptr, _ts_origin);
}

inline void ProcessorMotion::setOrigin(FrameBase* _origin_frame, const TimeStamp& _ts_origin)
{
    // make (empty) origin Capture
    origin_ptr_ = new CaptureMotion2(_ts_origin, this->getSensorPtr(), Eigen::VectorXs::Zero(data_size_),
                                     Eigen::MatrixXs::Zero(data_size_, data_size_));
    // Add origin capture to origin frame
    _origin_frame->addCapture(origin_ptr_);

    // Set timestamp to origin frame
    _origin_frame->setTimeStamp(_ts_origin);

    // make (emtpy) last Capture
    last_ptr_ = new CaptureMotion2(_ts_origin, this->getSensorPtr(), Eigen::VectorXs::Zero(data_size_),
                                   Eigen::MatrixXs::Zero(data_size_, data_size_));

    // Make frame at last Capture
    makeFrame(last_ptr_, _origin_frame->getState(), NON_KEY_FRAME);

    getBufferPtr()->get().clear();
    getBufferPtr()->get().push_back(
            Motion( {_ts_origin, deltaZero(), deltaZero(), Eigen::MatrixXs::Zero(delta_size_, delta_size_),
                     Eigen::MatrixXs::Zero(delta_size_, delta_size_)}));
}

inline void ProcessorMotion::process(CaptureBase* _incoming_ptr)
{
    incoming_ptr_ = (CaptureMotion2*)(_incoming_ptr);
    preProcess();
    integrate();
    postProcess();
}

inline void ProcessorMotion::integrate()
{
    // Set dt
    updateDt();

    // get data and convert it to delta
    data2delta(incoming_ptr_->getData(), incoming_ptr_->getDataCovariance(), dt_, delta_, delta_cov_);

    // then integrate delta
    deltaPlusDelta(getBufferPtr()->get().back().delta_integr_, delta_, delta_integrated_, jacobian_prev_,
                   jacobian_curr_);

    // and covariance
    deltaCovPlusDeltaCov(getBufferPtr()->get().back().delta_integr_cov_, delta_cov_, jacobian_prev_, jacobian_curr_,
                         delta_integrated_cov_);
    //std::cout << "delta_integrated_cov_" << std::endl;
    //std::cout << delta_integrated_cov_ << std::endl;

        // then push it into buffer
    getBufferPtr()->get().push_back(Motion( {incoming_ptr_->getTimeStamp(),
                                             delta_,
                                             delta_integrated_,
                                             delta_cov_,
                                             delta_integrated_cov_,
                                             Eigen::MatrixXs::Zero(delta_size_, delta_size_),
                                             Eigen::MatrixXs::Zero(delta_size_, delta_size_)}));
    //std::cout << "getBufferPtr()->get().back().delta_integrated_cov_" << std::endl;
    //std::cout << getBufferPtr()->get().back().delta_integr_cov_ << std::endl;
}

inline void ProcessorMotion::reintegrate()
{
    Motion zero_motion; // call constructor with params // TODO use motionZero(ts)
    zero_motion.ts_ = origin_ptr_->getTimeStamp();
    zero_motion.delta_ = deltaZero();
    zero_motion.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
    zero_motion.delta_integr_ = deltaZero();
    zero_motion.jacobian_0.setIdentity();
    zero_motion.delta_integr_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);

    this->getBufferPtr()->get().push_front(zero_motion);

    auto motion_it = getBufferPtr()->get().begin();
    auto prev_motion_it = motion_it;
    motion_it++;

    Eigen::MatrixXs jacobian_prev(delta_size_, delta_size_), jacobian_curr(delta_size_, delta_size_);
    while (motion_it != getBufferPtr()->get().end())
    {
        deltaPlusDelta(prev_motion_it->delta_integr_, motion_it->delta_, motion_it->delta_integr_, jacobian_prev,
                       jacobian_curr);
        //std::cout << "delta reintegrated" << std::endl;
        deltaCovPlusDeltaCov(prev_motion_it->delta_integr_cov_, motion_it->delta_cov_, jacobian_prev, jacobian_curr,
                             motion_it->delta_integr_cov_);
        //std::cout << "delta_cov reintegrated" << std::endl;
        motion_it++;
        prev_motion_it++;
    }
}

inline bool ProcessorMotion::keyFrameCallback(FrameBase* _keyframe_ptr, const Scalar& _time_tol)
{
    //std::cout << "ProcessorMotion::keyFrameCallback: " << std::endl;
    //std::cout << "\tnew keyframe " << _keyframe_ptr->id() << std::endl;
    //std::cout << "\torigin keyframe " << origin_ptr_->getFramePtr()->id() << std::endl;

    // get time stamp
    TimeStamp ts = _keyframe_ptr->getTimeStamp();
    // create motion capture
    CaptureMotion2* key_capture_ptr = new CaptureMotion2(ts, this->getSensorPtr(), Eigen::VectorXs::Zero(data_size_),
                                                         Eigen::MatrixXs::Zero(data_size_, data_size_));
    // add motion capture to keyframe
    _keyframe_ptr->addCapture(key_capture_ptr);

    // split the buffer
    // and give old buffer to capture
    splitBuffer(ts, *(key_capture_ptr->getBufferPtr()));

    // interpolate individual delta
    Motion mot = interpolate(key_capture_ptr->getBufferPtr()->get().back(), // last Motion of old buffer
                             getBufferPtr()->get().front(), // first motion of new buffer
                             ts);

    // add to old buffer
    key_capture_ptr->getBufferPtr()->get().push_back(mot);

    // create motion constraint and add it to the new keyframe
    FeatureBase* key_feature_ptr = new FeatureBase(FEATURE_MOTION,
                                                   key_capture_ptr->getBufferPtr()->get().back().delta_integr_,
                                                   key_capture_ptr->getBufferPtr()->get().back().delta_integr_cov_);
    key_capture_ptr->addFeature(key_feature_ptr);
    key_feature_ptr->addConstraint(createConstraint(key_feature_ptr, origin_ptr_->getFramePtr()));

    // reset processor origin
    origin_ptr_ = key_capture_ptr;

    // reintegrate own buffer
    reintegrate();

    return true;
}

inline void ProcessorMotion::splitBuffer(const TimeStamp& _t_split, MotionBuffer& _oldest_part)
{
    last_ptr_->getBufferPtr()->split(_t_split, _oldest_part);
}

inline FrameBase* ProcessorMotion::makeFrame(CaptureBase* _capture_ptr, const Eigen::VectorXs& _state, FrameKeyType _type)
{
    // We need to create the new free Frame to hold what will become the last Capture
    FrameBase* new_frame_ptr = getProblem()->createFrame(_type, _state, _capture_ptr->getTimeStamp());
    new_frame_ptr->addCapture(_capture_ptr); // Add incoming Capture to the new Frame
    return new_frame_ptr;
}

inline FrameBase* ProcessorMotion::makeFrame(CaptureBase* _capture_ptr, FrameKeyType _type)
{
    // We need to create the new free Frame to hold what will become the last Capture
    FrameBase* new_frame_ptr = getProblem()->createFrame(_type, _capture_ptr->getTimeStamp());
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
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBufferPtr()->get().back().delta_integr_, _x);
}

inline const Motion& ProcessorMotion::getMotion() const
{
    return getBufferPtr()->get().back();
}

inline const Motion& ProcessorMotion::getMotion(const TimeStamp& _ts) const
{
    return getBufferPtr()->getMotion(_ts);
}

inline void ProcessorMotion::getMotion(Motion& _motion) const
{
    _motion = getBufferPtr()->get().back();
}

inline void ProcessorMotion::getMotion(const TimeStamp& _ts, Motion& _motion) const
{
    getBufferPtr()->getMotion(_ts, _motion);
}

inline void ProcessorMotion::sumDeltas(CaptureMotion2* _cap1_ptr, CaptureMotion2* _cap2_ptr,
                                       Eigen::VectorXs& _delta1_plus_delta2)
{
    // TODO: what should it return, now? also covariance? jacobians? a new CaptureMotion..?
    //deltaPlusDelta(_cap1_ptr->getDelta(), _cap2_ptr->getDelta(), _delta1_plus_delta2);
}

inline void ProcessorMotion::updateDt()
{
    dt_ = incoming_ptr_->getTimeStamp() - getBufferPtr()->get().back().ts_;
}

inline const MotionBuffer* ProcessorMotion::getBufferPtr() const
{
    return last_ptr_->getBufferPtr();
}

inline MotionBuffer* ProcessorMotion::getBufferPtr()
{
    return last_ptr_->getBufferPtr();
}

inline Motion ProcessorMotion::motionZero(TimeStamp& _ts)
{
    return Motion(
            {_ts,
             deltaZero(),
             deltaZero(),
             Eigen::MatrixXs::Zero(delta_size_, delta_size_),
             Eigen::MatrixXs::Zero(delta_size_, delta_size_),
             Eigen::MatrixXs::Identity(delta_size_, delta_size_),
             Eigen::MatrixXs::Identity(delta_size_, delta_size_)});
}

} // namespace wolf

#endif /* PROCESSOR_MOTION2_H_ */
