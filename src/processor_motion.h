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

        // This is the main public interface
    public:
        ProcessorMotion(ProcessorType _tp, const std::string& _type, Size _state_size, Size _delta_size, Size _delta_cov_size, Size _data_size, const Scalar& _time_tolerance = 0.1);
        virtual ~ProcessorMotion();

        // Instructions to the processor:

        virtual void process(CaptureBase* _incoming_ptr);
        virtual void resetDerived();

        // Queries to the processor:

        virtual bool voteForKeyFrame();

        /** \brief Fills a reference to the state integrated so far
         * \param _x the returned state vector
         */
        const void getCurrentState(Eigen::VectorXs& _x);

        /** \brief Fills a reference to the state integrated so far and its stamp
         * \param _x the returned state vector
         * \param _ts the returned stamp
         */
        const void getCurrentState(Eigen::VectorXs& _x, TimeStamp& _ts);

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
        CaptureMotion* findCaptureContainingTimeStamp(const TimeStamp& _ts) const;

        /** Composes the deltas in two pre-integrated Captures
         * \param _cap1_ptr pointer to the first Capture
         * \param _cap2_ptr pointer to the second Capture. This is local wrt. the first Capture.
         * \param _delta1_plus_delta2 the concatenation of the deltas of Captures 1 and 2.
         */
        void sumDeltas(CaptureMotion* _cap1_ptr,
                       CaptureMotion* _cap2_ptr,
                       Eigen::VectorXs& _delta1_plus_delta2);

        /** Composes two delta covariances
         * \param _delta_cov1 covariance of the first delta
         * \param _delta_cov2 covariance of the delta to be composed
         * \param _Dt2 the second delta-state's time delta
         * \param _jacobian1 jacobian of the composition w.r.t. _delta1
         * \param _jacobian2 jacobian of the composition w.r.t. _delta2
         * \param _delta_cov1_plus_delta_cov2 the covariance of the composition.
         */
        void deltaCovPlusDeltaCov(const Eigen::MatrixXs& _delta1_cov,
                                  const Eigen::MatrixXs& _delta2_cov,
                                  const Eigen::MatrixXs& _jacobian1,
                                  const Eigen::MatrixXs& _jacobian2,
                                  Eigen::MatrixXs& _delta1_plus_delta2_cov);
        /** Set the origin of all motion for this processor
         * \param _origin_frame the key frame to be the origin
         */
        void setOrigin(FrameBase* _origin_frame);

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

        FrameBase* makeFrame(CaptureBase* _capture_ptr, const Eigen::VectorXs& _state, FrameKeyType _type);

        MotionBuffer* getBufferPtr();

        const MotionBuffer* getBufferPtr() const;

        virtual bool isMotion();

    protected:
        void updateDt();
        void integrate();
        void reintegrate(CaptureMotion* _capture_ptr);

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

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin) = 0;

        Motion motionZero(const TimeStamp& _ts);

    protected:
        // Attributes
        Size x_size_;           ///< The size of the state vector
        Size delta_size_;       ///< the size of the deltas
        Size delta_cov_size_;   ///< the size of the delta covariances matrix
        Size data_size_;        ///< the size of the incoming data
        CaptureBase* origin_ptr_;
        CaptureMotion* last_ptr_;
        CaptureMotion* incoming_ptr_;

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

inline ProcessorMotion::ProcessorMotion(ProcessorType _tp, const std::string& _type, Size _state_size, Size _delta_size, Size _delta_cov_size, Size _data_size, const Scalar& _time_tolerance) :
        ProcessorBase(_tp, _type, _time_tolerance), x_size_(_state_size), delta_size_(_delta_size), delta_cov_size_(_delta_cov_size), data_size_(_data_size), origin_ptr_(
                nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr), dt_(0.0), x_(_state_size), delta_(_delta_size), delta_cov_(
                _delta_cov_size, _delta_cov_size), delta_integrated_(_delta_size), delta_integrated_cov_(_delta_cov_size, _delta_cov_size), data_(
                _data_size), jacobian_delta_preint_(delta_cov_size_, delta_cov_size_), jacobian_delta_(delta_cov_size_, delta_cov_size_)
{
    //
}

inline ProcessorMotion::~ProcessorMotion()
{
    if (incoming_ptr_!= nullptr)
        incoming_ptr_->destruct();
}

inline void ProcessorMotion::deltaCovPlusDeltaCov(const Eigen::MatrixXs& _delta1_cov,
                                                  const Eigen::MatrixXs& _delta2_cov,
                                                  const Eigen::MatrixXs& _jacobian1,
                                                  const Eigen::MatrixXs& _jacobian2,
                                                  Eigen::MatrixXs& _delta1_plus_delta2_cov)
{

    _delta1_plus_delta2_cov =   _jacobian1 * _delta1_cov * _jacobian1.transpose()
                              + _jacobian2 * _delta2_cov * _jacobian2.transpose();

}

inline void ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin)
{
    // make a new key frame
    FrameBase* key_frame_ptr = getProblem()->createFrame(KEY_FRAME, _x_origin, _ts_origin);
    // set the key frame as origin
    setOrigin(key_frame_ptr);
}

inline void ProcessorMotion::setOrigin(FrameBase* _origin_frame)
{
    assert(_origin_frame->getTrajectoryPtr() != nullptr && "ProcessorMotion::setOrigin: origin frame must be in the trajectory.");
    assert(_origin_frame->isKey() && "ProcessorMotion::setOrigin: origin frame must be KEY FRAME.");

    // make (empty) origin Capture
    origin_ptr_ = new CaptureMotion(_origin_frame->getTimeStamp(), this->getSensorPtr(), Eigen::VectorXs::Zero(data_size_),
                                     Eigen::MatrixXs::Zero(data_size_, data_size_), nullptr);
    // Add origin capture to origin frame
    _origin_frame->addCapture(origin_ptr_);

    // make (emtpy) last Capture
    last_ptr_ = new CaptureMotion(_origin_frame->getTimeStamp(), this->getSensorPtr(), Eigen::VectorXs::Zero(data_size_),
                                   Eigen::MatrixXs::Zero(data_size_, data_size_), _origin_frame);

    // Make frame at last Capture
    makeFrame(last_ptr_, _origin_frame->getState(), NON_KEY_FRAME);

    // Reset deltas
    delta_ = deltaZero();
    delta_integrated_ = deltaZero();

    // clear and reset buffer
    getBufferPtr()->get().clear();
    getBufferPtr()->get().push_back(motionZero(_origin_frame->getTimeStamp()));

    // Reset derived things
    resetDerived();
}

inline void ProcessorMotion::process(CaptureBase* _incoming_ptr)
{
    incoming_ptr_ = (CaptureMotion*)(_incoming_ptr);
    preProcess();
    integrate();

    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // key_capture
        CaptureMotion* key_capture_ptr = last_ptr_;
        FrameBase* key_frame_ptr = key_capture_ptr->getFramePtr();

        // Set the frame as key
        key_frame_ptr->setState(getCurrentState());
        key_frame_ptr->setTimeStamp(getBufferPtr()->get().back().ts_);
        key_frame_ptr->setKey();

        // create motion constraint and add it to the new keyframe
        FeatureBase* key_feature_ptr = new FeatureBase(FEATURE_MOTION, "MOTION",
                                                       key_capture_ptr->getBufferPtr()->get().back().delta_integr_,
                                                       key_capture_ptr->getBufferPtr()->get().back().delta_integr_cov_.determinant() > 0 ?
                                                       key_capture_ptr->getBufferPtr()->get().back().delta_integr_cov_ :
                                                       Eigen::MatrixXs::Identity(delta_cov_size_, delta_cov_size_)*1e-8);


        key_capture_ptr->addFeature(key_feature_ptr);
        key_feature_ptr->addConstraint(createConstraint(key_feature_ptr, origin_ptr_->getFramePtr()));

        // new last capture
        last_ptr_ = new CaptureMotion(key_frame_ptr->getTimeStamp(), this->getSensorPtr(), Eigen::VectorXs::Zero(data_size_),
                                       Eigen::MatrixXs::Zero(data_size_, data_size_), key_frame_ptr);

        // create a new last frame
        makeFrame(last_ptr_, key_frame_ptr->getState(), NON_KEY_FRAME);

        // reset processor origin
        origin_ptr_ = key_capture_ptr;
        getBufferPtr()->get().push_back(Motion( {key_frame_ptr->getTimeStamp(),
                                                 deltaZero(),
                                                 deltaZero(),
                                                 Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_),
                                                 Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_)}));
        // reset derived things
        resetDerived();

        getProblem()->keyFrameCallback(key_frame_ptr, this, time_tolerance_);

            //// debug cout
            //Eigen::VectorXs interpolated_state(3);
            //xPlusDelta(origin_ptr_->getFramePtr()->getState(), key_capture_ptr->getBufferPtr()->get().back().delta_integr_, interpolated_state);
            //std::cout << "\tinterpolated state: " << interpolated_state.transpose() << std::endl;
    }
    postProcess();
}

inline void ProcessorMotion::integrate()
{
    // Set dt
    updateDt();

    // get data and convert it to delta, and obtain also the delta covariance
    data2delta(incoming_ptr_->getData(), incoming_ptr_->getDataCovariance(), dt_);

    // then integrate the current delta to pre-integrated measurements, and get Jacobians
    deltaPlusDelta(delta_integrated_,
                   delta_ ,
                   dt_,
                   delta_integrated_,
                   jacobian_delta_preint_,
                   jacobian_delta_);

    delta_integrated_cov_ =   jacobian_delta_preint_ * getBufferPtr()->get().back().delta_integr_cov_ * jacobian_delta_preint_.transpose()
                              + jacobian_delta_ * delta_cov_ * jacobian_delta_.transpose();


    // and covariance
//    deltaCovPlusDeltaCov(getBufferPtr()->get().back().delta_integr_cov_,
//                         delta_cov_,
//                         jacobian_delta_preint_,
//                         jacobian_delta_,
//                         delta_integrated_cov_);

    // then push it into buffer
    getBufferPtr()->get().push_back(Motion( {incoming_ptr_->getTimeStamp(),
                                             delta_,
                                             delta_integrated_,
                                             delta_cov_,
                                             delta_integrated_cov_
//                                             jacobian_delta_preint_,
//                                             jacobian_delta_
                                             }));
}

inline void ProcessorMotion::reintegrate(CaptureMotion* _capture_ptr)
{
    // start with empty motion
    _capture_ptr->getBufferPtr()->get().push_front(motionZero(_capture_ptr->getOriginFramePtr()->getTimeStamp()));

    auto motion_it = _capture_ptr->getBufferPtr()->get().begin();
    auto prev_motion_it = motion_it;
    motion_it++;

    while (motion_it != _capture_ptr->getBufferPtr()->get().end())
    {
        const Scalar dt = motion_it->ts_ - prev_motion_it->ts_;
        deltaPlusDelta(prev_motion_it->delta_integr_,
                       motion_it->delta_,
                       dt,
                       motion_it->delta_integr_,
                       jacobian_delta_preint_,
                       jacobian_delta_);

        delta_integrated_cov_ =   jacobian_delta_preint_ * getBufferPtr()->get().back().delta_integr_cov_ * jacobian_delta_preint_.transpose()
                                  + jacobian_delta_ * delta_cov_ * jacobian_delta_.transpose();

//        deltaCovPlusDeltaCov(prev_motion_it->delta_integr_cov_,
//                             motion_it->delta_cov_,
//                             jacobian_delta_preint_,
//                             jacobian_delta_,
//                             motion_it->delta_integr_cov_);

        motion_it++;
        prev_motion_it++;
    }
}

inline bool ProcessorMotion::keyFrameCallback(FrameBase* _keyframe_ptr, const Scalar& _time_tol)
{
    assert(_keyframe_ptr->getTrajectoryPtr() != nullptr && "ProcessorMotion::keyFrameCallback: key frame must be in the trajectory.");
    //std::cout << "ProcessorMotion::keyFrameCallback: ts = " << _keyframe_ptr->getTimeStamp().getSeconds() << "." << _keyframe_ptr->getTimeStamp().getNanoSeconds() << std::endl;
    //std::cout << "\tnew keyframe " << _keyframe_ptr->id() << ": " << _keyframe_ptr->getState().transpose() << std::endl;
    //std::cout << "\torigin keyframe " << origin_ptr_->getFramePtr()->id() << std::endl;

    // get time stamp
    TimeStamp ts = _keyframe_ptr->getTimeStamp();

    // find capture in which the new keyframe is interpolated
    CaptureMotion* capture_ptr = findCaptureContainingTimeStamp(ts);
    assert(capture_ptr != nullptr && "ProcessorMotion::keyFrameCallback: no motion capture containing the required TimeStamp found");

    FrameBase* key_capture_origin = capture_ptr->getOriginFramePtr();

    // create motion capture
    CaptureMotion* key_capture_ptr = new CaptureMotion(ts, this->getSensorPtr(), Eigen::VectorXs::Zero(data_size_),
                                                         Eigen::MatrixXs::Zero(data_size_, data_size_), key_capture_origin);

    // add motion capture to keyframe
    _keyframe_ptr->addCapture(key_capture_ptr);

    // split the buffer
    // and give old buffer to new key capture
    capture_ptr->getBufferPtr()->split(ts, *(key_capture_ptr->getBufferPtr()));

    // interpolate individual delta
    Motion mot = interpolate(key_capture_ptr->getBufferPtr()->get().back(), // last Motion of old buffer
                             capture_ptr->getBufferPtr()->get().front(), // first motion of new buffer
                             ts);

    // add to old buffer
    key_capture_ptr->getBufferPtr()->get().push_back(mot);

    //// debug cout
    //Eigen::VectorXs interpolated_state(3);
    //xPlusDelta(key_capture_origin->getState(), key_capture_ptr->getBufferPtr()->get().back().delta_integr_, interpolated_state);
    //std::cout << "\tinterpolated state: " << interpolated_state.transpose() << std::endl;

    // create motion constraint and add it to the new keyframe
    FeatureBase* key_feature_ptr = new FeatureBase(FEATURE_MOTION, "MOTION",
                                                   key_capture_ptr->getBufferPtr()->get().back().delta_integr_,
                                                   key_capture_ptr->getBufferPtr()->get().back().delta_integr_cov_.determinant() > 0 ?
                                                   key_capture_ptr->getBufferPtr()->get().back().delta_integr_cov_ :
                                                   Eigen::MatrixXs::Identity(delta_size_, delta_size_)*1e-8);
    key_capture_ptr->addFeature(key_feature_ptr);
    key_feature_ptr->addConstraint(createConstraint(key_feature_ptr, key_capture_origin));

    // Fix the remaining capture
    if (capture_ptr == last_ptr_)
        // reset processor origin
        origin_ptr_ = key_capture_ptr;

    capture_ptr->setOriginFramePtr(_keyframe_ptr);

    // reintegrate own buffer
    reintegrate(capture_ptr);

    // modify feature and constraint (if they exist)
    if (!capture_ptr->getFeatureListPtr()->empty())
    {
        FeatureBase* feature_ptr = capture_ptr->getFeatureListPtr()->front();
        // modify feature
        feature_ptr->setMeasurement(capture_ptr->getBufferPtr()->get().back().delta_integr_);
        feature_ptr->setMeasurementCovariance(capture_ptr->getBufferPtr()->get().back().delta_integr_cov_.determinant() > 0 ?
                                              capture_ptr->getBufferPtr()->get().back().delta_integr_cov_ :
                                              Eigen::MatrixXs::Identity(delta_size_, delta_size_)*1e-8);
        // modify constraint
        if (!feature_ptr->getConstraintListPtr()->empty())
        {
            feature_ptr->getConstraintListPtr()->front()->destruct();
            feature_ptr->addConstraint(createConstraint(feature_ptr, _keyframe_ptr));
        }
    }

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
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBufferPtr()->getDelta(_ts), _ts - origin_ptr_->getTimeStamp(), _x);
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

inline const void ProcessorMotion::getCurrentState(Eigen::VectorXs& _x)
{
    Scalar Dt = getBufferPtr()->get().back().ts_ - origin_ptr_->getTimeStamp();
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBufferPtr()->get().back().delta_integr_, Dt, _x);
}

inline const void ProcessorMotion::getCurrentState(Eigen::VectorXs& _x, TimeStamp& _ts)
{
    getCurrentState(_x);
    _ts = getBufferPtr()->get().back().ts_;
}

inline const Motion& ProcessorMotion::getMotion() const
{
    return getBufferPtr()->get().back();
}

inline const Motion& ProcessorMotion::getMotion(const TimeStamp& _ts) const
{
    auto capture_ptr = findCaptureContainingTimeStamp(_ts);
    assert(capture_ptr != nullptr && "ProcessorMotion::getMotion: timestamp older than first motion");

    return capture_ptr->getBufferPtr()->getMotion(_ts);
}

inline void ProcessorMotion::getMotion(Motion& _motion) const
{
    _motion = getBufferPtr()->get().back();
}

inline void ProcessorMotion::getMotion(const TimeStamp& _ts, Motion& _motion) const
{
    auto capture_ptr = findCaptureContainingTimeStamp(_ts);
    assert(capture_ptr != nullptr && "ProcessorMotion::getMotion: timestamp older than first motion");

    capture_ptr->getBufferPtr()->getMotion(_ts, _motion);
}

inline CaptureMotion* ProcessorMotion::findCaptureContainingTimeStamp(const TimeStamp& _ts) const
{
    //std::cout << "ProcessorMotion::findCaptureContainingTimeStamp: ts = " << _ts.getSeconds() << "." << _ts.getNanoSeconds() << std::endl;
    auto capture_ptr = last_ptr_;
    while (capture_ptr != nullptr)
    {
        // capture containing motion previous than the ts found:
        if (capture_ptr->getBufferPtr()->get().front().ts_ < _ts)
            return capture_ptr;

        // go to the previous motion capture
        else if (capture_ptr == last_ptr_)
            capture_ptr = (CaptureMotion*)origin_ptr_;
        else if (capture_ptr->getOriginFramePtr() == nullptr)
            return nullptr;
        else
        {
            CaptureBase* capture_base_ptr = capture_ptr->getOriginFramePtr()->hasCaptureOf(getSensorPtr());
            if (capture_base_ptr == nullptr)
                return nullptr;
            else
                capture_ptr = (CaptureMotion*)capture_base_ptr;
        }
    }
    return capture_ptr;
}


inline bool ProcessorMotion::isMotion()
{
    return true;
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
//             Eigen::MatrixXs::Identity(delta_cov_size_, delta_cov_size_),
//             Eigen::MatrixXs::Identity(delta_cov_size_, delta_cov_size_)
             });
}

} // namespace wolf

#endif /* PROCESSOR_MOTION2_H_ */
