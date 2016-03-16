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
#include "time_stamp.h"
#include "wolf.h"

// STL
#include <deque>


class ProcessorMotion2 : public ProcessorBase
{
    private: // The elements of the buffer
        typedef struct Instance{
            public:
                TimeStamp ts_;
                Eigen::VectorXs dx_;
                Eigen::VectorXs Dx_;
        };

    public: // This is the main public interface
        ProcessorMotion2(ProcessorType _tp, size_t _state_size, size_t _delta_size, size_t _data_size,
                         size_t _noise_size, WolfScalar _dt);
        virtual ~ProcessorMotion2();

        virtual void process(CaptureBase* _incoming_ptr);

        void init(const CaptureBase* _origin_ptr); ///< To be called once
        void makeKeyFrame(const TimeStamp& _t); ///< To be called when a key-frame is created

    protected: // These are the pieces for building the public interface
        void update(); ///< To be called after a solver update
        void reset(const TimeStamp& _t); ///< To be called at KeyFrame generation
        void advance(); ///< To be called at the end of each Capture processed

        virtual void state(const TimeStamp& _t, Eigen::VectorXs& _x); ///< provides the state at the given time-stamp
        virtual void deltaState(const TimeStamp& _t1, const TimeStamp& _t2, Eigen::VectorXs& _Delta); ///< Provides the delta-state between two time-stamps
        virtual void sumDeltas(const CaptureMotion* _cap1_ptr, const CaptureMotion* _cap2_ptr, Eigen::VectorXs& _delta_1_2) const; ///< Composes the deltas in two pre-integrated Captures

    protected: // Attributes
        size_t x_size_;    ///< The size of the state vector
        size_t dx_size_;   ///< the size of the delta integrator
        size_t data_size_; ///< the size of the incoming data
        size_t noise_size_; ///< the size of the noise vector

        CaptureBase* origin_ptr_;
        WolfScalar dt_; ///< Time step --- assumed constant
        TimeStamp ts_origin_; ///< Time step at the origin
        Eigen::Map<Eigen::VectorXs> x_origin_; ///< state at the origin
        std::deque<Instance> buffer_; ///< Buffer starts with the origin data (ts_origin_, x_origin_).
        Instance instance_;

    protected: // These are the pure virtual functions doing the mathematics
        /** \brief Extract data from a derived capture.
         * \param _capture_ptr A pointer to the Capture we want to extract data from.
         * This function needs to:
         *  - access the incoming Capture
         *  - cast it to DerivedCapture to be able to access its derived members.
         *  - Fill in the data_ field
         *  - Fill in the dx_ field
         *  - Fill in the ts_ field
         *  - Eventually compute the new dt_ field as the time lapse between the old ts_ and the new one.
         */
        virtual void extractData(const CaptureBase* _capture_ptr) = 0;
        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta) = 0;
        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2) = 0;
        /** \brief Computes the delta-state the goes from one state to another
         * \param _x1 the initial state
         * \param _x2 the final state
         * \param _delta the delta-state. It has the format of a delta-state.
         *
         * This function computes _delta = _x2 (-) _x1, so that _x2 satisfies by plus(_x1, _delta, _x2).
         * The operator (-) is the composition 'minus'.
         */
       virtual void xMinusX(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x2_minus_x1) = 0;

    protected:
        unsigned int index(const TimeStamp& _t); ///< Get index in buffer corresponding to time-stamp
        void pickDx(unsigned int _index, Eigen::Map<Eigen::VectorXs>& _Dx);

    protected:
        WolfScalar ts_;
        Eigen::VectorXs x1_, x2_, x_, dx_, Dx1_, Dx2_;
};

inline ProcessorMotion2::ProcessorMotion2(ProcessorType _tp, size_t _state_size, size_t _delta_size, size_t _data_size,
                                          size_t _noise_size, WolfScalar _dt) :
        ProcessorBase(_tp), x_size_(_state_size), dx_size_(_delta_size), data_size_(_data_size), noise_size_(
                _noise_size), origin_ptr_(nullptr), dt_(_dt), ts_(0)
{
    buffer_.clear(); // just to be explicit; probably not needed.
}

inline ProcessorMotion2::~ProcessorMotion2()
{
    // TODO Auto-generated destructor stub
}

inline void ProcessorMotion2::process(CaptureBase* _incoming_ptr)
{
    // First get data and push it into buffer
    extractData(_incoming_ptr);
    Dx1_ = buffer_.back().Dx_;
    instance_.ts_ = ts_;
    instance_.dx_ = dx_;
    deltaPlusDelta(Dx1_, dx_, instance_.Dx_);
    buffer_.push_back(instance_);
    // Then deal with other stuff... but what stuff? I don't know.
}

inline void ProcessorMotion2::init(const CaptureBase* _origin_ptr)
{
    x_origin_ = _origin_ptr->getFramePtr()->getState();
    extractData(_origin_ptr);
    buffer_.clear();
    instance_.ts_ = ts_origin_;
    instance_.dx_ = dx_;
    instance_.Dx_.setZero();
    buffer_.push_back(instance_);
}

inline void ProcessorMotion2::makeKeyFrame(const TimeStamp& _t)
{
    unsigned int i = index(_t);
}

inline void ProcessorMotion2::update()
{
}

inline void ProcessorMotion2::reset(const TimeStamp& _t)
{
}

inline void ProcessorMotion2::advance()
{
}

inline void ProcessorMotion2::state(const TimeStamp& _t, Eigen::VectorXs& _x)
{
}

inline void ProcessorMotion2::deltaState(const TimeStamp& _t1, const TimeStamp& _t2, Eigen::VectorXs& _Delta)
{
}

inline void ProcessorMotion2::sumDeltas(const CaptureMotion* _cap1_ptr, const CaptureMotion* _cap2_ptr,
                                        Eigen::VectorXs& _delta_1_2) const
{
}

inline unsigned int ProcessorMotion2::index(const TimeStamp& _t)
{
    assert (!buffer_.empty() && "Empty buffer. This should have never happened.");
    // Assume dt is constant and known, and exists in dt_
    return (buffer_.back().ts_ - buffer_.front().ts_) / dt_ + 0.5; // we rounded to the nearest entry in the buffer
}

#endif /* PROCESSOR_MOTION2_H_ */
