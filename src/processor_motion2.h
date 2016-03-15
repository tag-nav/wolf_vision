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
        ProcessorMotion2(ProcessorType _tp, size_t _state_size, size_t _delta_size, size_t _data_size);
        virtual ~ProcessorMotion2();

        virtual void process(CaptureBase* _incoming_ptr);

        void init(const CaptureBase* _origin_ptr); ///< To be called once
        void makeKeyFrame(const TimeStamp& _t); ///< To be called when a key-frame is created

    protected: // These are the pieces for building the public interface
        void update(); ///< To be called after a solver update
        void reset(const TimeStamp& _t); ///< To be called at KeyFrame generation
        void advance(); ///< To be called at the end of each Capture processed

        virtual Eigen::VectorXs& state(const TimeStamp& _t); ///< provides the state at the given time-stamp
        virtual Eigen::VectorXs& deltaState(const TimeStamp& _t1, const TimeStamp& _t2); ///< Provides the delta-state between two time-stamps
        virtual void sumDeltas(const CaptureBase* _cap1_ptr, const CaptureBase* _cap2_ptr, Eigen::VectorXs& _delta_1_2) const; ///< Composes the deltas in two pre-integrated Captures

    protected: // Attributes
        size_t x_size_;    ///< The size of the state vector
        size_t dx_size_;   ///< the size of the delta integrator
        size_t data_size_; ///< the size of the incoming data
        size_t noise_size; ///< the size of the noise vector

        CaptureBase* origin_ptr_;
        WolfScalar dt_; ///< Time step --- assumed constant
        TimeStamp ts_origin_; ///< Time step at the origin
        Eigen::Map<Eigen::VectorXs> x_origin_; ///< state at the origin
        std::deque<Instance> buffer_; ///< Buffer starts empty

    protected: // These are the pure virtual functions doing the mathematics
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta) = 0;
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2) = 0;
        virtual void xMinusX(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x2_minus_x1) = 0;

    protected:
        unsigned int index(const TimeStamp& _t); ///< Get index in buffer corresponding to time-stamp
        void dx(unsigned int _index, Eigen::Map<Eigen::VectorXs>& _dx);
        void Dx(unsigned int _index, Eigen::Map<Eigen::VectorXs>& _Dx);

    protected:
        Eigen::Map<Eigen::VectorXs> x1_, x2_, x_, dx_, Dx1_, Dx2_;
};

#endif /* PROCESSOR_MOTION2_H_ */
