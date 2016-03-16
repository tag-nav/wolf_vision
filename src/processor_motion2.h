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
    private: // The elements of the buffer ---> this will move to the CaptureMotion
        typedef struct {
            public:
                TimeStamp ts_;       ///< Time stamp
                Eigen::VectorXs dx_; ///< the individual delta
                Eigen::VectorXs Dx_; ///< the integrated delta
        } Motion; ///< One instance of the buffered data, corresponding to a particular time stamp.
        typedef std::deque<Motion> MotionBuffer;

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

    public:
        /** \brief Gets the state corresponding to provided time-stamp
         * \param _t the time stamp
         * \_x the returned state
         */
        virtual void state(const TimeStamp& _t, Eigen::VectorXs& _x);
        /** \brief Provides the delta-state between two time-stamps
         * \param _t1 initial time
         * \param _t2 final time
         * \param _Delta the delta-state between _t1 and _t2
         */
        virtual void deltaState(const TimeStamp& _t1, const TimeStamp& _t2, Eigen::VectorXs& _Delta);
        /** Composes the deltas in two pre-integrated Captures
         * \param _cap1_ptr pointer to the first Capture
         * \param _cap2_ptr pointer to the second Capture. This is local wrt. the first Capture.
         * \param _delta_1_2 the concatenation of the deltas of Captures 1 and 2.
         */
        virtual void sumDeltas(const CaptureMotion* _cap1_ptr, const CaptureMotion* _cap2_ptr, Eigen::VectorXs& _delta_1_2) const;

    protected: // Attributes
        size_t x_size_;    ///< The size of the state vector
        size_t dx_size_;   ///< the size of the delta integrator
        size_t data_size_; ///< the size of the incoming data
        size_t noise_size_; ///< the size of the noise vector

        CaptureBase* origin_ptr_;
        WolfScalar dt_; ///< Time step --- assumed constant
        TimeStamp ts_origin_; ///< Time step at the origin
        Eigen::VectorXs x_origin_; ///< state at the origin
        Motion motion_;
        MotionBuffer buffer_; ///< Buffer starts with the origin data (ts_origin_, x_origin_).

    protected: // These are the pure virtual functions doing the mathematics
        /** \brief Extract data from a derived capture.
         * \param _capture_ptr pointer to the Capture we want to extract data from.
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
         * This function implements the composition (-) so that _x2_minus_x1 = _x2 (-) _x1.
         */
       virtual void xMinusX(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x2_minus_x1) = 0;

       /** \brief Computes the delta-state the goes from one delta-state to another
        * \param _delta1 the initial delta
        * \param _delta2 the final delta
        * \param _delta2_minus_delta1 the delta-state. It has the format of a delta-state.
        *
         * This function implements the composition (-) so that _delta2_minus_delta1 = _delta2 (-) _delta1.
        */
      virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta2_minus_delta1) = 0;

    protected:
        unsigned int index(const TimeStamp& _t); ///< Get index in buffer corresponding to time-stamp
        void pickDx(unsigned int _index, Eigen::Map<Eigen::VectorXs>& _Dx);
        void integrate(CaptureBase* _incoming_ptr);

    protected:
        WolfScalar ts_;
        Eigen::VectorXs dx_;
};
#endif /* PROCESSOR_MOTION2_H_ */
