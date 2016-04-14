/**
 * \file processor_motion_base.h
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_MOTION_BASE_H_
#define PROCESSOR_MOTION_BASE_H_

// forward declarations
namespace wolf {
    class TimeStamp;
}

// Wolf
#include "processor_base.h"
#include "wolf.h"

namespace wolf {

class ProcessorMotionBase : public ProcessorBase
{

        // This is the main public interface
    public:
        ProcessorMotionBase(ProcessorType _tp);
        virtual ~ProcessorMotionBase();

        // Instructions to the processor:

        virtual void process(CaptureBase* _incoming_ptr) = 0;

        // Queries to the processor:
        virtual bool voteForKeyFrame() = 0;

        /** \brief Fills a reference to the state integrated so far
         * \param the returned state vector
         */
        virtual const void state(Eigen::VectorXs& _x) = 0;
        //virtual const void state(Eigen::Map<Eigen::VectorXs>& _x) = 0;
        /** \brief Gets a constant reference to the state integrated so far
         * \return the state vector
         */
        virtual const Eigen::VectorXs state() = 0;
        /** \brief Fills the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \param _x the returned state
         */
        virtual void state(const TimeStamp& _ts, Eigen::VectorXs& _x) = 0;
        //virtual void state(const TimeStamp& _ts, Eigen::Map<Eigen::VectorXs>& _x) = 0;
        /** \brief Gets the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \return the state vector
         */
        virtual Eigen::VectorXs state(const TimeStamp& _ts) = 0;

};

} // namespace wolf

// IMPLEMENTATION

#include "time_stamp.h"

namespace wolf
{

inline ProcessorMotionBase::ProcessorMotionBase(ProcessorType _tp) :
        ProcessorBase(_tp)
{
    //
}

inline ProcessorMotionBase::~ProcessorMotionBase()
{
    //
}

} // namespace wolf


#endif /* PROCESSOR_MOTION2_H_ */
