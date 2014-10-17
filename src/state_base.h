/*
 * state.h
 *
 *  Created on: Jun 2, 2014
 *      \author: jsola
 */

#ifndef STATE_BASE_H_
#define STATE_BASE_H_

//wolf
#include "state_root_base.h"

/** \brief Base class for regular states.
 * 
 * Base class for regular states. It implements the set/get methods to manipulate the state vector. 
 * See StateError for a base class for error states.
 * 
 */
class StateBase : public StateRootBase
{
    public:
        // Local Constructors
        /**
         * Local constructor from size
         * \param _size size of the state vector
         */
        StateBase(const unsigned int _size);

        /**
         * Local constructor from vector
         * \param _x the state vector
         */
        StateBase(const Eigen::VectorXs& _x);

        // Remote Constructors
        /**
         * Remote constructor from size
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size size of the state vector
         */
        StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size);

        /**
         * Remote constructor from vector
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _x);

        /**
         * Destructor
         */
        virtual ~StateBase();

    public:
        // Setters and getters

        /**
         * Get reference to state
         */
        Eigen::Map<Eigen::VectorXs>& x();

        /**
         * Get reference to state
         */
        const Eigen::Map<Eigen::VectorXs>& x() const;

        /**
         * Set state vector
         * \param _x the state vector
         */
        void x(const Eigen::VectorXs& _x);


};


////////////////////////////////
// IMPLEMENTATION
////////////////////////////////



inline void StateBase::x(const Eigen::VectorXs& _x)
{
    assert(state_estimated_map_.size() == _x.size());
    state_estimated_map_ = _x;
}

inline const Eigen::Map<Eigen::VectorXs>& StateBase::x() const
{
    return state_estimated_map_;
}

inline Eigen::Map<Eigen::VectorXs>& StateBase::x()
{
    return state_estimated_map_;
}

#endif /* STATE_BASE_H_ */
