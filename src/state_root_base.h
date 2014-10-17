/*
 * state_root_base.h
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

#ifndef STATE_ROOT_BASE_H_
#define STATE_ROOT_BASE_H_

// wolf includes
#include "wolf.h"

/** \brief Base class for all kind of states.
 * 
 * This class is the base for all state classes. Conceptually, it has a vector which represents the state, but this vector is stored either locally or remotely, depending on how the object has been constructed:
 *      - Local Case: an Eigen VectorXs member is stored by the object itself.
 *      - Remote Case: an Eigen Map member maps a remote storage Vector, so the object does not allocate memory for state Vector
 * In the local case the map member is pointing to the local vector, thus in both cases the accessors implemented in the inherited classes will be the same, since these accessors are implemented on the map member. 
 * 
 */
class StateRootBase
{
    protected:

        Eigen::VectorXs state_estimated_local_;   ///< Local storage
        Eigen::Map<Eigen::VectorXs> state_estimated_map_; ///< mapped vector, to remote storage

    protected:

        // Local Constructors
        /**
         * Local constructor from size. Map member will map local vector.
         * \param _size size of the state vector
         */
        StateRootBase(const unsigned int _size);

        /**
         * Local constructor from vector. Map member will map local vector.
         * \param _x the state vector
         */
        StateRootBase(const Eigen::VectorXs& _x);

        // Remote Constructors
        /**
         * Remote constructor from size. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size size of the state vector
         */
        StateRootBase(Eigen::VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size);

        /**
         * Remote constructor from vector. Map member will map remote storage vector.
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StateRootBase(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _x);

        /**
         * Destructor
         */
        virtual ~StateRootBase();

    public:

        Eigen::VectorXs& stateEstimatedLocal();

        const Eigen::VectorXs& stateEstimatedLocal() const;

        Eigen::Map<Eigen::VectorXs>& stateEstimatedMap();

        const Eigen::Map<Eigen::VectorXs>& stateEstimatedMap() const;

        /**
         * \brief Force implementation of a state getter and make this class Abstract.
         *
         * The implementation must take care of these facts:
         *  - If in a regular state, this corresponds to state_estimated_map_
         *  - If in an error state, this composes the nominal state and the error state.
         */
        virtual Eigen::Map<Eigen::VectorXs>& x() = 0;

        virtual void remap(Eigen::VectorXs& _st_remote, const unsigned int _idx);

        unsigned int size();

};

/////////////////////////////////
// IMPLEMENTATION
/////////////////////////////////



inline const Eigen::VectorXs& StateRootBase::stateEstimatedLocal() const
{
    return state_estimated_local_;
}

inline Eigen::VectorXs& StateRootBase::stateEstimatedLocal()
{
    return state_estimated_local_;
}

inline const Eigen::Map<Eigen::VectorXs>& StateRootBase::stateEstimatedMap() const
{
    return state_estimated_map_;
}

inline Eigen::Map<Eigen::VectorXs>& StateRootBase::stateEstimatedMap()
{
    return state_estimated_map_;
}

inline void StateRootBase::remap(Eigen::VectorXs& _st_remote, const unsigned int _idx)
{
    new (&state_estimated_map_) Eigen::Map<Eigen::VectorXs>(&_st_remote(_idx), this->size());
}

inline unsigned int StateRootBase::size()
{
    return state_estimated_map_.size();
}

#endif /* STATE_ROOT_BASE_H_ */
