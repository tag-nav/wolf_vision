/*
 * state_error.h
 *
 *  Created on: 29/05/2014
 *      \author: jsola
 */

#ifndef STATE_ERROR_H_
#define STATE_ERROR_H_

//wolf
#include "state_root_base.h"

/** \brief Base class for error states
 * 
 * Base class for error states. It implements set/get methods to the nominal, error and composed states. 
 * Error states involve implicitly three states: 
 *      - nominal: the state point from which the error is estimated. It is an Eigen Vector member of this class.
 *      - error: the error part, which is usually estimated or optimized. It is the inherited Eigen Map of StateBase class.
 *      - composed: It is the composition of both nominal and error, so it is usually the state point to publish. It is an Eigen Vector member of this class.
 */
class StateError : public StateRootBase
{
    private:
        Eigen::VectorXs state_nominal_;        ///< nominal state
        Eigen::VectorXs state_composed_;       ///< composed state

    public:
        // Local constructors

        /**
         * Local constructor from sizes
         * \param _size_nominal size of the nominal state vector
         * \param _size_error size of the error state vector
         *
         * Nominal and error states initialized to zero
         */
        StateError(const unsigned int _size_nominal, const unsigned int _size_error);

        /**
         * Local constructor from vector and size
         * \param _x_nominal the nominal state
         * \param _size_error the size of the error state
         *
         * The error state is initialized to zero
         */
        StateError(const Eigen::VectorXs& _x_nominal, const unsigned int _size_error);

        /**
         * Local constructor from vectors
         * \param _x_nominal the nominal state
         * \param _x_error the error state
         */
        StateError(const Eigen::VectorXs& _x_nominal, const Eigen::VectorXs& _x_error);

        // Remote constructors
        /**
         * Remote constructor from sizes
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size_nominal size of the nominal state vector
         * \param _size_error size of the error state vector
         */
        StateError(Eigen::VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size_nominal, const unsigned int _size_error);

        /**
         * Remote constructor from vector and size
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x_nominal the nominal state vector
         * \param _size_error size of the error state vector
         *
         * The error state is initialized to zero
         */
        StateError(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _x_nominal, const unsigned int _size_error);

        /**
         * Remote constructor from vectors
         * \param _st_remote storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x_nominal the nominal state
         * \param _x_error the error state
         */
        StateError(Eigen::VectorXs& _st_remote, const unsigned int _idx, const Eigen::VectorXs& _x_nominal, const Eigen::VectorXs& _x_error);

        // Destructor
        virtual ~StateError();

        // Accessors to nominal, error and composed states
        /**
         * Get reference to nominal state
         */
        Eigen::VectorXs& xn();

        /**
         * Get reference to nominal state
         */
        const Eigen::VectorXs& xn() const;

        /**
         * Get reference to error state
         */
        Eigen::Map<Eigen::VectorXs>& xe();

        /**
         * Get reference to error state
         */
        const Eigen::Map<Eigen::VectorXs>& xe() const;

        /**
         * Get reference to composed state
         */
        Eigen::VectorXs& xc();

        /**
         * Get reference to composed state
         */
        const Eigen::VectorXs& xc() const;

        /**
         * Clear error. Set it to 0. 
         */
        void clearError();

        /**
         * Compose nominal and error states
         */
        virtual void compose() = 0;

};

//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////



inline const Eigen::VectorXs& StateError::xn() const
{
    return state_nominal_;
}

inline Eigen::VectorXs& StateError::xn()
{
    return state_nominal_;
}

inline const Eigen::Map<Eigen::VectorXs>& StateError::xe() const
{
    return state_estimated_map_;
}

inline Eigen::Map<Eigen::VectorXs>& StateError::xe()
{
    return state_estimated_map_;
}

inline const Eigen::VectorXs& StateError::xc() const
{
    return state_composed_;
}

inline Eigen::VectorXs& StateError::xc()
{
    compose();
    return state_composed_;
}

inline void StateError::clearError()
{
    state_estimated_map_ = Eigen::VectorXs::Zero(state_estimated_map_.size());
}

#endif /* STATE_ERROR_H_ */
