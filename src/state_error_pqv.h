/*
 * state_error_pqv.h
 *
 *  Created on: Jun 2, 2014
 *      \author: jsola
 */

#ifndef STATE_ERROR_PQV_H_
#define STATE_ERROR_PQV_H_

//wolf
#include "state_error_pose.h"

/**
 * \brief Error state class for position-velocity-quaternion states
 */
class StateErrorPQV : public StateErrorPose
{

    public:
        static const unsigned int SIZE_NOMINAL_ = 10;   ///< Size of the nominal state
        static const unsigned int SIZE_ERROR_ = 9;      ///< Size of the error state

    private:
        Eigen::Map<Eigen::Vector3s> v_error_;         ///< Velocity error
        Eigen::Map<Eigen::Vector3s> v_nominal_;       ///< Nominal velocity
        Eigen::Map<Eigen::Vector3s> v_composed_;      ///< Composed velocity

    public:

        // Local constructors
        /**
         * Default Local constructor
         *
         * All states initialized to zero.
         */
        StateErrorPQV();

        /**
         * Local Constructor from nominal state.
         * \param _xn the nominal state vector
         *
         * Error state always initialized to zero.
         */
        StateErrorPQV(Eigen::VectorXs& _xn);

        /**
         * Local Constructor from nominal states.
         * \param _p_n the nominal position
         * \param _q_n the nominal quaternion
         * \param _v_n the nominal velocity
         *
         * Error states always initialized to zero.
         */
        StateErrorPQV(Eigen::Vector3s& _p_n, Eigen::Quaternions& _q_n, Eigen::Vector3s& _v_n);

        // Remote constructors
        /**
         * Default Remote constructor
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         *
         * All states initialized to zero.
         */
        StateErrorPQV(Eigen::VectorXs& _storage, unsigned int _idx);

        /**
         * Remote Constructor from nominal state.
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         * \param _xn the nominal state vector
         *
         * Error state initialized to zero.
         */
        StateErrorPQV(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::VectorXs& _xn);

        /**
         * Remote Constructor from nominal states.
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         * \param _p_n the nominal position
         * \param _q_n the nominal quaternion
         * \param _v_n the nominal velocity
         *
         * Error states initialized to zero.
         */
        StateErrorPQV(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::Vector3s& _p_n, Eigen::Quaternions& _q_n,
                      Eigen::Vector3s& _v_n);

        /**
         * Default destructor
         */
        virtual ~StateErrorPQV();

        /**
         * Set nominal velocity
         * \param _v_n the nominal velocity
         */
        void vn(Eigen::Vector3s& _v_n);

        /**
         * Get reference to nominal velocity
         */
        Eigen::Map<Eigen::Vector3s>& vn();

        /**
         * Get reference to velocity error
         */
        Eigen::Map<Eigen::Vector3s>& ve();

        /**
         * Get reference to composed velocity
         */
        Eigen::Map<Eigen::Vector3s>& vc();

        /**
         * Compose nominal and error states.
         */
        void compose();

//        virtual Eigen::Map<Eigen::VectorXs>& x();

};

//*///////////////////////////////
//       IMPLEMENTATION
//*///////////////////////////////

inline StateErrorPQV::~StateErrorPQV()
{
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorPQV::vn()
{
    return v_nominal_;
}

inline void StateErrorPQV::vn(Eigen::Vector3s& _v_n)
{
    v_nominal_ = _v_n;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorPQV::ve()
{
    return v_error_;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorPQV::vc()
{
    compose();
    return v_composed_;
}

#endif /* STATE_ERROR_PQV_H_ */
