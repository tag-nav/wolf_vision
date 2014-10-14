/**
 * \file state_error_pose.h
 *
 *  Created on: 13/08/2014
 *     \author: jsola
 */

#ifndef STATE_ERROR_POSE_H_
#define STATE_ERROR_POSE_H_

#include "state_error.h"

class StateErrorPose : public StateError
{
    private:
        Eigen::Map<Eigen::Vector3s> p_error_;         ///< Position error
        Eigen::Map<Eigen::Vector3s> q_error_;         ///< Local orientation error (rad in XYZ axes; with respect to nominal orientation)
        Eigen::Map<Eigen::Vector3s> p_nominal_;       ///< Nominal position
        Eigen::Map<Eigen::Quaternions> q_nominal_;    ///< Nominal quaternion
        Eigen::Map<Eigen::Vector3s> p_composed_;      ///< Composed position
        Eigen::Map<Eigen::Quaternions> q_composed_;   ///< Composed quaternion
    public:

        // Local constructors
        /**
         * Default Local constructor
         *
         * All states initialized to zero.
         */
        StateErrorPose(unsigned int _size_nom, unsigned int _size_err);

        /**
         * Local Constructor from nominal state.
         * \param _xn the nominal state vector
         * \param _size_err The size of the error state
         *
         * Error state always initialized to zero.
         */
        StateErrorPose(Eigen::VectorXs& _xn, unsigned int _size_err);

        /**
         * Local Constructor from nominal states.
         * \param _size_nom the size of the nominal state
         * \param _size_err the size of the error state
         * \param _p_n the nominal position
         * \param _q_n the nominal quaternion
         *
         * Error states always initialized to zero.
         */
        StateErrorPose(unsigned int _size_nom, unsigned int _size_err, Eigen::Vector3s& _p_n, Eigen::Quaternions& _q_n);

        // Remote constructors
        /**
         * Default Remote constructor
         * \param _size_nom the size of the nominal state
         * \param _size_err the size of the error state
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         *
         * All states initialized to zero.
         */
        StateErrorPose(Eigen::VectorXs& _storage, unsigned int _idx, unsigned int _size_nom, unsigned int _size_err);

        /**
         * Remote Constructor from nominal state.
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         * \param _xn the nominal state vector
         * \param _size_err the size of the error state
         *
         * Error state initialized to zero.
         */
        StateErrorPose(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::VectorXs& _xn, unsigned int _size_err);

        /**
         * Remote Constructor from nominal states.
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         * \param _size_nom the size of the nominal state
         * \param _size_err the size of the error state
         * \param _p_n the nominal position
         * \param _q_n the nominal quaternion
         *
         * Error states initialized to zero.
         */
        StateErrorPose(Eigen::VectorXs& _storage, unsigned int _idx, unsigned int _size_nom, unsigned int _size_err, Eigen::Vector3s& _p_n, Eigen::Quaternions& _q_n);

        /**
         * Default destructor
         */
        virtual ~StateErrorPose();

        /**
         * Set nominal position
         * \param _p_n the nominal position
         */
        void pn(Eigen::Vector3s& _p_n);

        /**
         * Get reference to nominal position
         */
        Eigen::Map<Eigen::Vector3s>& pn();

        /**
         * Set nominal quaternion
         * \param _q_n the nominal quaternion
         */
        void qn(Eigen::Quaternions& _q_n);

        /**
         * Get reference to nominal quaternion
         */
        Eigen::Map<Eigen::Quaternions>& qn();

        /**
         * Get reference to position error
         */
        Eigen::Map<Eigen::Vector3s>& pe();


        /**
         * Get reference to orientation error
         */
        Eigen::Map<Eigen::Vector3s>& qe();

        /**
         * Get reference to composed position
         */
        Eigen::Map<Eigen::Vector3s>& pc();

        /**
         * Get reference to composed quaternion
         */
        Eigen::Map<Eigen::Quaternions>& qc();


        /**
         * Compose nominal and error states.
         */
        void compose();


};

//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////



inline Eigen::Map<Eigen::Vector3s>& StateErrorPose::pn()
{
    return p_nominal_;
}

inline void StateErrorPose::pn(Eigen::Vector3s& _p_n)
{
    p_nominal_ = _p_n;
}

inline Eigen::Map<Eigen::Quaternions>& StateErrorPose::qn()
{
    return q_nominal_;
}

inline void StateErrorPose::qn(Eigen::Quaternions& _q_n)
{
    q_nominal_ = _q_n;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorPose::pe()
{
    return p_error_;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorPose::qe()
{
    return q_error_;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorPose::pc()
{
    compose();
    return p_composed_;
}

inline Eigen::Map<Eigen::Quaternions>& StateErrorPose::qc()
{
    compose();
    return q_composed_;
}

#endif /* STATE_ERROR_POSE_H_ */
