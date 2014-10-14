/*
 * state_imu.h
 *
 *  Created on: May 22, 2014
 *      \author: jsola
 */

#ifndef STATE_IMU_H_
#define STATE_IMU_H_

//wolf
#include "state_base.h"


/**
 * \brief Class for IMU bias states
 */
class StateIMU : public StateBase
{
    public:
        static const unsigned int SIZE_ = 6; ///< Size of the IMU bias state

    private:
        Eigen::Map<Eigen::Vector3s> a_bias_; ///< accelerometer bias
        Eigen::Map<Eigen::Vector3s> w_bias_; ///< gyrometer bias

    public:

        // Local constructors
        /**
         * Default local constructor
         */
        StateIMU();

        /**
         * Local constructor from vector
         * \param _x the state vector
         */
        StateIMU(Eigen::VectorXs& _x);

        /**
         * Local constructor from vectors
         * \param _abias the accelerometer bias
         * \param _wbias the gyrometer bias
         */
        StateIMU(Eigen::Vector3s& _abias, Eigen::Vector3s& _wbias);

        // Remote constructors
        /**
         * Remote constructor
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         */
        StateIMU(Eigen::VectorXs& _storage, unsigned int _idx);

        /**
         * Remote constructor from vector
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StateIMU(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::VectorXs& _x);

        /**
         * Remote constructor from vectors
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _abias the accelerometer bias
         * \param _wbias the gyrometer bias
         */
        StateIMU(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::Vector3s& _abias, Eigen::Vector3s& _wbias);

        /**
         * Destructor
         */
        virtual ~StateIMU();

        /**
         * Set acc bias
         * \param _abias acc bias
         */
        void ab(Eigen::Vector3s& _abias);

        /**
         * Get reference to acc bias
         */
        Eigen::Map<Eigen::Vector3s>& ab();

        /**
         * Set gyro bias
         * \param _wbias gyro bias
         */
        void wb(Eigen::Vector3s& _wbias);

        /**
         * Get reference to gyro bias
         */
        Eigen::Map<Eigen::Vector3s>& wb();

};

//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////



inline Eigen::Map<Eigen::Vector3s>& StateIMU::ab()
{
    return a_bias_;
}

inline void StateIMU::ab(Eigen::Vector3s& _abias)
{
    a_bias_ = _abias;
}

inline Eigen::Map<Eigen::Vector3s>& StateIMU::wb()
{
    return w_bias_;
}

inline void StateIMU::wb(Eigen::Vector3s& _wbias)
{
    w_bias_ = _wbias;
}

#endif /* STATE_IMU_H_ */
