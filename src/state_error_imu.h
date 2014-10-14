/*
 * state_error_imu.h
 *
 *  Created on: Jun 2, 2014
 *      \author: jsola
 */

#ifndef STATE_ERROR_IMU_H_
#define STATE_ERROR_IMU_H_

//wolf
#include <state_error.h>

/**
 * \brief Error-state class for IMU biases states
 */
class StateErrorIMU : public StateError
{
    public:
        static const unsigned int SIZE_NOMINAL_ = 6;    ///< size of the nominal state
        static const unsigned int SIZE_ERROR_ = 6;      ///< size of the error state

    private:
        Eigen::Map<Eigen::Vector3s> ab_error_;        ///< acc bias error
        Eigen::Map<Eigen::Vector3s> wb_error_;        ///< gyro bias error
        Eigen::Map<Eigen::Vector3s> ab_nominal_;      ///< nominal acc bias
        Eigen::Map<Eigen::Vector3s> wb_nominal_;      ///< nominal gyro bias
        Eigen::Map<Eigen::Vector3s> ab_composed_;     ///< composed acc bias
        Eigen::Map<Eigen::Vector3s> wb_composed_;     ///< composed gyro bias

    public:
        // Local constructors
        /**
         * Default Local constructor
         */
        StateErrorIMU();

        /**
         * Local Constructor from nominal state.
         * \param _xn the nominal state vector
         *
         * Error state always initialized to zero.
         */
        StateErrorIMU(Eigen::VectorXs& _xn);

        /**
         * Local Constructor from nominal biases.
         * \param _abias_n nominal acc bias
         * \param _wbias_n nominal gyro bias
         *
         * Error state always initialized to zero.
         */
        StateErrorIMU(Eigen::Vector3s& _abias_n, Eigen::Vector3s& _wbias_n);

        // Remote constructors
        /**
         * Default Remote constructor
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         *
         * All states initialized to zero.
         */
        StateErrorIMU(Eigen::VectorXs& _storage, unsigned int _idx);

        /**
         * Remote Constructor from nominal state.
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         * \param _xn the nominal state vector
         *
         * Error state initialized to zero.
         */
        StateErrorIMU(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::VectorXs& _xn);

        /**
         * Remote Constructor from nominal biases.
         * \param _storage the remote storage vector
         * \param _idx index to the remote storage where the error state will map
         * \param _abias_n the nominal acc bias
         * \param _wbias_n the nominal gyro bias
         *
         * Error state initialized to zero.
         */
        StateErrorIMU(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::Vector3s& _abias_n, Eigen::Vector3s& _wbias_n);

        virtual ~StateErrorIMU();

        /**
         * Set nominal acc bias
         * \param _abias_n nominal acc bias
         */
        void abn(Eigen::Vector3s& _abias_n);

        /**
         * Get reference to nominal acc bias
         */
        Eigen::Map<Eigen::Vector3s>& abn();

        /**
         * Set nominal gyro bias
         * \param _wbias_n nominal gyro bias
         */
        void wbn(Eigen::Vector3s& _wbias_n);

        /**
         * Get reference to nominal gyro bias
         */
        Eigen::Map<Eigen::Vector3s>& wbn();

        /**
         * Get reference to acc bias error
         */
        Eigen::Map<Eigen::Vector3s>& abe();

        /**
         * Get reference to gyro bias error
         */
        Eigen::Map<Eigen::Vector3s>& wbe();

        /**
         * Get reference to composed acc bias
         */
        Eigen::Map<Eigen::Vector3s>& abc();

        /**
         * Get reference to composed gyro bias
         */
        Eigen::Map<Eigen::Vector3s>& wbc();

        /**
         * Compose nominal and error states.
         */
        void compose()
        {
            state_composed_ = state_nominal_ + state_estimated_map_;
        }
};


////////////////////////////////
// IMPLEMENTATION
////////////////////////////////


inline Eigen::Map<Eigen::Vector3s>& StateErrorIMU::abn()
{
    return ab_nominal_;
}

inline void StateErrorIMU::abn(Eigen::Vector3s& _abias_n)
{
    ab_nominal_ = _abias_n;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorIMU::wbn()
{
    return wb_nominal_;
}

inline void StateErrorIMU::wbn(Eigen::Vector3s& _wbias_n)
{
    wb_nominal_ = _wbias_n;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorIMU::abe()
{
    return ab_error_;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorIMU::wbe()
{
    return wb_error_;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorIMU::abc()
{
    compose();
    return ab_composed_;
}

inline Eigen::Map<Eigen::Vector3s>& StateErrorIMU::wbc()
{
    compose();
    return wb_composed_;
}

#endif /* STATE_ERROR_IMU_H_ */
