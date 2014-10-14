/**
 * \file state_pq.h
 *
 *  Created on: Jun 19, 2014
 *      \author: jsola
 */

#ifndef STATE_PQ_H_
#define STATE_PQ_H_

#include "state_pose.h"

/**
 * \brief State pose with position and orientation only.
 *
 * This class is exactly equal to class StatePose in that it contains exactly a position p()
 * and an orientation quaternion q(), which can be mapped to local or remote storages as desired.
 *
 */
class StatePQ : public StatePose
{
    public:
        static const unsigned int SIZE_ = 7; ///< Size of the overall PQV state

    public:
        /**
         * Default local constructor
         */
        StatePQ();

        /**
         * Local constructor from vector
         * \param _x the state vector
         */
        StatePQ(const Eigen::VectorXs& _x);

        /**
         * Local constructor from vectors
         * \param _p position
         * \param _q orientation quaternion (must be normalized)
         */
        StatePQ(Eigen::Vector3s& _p, Eigen::Quaternions& _q);

        /**
         * Remote constructor
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         */
        StatePQ(Eigen::VectorXs& _storage, unsigned int _idx);

        /**
         * Remote constructor from vector
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StatePQ(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::VectorXs& _x);


        /**
         * Remote constructor from vectors
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _p position
         * \param _q orientation quaternion (must be normalized)
         */
        StatePQ(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::Vector3s& _p, Eigen::Quaternions& _q);

        /**
         * Destructor
         */
        virtual ~StatePQ();

};
#endif /* STATE_PQ_H_ */
