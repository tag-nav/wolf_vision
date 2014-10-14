/*
 * \file state_pqv.h
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

#ifndef STATE_PQV_H_
#define STATE_PQV_H_

//std
//#include <memory>

//eigen
//#include <eigen3/Eigen/Geometry>

//wolf
//#include "wolf.h"
#include "state_pose.h"


/** \brief Class for 3D position, velocity and orientation states. 
 * 
 * Class for 3D position, velocity and orientation states. Oreintation is represented with a quaternion. 
 * It inherits State (and StateBase, and StatePose), so it can be constructed as local or remote.
 * P,V and Q parts are mapped over the StateBase map. Q part is a Quaternion Map, so when it is returned it can be used as a standard Eigen Quaternion.
 * The class implements P,V,Q accessors, and also a method to integrate IMU (Inertial Measurement Units) measurements to such states.
 * 
 */
class StatePQV : public StatePose
{
    public:
        static const unsigned int SIZE_ = 10; ///< Size of the overall PQV state

    private:
        Eigen::Map<Eigen::Vector3s> v_; ///< velocity

    public:

        /**
         * Default local constructor
         */
        StatePQV();

        /**
         * Local constructor from vector
         * \param _x the state vector
         */
        StatePQV(Eigen::VectorXs& _x);

        /**
         * Local constructor from vectors
         * \param _p position
         * \param _q orientation quaternion (must be normalized)
         * \param _v velocity
         */
        StatePQV(Eigen::Vector3s& _p, Eigen::Quaternions& _q, Eigen::Vector3s& _v);

        /**
         * Remote constructor
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         */
        StatePQV(Eigen::VectorXs& _storage, unsigned int _idx);

        /**
         * Remote constructor from vector
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector
         */
        StatePQV(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::VectorXs& _x);

        /**
         * Remote constructor from vectors
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _p position
         * \param _q orientation quaternion (must be normalized)
         * \param _v velocity
         */
        StatePQV(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::Vector3s& _p, Eigen::Quaternions& _q, Eigen::Vector3s& _v);

        /**
         * Destructor
         */
        virtual ~StatePQV();

        /**
         * Set velocity
         * \param _vel velocity
         */
        void v(Eigen::Vector3s& _vel);

        /**
         * Get reference to velocity
         */
        Eigen::Map<Eigen::Vector3s>& v();

        /**
         * Get reference to velocity
         */
        const Eigen::Map<Eigen::Vector3s>& v() const;

        virtual void remap(Eigen::VectorXs& _st_remote, const unsigned int _idx);



};

//////////////////////////////////////
//         IMPLEMENTATION
//////////////////////////////////////


inline void StatePQV::v(Eigen::Vector3s& vel)
{
    v_ = vel;
}

inline Eigen::Map<Eigen::Vector3s>& StatePQV::v()
{
    return v_;
}

inline const Eigen::Map<Eigen::Vector3s>& StatePQV::v() const
{
    return v_;
}

inline void StatePQV::remap(Eigen::VectorXs& _st_remote, const unsigned int _idx)
{
    StatePose::remap(_st_remote, _idx);
    new (&v_) Eigen::Map<Eigen::VectorXs>(&state_estimated_map_(7), 3);
}

#endif /* STATE_PQV_H_ */
