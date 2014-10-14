/*
 * state_pose.h
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

#ifndef STATE_POSE_H_
#define STATE_POSE_H_

//wolf
#include "state_base.h"

#include <iostream>

/** \brief Base class for pose states.
 *
 * A pose state is a state containing, at least, a position P and an orientation Q.
 *
 * Derive from this class to include additional substates such as velocities or other possible
 * things related to motion.
 *
 * Orientation is represented with a quaternion.
 *
 * It inherits StateRootBase, so it can be constructed as local or remote.
 * P and Q parts are mapped over the StateRootBase map. The Q part is an Eigen Quaternion Map, i.e., Map < Quaternions >, so when
 * it is returned it can be used as a standard Eigen Quaternion.
 * 
 * The class implements p() and q() accessors returning const and non-const references to the Maps, Map<VectorXs>& and Map<Quaternions>&.
 *  - The const references allow no-cost read access of P and Q directly on the remote storage vector pointed by the Map.
 *  - The non-const references allow no-cost manipulations of P and Q directly on the remote storage vector pointed by the Map.
 * 
 */
class StatePose : public StateBase
{
    private:
        Eigen::Map<Eigen::Vector3s> p_; ///< position
        Eigen::Map<Eigen::Quaternions> q_; ///< orientation quaternion

    public:

        /** \brief Default local constructor. 
         * 
         * Default local constructor. The pose is set to the origin and the rest of states to zero.
         * 
         */
        StatePose();

        /** \brief Local constructor from size
         * 
         * Local constructor from size
         * \param _size dimension of the state vector
         * 
         */
        StatePose(unsigned int _size);

        /** \brief Local constructor from vector
         * 
         * Local constructor from vector. 
         * \param _x the state vector
         *  
         */
        StatePose(const Eigen::VectorXs & _x);
        
        /** \brief Local constructor from StatePose
         * 
         * Local constructor from StatePose. 
         * \param _x the state vector
         *  
         */
        StatePose(const StatePose& _pose);

        /**
         * Local constructor from state vector
         * \param _size dimension of the state vector
         * \param _p position
         * \param _q orientation quaternion (must be normalized)
         *
         * NOTE: Enter a valid unity Quaternion. It is not normalized by the constructor.
         */
        StatePose(unsigned int _size, Eigen::Vector3s& _p, Eigen::Quaternions& _q);

        /**
         * Remote constructor
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size dimension of the state vector
         *
         * Note: The state takes the values existing in the given storage vector.
         *
         * Note: Enter a valid unity Quaternion. It is not normalized by the constructor.
         */
        StatePose(Eigen::VectorXs& _storage, unsigned int _idx, unsigned int _size);

        /**
         * Remote constructor from vector
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _x the state vector.
         *
         * Note: The state takes the values existing in the given storage vector.
         *
         * Note: The Quaternion is not normalized by the constructor.
         */
        StatePose(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::VectorXs& _x);

        /**
         * Remote constructor from vectors
         * \param _storage storage vector
         * \param _idx index where the state maps to the remote storage vector
         * \param _size dimension of the state vector
         * \param _p position
         * \param _q orientation quaternion (must be normalized by the caller)
         */
        StatePose(Eigen::VectorXs& _storage, unsigned int _idx, unsigned int _size, Eigen::Vector3s& _p,
                  Eigen::Quaternions& _q);

        /**
         * Destructor
         */
        virtual ~StatePose();

        /**
         * \brief Set to origin
         *
         * Set state to the origin.
         *
         * Sets the position to 0, the quaternion to Quaternions::Identity(), and the rest of the states to 0.
         *
         * This class may be overloaded in case the states other than P, Q need a different origin than the default 0.
         */
        virtual void setToOrigin();

        /**
         * Set position
         * \param _pos position
         */
        void p(Eigen::Vector3s& _pos);

        /**
         * Get reference to position
         */
        Eigen::Map<Eigen::Vector3s>& p();

        /**
         * Get reference to position
         */
        const Eigen::Map<Eigen::Vector3s>& p() const;


        /**
         * Set quaternion
         * \param _quat quaternion
         */
        void q(Eigen::Quaternions& _quat);

        /**
         * Get reference to quaternion
         */
        Eigen::Map<Eigen::Quaternions>& q();

        /**
         * Get reference to quaternion
         */
        const Eigen::Map<Eigen::Quaternions>& q() const;

        /**
         * \brief compose this pose with a local pose
         * \param _local_pose local pose to compose with. Acts as local wrt this.
         * \return a composed pose, this * _other
         *
         * Composes another pose locally on top of this.
         */
        virtual StatePose composeWithLocal(const StatePose& _local_pose) const;
        virtual void composeWithLocal(const StatePose& _local_pose, StatePose& _return_pose) const;
        virtual void composeWithLocal(const StatePose& _local_pose);

        /**
         * \brief compose this pose with a base pose
         * \param _base_pose base pose to compose with. Acts as global or base wrt this.
         * \return a composed pose, _other * this
         *
         * Composes this pose on top of another.
         */
        virtual StatePose composeOnTopOf(const StatePose& _base_pose) const;
        virtual void composeOnTopOf(const StatePose& _base_pose, StatePose& _return_pose) const;
        virtual void composeOnTopOf(const StatePose& _base_pose);

        /**
         * \brief compose poses
         * \param _reference_pose the pose in relation to which we compose
         *
         * Expresses this pose relative to another
         */
        virtual StatePose composeRelativeTo(const StatePose& _reference_pose) const;
        virtual void composeRelativeTo(const StatePose& _reference_pose, StatePose& _return_pose) const;
        virtual void composeRelativeTo(const StatePose& _reference_pose);

        /**
         * \brief inverse pose
         * \return a State pose whose pose is the inverse of this.
         */
        virtual StatePose inverse() const;
        void inverse(StatePose& _result) const;

        virtual void remap(Eigen::VectorXs& _st_remote, const unsigned int _idx);

};


//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////

inline void StatePose::setToOrigin()
{
    state_estimated_map_ = Eigen::VectorXs::Zero(size());
    q_.w() = 1.0; // faster than q_.setIdentity();
}

inline const Eigen::Map<Eigen::Vector3s>& StatePose::p() const
{
    return p_;
}

inline Eigen::Map<Eigen::Vector3s>& StatePose::p()
{
    return p_;
}

inline void StatePose::p(Eigen::Vector3s& _pos)
{
    p_ = _pos;
}

inline const Eigen::Map<Eigen::Quaternions>& StatePose::q() const
{
    return q_;
}

inline Eigen::Map<Eigen::Quaternions>& StatePose::q()
{
    return q_;
}

inline void StatePose::q(Eigen::Quaternions& _quat)
{
    q_ = _quat;
}

inline StatePose StatePose::composeWithLocal(const StatePose& _local_pose) const
{
    Eigen::Vector3s p_new(this->q() * _local_pose.p() + this->p());
    Eigen::Quaternions q_new(this->q() * _local_pose.q());
    return StatePose(7, p_new, q_new);
}

inline StatePose StatePose::composeOnTopOf(const StatePose& _base_pose) const
{
    Eigen::Vector3s p_new(_base_pose.q() * this->p() + _base_pose.p());
    Eigen::Quaternions q_new(_base_pose.q() * this->q());
    return StatePose(7, p_new, q_new);
}

inline StatePose StatePose::composeRelativeTo(const StatePose& _reference_pose) const
{
    Eigen::Quaternions q_rel(_reference_pose.q().conjugate() * this->q());
    Eigen::Vector3s p_rel(_reference_pose.q().conjugate() * (this->p() - _reference_pose.p()));
    return StatePose(7, p_rel, q_rel);
}

inline void StatePose::composeWithLocal(const StatePose& _local_pose, StatePose& _return_pose) const
{
    _return_pose.p() = this->q() * _local_pose.p() + this->p();
    _return_pose.q() = this->q() * _local_pose.q();
}

inline void StatePose::composeOnTopOf(const StatePose& _base_pose, StatePose& _return_pose) const
{
    _return_pose.p() = _base_pose.q() * this->p() + _base_pose.p();
    _return_pose.q() = _base_pose.q() * this->q();
}

inline void StatePose::composeRelativeTo(const StatePose& _reference_pose, StatePose& _return_pose) const
{
    _return_pose.p() = _reference_pose.q().conjugate() * (this->p() - _reference_pose.p());
    _return_pose.q() = _reference_pose.q().conjugate() * this->q();
}

inline void StatePose::composeWithLocal(const StatePose& _local_pose)
{
    this->p() += this->q() * _local_pose.p();
    this->q() *= _local_pose.q();
}

inline void StatePose::composeOnTopOf(const StatePose& _base_pose)
{
    this->p() += _base_pose.q() * this->p();
    this->q() = _base_pose.q() * this->q();
}

inline void StatePose::composeRelativeTo(const StatePose& _reference_pose)
{
    this->p() = _reference_pose.q().conjugate() * (this->p() - _reference_pose.p());
    this->q() = _reference_pose.q().conjugate() * this->q();
}

inline void StatePose::inverse(StatePose& _result) const
{
    _result.q() = this->q().conjugate();
    _result.p() = -(_result.q() * this->p());
}

inline StatePose StatePose::inverse() const
{
    StatePose inv;
    inv.q() = this->q().conjugate();
    inv.p() = -(inv.q() * this->p());
    return inv;
}

inline void StatePose::remap(Eigen::VectorXs& _st_remote, const unsigned int _idx)
{
    StateRootBase::remap(_st_remote, _idx);
    new (&p_) Eigen::Map<Eigen::VectorXs>    (&state_estimated_map_(0), 3);
    new (&q_) Eigen::Map<Eigen::Quaternions> (&state_estimated_map_(3));
}

#endif /* STATE_POSE_H_ */
