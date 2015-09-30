#ifndef STATE_QUATERNION_H_
#define STATE_QUATERNION_H_

//std includes
#include <iostream>
#include <vector>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "state_orientation.h"

//class StateQuaternion
//Last component is the real part.
class StateQuaternion : public StateOrientation
{
    public:
        static const unsigned int BLOCK_SIZE = 4;

    public:

        /** \brief Constructor with whole state storage and index where starts the state unit
         *
         * Constructor with whole state storage and index where starts the state unit
         * \param _st_remote is the whole state vector
         * \param _idx is the index of the first element of the state in the whole state vector
         *
         **/
        StateQuaternion(Eigen::VectorXs& _st_remote, const unsigned int _idx);

        /** \brief Constructor with scalar pointer
         *
         * Constructor with scalar pointer
         * \param _st_ptr is the pointer of the first element of the state unit
         *
         **/
        StateQuaternion(WolfScalar* _st_ptr);

        /** \brief Destructor
         *
         * Destructor
         *
         **/
        virtual ~StateQuaternion();

        /** \brief Returns the parametrization of the state unit
         *
         * Returns the parametrizationType (see wolf.h) of the state unit
         *
         **/
        virtual StateType getStateType() const;

        /** \brief Returns the state unit size
         *
         * Returns the parametrizationType (see wolf.h) of the state unit
         *
         **/
        virtual unsigned int getStateSize() const;

        /** \brief Returns the 3x3 rotation matrix of the orientation
         *
         * Returns the 3x3 rotation matrix of the orientation
         *
         **/
        //virtual Eigen::Matrix3s getRotationMatrix() const;
        virtual void rotationMatrix(Eigen::Matrix3s& R) const;

        /** \brief Returns the yaw angle
         *
         * Returns the yaw angle
         *
         **/
        virtual WolfScalar getYaw() const;

        /** \brief Returns a (mapped) vector of the state unit
         *
         * Returns a (mapped) vector of the state unit
         *
         **/
        virtual Eigen::Map<const Eigen::VectorXs> getVector() const;

        /** \brief Returns a (mapped) quaternion of the state unit
         *
         * Returns a (mapped) quaternion of the state unit
         *
         **/
        virtual Eigen::Map<const Eigen::Quaternions> getQuaternion() const;

        /** \brief Normalizes the quaternion
         *
         * Normalizes the quaternion
         *
         **/        
        void normalize();
        
        /** \brief Prints all the elements of the state unit
         *
         * Prints all the elements of the state unit
         *
         **/
        virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
};
#endif
