
#ifndef LANDMARK_CONTAINER_H_
#define LANDMARK_CONTAINER_H_

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "landmark_base.h"
#include "landmark_corner_2D.h"

//class LandmarkContainer
class LandmarkContainer : public LandmarkBase
{
    protected:
        Eigen::MatrixXs corners_;

    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBase pointer to the position
         * \param _o_ptr StateOrientation pointer to the orientation
         * \param _witdh descriptor of the landmark: container width
         * \param _length descriptor of the landmark: container length
         *
         **/
		LandmarkContainer(StateBase* _p_ptr, StateOrientation* _o_ptr, const WolfScalar& _witdh=2.44, const WolfScalar& _length=12.2);

		/** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBase pointer to the position
         * \param _o_ptr StateOrientation pointer to the orientation
         * \param _corner_1_pose pose of corner 1
         * \param _corner_2_pose pose of corner 2
         * \param _corner_1_idx index of corner 1 (A = 0, B = 1, C = 2, D = 3)
         * \param _corner_2_idx index of corner 2 (A = 0, B = 1, C = 2, D = 3)
         * \param _witdh descriptor of the landmark: container width
         * \param _length descriptor of the landmark: container length
         *
         **/
		LandmarkContainer(StateBase* _p_ptr, StateOrientation* _o_ptr, const Eigen::Vector3s& _corner_1_pose, const Eigen::Vector3s& _corner_2_pose, const int& _corner_1_idx, const int& _corner_2_idx, const WolfScalar& _witdh=2.44, const WolfScalar& _length=12.2);

        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBase pointer to the position
         * \param _o_ptr StateOrientation pointer to the orientation
         * \param _corner_1_ptr LandmarkCorner2D pointer to one of its corners
         * \param _corner_2_ptr LandmarkCorner2D pointer to one of its corners
         * \param _witdh descriptor of the landmark: container width
         * \param _length descriptor of the landmark: container length
         *
         **/
		LandmarkContainer(StateBase* _p_ptr, StateOrientation* _o_ptr, LandmarkCorner2D* _corner_A_ptr, LandmarkCorner2D* _corner_B_ptr, LandmarkCorner2D* _corner_C_ptr, LandmarkCorner2D* _corner_D_ptr, const WolfScalar& _witdh=2.44, const WolfScalar& _length=12.2);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~LandmarkContainer();
        
        /** \brief Returns the container width
         *
         * Returns the container width
         *
         **/
        WolfScalar getWidth() const;

        /** \brief Returns the container length
         * 
         * Returns the container length
         * 
         **/
        WolfScalar getLength() const;

        /** \brief Returns the container corners in container coordinates
         *
         * Returns the container corners in container coordinates
         *
         **/
        Eigen::MatrixXs getCorners() const;

        /** \brief Returns a corner in container coordinates
         *
         * Returns a corner in container coordinates
         * \param _id the index of the corner to be provided
         *
         **/
        Eigen::VectorXs getCorner(const unsigned int _id) const;

        /** \brief Tries to add a corner of the container
         *
         * Tries to add a corner of the container. Return {0, 1, 2, 3} if it matches with any of its corners and -1 otherwise.
         *
         **/
        int tryCorner(LandmarkCorner2D* _corner_ptr);
        
};

// NOT USEFUL -> capture_laser_2D
//bool fitContainer(const Eigen::VectorXs& _corner_1_p, const WolfScalar& _corner_1_orientation, const Eigen::VectorXs& _corner_2_p, const WolfScalar& _corner_2_orientation, int& corner_1_idx, int& corner_2_idx);

#endif
