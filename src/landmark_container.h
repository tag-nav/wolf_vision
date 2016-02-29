
#ifndef LANDMARK_CONTAINER_H_
#define LANDMARK_CONTAINER_H_

//Wolf includes
#include "landmark_base.h"
#include "wolf.h"

// Std includes


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
         * \param _o_ptr StateBase pointer to the orientation
         * \param _witdh descriptor of the landmark: container width
         * \param _length descriptor of the landmark: container length
         *
         **/
		LandmarkContainer(StateBlock* _p_ptr, StateBlock* _o_ptr, const WolfScalar& _witdh=2.44, const WolfScalar& _length=12.2);

		/** \brief Constructor with type, time stamp and the position state pointer
		 *
		 *  The vertices are refered as A, B, C and D:
		 *
		 *          B ---------------------------- A
         *          |                              |
         *          |                              |
         *          |           <---+              |
         *          |               |              |
         *          |               v              |
         *          C ---------------------------- D
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBase pointer to the position
         * \param _o_ptr StateBase pointer to the orientation
         * \param _corner_1_pose pose of corner 1
         * \param _corner_2_pose pose of corner 2
         * \param _corner_1_idx index of corner 1 (A = 0, B = 1, C = 2, D = 3)
         * \param _corner_2_idx index of corner 2 (A = 0, B = 1, C = 2, D = 3)
         * \param _witdh descriptor of the landmark: container width
         * \param _length descriptor of the landmark: container length
         *
         **/
		LandmarkContainer(StateBlock* _p_ptr, StateBlock* _o_ptr, const Eigen::Vector3s& _corner_1_pose, const Eigen::Vector3s& _corner_2_pose, const int& _corner_1_idx, const int& _corner_2_idx, const WolfScalar& _witdh=2.44, const WolfScalar& _length=12.2);

        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBase pointer to the position
         * \param _o_ptr StateBase pointer to the orientation
         * \param _corner_A_ptr LandmarkCorner2D pointer to one of its corners
         * \param _corner_B_ptr LandmarkCorner2D pointer to one of its corners
         * \param _corner_C_ptr LandmarkCorner2D pointer to one of its corners
         * \param _corner_D_ptr LandmarkCorner2D pointer to one of its corners
         * \param _witdh descriptor of the landmark: container width
         * \param _length descriptor of the landmark: container length
         *
         **/
		//LandmarkContainer(StateBlock* _p_ptr, StateBlock* _o_ptr, LandmarkCorner2D* _corner_A_ptr, LandmarkCorner2D* _corner_B_ptr, LandmarkCorner2D* _corner_C_ptr, LandmarkCorner2D* _corner_D_ptr, const WolfScalar& _witdh=2.44, const WolfScalar& _length=12.2);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
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
        
};

#endif
