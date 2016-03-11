#ifndef LANDMARK_POINT_2D_H
#define LANDMARK_POINT_2D_H

//Wolf includes
#include "wolf.h"
#include "landmark_base.h"

//std includes

//class LandmarkPoint2D
class LandmarkPoint2D : public LandmarkBase
{
    protected:

    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBase shared pointer to the position
         * \param _o_ptr StateBase shared pointer to the orientation
         * \param _aperture descriptor of the landmark: aperture of the corner
         *
         **/
        LandmarkPoint2D(StateBlock* _p_ptr, const std::vector<float> & _descriptor);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~LandmarkPoint2D();

};
#endif // LANDMARK_POINT_2D_H
