
#ifndef LANDMARK_CORNER_H_
#define LANDMARK_CORNER_H_

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "landmark_base.h"

//class LandmarkCorner2D
class LandmarkCorner2D : public LandmarkBase
{
    protected:
        
    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _p_ptr StateBase shared pointer to the position
         * \param _p_ptr StateBase shared pointer to the position
         * \param _aperture descriptor of the landmark: aperture of the corner
         *
         **/
		LandmarkCorner2D(StateBase* _p_ptr, StateOrientation* _o_ptr, const WolfScalar& _aperture=0);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~LandmarkCorner2D();
        
        /** \brief Returns aperture
         * 
         * Returns aperture
         * 
         **/
        WolfScalar getAperture() const;         
        
};
#endif
