
#ifndef LANDMARK_CORNER_H_
#define LANDMARK_CORNER_H_

//Wolf includes
#include "landmark_base.h"

// Std includes


namespace wolf {

//class LandmarkCorner2D
class LandmarkCorner2D : public LandmarkBase
{
    protected:
        
    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBlock shared pointer to the position
         * \param _o_ptr StateBlock shared pointer to the orientation
         * \param _aperture descriptor of the landmark: aperture of the corner
         *
         **/
		LandmarkCorner2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const Scalar& _aperture=0);

        virtual ~LandmarkCorner2D();
        
        /** \brief Returns aperture
         * 
         * Returns aperture
         * 
         **/
        Scalar getAperture() const;         
        
};

} // namespace wolf

#endif
