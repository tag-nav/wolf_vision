
#ifndef LANDMARK_CORNER_H_
#define LANDMARK_CORNER_H_

//Wolf includes
#include "landmark_base.h"

// Std includes


namespace wolf {

//forward declaration to typedef class pointers
class LandmarkCorner2D;
typedef std::shared_ptr<LandmarkCorner2D> LandmarkCorner2DPtr;
typedef std::shared_ptr<const LandmarkCorner2D> LandmarkCorner2DConstPtr;
typedef std::weak_ptr<LandmarkCorner2D> LandmarkCorner2DWPtr;
    
//class LandmarkCorner2D
class LandmarkCorner2D : public LandmarkBase
{
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
