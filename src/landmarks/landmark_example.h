#ifndef _LANDMARK_LANDMARK_BASE_LANDMARK_EXAMPLE_H_
#define _LANDMARK_LANDMARK_BASE_LANDMARK_EXAMPLE_H_

//Wolf includes
#include "wolf.h"
#include "landmark_base.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(LandmarkLandmarkLandmarkExample);

class LandmarkLandmarkLandmarkExample : public LandmarkLandmarkBase
{
    public:

        /** \brief Class constructor
         */
        // TODO Modify this default API to suit your class needs
        LandmarkLandmarkLandmarkExample( StateBlockPtr _p_ptr, StateBlockPtr _o_ptr = nullptr);

        /** \brief Class Destructor
         */
        virtual ~LandmarkLandmarkLandmarkExample();
};

} // namespace wolf

#endif /* _LANDMARK_LANDMARK_BASE_LANDMARK_EXAMPLE_H_ */
