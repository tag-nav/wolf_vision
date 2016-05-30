#ifndef LANDMARK_POINT_3D_H
#define LANDMARK_POINT_3D_H


//Wolf includes
#include "landmark_base.h"

// Std includes

namespace wolf {

class LandmarkPoint3D : public LandmarkBase
{
    public:
        LandmarkPoint3D(StateBlock* _p_ptr, StateBlock* _o_ptr);

        virtual ~LandmarkPoint3D();
};

} // namespace wolf

#endif // LANDMARK_POINT_3D_H
