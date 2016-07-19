/*
 * create_auto_diff_cost_function.cpp
 *
 *  Created on: May 18, 2016
 *      Author: jvallve
 */

#include "create_auto_diff_cost_function.h"

// Constraints
#include "../constraint_sparse.h"
#include "../constraint_fix.h"
#include "../constraint_gps_2D.h"
#include "../constraint_gps_pseudorange_3D.h"
#include "../constraint_gps_pseudorange_2D.h"
#include "../constraint_odom_2D.h"
#include "../constraint_corner_2D.h"
#include "../constraint_point_2D.h"
#include "../constraint_point_to_line_2D.h"
#include "../constraint_container.h"
#include "../constraint_image.h"
#include "../constraint_image_new_landmark.h"


// Wolf and ceres auto_diff creators
#include "create_auto_diff_cost_function_wrapper.h"
#include "create_auto_diff_cost_function_ceres.h"

namespace wolf {

ceres::CostFunction* createAutoDiffCostFunction(ConstraintBase* _ctr_ptr, bool _use_wolf_autodiff)
{
    switch (_ctr_ptr->getType())
    {
        case CTR_GPS_FIX_2D:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintGPS2D>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintGPS2D>(_ctr_ptr);

        case CTR_FIX:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintFix>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintFix>(_ctr_ptr);

        case CTR_ODOM_2D:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintOdom2D>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintOdom2D>(_ctr_ptr);

        case CTR_CORNER_2D:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintCorner2D>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintCorner2D>(_ctr_ptr);

        case CTR_CONTAINER:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintContainer>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintContainer>(_ctr_ptr);

        case CTR_GPS_PR_3D:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintGPSPseudorange3D>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintGPSPseudorange3D>(_ctr_ptr);

        case CTR_GPS_PR_2D:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintGPSPseudorange2D>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintGPSPseudorange2D>(_ctr_ptr);

        case CTR_POINT_2D:
            if (_use_wolf_autodiff)
            	return createAutoDiffCostFunctionWrapper<ConstraintPoint2D>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintPoint2D>(_ctr_ptr);

        case CTR_POINT_TO_LINE_2D:
            if (_use_wolf_autodiff)
            	return createAutoDiffCostFunctionWrapper<ConstraintPointToLine2D>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintPointToLine2D>(_ctr_ptr);

        case CTR_EPIPOLAR:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintImage>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintImage>(_ctr_ptr);

        case CTR_EPIPOLAR_NL:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintImageNewLandmark>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintImageNewLandmark>(_ctr_ptr);


            /* For adding a new constraint, add the #include and a case:
            case CTR_ENUM:
                if (_use_wolf_autodiff)
                    return createAutoDiffCostFunctionWrapper<ConstraintType>(_ctr_ptr);
                else
                    return createAutoDiffCostFunctionCeres<ConstraintType>(_ctr_ptr);
             */

        default:
            throw std::invalid_argument( "Unknown constraint type! Please add it in the file: ceres_wrapper/create_auto_diff_cost_function.cpp" );
    }
}

} // namespace wolf
