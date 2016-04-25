/*
 * ceres.h
 *
 *  Created on: Apr 4, 2016
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_H_
#define SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_H_

// Constraints
#include "../constraint_sparse.h"
#include "../constraint_fix.h"
#include "../constraint_gps_2D.h"
#include "../constraint_gps_pseudorange_3D.h"
#include "../constraint_gps_pseudorange_2D.h"
#include "../constraint_odom_2D.h"
#include "../constraint_corner_2D.h"
#include "../constraint_container.h"

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

        /* For adding a new constraint, add the #include and a case:
        case CTR_ENUM:
            if (_use_wolf_autodiff)
                return createAutoDiffCostFunctionWrapper<ConstraintType>(_ctr_ptr);
            else
                return createAutoDiffCostFunctionCeres<ConstraintType>(_ctr_ptr);
         */

        default:
            throw std::invalid_argument( "Unknown constraint type! Please add it in the file: ceres_wrapper/create_auto_diff_cost_function.h" );
    }
}

} // namespace wolf

#endif /* SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_H_ */
