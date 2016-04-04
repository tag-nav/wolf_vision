/*
 * ceres.h
 *
 *  Created on: Apr 4, 2016
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_DERIVED_CONSTRAINT_AUTO_DIFF_COST_FUNCTION_H_
#define SRC_CERES_WRAPPER_DERIVED_CONSTRAINT_AUTO_DIFF_COST_FUNCTION_H_

#include "auto_diff_cost_function_wrapper.h"
// Constraints includes
#include "../constraint_sparse.h"
#include "../constraint_fix.h"
#include "../constraint_gps_2D.h"
#include "../constraint_gps_pseudorange_3D.h"
#include "../constraint_gps_pseudorange_2D.h"
#include "../constraint_odom_2D.h"
#include "../constraint_corner_2D.h"
#include "../constraint_container.h"

ceres::CostFunction* derivedConstraintAutoDiffCostFunction(ConstraintBase* _ctr_ptr)
{
    switch (_ctr_ptr->getType())
    {
        case CTR_GPS_FIX_2D:
            return new AutoDiffCostFunctionWrapper<ConstraintGPS2D>(_ctr_ptr);

        case CTR_FIX:
            return new AutoDiffCostFunctionWrapper<ConstraintFix>(_ctr_ptr);

        case CTR_ODOM_2D:
            return new AutoDiffCostFunctionWrapper<ConstraintOdom2D>(_ctr_ptr);

        case CTR_CORNER_2D:
            return new AutoDiffCostFunctionWrapper<ConstraintCorner2D>(_ctr_ptr);

        case CTR_CONTAINER:
            return new AutoDiffCostFunctionWrapper<ConstraintContainer>(_ctr_ptr);

        case CTR_GPS_PR_3D:
            return new AutoDiffCostFunctionWrapper<ConstraintGPSPseudorange3D>(_ctr_ptr);

        case CTR_GPS_PR_2D:
            return new AutoDiffCostFunctionWrapper<ConstraintGPSPseudorange2D>(_ctr_ptr);

        default:
            throw std::invalid_argument( "Unknown constraint type! Please add it in the file: ceres_wrapper/derived_constraint_auto_diff_cost_function.h" );
    }
}

#endif /* SRC_CERES_WRAPPER_DERIVED_CONSTRAINT_AUTO_DIFF_COST_FUNCTION_H_ */
