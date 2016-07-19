/*
 * create_numerical_diff_cost_function.cpp
 *
 *  Created on: May 18, 2016
 *      Author: jvallve
 */

#include "create_numeric_diff_cost_function.h"

// Constraints
#include "../constraint_odom_2D.h"

// Wolf and ceres auto_diff creators
#include "create_numeric_diff_cost_function_ceres.h"

namespace wolf {

ceres::CostFunction* createNumericDiffCostFunction(ConstraintBase* _ctr_ptr, bool _use_wolf_numericdiff)
{
    if (_use_wolf_numericdiff)
        throw std::invalid_argument( "Numeric differentiation not implemented in wolf" );

    switch (_ctr_ptr->getType())
    {
        // just for testing
        case CTR_ODOM_2D:
            return createNumericDiffCostFunctionCeres<ConstraintOdom2D>(_ctr_ptr);

        /* For adding a new constraint, add the #include and a case:
        case CTR_ENUM:
            return createNumericDiffCostFunctionCeres<ConstraintType>(_ctr_ptr);
         */

        default:
            throw std::invalid_argument( "Unknown constraint type! Please add it in the file: ceres_wrapper/create_Numeric_diff_cost_function.h" );
    }
}

} // namespace wolf
