/*
 * create_numerical_diff_cost_function.h
 *
 *  Created on: Apr 5, 2016
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_
#define SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_

// Constraints
#include "../constraint_odom_2D.h"

// Wolf and ceres auto_diff creators
#include "create_numeric_diff_cost_function_ceres.h"

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

#endif /* SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_ */
