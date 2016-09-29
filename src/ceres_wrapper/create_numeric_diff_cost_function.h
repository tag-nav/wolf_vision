/*
 * create_numerical_diff_cost_function.h
 *
 *  Created on: Apr 5, 2016
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_
#define SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_

#include "ceres/cost_function.h"
#include "../constraint_base.h"

namespace wolf {

ceres::CostFunction* createNumericDiffCostFunction(ConstraintBasePtr _ctr_ptr, bool _use_wolf_numericdiff);

} // namespace wolf

#endif /* SRC_CERES_WRAPPER_CREATE_NUMERIC_DIFF_COST_FUNCTION_H_ */
