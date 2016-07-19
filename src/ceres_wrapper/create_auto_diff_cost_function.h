/*
 * create_auto_diff_cost_function.h
 *
 *  Created on: Apr 4, 2016
 *      Author: jvallve
 */

#ifndef SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_H_
#define SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_H_

#include "../constraint_base.h"
#include "ceres/cost_function.h"

namespace wolf {
    ceres::CostFunction* createAutoDiffCostFunction(ConstraintBase* _ctr_ptr, bool _use_wolf_autodiff);

}

#endif /* SRC_CERES_WRAPPER_CREATE_AUTO_DIFF_COST_FUNCTION_H_ */
