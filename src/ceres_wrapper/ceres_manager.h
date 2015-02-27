#ifndef CERES_MANAGER_H_
#define CERES_MANAGER_H_

//Ceres includes
#include "ceres/jet.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

//wof includes
#include "wolf.h"
#include "state_base.h"
#include "state_point.h"
#include "state_complex_angle.h"
#include "constraint_sparse.h"
#include "constraint_gps_2D.h"
#include "constraint_odom_2D_theta.h"
#include "constraint_odom_2D_complex_angle.h"

// ceres wrapper includes
#include "ceres_wrapper/complex_angle_parametrization.h"

/** \brief Ceres manager for WOLF
 *
 */

class CeresManager
{
	protected:
		std::vector<std::pair<ceres::ResidualBlockId, ConstraintBasePtr>> constraint_list_;
		ceres::Problem* ceres_problem_;

	public:
		CeresManager(ceres::Problem* _ceres_problem);

		~CeresManager();

		ceres::Solver::Summary solve(const ceres::Solver::Options& _ceres_options);

		void addConstraints(const ConstraintBasePtrList& _new_constraints);

		void removeConstraints();

		void addConstraint(const ConstraintBasePtr& _corr_ptr);

		void addStateUnits(const StateBasePtrList& _new_state_units);

		void removeStateUnit(WolfScalar* _st_ptr);

		void removeStateUnits(std::list<WolfScalar*> _st_ptr_list);

		void addStateUnit(const StateBasePtr& _st_ptr);

		ceres::CostFunction* createCostFunction(const ConstraintBasePtr& _corrPtr);
};

#endif
