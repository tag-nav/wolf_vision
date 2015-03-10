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
#include "constraint_corner_2D_theta.h"

// ceres wrapper includes
#include "ceres_wrapper/complex_angle_parametrization.h"

/** \brief Ceres manager for WOLF
 *
 */

class CeresManager
{
	protected:
//		std::map<unsigned int, ceres::ResidualBlockId> constraint_map_;
		ceres::Problem* ceres_problem_;
		ceres::Covariance* covariance_;

	public:
		CeresManager(ceres::Problem::Options _options);

		~CeresManager();

		ceres::Solver::Summary solve(const ceres::Solver::Options& _ceres_options);

		void computeCovariances(WolfProblem* _problem_ptr);//StateBaseList* _state_units_list, StateBase* _current_state_unit);

		void update(const WolfProblemPtr _problem_ptr);

		void addConstraint(ConstraintBase* _corr_ptr);

		// TODO: not necessary
		void addConstraints(ConstraintBaseList* _new_constraints_list_ptr);

		// TODO: not necessary
		void removeConstraint(const unsigned int& _corr_idx);

		// TODO: not necessary
		void removeConstraints(const std::list<unsigned int>& _corr_idx_list);

		void addStateUnit(StateBase* _st_ptr);

		// TODO: not necessary
		void addStateUnits(StateBaseList* _st_ptr_list);

		void removeStateUnit(WolfScalar* _st_ptr);

		void removeStateUnits(std::list<WolfScalar*> _st_ptr_list);

		void updateStateUnitStatus(StateBase* _st_ptr);

		// TODO: not necessary
		void updateStateUnitStatus(StateBaseList* _st_ptr_list);

		ceres::CostFunction* createCostFunction(ConstraintBase* _corrPtr);
};

#endif
