#ifndef CERES_MANAGER_H_
#define CERES_MANAGER_H_

//Ceres includes
#include "ceres/jet.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

//wof includes
#include "../wolf.h"
#include "../state_base.h"
#include "../state_point.h"
#include "../state_complex_angle.h"
#include "../state_theta.h"
#include "../constraint_sparse.h"
#include "../constraint_fix.h"
#include "../constraint_gps_2D.h"
#include "../constraint_odom_2D_theta.h"
#include "../constraint_odom_2D_complex_angle.h"
#include "../constraint_corner_2D_theta.h"
#include "../constraint_container.h"

// ceres wrapper includes
#include "complex_angle_parametrization.h"

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

		void computeCovariances(WolfProblem* _problem_ptr);

		void update(const WolfProblemPtr _problem_ptr);

		void addConstraint(ConstraintBase* _corr_ptr);

		// TODO: not necessary?
		void removeConstraint(const unsigned int& _corr_idx);

		void addStateUnit(StateBase* _st_ptr);

		void removeAllStateUnits();

		void updateStateUnitStatus(StateBase* _st_ptr);

		ceres::CostFunction* createCostFunction(ConstraintBase* _corrPtr);
};

#endif
