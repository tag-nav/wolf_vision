#ifndef CERES_MANAGER_H_
#define CERES_MANAGER_H_

//wolf includes
#include "../wolf.h"
#include "../state_block.h"
#include "../state_point.h"
#include "../state_complex_angle.h"
#include "../state_theta.h"
#include "../constraint_sparse.h"
#include "../constraint_gps_2D.h"
#include "../constraint_odom_2D_theta.h"
#include "../constraint_odom_2D_complex_angle.h"
#include "../constraint_corner_2D_theta.h"

/** \brief solver manager for WOLF
 *
 */

class SolverManager
{
	protected:


	public:
		SolverManager(ceres::Problem::Options _options);

		~SolverManager();

		ceres::Solver::Summary solve(const ceres::Solver::Options& _ceres_options);

		//void computeCovariances(WolfProblemPtr _problem_ptr);

		void update(const WolfProblemPtr _problem_ptr);

		void addConstraint(ConstraintBasePtr _corr_ptr);

		void removeConstraint(const unsigned int& _corr_idx);

		void addStateUnit(StateBlock* _st_ptr);

		void removeAllStateUnits();

		void updateStateUnitStatus(StateBlock* _st_ptr);

		ceres::CostFunction* createCostFunction(ConstraintBasePtr _corrPtr);
};

#endif
