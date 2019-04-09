#ifndef CERES_MANAGER_H_
#define CERES_MANAGER_H_

//wolf includes
#include "base/factor/factor_GPS_2D.h"
#include "base/wolf.h"
#include "base/state_block.h"
#include "../state_point.h"
#include "../state_complex_angle.h"
#include "../state_theta.h"
#include "../factor_sparse.h"
#include "../factor_odom_2D_theta.h"
#include "../factor_odom_2D_complex_angle.h"
#include "../factor_corner_2D_theta.h"

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

		void addFactor(FactorBasePtr _fac_ptr);

		void removeFactor(const unsigned int& _fac_idx);

		void addStateUnit(StateBlockPtr _st_ptr);

		void removeAllStateUnits();

		void updateStateUnitStatus(StateBlockPtr _st_ptr);

		ceres::CostFunction* createCostFunction(FactorBasePtr _fac_ptr);
};

#endif
