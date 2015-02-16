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
#include "correspondence_sparse.h"
#include "correspondence_gps_2D.h"
#include "correspondence_odom_2D_theta.h"
#include "correspondence_odom_2D_complex_angle.h"

// ceres wrapper includes
#include "ceres_wrapper/complex_angle_parametrization.h"

/** \brief Ceres manager for WOLF
 *
 */

class CeresManager
{
	protected:
		std::vector<std::pair<ceres::ResidualBlockId, CorrespondenceBasePtr>> correspondence_list_;
		ceres::Problem* ceres_problem_;

	public:
		CeresManager(ceres::Problem* _ceres_problem);

		~CeresManager();

		ceres::Solver::Summary solve(const ceres::Solver::Options& _ceres_options);

		void addCorrespondences(std::list<CorrespondenceBasePtr>& _new_correspondences);

		void removeCorrespondences();

		void addCorrespondence(const CorrespondenceBasePtr& _corr_ptr);

		void addStateUnits(std::list<StateBasePtr>& _new_state_units);

		void removeStateUnit(WolfScalar* _st_ptr);

		void addStateUnit(const StateBasePtr& _st_ptr);

		ceres::CostFunction* createCostFunction(const CorrespondenceBasePtr& _corrPtr);
};

#endif
