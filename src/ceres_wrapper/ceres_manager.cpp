#include "ceres_manager.h"

CeresManager::CeresManager(ceres::Problem* _ceres_problem) :
	ceres_problem_(_ceres_problem)
{
}

CeresManager::~CeresManager()
{
//			std::vector<double*> state_units;
//			ceres_problem_->GetParameterBlocks(&state_units);
//
//			for (uint i = 0; i< state_units.size(); i++)
//				removeStateUnit(state_units.at(i));
//
//			std::cout << "all state units removed! \n";
//	std::cout << "residual blocks: " << ceres_problem_->NumResidualBlocks() << "\n";
//	std::cout << "parameter blocks: " << ceres_problem_->NumParameterBlocks() << "\n";
}

ceres::Solver::Summary CeresManager::solve(const ceres::Solver::Options& _ceres_options)
{
	// create summary
	ceres::Solver::Summary ceres_summary_;

	// run Ceres Solver
	ceres::Solve(_ceres_options, ceres_problem_, &ceres_summary_);

	//display results
	return ceres_summary_;
}

void CeresManager::addConstraints(const ConstraintBasePtrList& _new_constraints)
{
	//std::cout << _new_constraints.size() << " new constraints\n";
	for(auto constraint_it = _new_constraints.begin(); constraint_it!=_new_constraints.end(); constraint_it++)
		addConstraint(*constraint_it);
}

void CeresManager::removeConstraints()
{
	for (uint i = 0; i<constraint_list_.size(); i++)
	{
		ceres_problem_->RemoveResidualBlock(constraint_list_.at(i).first);
	}
	constraint_list_.clear();
}

void CeresManager::addConstraint(const ConstraintBasePtr& _corr_ptr)
{
	ceres::ResidualBlockId blockIdx = ceres_problem_->AddResidualBlock(createCostFunction(_corr_ptr), NULL, _corr_ptr->getStateBlockPtrVector());
	constraint_list_.push_back(std::pair<ceres::ResidualBlockId, ConstraintBasePtr>(blockIdx,_corr_ptr));
}

void CeresManager::addStateUnits(const StateBasePtrList& _new_state_units)
{
	for(auto state_unit_it = _new_state_units.begin(); state_unit_it!=_new_state_units.end(); state_unit_it++)
		addStateUnit(*state_unit_it);
}

void CeresManager::removeStateUnit(WolfScalar* _st_ptr)
{
	ceres_problem_->RemoveParameterBlock(_st_ptr);
}

void CeresManager::removeStateUnits(std::list<WolfScalar*> _st_ptr_list)
{
	for(auto state_unit_it = _st_ptr_list.begin(); state_unit_it!=_st_ptr_list.end(); state_unit_it++)
		ceres_problem_->RemoveParameterBlock(*state_unit_it);
}

void CeresManager::addStateUnit(const StateBasePtr& _st_ptr)
{
	//std::cout << "Adding a State Unit to wolf_problem... " << std::endl;
	//_st_ptr->print();

	switch (_st_ptr->getStateType())
	{
		case ST_COMPLEX_ANGLE:
		{
			//std::cout << "Adding Complex angle Local Parametrization to the List... " << std::endl;
			//ceres_problem_->SetParameterization(_st_ptr->getPtr(), new ComplexAngleParameterization);
			ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), ((StateComplexAngle*)_st_ptr)->BLOCK_SIZE, new ComplexAngleParameterization);
			break;
		}
//				case PARAM_QUATERNION:
//				{
//					std::cout << "Adding Quaternion Local Parametrization to the List... " << std::endl;
//					ceres_problem_->SetParameterization(_st_ptr->getPtr(), new EigenQuaternionParameterization);
//					ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), ((StateQuaternion*)_st_ptr.get())->BLOCK_SIZE, new QuaternionParameterization);
//					break;
//				}
		case ST_POINT_1D:
		case ST_THETA:
		{
			//std::cout << "No Local Parametrization to be added" << std::endl;
			ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), ((StatePoint1D*)_st_ptr)->BLOCK_SIZE, nullptr);
			break;
		}
		case ST_POINT_2D:
		{
			//std::cout << "No Local Parametrization to be added" << std::endl;
			ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), ((StatePoint2D*)_st_ptr)->BLOCK_SIZE, nullptr);
			break;
		}
		case ST_POINT_3D:
		{
			//std::cout << "No Local Parametrization to be added" << std::endl;
			ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), ((StatePoint3D*)_st_ptr)->BLOCK_SIZE, nullptr);
			break;
		}
		default:
			std::cout << "Unknown  Local Parametrization type!" << std::endl;
	}
}

ceres::CostFunction* CeresManager::createCostFunction(const ConstraintBasePtr& _corrPtr)
{
	switch (_corrPtr->getConstraintType())
	{
		case CTR_GPS_FIX_2D:
		{
			ConstraintGPS2D* specific_ptr = (ConstraintGPS2D*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<ConstraintGPS2D,
													specific_ptr->measurementSize,
													specific_ptr->block0Size,
													specific_ptr->block1Size,
													specific_ptr->block2Size,
													specific_ptr->block3Size,
													specific_ptr->block4Size,
													specific_ptr->block5Size,
													specific_ptr->block6Size,
													specific_ptr->block7Size,
													specific_ptr->block8Size,
													specific_ptr->block9Size>(specific_ptr);
			break;
		}
		case CTR_ODOM_2D_COMPLEX_ANGLE:
		{
			ConstraintOdom2DComplexAngle* specific_ptr = (ConstraintOdom2DComplexAngle*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<ConstraintOdom2DComplexAngle,
													specific_ptr->measurementSize,
													specific_ptr->block0Size,
													specific_ptr->block1Size,
													specific_ptr->block2Size,
													specific_ptr->block3Size,
													specific_ptr->block4Size,
													specific_ptr->block5Size,
													specific_ptr->block6Size,
													specific_ptr->block7Size,
													specific_ptr->block8Size,
													specific_ptr->block9Size>(specific_ptr);
			break;
		}
		case CTR_ODOM_2D_THETA:
		{
			ConstraintOdom2DTheta* specific_ptr = (ConstraintOdom2DTheta*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<ConstraintOdom2DTheta,
													specific_ptr->measurementSize,
													specific_ptr->block0Size,
													specific_ptr->block1Size,
													specific_ptr->block2Size,
													specific_ptr->block3Size,
													specific_ptr->block4Size,
													specific_ptr->block5Size,
													specific_ptr->block6Size,
													specific_ptr->block7Size,
													specific_ptr->block8Size,
													specific_ptr->block9Size>(specific_ptr);
			break;
		}
		default:
			std::cout << "Unknown constraint type! Please add it in the CeresWrapper::createCostFunction()" << std::endl;

			return nullptr;
	}
}
