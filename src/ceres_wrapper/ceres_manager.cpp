#include "ceres_manager.h"

CeresManager::CeresManager(ceres::Problem::Options _options) :
	ceres_problem_(new ceres::Problem(_options)),
	covariance_(covariance_options_)
{
}

CeresManager::~CeresManager()
{
	std::vector<double*> state_units;

	ceres_problem_->GetParameterBlocks(&state_units);

	for (uint i = 0; i< state_units.size(); i++)
		removeStateUnit(state_units.at(i));

	std::cout << "all state units removed! \n";
	std::cout << "residual blocks: " << ceres_problem_->NumResidualBlocks() << "\n";
	std::cout << "parameter blocks: " << ceres_problem_->NumParameterBlocks() << "\n";
	delete ceres_problem_;
}

ceres::Solver::Summary CeresManager::solve(const ceres::Solver::Options& _ceres_options)
{
	//std::cout << "Residual blocks: " << ceres_problem_->NumResidualBlocks() <<  " Parameter blocks: " << ceres_problem_->NumParameterBlocks() << std::endl;

	// create summary
	ceres::Solver::Summary ceres_summary_;

	// run Ceres Solver
	ceres::Solve(_ceres_options, ceres_problem_, &ceres_summary_);
	//std::cout << "solved" << std::endl;

	//return results
	return ceres_summary_;
}

void CeresManager::computeCovariances(StateBaseList* _state_list_ptr, StateBase* _current_state_unit)
{
	std::cout << "_state_list_ptr.size() " << _state_list_ptr->size() << std::endl;

	// create vector of pointer pairs
	std::vector<std::pair<const double*, const double*>> covariance_blocks;
//	for (auto st_it = _state_list_ptr->begin(); st_it != _state_list_ptr->end(); st_it++)
//		if ((*st_it)->getPtr() != _current_state_unit->getPtr())
//			covariance_blocks.push_back(std::pair<const double*, const double*>((*st_it)->getPtr(),_current_state_unit->getPtr()));

	WolfScalar* block_1_ptr = _current_state_unit->getPtr();
	WolfScalar* block_2_ptr = _current_state_unit->getPtr();
	std::cout << "are blocks? " << ceres_problem_->HasParameterBlock(block_1_ptr) << ceres_problem_->HasParameterBlock(block_2_ptr) << std::endl;
	covariance_blocks.push_back(std::make_pair(block_1_ptr,block_2_ptr));
	std::cout << "covariance_blocks.size() " << covariance_blocks.size() << std::endl;
	// Compute covariances
	covariance_.Compute(covariance_blocks, ceres_problem_);
}

void CeresManager::update(WolfProblem* _problem_ptr)
{
	// ADD/UPDATE STATE UNITS
	for(auto state_unit_it = _problem_ptr->getStateListPtr()->begin(); state_unit_it!=_problem_ptr->getStateListPtr()->end(); state_unit_it++)
	{
		if ((*state_unit_it)->getPendingStatus() == ADD_PENDING)
			addStateUnit(*state_unit_it);

		else if((*state_unit_it)->getPendingStatus() == UPDATE_PENDING)
			updateStateUnitStatus(*state_unit_it);
	}
	//std::cout << "state units updated!" << std::endl;

	// REMOVE STATE UNITS
	while (!_problem_ptr->getRemovedStateListPtr()->empty())
	{
		removeStateUnit(_problem_ptr->getRemovedStateListPtr()->front());
		_problem_ptr->getRemovedStateListPtr()->pop_front();
	}
	//std::cout << "state units removed!" << std::endl;

	// ADD CONSTRAINTS
	ConstraintBaseList ctr_list;
	_problem_ptr->getTrajectoryPtr()->getConstraintList(ctr_list);
	//std::cout << "ctr_list.size() = " << ctr_list.size() << std::endl;
	for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
		if ((*ctr_it)->getPendingStatus() == ADD_PENDING)
			addConstraint((*ctr_it));

	//std::cout << "constraints updated!" << std::endl;
}

void CeresManager::addConstraint(ConstraintBase* _corr_ptr)
{
	ceres::ResidualBlockId blockIdx = ceres_problem_->AddResidualBlock(createCostFunction(_corr_ptr), NULL, _corr_ptr->getStateBlockPtrVector());
//	constraint_map_[_corr_ptr->nodeId()] = blockIdx;

	_corr_ptr->setPendingStatus(NOT_PENDING);
}

void CeresManager::addConstraints(ConstraintBaseList* _new_constraints_list_ptr)
{
	//std::cout << _new_constraints.size() << " new constraints\n";
	for(auto constraint_it = _new_constraints_list_ptr->begin(); constraint_it!=_new_constraints_list_ptr->end(); constraint_it++)
		addConstraint(*constraint_it);
}

void CeresManager::removeConstraint(const unsigned int& _corr_idx)
{
//	ceres_problem_->RemoveResidualBlock(constraint_map_[_corr_idx]);
//	constraint_map_.erase(_corr_idx);
}

void CeresManager::removeConstraints(const std::list<unsigned int>& _corr_idx_list)
{
	for (auto idx_it=_corr_idx_list.begin(); idx_it!=_corr_idx_list.end(); idx_it++)
		removeConstraint(*idx_it);
}

void CeresManager::addStateUnit(StateBase* _st_ptr)
{
	//std::cout << "Adding State Unit " << _st_ptr->nodeId() << std::endl;
	//_st_ptr->print();

	switch (_st_ptr->getStateType())
	{
		case ST_COMPLEX_ANGLE:
		{
			//std::cout << "Adding Complex angle Local Parametrization to the List... " << std::endl;
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
		case ST_POINT_1D: // equivalent ST_THETA:
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
	if (_st_ptr->getStateStatus() != ST_ESTIMATED)
		updateStateUnitStatus(_st_ptr);

	_st_ptr->setPendingStatus(NOT_PENDING);
}

void CeresManager::addStateUnits(StateBaseList* _new_state_units)
{
	for(auto state_unit_it = _new_state_units->begin(); state_unit_it!=_new_state_units->end(); state_unit_it++)
		addStateUnit(*state_unit_it);
}

void CeresManager::removeStateUnit(WolfScalar* _st_ptr)
{
	ceres_problem_->RemoveParameterBlock(_st_ptr);
}

void CeresManager::removeStateUnits(std::list<WolfScalar*> _st_ptr_list)
{
	for(auto state_unit_it = _st_ptr_list.begin(); state_unit_it!=_st_ptr_list.end(); state_unit_it++)
		removeStateUnit(*state_unit_it);
}

void CeresManager::updateStateUnitStatus(StateBase* _st_ptr)
{
	if (_st_ptr->getStateStatus() == ST_ESTIMATED)
		ceres_problem_->SetParameterBlockVariable(_st_ptr->getPtr());
	else if (_st_ptr->getStateStatus() == ST_FIXED)
		ceres_problem_->SetParameterBlockConstant(_st_ptr->getPtr());
	else
		printf("\nERROR: Update state unit status with unknown status");

	_st_ptr->setPendingStatus(NOT_PENDING);
}

void CeresManager::updateStateUnitStatus(StateBaseList* _st_ptr_list)
{
	for(auto state_unit_it = _st_ptr_list->begin(); state_unit_it!=_st_ptr_list->end(); state_unit_it++)
		updateStateUnitStatus(*state_unit_it);
}

ceres::CostFunction* CeresManager::createCostFunction(ConstraintBase* _corrPtr)
{
	//std::cout << "adding ctr " << _corrPtr->nodeId() << std::endl;
	//_corrPtr->print();

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
		case CTR_CORNER_2D_THETA:
		{
			ConstraintCorner2DTheta* specific_ptr = (ConstraintCorner2DTheta*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<ConstraintCorner2DTheta,
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
