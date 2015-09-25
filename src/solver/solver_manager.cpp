#include "ceres_manager.h"

SolverManager::SolverManager()
{

}

SolverManager::~SolverManager()
{
	removeAllStateUnits();
}

void SolverManager::solve()
{

}

//void SolverManager::computeCovariances(WolfProblem* _problem_ptr)
//{
//}

void SolverManager::update(const WolfProblemPtr _problem_ptr)
{
	// IF REALLOCATION OF STATE, REMOVE EVERYTHING AND BUILD THE PROBLEM AGAIN
	if (_problem_ptr->isReallocated())
	{
	    // todo: reallocate x
	}
	else
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
			// TODO: remove state unit
			//_problem_ptr->getRemovedStateListPtr()->pop_front();
		}
		//std::cout << "state units removed!" << std::endl;

		// ADD CONSTRAINTS
		ConstraintBaseList ctr_list;
		_problem_ptr->getTrajectoryPtr()->getConstraintList(ctr_list);
		//std::cout << "ctr_list.size() = " << ctr_list.size() << std::endl;
		for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
			if ((*ctr_it)->getPendingStatus() == ADD_PENDING)
				addConstraint(*ctr_it);

		//std::cout << "constraints updated!" << std::endl;
	}
}

void SolverManager::addConstraint(ConstraintBase* _corr_ptr)
{
	//TODO MatrixXs J; Vector e;
    // getResidualsAndJacobian(_corr_ptr, J, e);
    // solverQR->addConstraint(_corr_ptr, J, e);

//	constraint_map_[_corr_ptr->nodeId()] = blockIdx;
	_corr_ptr->setPendingStatus(NOT_PENDING);
}

void SolverManager::removeConstraint(const unsigned int& _corr_idx)
{
    // TODO
}

void SolverManager::addStateUnit(StateBase* _st_ptr)
{
	//std::cout << "Adding State Unit " << _st_ptr->nodeId() << std::endl;
	//_st_ptr->print();

	switch (_st_ptr->getStateType())
	{
		case ST_COMPLEX_ANGLE:
		{
		    // TODO
			//std::cout << "Adding Complex angle Local Parametrization to the List... " << std::endl;
			//ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), ((StateComplexAngle*)_st_ptr)->BLOCK_SIZE, new ComplexAngleParameterization);
			break;
		}
		case ST_THETA:
		{
			//std::cout << "No Local Parametrization to be added" << std::endl;
			ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), ((StateTheta*)_st_ptr)->BLOCK_SIZE, nullptr);
			break;
		}
		case ST_POINT_1D:
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

void SolverManager::removeAllStateUnits()
{
	std::vector<double*> parameter_blocks;

	ceres_problem_->GetParameterBlocks(&parameter_blocks);

	for (unsigned int i = 0; i< parameter_blocks.size(); i++)
		ceres_problem_->RemoveParameterBlock(parameter_blocks[i]);
}

void SolverManager::updateStateUnitStatus(StateBase* _st_ptr)
{
    // TODO

//	if (_st_ptr->getStateStatus() == ST_ESTIMATED)
//		ceres_problem_->SetParameterBlockVariable(_st_ptr->getPtr());
//	else if (_st_ptr->getStateStatus() == ST_FIXED)
//		ceres_problem_->SetParameterBlockConstant(_st_ptr->getPtr());
//	else
//		printf("\nERROR: Update state unit status with unknown status");
//
//	_st_ptr->setPendingStatus(NOT_PENDING);
}

ceres::CostFunction* SolverManager::createCostFunction(ConstraintBase* _corrPtr)
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
