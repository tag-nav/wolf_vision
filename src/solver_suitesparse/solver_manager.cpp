#include "base/ceres_wrapper/ceres_manager.h"

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

//void SolverManager::computeCovariances(WolfProblemPtr _problem_ptr)
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
		for(auto state_unit_it = _problem_ptr->getStateList().begin(); state_unit_it!=_problem_ptr->getStateList().end(); state_unit_it++)
		{
			if ((*state_unit_it)->getPendingStatus() == ADD_PENDING)
				addStateUnit(*state_unit_it);

			else if((*state_unit_it)->getPendingStatus() == UPDATE_PENDING)
				updateStateUnitStatus(*state_unit_it);
		}
		//std::cout << "state units updated!" << std::endl;

		// REMOVE STATE UNITS
		while (!_problem_ptr->getRemovedStateList().empty())
		{
			// TODO: remove state unit
			//_problem_ptr->getRemovedStateList().pop_front();
		}
		//std::cout << "state units removed!" << std::endl;

		// ADD CONSTRAINTS
		FactorBasePtrList fac_list;
		_problem_ptr->getTrajectory()->getFactorList(fac_list);
		//std::cout << "fac_list.size() = " << fac_list.size() << std::endl;
		for(auto fac_it = fac_list.begin(); fac_it!=fac_list.end(); fac_it++)
			if ((*fac_it)->getPendingStatus() == ADD_PENDING)
				addFactor(*fac_it);

		//std::cout << "factors updated!" << std::endl;
	}
}

void SolverManager::addFactor(FactorBasePtr _corr_ptr)
{
	//TODO MatrixXs J; Vector e;
    // getResidualsAndJacobian(_corr_ptr, J, e);
    // solverQR->addFactor(_corr_ptr, J, e);

//	factor_map_[_corr_ptr->id()] = blockIdx;
	_corr_ptr->setPendingStatus(NOT_PENDING);
}

void SolverManager::removeFactor(const unsigned int& _corr_idx)
{
    // TODO
}

void SolverManager::addStateUnit(StateBlockPtr _st_ptr)
{
	//std::cout << "Adding State Unit " << _st_ptr->id() << std::endl;
	//_st_ptr->print();

	switch (_st_ptr->getStateType())
	{
		case ST_COMPLEX_ANGLE:
		{
		    // TODO
			//std::cout << "Adding Complex angle Local Parametrization to the List... " << std::endl;
			//ceres_problem_->AddParameterBlock(_st_ptr->get(), ((StateComplexAngle*)_st_ptr)->BLOCK_SIZE, new ComplexAngleParameterization);
			break;
		}
		case ST_THETA:
		{
			//std::cout << "No Local Parametrization to be added" << std::endl;
			ceres_problem_->AddParameterBlock(_st_ptr->get(), ((StateBlockPtr)_st_ptr)->BLOCK_SIZE, nullptr);
			break;
		}
		case ST_POINT_1D:
		{
			//std::cout << "No Local Parametrization to be added" << std::endl;
			ceres_problem_->AddParameterBlock(_st_ptr->get(), ((StatePoint1D*)_st_ptr)->BLOCK_SIZE, nullptr);
			break;
		}
		case ST_VECTOR:
		{
			//std::cout << "No Local Parametrization to be added" << std::endl;
			ceres_problem_->AddParameterBlock(_st_ptr->get(), ((StateBlockPtr)_st_ptr)->BLOCK_SIZE, nullptr);
			break;
		}
		case ST_POINT_3D:
		{
			//std::cout << "No Local Parametrization to be added" << std::endl;
			ceres_problem_->AddParameterBlock(_st_ptr->get(), ((StateBlockPtr)_st_ptr)->BLOCK_SIZE, nullptr);
			break;
		}
		default:
			std::cout << "Unknown  Local Parametrization type!" << std::endl;
	}
	if (_st_ptr->isFixed())
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

void SolverManager::updateStateUnitStatus(StateBlockPtr _st_ptr)
{
    // TODO

//	if (!_st_ptr->isFixed())
//		ceres_problem_->SetParameterBlockVariable(_st_ptr->get());
//	else if (_st_ptr->isFixed())
//		ceres_problem_->SetParameterBlockConstant(_st_ptr->get());
//	else
//		printf("\nERROR: Update state unit status with unknown status");
//
//	_st_ptr->setPendingStatus(NOT_PENDING);
}

ceres::CostFunction* SolverManager::createCostFunction(FactorBasePtr _corrPtr)
{
	//std::cout << "adding ctr " << _corrPtr->id() << std::endl;
	//_corrPtr->print();

	switch (_corrPtr->getFactorType())
	{
		case FAC_GPS_FIX_2D:
		{
			FactorGPS2D* specific_ptr = (FactorGPS2D*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<FactorGPS2D,
													specific_ptr->residualSize,
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
		case FAC_ODOM_2D_COMPLEX_ANGLE:
		{
			FactorOdom2DComplexAngle* specific_ptr = (FactorOdom2DComplexAngle*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<FactorOdom2DComplexAngle,
													specific_ptr->residualSize,
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
		case FAC_ODOM_2D:
		{
			FactorOdom2D* specific_ptr = (FactorOdom2D*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<FactorOdom2D,
													specific_ptr->residualSize,
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
		case FAC_CORNER_2D:
		{
			FactorCorner2D* specific_ptr = (FactorCorner2D*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<FactorCorner2D,
													specific_ptr->residualSize,
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
		case FAC_IMU:
		{
			FactorIMU* specific_ptr = (FactorIMU*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<FactorIMU,
													specific_ptr->residualSize,
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
			std::cout << "Unknown factor type! Please add it in the CeresWrapper::createCostFunction()" << std::endl;

			return nullptr;
	}
}
