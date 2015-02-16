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

void CeresManager::addCorrespondences(std::list<CorrespondenceBasePtr>& _new_correspondences)
{
	//std::cout << _new_correspondences.size() << " new correspondences\n";
	while (!_new_correspondences.empty())
	{
		addCorrespondence(_new_correspondences.front());
		_new_correspondences.pop_front();
	}
}

void CeresManager::removeCorrespondences()
{
	for (uint i = 0; i<correspondence_list_.size(); i++)
	{
		ceres_problem_->RemoveResidualBlock(correspondence_list_.at(i).first);
	}
	correspondence_list_.clear();
}

void CeresManager::addCorrespondence(const CorrespondenceBasePtr& _corr_ptr)
{
	ceres::ResidualBlockId blockIdx = ceres_problem_->AddResidualBlock(createCostFunction(_corr_ptr), NULL, _corr_ptr->getStateBlockPtrVector());
	correspondence_list_.push_back(std::pair<ceres::ResidualBlockId, CorrespondenceBasePtr>(blockIdx,_corr_ptr));
}

void CeresManager::addStateUnits(std::list<StateBasePtr>& _new_state_units)
{
	while (!_new_state_units.empty())
	{
		addStateUnit(_new_state_units.front());
		_new_state_units.pop_front();
	}
}

void CeresManager::removeStateUnit(WolfScalar* _st_ptr)
{
	ceres_problem_->RemoveParameterBlock(_st_ptr);
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

ceres::CostFunction* CeresManager::createCostFunction(const CorrespondenceBasePtr& _corrPtr)
{
	switch (_corrPtr->getCorrespondenceType())
	{
		case CORR_GPS_FIX_2D:
		{
			CorrespondenceGPS2D* specific_ptr = (CorrespondenceGPS2D*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<CorrespondenceGPS2D,
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
		case CORR_ODOM_2D_COMPLEX_ANGLE:
		{
			CorrespondenceOdom2DComplexAngle* specific_ptr = (CorrespondenceOdom2DComplexAngle*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<CorrespondenceOdom2DComplexAngle,
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
		case CORR_ODOM_2D_THETA:
		{
			CorrespondenceOdom2DTheta* specific_ptr = (CorrespondenceOdom2DTheta*)(_corrPtr);
			return new ceres::AutoDiffCostFunction<CorrespondenceOdom2DTheta,
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
			std::cout << "Unknown correspondence type! Please add it in the CeresWrapper::createCostFunction()" << std::endl;

			return nullptr;
	}
}
