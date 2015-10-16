#include "ceres_manager.h"

CeresManager::CeresManager(ceres::Problem::Options _options) :
	ceres_problem_(new ceres::Problem(_options))
{
	ceres::Covariance::Options covariance_options;
    //covariance_options.algorithm_type = ceres::SUITE_SPARSE_QR;//ceres::DENSE_SVD;
    covariance_options.num_threads = 8;//ceres::DENSE_SVD;
	//covariance_options.null_space_rank = -1;
	covariance_ = new ceres::Covariance(covariance_options);
}

CeresManager::~CeresManager()
{
	removeAllStateUnits();

	std::cout << "all state units removed! \n";
	std::cout << "residual blocks: " << ceres_problem_->NumResidualBlocks() << "\n";
	std::cout << "parameter blocks: " << ceres_problem_->NumParameterBlocks() << "\n";
	delete ceres_problem_;
	delete covariance_;
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

void CeresManager::computeCovariances(WolfProblem* _problem_ptr)
{
    //std::cout << "computing covariances..." << std::endl;

    // CREATE DESIRED COVARIANCES LIST
	std::vector<std::pair<const double*, const double*>> covariance_blocks;

	// Last frame
    StateBase* current_position = _problem_ptr->getTrajectoryPtr()->getFrameListPtr()->back()->getPPtr();
    StateBase* current_orientation = _problem_ptr->getTrajectoryPtr()->getFrameListPtr()->back()->getOPtr();
	double* current_position_ptr = current_position->getPtr();
	double* current_orientation_ptr = current_orientation->getPtr();
	covariance_blocks.push_back(std::make_pair(current_position_ptr,current_position_ptr));
	covariance_blocks.push_back(std::make_pair(current_position_ptr,current_orientation_ptr));
	covariance_blocks.push_back(std::make_pair(current_orientation_ptr,current_orientation_ptr));

	// Landmarks and cross-covariance with current frame
	for(auto landmark_it = _problem_ptr->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it!=_problem_ptr->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++)
	{
            double* landmark_position_ptr = (*landmark_it)->getPPtr()->getPtr();
            double* landmark_orientation_ptr = (*landmark_it)->getOPtr()->getPtr();

            covariance_blocks.push_back(std::make_pair(landmark_position_ptr,landmark_position_ptr));
            covariance_blocks.push_back(std::make_pair(landmark_position_ptr,landmark_orientation_ptr));
            covariance_blocks.push_back(std::make_pair(landmark_orientation_ptr,landmark_orientation_ptr));
            covariance_blocks.push_back(std::make_pair(landmark_position_ptr,current_position_ptr));
            covariance_blocks.push_back(std::make_pair(landmark_position_ptr,current_orientation_ptr));
            covariance_blocks.push_back(std::make_pair(landmark_orientation_ptr,current_position_ptr));
            covariance_blocks.push_back(std::make_pair(landmark_orientation_ptr,current_orientation_ptr));
	}

	// COMPUTE DESIRED COVARIANCES
	if (covariance_->Compute(covariance_blocks, ceres_problem_))
	{
        // STORE DESIRED COVARIANCES
        // Last frame
        Eigen::MatrixXs m_pp(current_position->getStateSize(),current_position->getStateSize());
        Eigen::MatrixXs m_oo(current_orientation->getStateSize(),current_orientation->getStateSize());
        Eigen::MatrixXs m_po(current_position->getStateSize(),current_orientation->getStateSize());

        //std::cout << "getting m_pp covariance block... " << m_pp.rows() << "x" << m_pp.cols() << std::endl;
        covariance_->GetCovarianceBlock(current_position_ptr, current_position_ptr, m_pp.data());
        //std::cout << "getting m_oo covariance block... " << m_oo.rows() << "x" << m_oo.cols() << std::endl;
        covariance_->GetCovarianceBlock(current_position_ptr, current_orientation_ptr, m_po.data());
        //std::cout << "getting m_po covariance block... " << m_po.rows() << "x" << m_po.cols() << std::endl;
        covariance_->GetCovarianceBlock(current_orientation_ptr, current_orientation_ptr, m_oo.data());

        _problem_ptr->addCovarianceBlock(current_position, current_position, m_pp);
        _problem_ptr->addCovarianceBlock(current_orientation, current_orientation, m_oo);
        _problem_ptr->addCovarianceBlock(current_position, current_orientation, m_po);

        // Landmarks and cross-covariance with current frame
        for(auto landmark_it = _problem_ptr->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it!=_problem_ptr->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++)
        {
                StateBase* landmark_position = (*landmark_it)->getPPtr();
                StateBase* landmark_orientation = (*landmark_it)->getOPtr();
                double* landmark_position_ptr = landmark_position->getPtr();
                double* landmark_orientation_ptr = landmark_orientation->getPtr();

                Eigen::MatrixXs m_landmark_pp(landmark_position->getStateSize(),landmark_position->getStateSize());
                Eigen::MatrixXs m_landmark_po(landmark_position->getStateSize(),landmark_orientation->getStateSize());
                Eigen::MatrixXs m_landmark_oo(landmark_orientation->getStateSize(),landmark_orientation->getStateSize());
                Eigen::MatrixXs m_landmark_p_frame_p(landmark_position->getStateSize(),current_position->getStateSize());
                Eigen::MatrixXs m_landmark_p_frame_o(landmark_position->getStateSize(),current_orientation->getStateSize());
                Eigen::MatrixXs m_landmark_o_frame_p(landmark_orientation->getStateSize(),current_position->getStateSize());
                Eigen::MatrixXs m_landmark_o_frame_o(landmark_orientation->getStateSize(),current_orientation->getStateSize());

                //std::cout << "getting m_landmark_pp covariance block... " << m_landmark_pp.rows() << "x" << m_landmark_pp.cols() << std::endl;
                covariance_->GetCovarianceBlock(landmark_position_ptr, landmark_position_ptr, m_landmark_pp.data());
                //std::cout << "getting m_landmark_po covariance block... " << m_landmark_po.rows() << "x" << m_landmark_po.cols() << std::endl;
                covariance_->GetCovarianceBlock(landmark_position_ptr, landmark_orientation_ptr, m_landmark_po.data());
                //std::cout << "getting m_landmark_oo covariance block... " << m_landmark_oo.rows() << "x" << m_landmark_oo.cols() << std::endl;
                covariance_->GetCovarianceBlock(landmark_orientation_ptr, landmark_orientation_ptr, m_landmark_oo.data());
                //std::cout << "getting m_landmark_p_frame_p covariance block... " << m_landmark_p_frame_p.rows() << "x" << m_landmark_p_frame_p.cols() << std::endl;
                covariance_->GetCovarianceBlock(landmark_position_ptr, current_position_ptr, m_landmark_p_frame_p.data());
                //std::cout << "getting m_landmark_p_frame_o covariance block... " << m_landmark_p_frame_o.rows() << "x" << m_landmark_p_frame_o.cols() << std::endl;
                covariance_->GetCovarianceBlock(landmark_position_ptr, current_orientation_ptr, m_landmark_p_frame_o.data());
                //std::cout << "getting m_landmark_o_frame_p covariance block... " << m_landmark_o_frame_p.rows() << "x" << m_landmark_o_frame_p.cols() << std::endl;
                covariance_->GetCovarianceBlock(landmark_orientation_ptr, current_position_ptr, m_landmark_o_frame_p.data());
                //std::cout << "getting m_landmark_o_frame_o covariance block... " << m_landmark_o_frame_o.rows() << "x" << m_landmark_o_frame_o.cols() << std::endl;
                covariance_->GetCovarianceBlock(landmark_orientation_ptr, current_orientation_ptr, m_landmark_o_frame_o.data());

                _problem_ptr->addCovarianceBlock(landmark_position, landmark_position, m_landmark_pp);
                _problem_ptr->addCovarianceBlock(landmark_position, landmark_orientation, m_landmark_po);
                _problem_ptr->addCovarianceBlock(landmark_orientation, landmark_orientation, m_landmark_oo);
                _problem_ptr->addCovarianceBlock(landmark_position, current_position, m_landmark_p_frame_p);
                _problem_ptr->addCovarianceBlock(landmark_position, current_orientation, m_landmark_p_frame_o);
                _problem_ptr->addCovarianceBlock(landmark_orientation, current_position, m_landmark_o_frame_p);
                _problem_ptr->addCovarianceBlock(landmark_orientation, current_orientation, m_landmark_o_frame_o);
        }
	}
	else
	    std::cout << "WARNING: Couldn't compute covariances!" << std::endl;
}

void CeresManager::update(WolfProblem* _problem_ptr)
{
	// IF REALLOCATION OF STATE, REMOVE EVERYTHING AND BUILD THE PROBLEM AGAIN
	if (_problem_ptr->isReallocated())
	{
		// Remove all parameter blocks (residual blocks will be also removed)
		removeAllStateUnits();

		// Add all parameter blocks
		for(auto state_unit_it = _problem_ptr->getStateListPtr()->begin(); state_unit_it!=_problem_ptr->getStateListPtr()->end(); state_unit_it++)
			addStateUnit(*state_unit_it);

		// Add all residual blocks
		ConstraintBaseList ctr_list;
		_problem_ptr->getTrajectoryPtr()->getConstraintList(ctr_list);
		for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
			addConstraint(*ctr_it);

		// set the wolf problem reallocation flag to false
		_problem_ptr->reallocationDone();
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
			ceres_problem_->RemoveParameterBlock(_problem_ptr->getRemovedStateListPtr()->front());
			_problem_ptr->getRemovedStateListPtr()->pop_front();
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

void CeresManager::addConstraint(ConstraintBase* _corr_ptr)
{
	ceres::ResidualBlockId blockIdx = ceres_problem_->AddResidualBlock(createCostFunction(_corr_ptr), NULL, _corr_ptr->getStateBlockPtrVector());
//	constraint_map_[_corr_ptr->nodeId()] = blockIdx;
	_corr_ptr->setPendingStatus(NOT_PENDING);
}

void CeresManager::removeConstraint(const unsigned int& _corr_idx)
{
	// TODO: necessari? outliers?
//	ceres_problem_->RemoveResidualBlock(constraint_map_[_corr_idx]);
//	constraint_map_.erase(_corr_idx);
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

void CeresManager::removeAllStateUnits()
{
	std::vector<double*> parameter_blocks;

	ceres_problem_->GetParameterBlocks(&parameter_blocks);

	for (unsigned int i = 0; i< parameter_blocks.size(); i++)
		ceres_problem_->RemoveParameterBlock(parameter_blocks[i]);
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
        case CTR_FIX:
        {
            ConstraintFix* specific_ptr = (ConstraintFix*)(_corrPtr);
            return new ceres::AutoDiffCostFunction<ConstraintFix,
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
        case CTR_CONTAINER:
        {
            ConstraintContainer* specific_ptr = (ConstraintContainer*)(_corrPtr);
            return new ceres::AutoDiffCostFunction<ConstraintContainer,
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
