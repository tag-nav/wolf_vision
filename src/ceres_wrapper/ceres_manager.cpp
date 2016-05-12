#include "ceres_manager.h"

namespace wolf {

CeresManager::CeresManager(Problem* _wolf_problem, const ceres::Solver::Options& _ceres_options, const bool _use_wolf_auto_diff) :
    ceres_options_(_ceres_options),
    wolf_problem_(_wolf_problem),
    use_wolf_auto_diff_(_use_wolf_auto_diff)
{
    ceres::Covariance::Options covariance_options;
    covariance_options.algorithm_type = ceres::SUITE_SPARSE_QR;//ceres::DENSE_SVD;
    covariance_options.num_threads = 8;//ceres::DENSE_SVD;
    covariance_options.apply_loss_function = false;
    //covariance_options.null_space_rank = -1;
    covariance_ = new ceres::Covariance(covariance_options);

    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;//ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres_problem_ = new ceres::Problem(problem_options);
}

CeresManager::~CeresManager()
{
    while (!id_2_residual_idx_.empty())
        removeConstraint(id_2_residual_idx_.begin()->first);

    removeAllStateBlocks();

	//std::cout << "all state units removed! \n";
	//std::cout << "residual blocks: " << ceres_problem_->NumResidualBlocks() << "\n";
	//std::cout << "parameter blocks: " << ceres_problem_->NumParameterBlocks() << "\n";
	delete covariance_;
    //std::cout << "covariance deleted! \n";
    delete ceres_problem_;
    //std::cout << "ceres problem deleted! \n";
}

ceres::Solver::Summary CeresManager::solve()
{
	//std::cout << "Residual blocks: " << ceres_problem_->NumResidualBlocks() <<  " Parameter blocks: " << ceres_problem_->NumParameterBlocks() << std::endl;

    // update problem
    update();

	// create summary
	ceres::Solver::Summary ceres_summary_;

	// run Ceres Solver
	ceres::Solve(ceres_options_, ceres_problem_, &ceres_summary_);

	//return results
	return ceres_summary_;
}

void CeresManager::computeCovariances(CovarianceBlocksToBeComputed _blocks)
{
    //std::cout << "CeresManager: computing covariances..." << std::endl;

    // update problem
    update();

    // CLEAR STORED COVARIANCE BLOCKS IN WOLF PROBLEM
    wolf_problem_->clearCovariance();

    // CREATE DESIRED COVARIANCES LIST
    std::vector<std::pair<StateBlock*, StateBlock*>> state_block_pairs;
    std::vector<std::pair<const double*, const double*>> double_pairs;

    switch (_blocks)
    {
        case ALL:
        {
            // first create a vector containing all state blocks
            std::vector<StateBlock*> all_state_blocks;
            //frame state blocks
            for(auto fr_ptr : *(wolf_problem_->getTrajectoryPtr()->getFrameListPtr()))
            {
                if (fr_ptr->isKey())
                {
                    all_state_blocks.push_back(fr_ptr->getPPtr());
                    all_state_blocks.push_back(fr_ptr->getOPtr());
                }
            }
            // landmark state blocks
            for(auto l_ptr : *(wolf_problem_->getMapPtr()->getLandmarkListPtr()))
            {
                all_state_blocks.push_back(l_ptr->getPPtr());
                all_state_blocks.push_back(l_ptr->getOPtr());
            }
            // double loop all against all (without repetitions)
            for (unsigned int i = 0; i < all_state_blocks.size(); i++)
            {
                for  (unsigned int j = i; j < all_state_blocks.size(); j++)
                {
                    state_block_pairs.push_back(std::make_pair(all_state_blocks[i],all_state_blocks[j]));
                    double_pairs.push_back(std::make_pair(all_state_blocks[i]->getPtr(),all_state_blocks[j]->getPtr()));
                }
            }
            break;
        }
        case ALL_MARGINALS:
        {
            // first create a vector containing all state blocks
            std::vector<StateBlock*> all_state_blocks;
            //frame state blocks
            for(auto fr_ptr : *(wolf_problem_->getTrajectoryPtr()->getFrameListPtr()))
            {
                if (fr_ptr->isKey())
                {
                    all_state_blocks.push_back(fr_ptr->getPPtr());
                    all_state_blocks.push_back(fr_ptr->getOPtr());
                }
            }
            // landmark state blocks
            for(auto l_ptr : *(wolf_problem_->getMapPtr()->getLandmarkListPtr()))
            {
                all_state_blocks.push_back(l_ptr->getPPtr());
                all_state_blocks.push_back(l_ptr->getOPtr());
            }
            // loop all marginals (PO marginals)
            for (unsigned int i = 0; 2*i+1 < all_state_blocks.size(); i++)
            {
                state_block_pairs.push_back(std::make_pair(all_state_blocks[2*i],all_state_blocks[2*i]));
                state_block_pairs.push_back(std::make_pair(all_state_blocks[2*i],all_state_blocks[2*i+1]));
                state_block_pairs.push_back(std::make_pair(all_state_blocks[2*i+1],all_state_blocks[2*i+1]));

                double_pairs.push_back(std::make_pair(all_state_blocks[2*i]->getPtr(),all_state_blocks[2*i]->getPtr()));
                double_pairs.push_back(std::make_pair(all_state_blocks[2*i]->getPtr(),all_state_blocks[2*i+1]->getPtr()));
                double_pairs.push_back(std::make_pair(all_state_blocks[2*i+1]->getPtr(),all_state_blocks[2*i+1]->getPtr()));
            }
            break;
        }
        case ROBOT_LANDMARKS:
        {
            //robot-robot
            auto last_key_frame = wolf_problem_->getLastKeyFramePtr();

            state_block_pairs.push_back(std::make_pair(last_key_frame->getPPtr(), last_key_frame->getPPtr()));
            state_block_pairs.push_back(std::make_pair(last_key_frame->getPPtr(), last_key_frame->getOPtr()));
            state_block_pairs.push_back(std::make_pair(last_key_frame->getOPtr(), last_key_frame->getOPtr()));

            double_pairs.push_back(std::make_pair(last_key_frame->getPPtr()->getPtr(), last_key_frame->getPPtr()->getPtr()));
            double_pairs.push_back(std::make_pair(last_key_frame->getPPtr()->getPtr(), last_key_frame->getOPtr()->getPtr()));
            double_pairs.push_back(std::make_pair(last_key_frame->getOPtr()->getPtr(), last_key_frame->getOPtr()->getPtr()));

            for(auto l_ptr : *(wolf_problem_->getMapPtr()->getLandmarkListPtr()))
            {
                state_block_pairs.push_back(std::make_pair(last_key_frame->getPPtr(), l_ptr->getPPtr()));
                state_block_pairs.push_back(std::make_pair(last_key_frame->getPPtr(), l_ptr->getOPtr()));
                state_block_pairs.push_back(std::make_pair(last_key_frame->getOPtr(), l_ptr->getPPtr()));
                state_block_pairs.push_back(std::make_pair(last_key_frame->getOPtr(), l_ptr->getOPtr()));
                state_block_pairs.push_back(std::make_pair(l_ptr->getPPtr(), l_ptr->getPPtr()));
                state_block_pairs.push_back(std::make_pair(l_ptr->getPPtr(), l_ptr->getOPtr()));
                state_block_pairs.push_back(std::make_pair(l_ptr->getOPtr(), l_ptr->getOPtr()));

                double_pairs.push_back(std::make_pair(last_key_frame->getPPtr()->getPtr(), l_ptr->getPPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(last_key_frame->getPPtr()->getPtr(), l_ptr->getOPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(last_key_frame->getOPtr()->getPtr(), l_ptr->getPPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(last_key_frame->getOPtr()->getPtr(), l_ptr->getOPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(l_ptr->getPPtr()->getPtr(), l_ptr->getPPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(l_ptr->getPPtr()->getPtr(), l_ptr->getOPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(l_ptr->getOPtr()->getPtr(), l_ptr->getOPtr()->getPtr()));
            }
            break;
        }
    }
    //std::cout << "pairs... " << double_pairs.size() << std::endl;

    // COMPUTE DESIRED COVARIANCES
    if (covariance_->Compute(double_pairs, ceres_problem_))
    {
        // STORE DESIRED COVARIANCES
        for (unsigned int i = 0; i < double_pairs.size(); i++)
        {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov(state_block_pairs[i].first->getSize(),state_block_pairs[i].second->getSize());
            covariance_->GetCovarianceBlock(double_pairs[i].first, double_pairs[i].second, cov.data());
            wolf_problem_->addCovarianceBlock(state_block_pairs[i].first, state_block_pairs[i].second, cov);
            //std::cout << "getted covariance " << std::endl << cov << std::endl;
        }
    }
    else
        std::cout << "WARNING: Couldn't compute covariances!" << std::endl;
}

void CeresManager::update()
{
    //std::cout << "CeresManager: updating... " << std::endl;
    //std::cout << wolf_problem_->getStateBlockNotificationList().size() << " state block notifications" << std::endl;
    //std::cout << wolf_problem_->getConstraintNotificationList().size() << " constraint notifications" << std::endl;

    // UPDATE STATE BLOCKS
    while (!wolf_problem_->getStateBlockNotificationList().empty())
    {
        switch (wolf_problem_->getStateBlockNotificationList().front().notification_)
        {
            case ADD:
            {
                addStateBlock(wolf_problem_->getStateBlockNotificationList().front().state_block_ptr_);
                break;
            }
            case UPDATE:
            {
                updateStateBlockStatus(wolf_problem_->getStateBlockNotificationList().front().state_block_ptr_);
                break;
            }
            case REMOVE:
            {
                removeStateBlock((double *)(wolf_problem_->getStateBlockNotificationList().front().scalar_ptr_));
                break;
            }
            default:
                throw std::runtime_error("CeresManager::update: State Block notification must be ADD, UPATE or REMOVE.");
        }
        wolf_problem_->getStateBlockNotificationList().pop_front();
    }
    // UPDATE CONSTRAINTS
    while (!wolf_problem_->getConstraintNotificationList().empty())
    {
        switch (wolf_problem_->getConstraintNotificationList().front().notification_)
        {
            case ADD:
            {
                addConstraint(wolf_problem_->getConstraintNotificationList().front().constraint_ptr_,wolf_problem_->getConstraintNotificationList().front().id_);
                break;
            }
            case REMOVE:
            {
                removeConstraint(wolf_problem_->getConstraintNotificationList().front().id_);
                break;
            }
            default:
                throw std::runtime_error("CeresManager::update: Constraint notification must be ADD or REMOVE.");
        }
        wolf_problem_->getConstraintNotificationList().pop_front();
    }
}

void CeresManager::addConstraint(ConstraintBase* _ctr_ptr, unsigned int _id)
{
    id_2_costfunction_[_id] = createCostFunction(_ctr_ptr);

    if (_ctr_ptr->getApplyLossFunction())
        id_2_residual_idx_[_id] = ceres_problem_->AddResidualBlock(id_2_costfunction_[_id], new ceres::CauchyLoss(0.5), _ctr_ptr->getStateBlockPtrVector());
    else
        id_2_residual_idx_[_id] = ceres_problem_->AddResidualBlock(id_2_costfunction_[_id], NULL, _ctr_ptr->getStateBlockPtrVector());
}

void CeresManager::removeConstraint(const unsigned int& _corr_id)
{
    //std::cout << "removing constraint " << _corr_id << std::endl;
    assert(id_2_residual_idx_.find(_corr_id) != id_2_residual_idx_.end());
	ceres_problem_->RemoveResidualBlock(id_2_residual_idx_[_corr_id]);
    //std::cout << "residual block removed!" << std::endl;
	id_2_residual_idx_.erase(_corr_id);
	// The cost functions will be deleted by ceres_problem destructor (IT MUST HAVE THE OWNERSHIP)
}

void CeresManager::addStateBlock(StateBlock* _st_ptr)
{
    //std::cout << "Adding State Unit with size: " <<  _st_ptr->getSize() << std::endl;

    if (_st_ptr->hasLocalParametrization())
    {
        //std::cout << "Local Parametrization to be added" << std::endl;
        ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), _st_ptr->getSize(), new LocalParametrizationWrapper(_st_ptr->getLocalParametrizationPtr()));
    }
    else
    {
        //std::cout << "No Local Parametrization to be added" << std::endl;
        ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), _st_ptr->getSize(), nullptr);
    }
    if (_st_ptr->isFixed())
        updateStateBlockStatus(_st_ptr);
}

void CeresManager::removeStateBlock(double* _st_ptr)
{
    ceres_problem_->RemoveParameterBlock(_st_ptr);
}

void CeresManager::removeAllStateBlocks()
{
	std::vector<double*> parameter_blocks;

	ceres_problem_->GetParameterBlocks(&parameter_blocks);

	for (unsigned int i = 0; i< parameter_blocks.size(); i++)
		ceres_problem_->RemoveParameterBlock(parameter_blocks[i]);
}

void CeresManager::updateStateBlockStatus(StateBlock* _st_ptr)
{
	if (_st_ptr->isFixed())
		ceres_problem_->SetParameterBlockConstant(_st_ptr->getPtr());
	else
		ceres_problem_->SetParameterBlockVariable(_st_ptr->getPtr());
}

ceres::CostFunction* CeresManager::createCostFunction(ConstraintBase* _corrPtr)
{
	//std::cout << "adding ctr " << _corrPtr->nodeId() << std::endl;

    // analitic jacobian
    if (_corrPtr->getJacobianMethod() == JAC_ANALYTIC)
        return new CostFunctionWrapper((ConstraintAnalytic*)_corrPtr);

    // auto jacobian
    else if (_corrPtr->getJacobianMethod() == JAC_AUTO)
        return createAutoDiffCostFunction(_corrPtr, use_wolf_auto_diff_);

    // numeric jacobian
    else if (_corrPtr->getJacobianMethod() == JAC_NUMERIC)
        return createNumericDiffCostFunction(_corrPtr, use_wolf_auto_diff_);

    else
        throw std::invalid_argument( "Bad Jacobian Method!" );
}

} // namespace wolf

