#include "ceres_manager.h"

namespace wolf {

CeresManager::CeresManager(WolfProblem*  _wolf_problem, ceres::Problem::Options _options) :
    ceres_problem_(new ceres::Problem(_options)),
    wolf_problem_(_wolf_problem)
{
	ceres::Covariance::Options covariance_options;
    covariance_options.algorithm_type = ceres::SUITE_SPARSE_QR;//ceres::DENSE_SVD;
    covariance_options.num_threads = 8;//ceres::DENSE_SVD;
    covariance_options.apply_loss_function = false;
	//covariance_options.null_space_rank = -1;
	covariance_ = new ceres::Covariance(covariance_options);
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

ceres::Solver::Summary CeresManager::solve(const ceres::Solver::Options& _ceres_options)
{
	//std::cout << "Residual blocks: " << ceres_problem_->NumResidualBlocks() <<  " Parameter blocks: " << ceres_problem_->NumParameterBlocks() << std::endl;

	// create summary
	ceres::Solver::Summary ceres_summary_;

	// run Ceres Solver
	ceres::Solve(_ceres_options, ceres_problem_, &ceres_summary_);

	//return results
	return ceres_summary_;
}

void CeresManager::computeCovariances(CovarianceBlocksToBeComputed _blocks)
{
    //std::cout << "computing covariances..." << std::endl;

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
            for(auto f_it = wolf_problem_->getTrajectoryPtr()->getFrameListPtr()->begin(); f_it!=wolf_problem_->getTrajectoryPtr()->getFrameListPtr()->end(); f_it++)
            {
                all_state_blocks.push_back((*f_it)->getPPtr());
                all_state_blocks.push_back((*f_it)->getOPtr());
            }
            // landmark state blocks
            for(auto l_it = wolf_problem_->getMapPtr()->getLandmarkListPtr()->begin(); l_it!=wolf_problem_->getMapPtr()->getLandmarkListPtr()->end(); l_it++)
            {
                all_state_blocks.push_back((*l_it)->getPPtr());
                all_state_blocks.push_back((*l_it)->getOPtr());
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
            for(auto f_it = wolf_problem_->getTrajectoryPtr()->getFrameListPtr()->begin(); f_it!=wolf_problem_->getTrajectoryPtr()->getFrameListPtr()->end(); f_it++)
            {
                all_state_blocks.push_back((*f_it)->getPPtr());
                all_state_blocks.push_back((*f_it)->getOPtr());

            }
            // landmark state blocks
            for(auto l_it = wolf_problem_->getMapPtr()->getLandmarkListPtr()->begin(); l_it!=wolf_problem_->getMapPtr()->getLandmarkListPtr()->end(); l_it++)
            {
                all_state_blocks.push_back((*l_it)->getPPtr());
                all_state_blocks.push_back((*l_it)->getOPtr());
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
            FrameBase* last_frame = wolf_problem_->getTrajectoryPtr()->getFrameListPtr()->back();

            state_block_pairs.push_back(std::make_pair(last_frame->getPPtr(), last_frame->getPPtr()));
            state_block_pairs.push_back(std::make_pair(last_frame->getPPtr(), last_frame->getOPtr()));
            state_block_pairs.push_back(std::make_pair(last_frame->getOPtr(), last_frame->getOPtr()));

            double_pairs.push_back(std::make_pair(last_frame->getPPtr()->getPtr(), last_frame->getPPtr()->getPtr()));
            double_pairs.push_back(std::make_pair(last_frame->getPPtr()->getPtr(), last_frame->getOPtr()->getPtr()));
            double_pairs.push_back(std::make_pair(last_frame->getOPtr()->getPtr(), last_frame->getOPtr()->getPtr()));

            for(auto l_it = wolf_problem_->getMapPtr()->getLandmarkListPtr()->begin(); l_it!=wolf_problem_->getMapPtr()->getLandmarkListPtr()->end(); l_it++)
            {
                state_block_pairs.push_back(std::make_pair(last_frame->getPPtr(), (*l_it)->getPPtr()));
                state_block_pairs.push_back(std::make_pair(last_frame->getPPtr(), (*l_it)->getOPtr()));
                state_block_pairs.push_back(std::make_pair(last_frame->getOPtr(), (*l_it)->getPPtr()));
                state_block_pairs.push_back(std::make_pair(last_frame->getOPtr(), (*l_it)->getOPtr()));
                state_block_pairs.push_back(std::make_pair((*l_it)->getPPtr(), (*l_it)->getPPtr()));
                state_block_pairs.push_back(std::make_pair((*l_it)->getPPtr(), (*l_it)->getOPtr()));
                state_block_pairs.push_back(std::make_pair((*l_it)->getOPtr(), (*l_it)->getOPtr()));

                double_pairs.push_back(std::make_pair(last_frame->getPPtr()->getPtr(), (*l_it)->getPPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(last_frame->getPPtr()->getPtr(), (*l_it)->getOPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(last_frame->getOPtr()->getPtr(), (*l_it)->getPPtr()->getPtr()));
                double_pairs.push_back(std::make_pair(last_frame->getOPtr()->getPtr(), (*l_it)->getOPtr()->getPtr()));
                double_pairs.push_back(std::make_pair((*l_it)->getPPtr()->getPtr(), (*l_it)->getPPtr()->getPtr()));
                double_pairs.push_back(std::make_pair((*l_it)->getPPtr()->getPtr(), (*l_it)->getOPtr()->getPtr()));
                double_pairs.push_back(std::make_pair((*l_it)->getOPtr()->getPtr(), (*l_it)->getOPtr()->getPtr()));
            }
            break;
        }
    }

    // COMPUTE DESIRED COVARIANCES
    if (covariance_->Compute(double_pairs, ceres_problem_))
    {
        // STORE DESIRED COVARIANCES
        for (unsigned int i = 0; i < double_pairs.size(); i++)
        {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov(state_block_pairs[i].first->getSize(),state_block_pairs[i].second->getSize());
            covariance_->GetCovarianceBlock(double_pairs[i].first, double_pairs[i].second, cov.data());
            //std::cout << "getted covariance " << std::endl << cov << std::endl;
            wolf_problem_->addCovarianceBlock(state_block_pairs[i].first, state_block_pairs[i].second, cov);
        }
    }
    else
        std::cout << "WARNING: Couldn't compute covariances!" << std::endl;
}

void CeresManager::update(const bool _self_auto_diff, const bool _apply_loss_function)
{
    //std::cout << "CeresManager: updating... getConstraintRemoveList()->size()" << wolf_problem_->getConstraintRemoveList()->size() << std::endl;

    // REMOVE CONSTRAINTS
    while (!wolf_problem_->getConstraintRemoveList()->empty())
    {
        removeConstraint(wolf_problem_->getConstraintRemoveList()->front());
        wolf_problem_->getConstraintRemoveList()->pop_front();
    }
    // REMOVE STATE BLOCKS
    while (!wolf_problem_->getStateBlockRemoveList()->empty())
    {
        removeStateBlock((double *)(wolf_problem_->getStateBlockRemoveList()->front()));
        wolf_problem_->getStateBlockRemoveList()->pop_front();
    }
    // ADD STATE BLOCKS
    while (!wolf_problem_->getStateBlockAddList()->empty())
    {
        addStateBlock(wolf_problem_->getStateBlockAddList()->front());
        wolf_problem_->getStateBlockAddList()->pop_front();
    }
    // UPDATE STATE BLOCKS
    while (!wolf_problem_->getStateBlockUpdateList()->empty())
    {
        updateStateBlockStatus(wolf_problem_->getStateBlockUpdateList()->front());
        wolf_problem_->getStateBlockUpdateList()->pop_front();
    }
    // ADD CONSTRAINTS
    while (!wolf_problem_->getConstraintAddList()->empty())
    {
        addConstraint(wolf_problem_->getConstraintAddList()->front(), _self_auto_diff, _apply_loss_function);
        wolf_problem_->getConstraintAddList()->pop_front();
    }
}

void CeresManager::addConstraint(ConstraintBase* _corr_ptr, const bool _self_auto_diff, const bool _apply_loss)
{
    id_2_costfunction_[_corr_ptr->nodeId()] = createCostFunction(_corr_ptr, _self_auto_diff);

    if (_apply_loss)
        id_2_residual_idx_[_corr_ptr->nodeId()] = ceres_problem_->AddResidualBlock(id_2_costfunction_[_corr_ptr->nodeId()], new ceres::CauchyLoss(0.5), _corr_ptr->getStateBlockPtrVector());
    else
        id_2_residual_idx_[_corr_ptr->nodeId()] = ceres_problem_->AddResidualBlock(id_2_costfunction_[_corr_ptr->nodeId()], NULL, _corr_ptr->getStateBlockPtrVector());
}

void CeresManager::removeConstraint(const unsigned int& _corr_id)
{
    //std::cout << "removing constraint " << _corr_id << std::endl;
    assert(id_2_residual_idx_.find(_corr_id) != id_2_residual_idx_.end());
	ceres_problem_->RemoveResidualBlock(id_2_residual_idx_[_corr_id]);
    //std::cout << "residual block removed!" << std::endl;
	id_2_residual_idx_.erase(_corr_id);
//    std::cout << "deleting cost function" << std::endl;
//    assert(id_2_costfunction_.find(_corr_id) != id_2_costfunction_.end());
//    delete id_2_costfunction_[_corr_id];
//    std::cout << "cost function deleted!" << std::endl;
	id_2_costfunction_.erase(_corr_id);
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

ceres::CostFunction* CeresManager::createCostFunction(ConstraintBase* _corrPtr, const bool _use_wolf_auto_diff)
{
	//std::cout << "adding ctr " << _corrPtr->nodeId() << std::endl;

    // analitic jacobian
    if (_corrPtr->getJacobianMethod() == JAC_ANALYTIC)
        return new CostFunctionWrapper((ConstraintAnalytic*)_corrPtr);

    // auto jacobian
    else if (_corrPtr->getJacobianMethod() == JAC_AUTO)
        return createAutoDiffCostFunction(_corrPtr, _use_wolf_auto_diff);

    // numeric jacobian
    else if (_corrPtr->getJacobianMethod() == JAC_NUMERIC)
        return createNumericDiffCostFunction(_corrPtr, _use_wolf_auto_diff);

    else
        throw std::invalid_argument( "Bad Jacobian Method!" );
}

} // namespace wolf

