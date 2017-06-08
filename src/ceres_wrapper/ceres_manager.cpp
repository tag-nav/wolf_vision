#include "ceres_manager.h"
#include "create_numeric_diff_cost_function.h"
#include "../trajectory_base.h"
#include "../map_base.h"
#include "../landmark_base.h"

namespace wolf {

CeresManager::CeresManager(ProblemPtr _wolf_problem, const ceres::Solver::Options& _ceres_options) :
    SolverManager(_wolf_problem),
    ceres_options_(_ceres_options)
{
    ceres::Covariance::Options covariance_options;
    covariance_options.algorithm_type = ceres::SPARSE_QR;//ceres::DENSE_SVD;
    covariance_options.num_threads = 8;//ceres::DENSE_SVD;
    covariance_options.apply_loss_function = false;
    //covariance_options.null_space_rank = -1;
    covariance_ = new ceres::Covariance(covariance_options);

    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres_problem_ = new ceres::Problem(problem_options);
}

CeresManager::~CeresManager()
{
//	std::cout << "ceres residual blocks:   " << ceres_problem_->NumResidualBlocks() << std::endl;
//	std::cout << "ceres parameter blocks:  " << ceres_problem_->NumParameterBlocks() << std::endl;
    while (!ctr_2_residual_idx_.empty())
        removeConstraint(ctr_2_residual_idx_.begin()->first);
//	std::cout << "all residuals removed! \n";

	delete covariance_;
    //std::cout << "covariance deleted! \n";
    delete ceres_problem_;
    //std::cout << "ceres problem deleted! \n";
}

std::string CeresManager::solve(const unsigned int& _report_level)
{
	//std::cout << "Residual blocks: " << ceres_problem_->NumResidualBlocks() <<  " Parameter blocks: " << ceres_problem_->NumParameterBlocks() << std::endl;

    // update problem
    update();

    //std::cout << "After Update: Residual blocks: " << ceres_problem_->NumResidualBlocks() <<  " Parameter blocks: " << ceres_problem_->NumParameterBlocks() << std::endl;

	// run Ceres Solver
	ceres::Solve(ceres_options_, ceres_problem_, &summary_);
	//std::cout << "solved" << std::endl;

	//return report
	if (_report_level == 0)
	    return std::string();
    else if (_report_level == 1)
        return summary_.BriefReport();
    else if (_report_level == 1)
        return summary_.FullReport();
    else
        throw std::invalid_argument( "Report level should be 0, 1 or 2!" );
}

void CeresManager::computeCovariances(CovarianceBlocksToBeComputed _blocks)
{
    //std::cout << "CeresManager: computing covariances..." << std::endl;

    // update problem
    update();

    // CLEAR STORED COVARIANCE BLOCKS IN WOLF PROBLEM
    wolf_problem_->clearCovariance();

    // CREATE DESIRED COVARIANCES LIST
    std::vector<std::pair<StateBlockPtr, StateBlockPtr>> state_block_pairs;
    std::vector<std::pair<const double*, const double*>> double_pairs;

    switch (_blocks)
    {
        case ALL:
        {
            // first create a vector containing all state blocks
            std::vector<StateBlockPtr> all_state_blocks, landmark_state_blocks;
            //frame state blocks
            for(auto fr_ptr : wolf_problem_->getTrajectoryPtr()->getFrameList())
                if (fr_ptr->isKey())
                    for (auto sb : fr_ptr->getStateBlockVec())
                        if (sb)
                            all_state_blocks.push_back(sb);

            // landmark state blocks
            for(auto l_ptr : wolf_problem_->getMapPtr()->getLandmarkList())
            {
                landmark_state_blocks = l_ptr->getUsedStateBlockVec();
                all_state_blocks.insert(all_state_blocks.end(), landmark_state_blocks.begin(), landmark_state_blocks.end());
            }
            // double loop all against all (without repetitions)
            for (unsigned int i = 0; i < all_state_blocks.size(); i++)
                for  (unsigned int j = i; j < all_state_blocks.size(); j++)
                {
                    state_block_pairs.push_back(std::make_pair(all_state_blocks[i],all_state_blocks[j]));
                    double_pairs.push_back(std::make_pair(all_state_blocks[i]->getPtr(),all_state_blocks[j]->getPtr()));
                }
            break;
        }
        case ALL_MARGINALS:
        {
            // first create a vector containing all state blocks
            for(auto fr_ptr : wolf_problem_->getTrajectoryPtr()->getFrameList())
                if (fr_ptr->isKey())
                    for (auto sb : fr_ptr->getStateBlockVec())
                        if (sb)
                            for(auto sb2 : fr_ptr->getStateBlockVec())
                                if (sb)
                                {
                                    state_block_pairs.push_back(std::make_pair(sb, sb2));
                                    double_pairs.push_back(std::make_pair(sb->getPtr(), sb2->getPtr()));
                                    if (sb == sb2)
                                        break;
                                }

            // landmark state blocks
            for(auto l_ptr : wolf_problem_->getMapPtr()->getLandmarkList())
                for (auto sb : l_ptr->getUsedStateBlockVec())
                    for(auto sb2 : l_ptr->getUsedStateBlockVec())
                    {
                        state_block_pairs.push_back(std::make_pair(sb, sb2));
                        double_pairs.push_back(std::make_pair(sb->getPtr(), sb2->getPtr()));
                        if (sb == sb2) break;
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

            // landmarks
            std::vector<StateBlockPtr> landmark_state_blocks;
            for(auto l_ptr : wolf_problem_->getMapPtr()->getLandmarkList())
            {
                // load state blocks vector
                landmark_state_blocks = l_ptr->getUsedStateBlockVec();

                for (auto state_it = landmark_state_blocks.begin(); state_it != landmark_state_blocks.end(); state_it++)
                {
                    // robot - landmark
                    state_block_pairs.push_back(std::make_pair(last_key_frame->getPPtr(), *state_it));
                    state_block_pairs.push_back(std::make_pair(last_key_frame->getOPtr(), *state_it));
                    double_pairs.push_back(std::make_pair(last_key_frame->getPPtr()->getPtr(), (*state_it)->getPtr()));
                    double_pairs.push_back(std::make_pair(last_key_frame->getOPtr()->getPtr(), (*state_it)->getPtr()));

                    // landmark marginal
                    for (auto next_state_it = state_it; next_state_it != landmark_state_blocks.end(); next_state_it++)
                    {
                        state_block_pairs.push_back(std::make_pair(*state_it, *next_state_it));
                        double_pairs.push_back(std::make_pair((*state_it)->getPtr(), (*next_state_it)->getPtr()));
                    }
                }
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

void CeresManager::computeCovariances(const StateBlockList& st_list)
{
    //std::cout << "CeresManager: computing covariances..." << std::endl;

    // update problem
    update();

    // CLEAR STORED COVARIANCE BLOCKS IN WOLF PROBLEM
    wolf_problem_->clearCovariance();

    // CREATE DESIRED COVARIANCES LIST
    std::vector<std::pair<StateBlockPtr, StateBlockPtr>> state_block_pairs;
    std::vector<std::pair<const double*, const double*>> double_pairs;

    // double loop all against all (without repetitions)
    for (auto st_it1 = st_list.begin(); st_it1 != st_list.end(); st_it1++)
        for (auto st_it2 = st_it1; st_it2 != st_list.end(); st_it2++)
        {
            state_block_pairs.push_back(std::pair<StateBlockPtr, StateBlockPtr>(*st_it1,*st_it2));
            double_pairs.push_back(std::pair<const double*, const double*>((*st_it1)->getPtr(),(*st_it2)->getPtr()));
        }

    //std::cout << "pairs... " << double_pairs.size() << std::endl;

    // COMPUTE DESIRED COVARIANCES
    if (covariance_->Compute(double_pairs, ceres_problem_))
        // STORE DESIRED COVARIANCES
        for (unsigned int i = 0; i < double_pairs.size(); i++)
        {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov(state_block_pairs[i].first->getSize(),state_block_pairs[i].second->getSize());
            covariance_->GetCovarianceBlock(double_pairs[i].first, double_pairs[i].second, cov.data());
            wolf_problem_->addCovarianceBlock(state_block_pairs[i].first, state_block_pairs[i].second, cov);
            //std::cout << "getted covariance " << std::endl << cov << std::endl;
        }
    else
        std::cout << "WARNING: Couldn't compute covariances!" << std::endl;
}

void CeresManager::addConstraint(ConstraintBasePtr _ctr_ptr)
{
    ctr_2_costfunction_[_ctr_ptr] = createCostFunction(_ctr_ptr);

//    std::cout << "adding constraint " << _ctr_ptr->id() << std::endl;
//    std::cout << "constraint pointer " << _ctr_ptr << std::endl;

    if (_ctr_ptr->getApplyLossFunction())
        ctr_2_residual_idx_[_ctr_ptr] = ceres_problem_->AddResidualBlock(ctr_2_costfunction_[_ctr_ptr].get(), new ceres::CauchyLoss(0.5), _ctr_ptr->getStateScalarPtrVector());
    else
        ctr_2_residual_idx_[_ctr_ptr] = ceres_problem_->AddResidualBlock(ctr_2_costfunction_[_ctr_ptr].get(), NULL, _ctr_ptr->getStateScalarPtrVector());

    assert(ceres_problem_->NumResidualBlocks() == ctr_2_residual_idx_.size() && "ceres residuals different from wrapper residuals");
}

void CeresManager::removeConstraint(ConstraintBasePtr _ctr_ptr)
{
//  std::cout << "removing constraint " << _ctr_ptr->id() << std::endl;

    assert(ctr_2_residual_idx_.find(_ctr_ptr) != ctr_2_residual_idx_.end());
	ceres_problem_->RemoveResidualBlock(ctr_2_residual_idx_[_ctr_ptr]);
	ctr_2_residual_idx_.erase(_ctr_ptr);
	ctr_2_costfunction_.erase(_ctr_ptr);

//	std::cout << "removingremoved!" << std::endl;
	assert(ceres_problem_->NumResidualBlocks() == ctr_2_residual_idx_.size() && "ceres residuals different from wrapper residuals");
}

void CeresManager::addStateBlock(StateBlockPtr _st_ptr)
{
//    std::cout << "Adding State Block " << _st_ptr->getPtr() << std::endl;
//    std::cout << " size: " <<  _st_ptr->getSize() << std::endl;
//    std::cout << " vector: " <<  _st_ptr->getVector().transpose() << std::endl;

    if (_st_ptr->hasLocalParametrization())
    {
//        std::cout << "Local Parametrization to be added:" << _st_ptr->getLocalParametrizationPtr() << std::endl;
        ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), _st_ptr->getSize(), new LocalParametrizationWrapper(_st_ptr->getLocalParametrizationPtr()));
    }
    else
    {
//        std::cout << "No Local Parametrization to be added" << std::endl;
        ceres_problem_->AddParameterBlock(_st_ptr->getPtr(), _st_ptr->getSize(), nullptr);
    }
}

void CeresManager::removeStateBlock(StateBlockPtr _st_ptr)
{
    //std::cout << "Removing State Block " << _st_ptr << std::endl;
	assert(_st_ptr);
    ceres_problem_->RemoveParameterBlock(_st_ptr->getPtr());
}

void CeresManager::updateStateBlockStatus(StateBlockPtr _st_ptr)
{
	assert(_st_ptr != nullptr);
	if (_st_ptr->isFixed())
		ceres_problem_->SetParameterBlockConstant(_st_ptr->getPtr());
	else
		ceres_problem_->SetParameterBlockVariable(_st_ptr->getPtr());
}

ceres::CostFunctionPtr CeresManager::createCostFunction(ConstraintBasePtr _ctr_ptr)
{
	assert(_ctr_ptr != nullptr);
	//std::cout << "creating cost function for constraint " << _ctr_ptr->id() << std::endl;

    // analitic jacobian
    if (_ctr_ptr->getJacobianMethod() == JAC_ANALYTIC || _ctr_ptr->getJacobianMethod() == JAC_AUTO)
        return std::make_shared<CostFunctionWrapper>(_ctr_ptr);

    // numeric jacobian
    else if (_ctr_ptr->getJacobianMethod() == JAC_NUMERIC)
        return createNumericDiffCostFunction(_ctr_ptr);

    else
        throw std::invalid_argument( "Bad Jacobian Method!" );
}

} // namespace wolf

