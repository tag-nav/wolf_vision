#include "ceres_manager.h"
#include "create_numeric_diff_cost_function.h"
#include "../trajectory_base.h"
#include "../map_base.h"
#include "../landmark_base.h"
#include "../make_unique.h"

namespace wolf {

CeresManager::CeresManager(const ProblemPtr& _wolf_problem,
                           const ceres::Solver::Options& _ceres_options) :
    SolverManager(_wolf_problem),
    ceres_options_(_ceres_options)
{
    ceres::Covariance::Options covariance_options;
#if CERES_VERSION_MINOR >= 13
    covariance_options.algorithm_type = ceres::SPARSE_QR;//ceres::DENSE_SVD;
    covariance_options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
#elif CERES_VERSION_MINOR >= 10
    covariance_options.algorithm_type = ceres::SUITE_SPARSE_QR;//ceres::DENSE_SVD;
#else
    covariance_options.algorithm_type = ceres::SPARSE_QR;//ceres::DENSE_SVD;
#endif
    covariance_options.num_threads = 1;
    covariance_options.apply_loss_function = false;
    //covariance_options.null_space_rank = -1;
    covariance_ = wolf::make_unique<ceres::Covariance>(covariance_options);

    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

    ceres_problem_ = wolf::make_unique<ceres::Problem>(problem_options);
}

CeresManager::~CeresManager()
{
    while (!ctr_2_residual_idx_.empty())
        removeConstraint(ctr_2_residual_idx_.begin()->first);
}

std::string CeresManager::solveImpl(const ReportVerbosity report_level)
{
    // Check
    #ifdef _WOLF_DEBUG
        check();
    #endif

    // run Ceres Solver
    ceres::Solve(ceres_options_, ceres_problem_.get(), &summary_);

    std::string report;

    //return report
    if (report_level == ReportVerbosity::BRIEF)
        report = summary_.BriefReport();
    else if (report_level == ReportVerbosity::FULL)
        report = summary_.FullReport();

    return report;
}

void CeresManager::computeCovariances(const CovarianceBlocksToBeComputed _blocks)
{
    // update problem
    update();

    // CLEAR STORED COVARIANCE BLOCKS IN WOLF PROBLEM
    wolf_problem_->clearCovariance();

    // CREATE DESIRED COVARIANCES LIST
    std::vector<std::pair<StateBlockPtr, StateBlockPtr>> state_block_pairs;
    std::vector<std::pair<const double*, const double*>> double_pairs;

    switch (_blocks)
    {
        case CovarianceBlocksToBeComputed::ALL:
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
                    state_block_pairs.emplace_back(all_state_blocks[i],all_state_blocks[j]);
                    double_pairs.emplace_back(getAssociatedMemBlockPtr(all_state_blocks[i]),
                                              getAssociatedMemBlockPtr(all_state_blocks[j]));
                }
            break;
        }
        case CovarianceBlocksToBeComputed::ALL_MARGINALS:
        {
            // first create a vector containing all state blocks
            for(auto fr_ptr : wolf_problem_->getTrajectoryPtr()->getFrameList())
                if (fr_ptr->isKey())
                    for (auto sb : fr_ptr->getStateBlockVec())
                        if (sb)
                            for(auto sb2 : fr_ptr->getStateBlockVec())
                                if (sb)
                                {
                                    state_block_pairs.emplace_back(sb, sb2);
                                    double_pairs.emplace_back(getAssociatedMemBlockPtr(sb), getAssociatedMemBlockPtr(sb2));
                                    if (sb == sb2)
                                        break;
                                }

            // landmark state blocks
            for(auto l_ptr : wolf_problem_->getMapPtr()->getLandmarkList())
                for (auto sb : l_ptr->getUsedStateBlockVec())
                    for(auto sb2 : l_ptr->getUsedStateBlockVec())
                    {
                        state_block_pairs.emplace_back(sb, sb2);
                        double_pairs.emplace_back(getAssociatedMemBlockPtr(sb), getAssociatedMemBlockPtr(sb2));
                        if (sb == sb2)
                            break;
                    }
            //            // loop all marginals (PO marginals)
            //            for (unsigned int i = 0; 2*i+1 < all_state_blocks.size(); i++)
            //            {
            //                state_block_pairs.emplace_back(all_state_blocks[2*i],all_state_blocks[2*i]);
            //                state_block_pairs.emplace_back(all_state_blocks[2*i],all_state_blocks[2*i+1]);
            //                state_block_pairs.emplace_back(all_state_blocks[2*i+1],all_state_blocks[2*i+1]);
            //
            //                double_pairs.emplace_back(all_state_blocks[2*i]->getPtr(),all_state_blocks[2*i]->getPtr());
            //                double_pairs.emplace_back(all_state_blocks[2*i]->getPtr(),all_state_blocks[2*i+1]->getPtr());
            //                double_pairs.emplace_back(all_state_blocks[2*i+1]->getPtr(),all_state_blocks[2*i+1]->getPtr());
            //            }
            break;
        }
        case CovarianceBlocksToBeComputed::ROBOT_LANDMARKS:
        {
            //robot-robot
            auto last_key_frame = wolf_problem_->getLastKeyFramePtr();

            state_block_pairs.emplace_back(last_key_frame->getPPtr(), last_key_frame->getPPtr());
            state_block_pairs.emplace_back(last_key_frame->getPPtr(), last_key_frame->getOPtr());
            state_block_pairs.emplace_back(last_key_frame->getOPtr(), last_key_frame->getOPtr());

            double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getPPtr()),
                                      getAssociatedMemBlockPtr(last_key_frame->getPPtr()));
            double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getPPtr()),
                                      getAssociatedMemBlockPtr(last_key_frame->getOPtr()));
            double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getOPtr()),
                                      getAssociatedMemBlockPtr(last_key_frame->getOPtr()));

            // landmarks
            std::vector<StateBlockPtr> landmark_state_blocks;
            for(auto l_ptr : wolf_problem_->getMapPtr()->getLandmarkList())
            {
                // load state blocks vector
                landmark_state_blocks = l_ptr->getUsedStateBlockVec();

                for (auto state_it = landmark_state_blocks.begin(); state_it != landmark_state_blocks.end(); state_it++)
                {
                    // robot - landmark
                    state_block_pairs.emplace_back(last_key_frame->getPPtr(), *state_it);
                    state_block_pairs.emplace_back(last_key_frame->getOPtr(), *state_it);
                    double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getPPtr()),
                                              getAssociatedMemBlockPtr((*state_it)));
                    double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getOPtr()),
                                              getAssociatedMemBlockPtr((*state_it)));

                    // landmark marginal
                    for (auto next_state_it = state_it; next_state_it != landmark_state_blocks.end(); next_state_it++)
                    {
                      state_block_pairs.emplace_back(*state_it, *next_state_it);
                      double_pairs.emplace_back(getAssociatedMemBlockPtr((*state_it)),
                                                getAssociatedMemBlockPtr((*next_state_it)));
                    }
                }
            }
            break;
        }
    }
    //std::cout << "pairs... " << double_pairs.size() << std::endl;

    // COMPUTE DESIRED COVARIANCES
    if (covariance_->Compute(double_pairs, ceres_problem_.get()))
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
            state_block_pairs.emplace_back(*st_it1, *st_it2);
            double_pairs.emplace_back(getAssociatedMemBlockPtr((*st_it1)),
                                      getAssociatedMemBlockPtr((*st_it2)));
        }

    //std::cout << "pairs... " << double_pairs.size() << std::endl;

    // COMPUTE DESIRED COVARIANCES
    if (covariance_->Compute(double_pairs, ceres_problem_.get()))
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

void CeresManager::addConstraint(const ConstraintBasePtr& ctr_ptr)
{
    assert(ctr_2_costfunction_.find(ctr_ptr) == ctr_2_costfunction_.end() && "adding a constraint that is already in the ctr_2_costfunction_ map");

    auto cost_func_ptr = createCostFunction(ctr_ptr);
    ctr_2_costfunction_[ctr_ptr] = cost_func_ptr;

    std::vector<Scalar*> res_block_mem;
    res_block_mem.reserve(ctr_ptr->getStateBlockPtrVector().size());
    for (const StateBlockPtr& st : ctr_ptr->getStateBlockPtrVector())
    {
        res_block_mem.emplace_back( getAssociatedMemBlockPtr(st) );
    }

    auto loss_func_ptr = (ctr_ptr->getApplyLossFunction()) ? new ceres::CauchyLoss(0.5) : nullptr;

    assert(ctr_2_residual_idx_.find(ctr_ptr) == ctr_2_residual_idx_.end() && "adding a constraint that is already in the ctr_2_residual_idx_ map");

    ctr_2_residual_idx_[ctr_ptr] = ceres_problem_->AddResidualBlock(cost_func_ptr.get(),
                                                                    loss_func_ptr,
                                                                    res_block_mem);

    assert((unsigned int)(ceres_problem_->NumResidualBlocks()) == ctr_2_residual_idx_.size() && "ceres residuals different from wrapper residuals");
}

void CeresManager::removeConstraint(const ConstraintBasePtr& _ctr_ptr)
{
    assert(ctr_2_residual_idx_.find(_ctr_ptr) != ctr_2_residual_idx_.end() && "removing a constraint that is not in the ctr_2_residual map");

    ceres_problem_->RemoveResidualBlock(ctr_2_residual_idx_[_ctr_ptr]);
    ctr_2_residual_idx_.erase(_ctr_ptr);
    ctr_2_costfunction_.erase(_ctr_ptr);

    assert((unsigned int)(ceres_problem_->NumResidualBlocks()) == ctr_2_residual_idx_.size() && "ceres residuals different from wrapper residuals");
}

void CeresManager::addStateBlock(const StateBlockPtr& state_ptr)
{
    ceres::LocalParameterization* local_parametrization_ptr = nullptr;

    if (state_ptr->hasLocalParametrization() &&
        state_blocks_local_param_.find(state_ptr)==state_blocks_local_param_.end())
    {
        auto p = state_blocks_local_param_.emplace(state_ptr,
                                                   std::make_shared<LocalParametrizationWrapper>(state_ptr->getLocalParametrizationPtr()));

        local_parametrization_ptr = p.first->second.get();
    }

    ceres_problem_->AddParameterBlock(getAssociatedMemBlockPtr(state_ptr),
                                      state_ptr->getSize(),
                                      local_parametrization_ptr);

    updateStateBlockStatus(state_ptr);
}

void CeresManager::removeStateBlock(const StateBlockPtr& state_ptr)
{
    assert(state_ptr);
    ceres_problem_->RemoveParameterBlock(getAssociatedMemBlockPtr(state_ptr));
}

void CeresManager::updateStateBlockStatus(const StateBlockPtr& state_ptr)
{
    assert(state_ptr != nullptr);
    if (state_ptr->isFixed())
        ceres_problem_->SetParameterBlockConstant(getAssociatedMemBlockPtr(state_ptr));
    else
        ceres_problem_->SetParameterBlockVariable(getAssociatedMemBlockPtr(state_ptr));
}

ceres::CostFunctionPtr CeresManager::createCostFunction(const ConstraintBasePtr& _ctr_ptr)
{
    assert(_ctr_ptr != nullptr);

    // analitic & autodiff jacobian
    if (_ctr_ptr->getJacobianMethod() == JAC_ANALYTIC || _ctr_ptr->getJacobianMethod() == JAC_AUTO)
        return std::make_shared<CostFunctionWrapper>(_ctr_ptr);

    // numeric jacobian
    else if (_ctr_ptr->getJacobianMethod() == JAC_NUMERIC)
        return createNumericDiffCostFunction(_ctr_ptr);

    else
        throw std::invalid_argument( "Wrong Jacobian Method!" );
}

void CeresManager::check()
{
    // Check numbers
    assert(ceres_problem_->NumResidualBlocks() == ctr_2_costfunction_.size());
    assert(ceres_problem_->NumResidualBlocks() == ctr_2_residual_idx_.size());
    assert(ceres_problem_->NumParameterBlocks() == state_blocks_.size());

    // Check parameter blocks
    for (auto&& state_block_pair : state_blocks_)
        assert(ceres_problem_->HasParameterBlock(state_block_pair.second.data()));

    // Check residual blocks
    for (auto&& ctr_res_pair : ctr_2_residual_idx_)
    {
        // costfunction - residual
        assert(ctr_2_costfunction_.find(ctr_res_pair.first) != ctr_2_costfunction_.end());
        assert(ctr_2_costfunction_[ctr_res_pair.first].get() == ceres_problem_->GetCostFunctionForResidualBlock(ctr_res_pair.second));

        // constraint - residual
        assert(ctr_res_pair.first == static_cast<const CostFunctionWrapper*>(ceres_problem_->GetCostFunctionForResidualBlock(ctr_res_pair.second))->constraint_ptr_);

        // parameter blocks - state blocks
        std::vector<Scalar*> param_blocks;
        ceres_problem_->GetParameterBlocksForResidualBlock(ctr_res_pair.second, &param_blocks);
        auto i = 0;
        for (const StateBlockPtr& st : ctr_res_pair.first->getStateBlockPtrVector())
        {
            assert(getAssociatedMemBlockPtr(st) == param_blocks[i]);
            i++;
        }
    }
}

} // namespace wolf

