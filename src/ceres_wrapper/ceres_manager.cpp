#include "base/ceres_wrapper/ceres_manager.h"
#include "base/ceres_wrapper/create_numeric_diff_cost_function.h"
#include "base/trajectory/trajectory_base.h"
#include "base/map/map_base.h"
#include "base/landmark/landmark_base.h"
#include "base/utils/make_unique.h"

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
    while (!fac_2_residual_idx_.empty())
        removeFactor(fac_2_residual_idx_.begin()->first);
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
            for(auto fr_ptr : wolf_problem_->getTrajectory()->getFrameList())
                if (fr_ptr->isKey())
                    for (auto sb : fr_ptr->getStateBlockVec())
                        if (sb)
                            all_state_blocks.push_back(sb);

            // landmark state blocks
            for(auto l_ptr : wolf_problem_->getMap()->getLandmarkList())
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
            for(auto fr_ptr : wolf_problem_->getTrajectory()->getFrameList())
                if (fr_ptr->isKey())
                    for (auto sb : fr_ptr->getStateBlockVec())
                        if (sb)
                            for(auto sb2 : fr_ptr->getStateBlockVec())
                                if (sb2)
                                {
                                    state_block_pairs.emplace_back(sb, sb2);
                                    double_pairs.emplace_back(getAssociatedMemBlockPtr(sb), getAssociatedMemBlockPtr(sb2));
                                    if (sb == sb2)
                                        break;
                                }

            // landmark state blocks
            for(auto l_ptr : wolf_problem_->getMap()->getLandmarkList())
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
            //                double_pairs.emplace_back(all_state_blocks[2*i]->get(),all_state_blocks[2*i]->get());
            //                double_pairs.emplace_back(all_state_blocks[2*i]->get(),all_state_blocks[2*i+1]->get());
            //                double_pairs.emplace_back(all_state_blocks[2*i+1]->get(),all_state_blocks[2*i+1]->get());
            //            }
            break;
        }
        case CovarianceBlocksToBeComputed::ROBOT_LANDMARKS:
        {
            //robot-robot
            auto last_key_frame = wolf_problem_->getLastKeyFrame();

            state_block_pairs.emplace_back(last_key_frame->getP(), last_key_frame->getP());
            state_block_pairs.emplace_back(last_key_frame->getP(), last_key_frame->getO());
            state_block_pairs.emplace_back(last_key_frame->getO(), last_key_frame->getO());

            double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getP()),
                                      getAssociatedMemBlockPtr(last_key_frame->getP()));
            double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getP()),
                                      getAssociatedMemBlockPtr(last_key_frame->getO()));
            double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getO()),
                                      getAssociatedMemBlockPtr(last_key_frame->getO()));

            // landmarks
            std::vector<StateBlockPtr> landmark_state_blocks;
            for(auto l_ptr : wolf_problem_->getMap()->getLandmarkList())
            {
                // load state blocks vector
                landmark_state_blocks = l_ptr->getUsedStateBlockVec();

                for (auto state_it = landmark_state_blocks.begin(); state_it != landmark_state_blocks.end(); state_it++)
                {
                    // robot - landmark
                    state_block_pairs.emplace_back(last_key_frame->getP(), *state_it);
                    state_block_pairs.emplace_back(last_key_frame->getO(), *state_it);
                    double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getP()),
                                              getAssociatedMemBlockPtr((*state_it)));
                    double_pairs.emplace_back(getAssociatedMemBlockPtr(last_key_frame->getO()),
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
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov(state_block_pairs[i].first->getLocalSize(),state_block_pairs[i].second->getLocalSize());
            covariance_->GetCovarianceBlockInTangentSpace(double_pairs[i].first, double_pairs[i].second, cov.data());
            wolf_problem_->addCovarianceBlock(state_block_pairs[i].first, state_block_pairs[i].second, cov);
            // std::cout << "covariance got switch: " << std::endl << cov << std::endl;
        }
    }
    else
        std::cout << "WARNING: Couldn't compute covariances!" << std::endl;
}
void CeresManager::computeCovariances(const std::vector<StateBlockPtr>& st_list)
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
    for (auto st_it1 = st_list.begin(); st_it1 != st_list.end(); st_it1++){
        if (*st_it1 == nullptr){
            continue;
        }
        for (auto st_it2 = st_it1; st_it2 != st_list.end(); st_it2++)
        {
            if (*st_it2 == nullptr){
                continue;
            }
            state_block_pairs.emplace_back(*st_it1, *st_it2);
            double_pairs.emplace_back(getAssociatedMemBlockPtr((*st_it1)),
                                      getAssociatedMemBlockPtr((*st_it2)));
        }
    }

    //std::cout << "pairs... " << double_pairs.size() << std::endl;

    // COMPUTE DESIRED COVARIANCES
    if (covariance_->Compute(double_pairs, ceres_problem_.get()))
        // STORE DESIRED COVARIANCES
        for (unsigned int i = 0; i < double_pairs.size(); i++)
        {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov(state_block_pairs[i].first->getLocalSize(),state_block_pairs[i].second->getLocalSize());
            covariance_->GetCovarianceBlockInTangentSpace(double_pairs[i].first, double_pairs[i].second, cov.data());
            wolf_problem_->addCovarianceBlock(state_block_pairs[i].first, state_block_pairs[i].second, cov);
            // std::cout << "covariance got from st_list: " << std::endl << cov << std::endl;
        }
    else
        std::cout << "WARNING: Couldn't compute covariances!" << std::endl;
}

void CeresManager::addFactor(const FactorBasePtr& fac_ptr)
{
    assert(fac_2_costfunction_.find(fac_ptr) == fac_2_costfunction_.end() && "adding a factor that is already in the fac_2_costfunction_ map");

    auto cost_func_ptr = createCostFunction(fac_ptr);
    fac_2_costfunction_[fac_ptr] = cost_func_ptr;

    std::vector<Scalar*> res_block_mem;
    res_block_mem.reserve(fac_ptr->getStateBlockPtrVector().size());
    for (const StateBlockPtr& st : fac_ptr->getStateBlockPtrVector())
    {
        res_block_mem.emplace_back( getAssociatedMemBlockPtr(st) );
    }

    auto loss_func_ptr = (fac_ptr->getApplyLossFunction()) ? new ceres::CauchyLoss(0.5) : nullptr;

    //std::cout << "Added residual block corresponding to constraint " << std::static_pointer_cast<CostFunctionWrapper>(cost_func_ptr)->getConstraintPtr()->getType() << std::static_pointer_cast<CostFunctionWrapper>(cost_func_ptr)->getConstraintPtr()->id() <<std::endl;

    assert((unsigned int)(ceres_problem_->NumResidualBlocks()) == fac_2_residual_idx_.size() && "ceres residuals different from wrapper residuals");
    assert(fac_2_residual_idx_.find(fac_ptr) == fac_2_residual_idx_.end() && "adding a factor that is already in the fac_2_residual_idx_ map");

    fac_2_residual_idx_[fac_ptr] = ceres_problem_->AddResidualBlock(cost_func_ptr.get(),
                                                                    loss_func_ptr,
                                                                    res_block_mem);

    assert((unsigned int)(ceres_problem_->NumResidualBlocks()) == fac_2_residual_idx_.size() && "ceres residuals different from wrapper residuals");
}

void CeresManager::removeFactor(const FactorBasePtr& _fac_ptr)
{
    assert(fac_2_residual_idx_.find(_fac_ptr) != fac_2_residual_idx_.end() && "removing a factor that is not in the fac_2_residual map");

    ceres_problem_->RemoveResidualBlock(fac_2_residual_idx_[_fac_ptr]);
    fac_2_residual_idx_.erase(_fac_ptr);
    fac_2_costfunction_.erase(_fac_ptr);

    assert((unsigned int)(ceres_problem_->NumResidualBlocks()) == fac_2_residual_idx_.size() && "ceres residuals different from wrapper residuals");
}

void CeresManager::addStateBlock(const StateBlockPtr& state_ptr)
{
    ceres::LocalParameterization* local_parametrization_ptr = nullptr;

    if (state_ptr->hasLocalParametrization() &&
        state_blocks_local_param_.find(state_ptr)==state_blocks_local_param_.end())
    {
        auto p = state_blocks_local_param_.emplace(state_ptr,
                                                   std::make_shared<LocalParametrizationWrapper>(state_ptr->getLocalParametrization()));

        local_parametrization_ptr = p.first->second.get();
    }

    ceres_problem_->AddParameterBlock(getAssociatedMemBlockPtr(state_ptr),
                                      state_ptr->getSize(),
                                      local_parametrization_ptr);

    if (state_ptr->isFixed())
        ceres_problem_->SetParameterBlockConstant(getAssociatedMemBlockPtr(state_ptr));

    updateStateBlockStatus(state_ptr);
}

void CeresManager::removeStateBlock(const StateBlockPtr& state_ptr)
{
    //std::cout << "CeresManager::removeStateBlock " << state_ptr.get() << " - " << getAssociatedMemBlockPtr(state_ptr) << std::endl;
    assert(state_ptr);
    ceres_problem_->RemoveParameterBlock(getAssociatedMemBlockPtr(state_ptr));
    state_blocks_local_param_.erase(state_ptr);
}

void CeresManager::updateStateBlockStatus(const StateBlockPtr& state_ptr)
{
    assert(state_ptr != nullptr);
    if (state_ptr->isFixed())
        ceres_problem_->SetParameterBlockConstant(getAssociatedMemBlockPtr(state_ptr));
    else
        ceres_problem_->SetParameterBlockVariable(getAssociatedMemBlockPtr(state_ptr));
}

void CeresManager::updateStateBlockLocalParametrization(const StateBlockPtr& state_ptr)
{
    assert(state_ptr != nullptr);

    /* in ceres the easiest way to update (add or remove) a local parameterization
     * of a state block (parameter block in ceres) is remove & add:
     *    - the state block: The associated memory block (that identified the parameter_block) is and MUST be the same
     *    - all involved factors (residual_blocks in ceres)
     */

    // get all involved factors
    FactorBasePtrList involved_factors;
    for (auto pair : fac_2_costfunction_)
        for (const StateBlockPtr& st : pair.first->getStateBlockPtrVector())
            if (st == state_ptr)
            {
                // store
                involved_factors.push_back(pair.first);
                break;
            }

    // Remove all involved factors (it does not remove any parameter block)
    for (auto fac : involved_factors)
        removeFactor(fac);

    // Remove state block (it removes all involved residual blocks but they just were removed)
    removeStateBlock(state_ptr);

    // Add state block
    addStateBlock(state_ptr);

    // Add all involved factors
    for (auto fac : involved_factors)
        addFactor(fac);
}

bool CeresManager::hasConverged()
{
    return summary_.termination_type == ceres::CONVERGENCE;
}

SizeStd CeresManager::iterations()
{
    return summary_.iterations.size();
}

Scalar CeresManager::initialCost()
{
    return Scalar(summary_.initial_cost);
}

Scalar CeresManager::finalCost()
{
    return Scalar(summary_.final_cost);
}

ceres::CostFunctionPtr CeresManager::createCostFunction(const FactorBasePtr& _fac_ptr)
{
    assert(_fac_ptr != nullptr);

    // analitic & autodiff jacobian
    if (_fac_ptr->getJacobianMethod() == JAC_ANALYTIC || _fac_ptr->getJacobianMethod() == JAC_AUTO)
        return std::make_shared<CostFunctionWrapper>(_fac_ptr);

    // numeric jacobian
    else if (_fac_ptr->getJacobianMethod() == JAC_NUMERIC)
        return createNumericDiffCostFunction(_fac_ptr);

    else
        throw std::invalid_argument( "Wrong Jacobian Method!" );
}

void CeresManager::check()
{
    // Check numbers
    assert(ceres_problem_->NumResidualBlocks() == fac_2_costfunction_.size());
    assert(ceres_problem_->NumResidualBlocks() == fac_2_residual_idx_.size());
    assert(ceres_problem_->NumParameterBlocks() == state_blocks_.size());

    // Check parameter blocks
    for (auto&& state_block_pair : state_blocks_)
        assert(ceres_problem_->HasParameterBlock(state_block_pair.second.data()));

    // Check residual blocks
    for (auto&& fac_res_pair : fac_2_residual_idx_)
    {
        // costfunction - residual
        assert(fac_2_costfunction_.find(fac_res_pair.first) != fac_2_costfunction_.end());
        assert(fac_2_costfunction_[fac_res_pair.first].get() == ceres_problem_->GetCostFunctionForResidualBlock(fac_res_pair.second));

        // factor - residual
        assert(fac_res_pair.first == static_cast<const CostFunctionWrapper*>(ceres_problem_->GetCostFunctionForResidualBlock(fac_res_pair.second))->getFactor());

        // parameter blocks - state blocks
        std::vector<Scalar*> param_blocks;
        ceres_problem_->GetParameterBlocksForResidualBlock(fac_res_pair.second, &param_blocks);
        auto i = 0;
        for (const StateBlockPtr& st : fac_res_pair.first->getStateBlockPtrVector())
        {
            assert(getAssociatedMemBlockPtr(st) == param_blocks[i]);
            i++;
        }
    }
}

} // namespace wolf

