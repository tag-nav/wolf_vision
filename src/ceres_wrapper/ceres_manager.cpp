#include "ceres_manager.h"
#include "../trajectory_base.h"
#include "../map_base.h"
#include "../landmark_base.h"

namespace wolf {

CeresManager::CeresManager(ProblemPtr _wolf_problem, const ceres::Solver::Options& _ceres_options, const bool _use_wolf_auto_diff) :
    ceres_options_(_ceres_options),
    wolf_problem_(_wolf_problem),
    use_wolf_auto_diff_(_use_wolf_auto_diff)
{
    ceres::Covariance::Options covariance_options;
    covariance_options.algorithm_type = ceres::SPARSE_QR;//ceres::DENSE_SVD;
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
//	std::cout << "ceres residual blocks:   " << ceres_problem_->NumResidualBlocks() << std::endl;
//	std::cout << "ceres parameter blocks:  " << ceres_problem_->NumParameterBlocks() << std::endl;
    while (!id_2_residual_idx_.empty())
        removeConstraint(id_2_residual_idx_.begin()->first);
//	std::cout << "all residuals removed! \n";
    removeAllStateBlocks();
//    std::cout << "all parameter blocks removed! \n";

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

    //std::cout << "After Update: Residual blocks: " << ceres_problem_->NumResidualBlocks() <<  " Parameter blocks: " << ceres_problem_->NumParameterBlocks() << std::endl;

	// create summary
	ceres::Solver::Summary ceres_summary_;

	// run Ceres Solver
	ceres::Solve(ceres_options_, ceres_problem_, &ceres_summary_);
	//std::cout << "solved" << std::endl;
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
//            std::vector<StateBlockPtr> all_state_blocks, landmark_state_blocks;
            //frame state blocks
            for(auto fr_ptr : wolf_problem_->getTrajectoryPtr()->getFrameList())
            {
                if (fr_ptr->isKey())
                    for (auto sb : fr_ptr->getStateBlockVec())
                        if (sb)
                        {
//                            std::cout << "CeresManager::computeCovariances(): State block @ " << sb.get() << std::endl;
//                            all_state_blocks.push_back(sb);
                            for(auto sb2 : fr_ptr->getStateBlockVec())
                            {
                                if (sb)
                                {
                                    state_block_pairs.push_back(std::make_pair(sb, sb2));
                                    double_pairs.push_back(std::make_pair(sb->getPtr(), sb2->getPtr()));
                                    if (sb == sb2) break;
                                }
                            }
                        }
            }

            // landmark state blocks
            for(auto l_ptr : wolf_problem_->getMapPtr()->getLandmarkList())
            {
//                landmark_state_blocks = l_ptr->getUsedStateBlockVec();
//                all_state_blocks.insert(all_state_blocks.end(), landmark_state_blocks.begin(), landmark_state_blocks.end());
                //all_state_blocks.push_back(l_ptr->getPPtr());
                //all_state_blocks.push_back(l_ptr->getOPtr());


                for (auto sb : l_ptr->getUsedStateBlockVec())
                {
//                    std::cout << "CeresManager::computeCovariances(): State block @ " << sb.get() << std::endl;
//                    all_state_blocks.push_back(sb);
                    for(auto sb2 : l_ptr->getUsedStateBlockVec())
                    {
                        state_block_pairs.push_back(std::make_pair(sb, sb2));
                        double_pairs.push_back(std::make_pair(sb->getPtr(), sb2->getPtr()));
                        if (sb == sb2) break;
                    }
                }



            }
//            // loop all marginals (PO marginals)
//            for (unsigned int i = 0; 2*i+1 < all_state_blocks.size(); i++)
//            {
//                state_block_pairs.push_back(std::make_pair(all_state_blocks[2*i],all_state_blocks[2*i]));
//                state_block_pairs.push_back(std::make_pair(all_state_blocks[2*i],all_state_blocks[2*i+1]));
//                state_block_pairs.push_back(std::make_pair(all_state_blocks[2*i+1],all_state_blocks[2*i+1]));
//
//                double_pairs.push_back(std::make_pair(all_state_blocks[2*i]->getPtr(),all_state_blocks[2*i]->getPtr()));
//                double_pairs.push_back(std::make_pair(all_state_blocks[2*i]->getPtr(),all_state_blocks[2*i+1]->getPtr()));
//                double_pairs.push_back(std::make_pair(all_state_blocks[2*i+1]->getPtr(),all_state_blocks[2*i+1]->getPtr()));
//            }
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

std::vector<Eigen::MatrixXs> CeresManager::computeJacobian(const unsigned int& _ctr_id, const std::vector<Scalar*>& _states_ptr) const
{
    // find costfunction pair
    auto cost_function_pair = id_2_costfunction_.find(_ctr_id);
	assert(cost_function_pair != id_2_costfunction_.end());
	auto cost_function_ptr = cost_function_pair->second;

	// init sizes
	const std::vector<int>& parameter_block_sizes = cost_function_ptr->parameter_block_sizes();
	int residual_size = cost_function_ptr->num_residuals();
	int N_parameters = parameter_block_sizes.size();
	assert(_states_ptr.size() == N_parameters);

	// build raw pointers
	double const* const* parameters = _states_ptr.data();
	double* residuals = new double[residual_size];

	// init raw pointers jacobian
    double** jacobians = new double*[N_parameters];
    for(int i = 0; i < N_parameters; ++i)
        jacobians[i] = new double[residual_size*parameter_block_sizes[i]];

    // evaluate
	cost_function_ptr->Evaluate(parameters, residuals, jacobians);

	// fill jacobian matrices
    std::vector<Eigen::MatrixXs> J;
    for (auto i = 0; i < N_parameters; i++)
    {
        Eigen::MatrixXs Ji = Eigen::Map<Eigen::MatrixXs>(jacobians[i], residual_size, parameter_block_sizes[i]);

        J.push_back(Ji);

        //std::cout << J[i] << std::endl << std::endl;
    }

	// delete
	delete residuals;
	for(int i = 0; i < N_parameters; ++i)
	    delete [] jacobians[i];
	delete [] jacobians;

	// return
	return J;
}

void CeresManager::update()
{
	//std::cout << "CeresManager: updating... " << std::endl;
	//std::cout << wolf_problem_->getStateBlockNotificationList().size() << " state block notifications" << std::endl;
	//std::cout << wolf_problem_->getConstraintNotificationList().size() << " constraint notifications" << std::endl;

	// REMOVE CONSTRAINTS
	auto ctr_notification_it = wolf_problem_->getConstraintNotificationList().begin();
	while ( ctr_notification_it != wolf_problem_->getConstraintNotificationList().end() )
	{
		if (ctr_notification_it->notification_ == REMOVE)
		{
			removeConstraint(ctr_notification_it->id_);
			ctr_notification_it = wolf_problem_->getConstraintNotificationList().erase(ctr_notification_it);
		}
		else
			ctr_notification_it++;
	}

	// REMOVE STATE BLOCKS
	auto state_notification_it = wolf_problem_->getStateBlockNotificationList().begin();
	while ( state_notification_it != wolf_problem_->getStateBlockNotificationList().end() )
	{
		if (state_notification_it->notification_ == REMOVE)
		{
			removeStateBlock((double *)(state_notification_it->scalar_ptr_));
			state_notification_it = wolf_problem_->getStateBlockNotificationList().erase(state_notification_it);
		}
		else
			state_notification_it++;
	}

    // ADD/UPDATE STATE BLOCKS
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
            default:
                throw std::runtime_error("CeresManager::update: State Block notification must be ADD, UPATE or REMOVE.");
        }
        wolf_problem_->getStateBlockNotificationList().pop_front();
    }
    // ADD CONSTRAINTS
    while (!wolf_problem_->getConstraintNotificationList().empty())
    {
        switch (wolf_problem_->getConstraintNotificationList().front().notification_)
        {
            case ADD:
            {
//                std::cout << "adding constraint" << std::endl;
                addConstraint(wolf_problem_->getConstraintNotificationList().front().constraint_ptr_,wolf_problem_->getConstraintNotificationList().front().id_);
                //std::cout << "added" << std::endl;
                break;
            }
            default:
                throw std::runtime_error("CeresManager::update: Constraint notification must be ADD or REMOVE.");
        }
        wolf_problem_->getConstraintNotificationList().pop_front();
    }
//	std::cout << "all constraints added" << std::endl;
//	std::cout << "ceres residual blocks:   " << ceres_problem_->NumResidualBlocks() << std::endl;
//	std::cout << "wrapper residual blocks: " << id_2_residual_idx_.size() << std::endl;
//	std::cout << "parameter blocks: " << ceres_problem_->NumParameterBlocks() << std::endl;

    assert(ceres_problem_->NumResidualBlocks() == id_2_residual_idx_.size() && "ceres residuals different from wrapper residuals");
    assert(wolf_problem_->getConstraintNotificationList().empty() && "wolf problem's constraints notification list not empty after update");
    assert(wolf_problem_->getStateBlockNotificationList().empty() && "wolf problem's state_blocks notification list not empty after update");
}

void CeresManager::addConstraint(ConstraintBasePtr _ctr_ptr, unsigned int _id)
{
    id_2_costfunction_[_id] = createCostFunction(_ctr_ptr);

//    std::cout << "adding residual " << _ctr_ptr->id() << std::endl;
//    std::cout << "residual pointer " << _ctr_ptr << std::endl;

    if (_ctr_ptr->getApplyLossFunction())
        id_2_residual_idx_[_id] = ceres_problem_->AddResidualBlock(id_2_costfunction_[_id], new ceres::CauchyLoss(0.5), _ctr_ptr->getStateScalarPtrVector());
    else
        id_2_residual_idx_[_id] = ceres_problem_->AddResidualBlock(id_2_costfunction_[_id], NULL, _ctr_ptr->getStateScalarPtrVector());
}

void CeresManager::removeConstraint(const unsigned int& _corr_id)
{
//    std::cout << "removing constraint " << _corr_id << std::endl;

    assert(id_2_residual_idx_.find(_corr_id) != id_2_residual_idx_.end());
	ceres_problem_->RemoveResidualBlock(id_2_residual_idx_[_corr_id]);
	id_2_residual_idx_.erase(_corr_id);

//	std::cout << "removingremoved!" << std::endl;
	// The cost functions will be deleted by ceres_problem destructor (IT MUST HAVE THE OWNERSHIP)
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
    if (_st_ptr->isFixed())
        updateStateBlockStatus(_st_ptr);
}

void CeresManager::removeStateBlock(double* _st_ptr)
{
    //std::cout << "Removing State Block " << _st_ptr << std::endl;
	assert(_st_ptr != nullptr);
    ceres_problem_->RemoveParameterBlock(_st_ptr);
}

void CeresManager::removeAllStateBlocks()
{
	std::vector<double*> parameter_blocks;

	ceres_problem_->GetParameterBlocks(&parameter_blocks);

	for (unsigned int i = 0; i< parameter_blocks.size(); i++)
		ceres_problem_->RemoveParameterBlock(parameter_blocks[i]);
}

void CeresManager::updateStateBlockStatus(StateBlockPtr _st_ptr)
{
	assert(_st_ptr != nullptr);
	if (_st_ptr->isFixed())
		ceres_problem_->SetParameterBlockConstant(_st_ptr->getPtr());
	else
		ceres_problem_->SetParameterBlockVariable(_st_ptr->getPtr());
}

ceres::CostFunction* CeresManager::createCostFunction(ConstraintBasePtr _corrPtr)
{
	assert(_corrPtr != nullptr);
	//std::cout << "creating cost function for constraint " << _corrPtr->id() << std::endl;

    // analitic jacobian
    if (_corrPtr->getJacobianMethod() == JAC_ANALYTIC)
        return new CostFunctionWrapper((ConstraintAnalytic*)(_corrPtr.get())); // TODO revise pointer types

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

