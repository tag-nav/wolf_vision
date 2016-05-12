/*
 * qr_solver.h
 *
 *  Created on: Jul 2, 2015
 *      Author: jvallve
 */

#ifndef TRUNK_SRC_SOLVER_QR_SOLVER_H_
#define TRUNK_SRC_SOLVER_QR_SOLVER_H_

//std includes
#include <iostream>
#include <ctime>

//Wolf includes
#include "state_block.h"
#include "../constraint_sparse.h"
#include "../constraint_fix.h"
//#include "../constraint_gps_2D.h"
//#include "../constraint_gps_pseudorange_2D.h"
#include "../constraint_odom_2D.h"
#include "../constraint_corner_2D.h"
#include "../constraint_container.h"
#include "sparse_utils.h"

// wolf solver
#include "solver/ccolamd_ordering.h"
#include "solver/cost_function_sparse.h"

// eigen includes
#include <eigen3/Eigen/OrderingMethods>
#include <eigen3/Eigen/SparseQR>

namespace wolf
{

class SolverQR
{
    protected:
        Problem* problem_ptr_;
        Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::NaturalOrdering<int>> solver_;
        Eigen::SparseMatrix<double> A_, R_;
        Eigen::VectorXd b_, x_incr_;
        std::vector<StateBlock*> nodes_;
        std::vector<ConstraintBase*> constraints_;
        std::vector<CostFunctionBase*> cost_functions_;

        // ordering
        Eigen::SparseMatrix<int> A_nodes_;
        Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> acc_node_permutation_;
        std::map<double*, unsigned int> id_2_idx_;
        Eigen::CCOLAMDOrdering<int> orderer_;
        Eigen::VectorXi node_ordering_restrictions_;
        Eigen::ArrayXi node_locations_;
        std::vector<unsigned int> constraint_locations_;
        unsigned int n_new_constraints_;

        // time
        clock_t t_ordering_, t_solving_, t_managing_;
        double time_ordering_, time_solving_, time_managing_;

    public:
        SolverQR(Problem* problem_ptr_) :
                problem_ptr_(problem_ptr_), A_(0, 0), R_(0, 0), A_nodes_(0, 0), acc_node_permutation_(0), n_new_constraints_(
                        0), time_ordering_(0), time_solving_(0), time_managing_(0)
        {
            node_locations_.resize(0);
            constraint_locations_.resize(0);
        }

        virtual ~SolverQR()
        {

        }

        void update()
        {
            // UPDATE STATE BLOCKS
            while (!problem_ptr_->getStateBlockNotificationList().empty())
            {
                switch (problem_ptr_->getStateBlockNotificationList().front().notification_)
                {
                    case ADD:
                    {
                        addStateBlock(problem_ptr_->getStateBlockNotificationList().front().state_block_ptr_);
                        break;
                    }
                    case UPDATE:
                    {
                        updateStateBlockStatus(problem_ptr_->getStateBlockNotificationList().front().state_block_ptr_);
                        break;
                    }
                    case REMOVE:
                    {
                        // TODO removeStateBlock((double *)(problem_ptr_->getStateBlockNotificationList().front().scalar_ptr_));
                        break;
                    }
                    default:
                        throw std::runtime_error("SolverQR::update: State Block notification must be ADD, UPATE or REMOVE.");
                }
                problem_ptr_->getStateBlockNotificationList().pop_front();
            }
            // UPDATE CONSTRAINTS
            while (!problem_ptr_->getConstraintNotificationList().empty())
            {
                switch (problem_ptr_->getConstraintNotificationList().front().notification_)
                {
                    case ADD:
                    {
                        addConstraint(problem_ptr_->getConstraintNotificationList().front().constraint_ptr_);
                        break;
                    }
                    case REMOVE:
                    {
                        // TODO: removeConstraint(problem_ptr_->getConstraintNotificationList().front().id_);
                        break;
                    }
                    default:
                        throw std::runtime_error("SolverQR::update: Constraint notification must be ADD or REMOVE.");
                }
                problem_ptr_->getConstraintNotificationList().pop_front();
            }
        }

        void addStateBlock(StateBlock* _state_ptr)
        {
            t_managing_ = clock();

            std::cout << "adding state unit " << _state_ptr->getPtr() << std::endl;
            if (!_state_ptr->isFixed())
            {
                nodes_.push_back(_state_ptr);
                id_2_idx_[_state_ptr->getPtr()] = nodes_.size() - 1;

                std::cout << "idx " << id_2_idx_[_state_ptr->getPtr()] << std::endl;

                // Resize accumulated permutations
                augmentPermutation(acc_node_permutation_, nNodes());

                // Resize state
                x_incr_.conservativeResize(x_incr_.size() + _state_ptr->getSize());

                // Resize problem
                A_.conservativeResize(A_.rows(), A_.cols() + _state_ptr->getSize());
                R_.conservativeResize(R_.cols() + _state_ptr->getSize(), R_.cols() + _state_ptr->getSize());

            }
            time_managing_ += ((double)clock() - t_managing_) / CLOCKS_PER_SEC;
        }

        void updateStateBlockStatus(StateBlock* _state_ptr)
        {
            //TODO
        }

        void addConstraint(ConstraintBase* _constraint_ptr)
        {
            std::cout << "adding constraint " << _constraint_ptr->nodeId() << std::endl;
            t_managing_ = clock();

            constraints_.push_back(_constraint_ptr);
            cost_functions_.push_back(createCostFunction(_constraint_ptr));

            unsigned int meas_dim = _constraint_ptr->getSize();

            std::vector<Eigen::MatrixXs> jacobians(_constraint_ptr->getStatePtrVector().size());
            Eigen::VectorXs error(meas_dim);

            cost_functions_.back()->evaluateResidualJacobians();
            cost_functions_.back()->getResidual(error);
            cost_functions_.back()->getJacobians(jacobians);

            std::vector<unsigned int> idxs;
            for (unsigned int i = 0; i < _constraint_ptr->getStatePtrVector().size(); i++)
                if (!_constraint_ptr->getStatePtrVector().at(i)->isFixed())
                    idxs.push_back(id_2_idx_[_constraint_ptr->getStatePtrVector().at(i)->getPtr()]);

            n_new_constraints_++;
            constraint_locations_.push_back(A_.rows());

            // Resize problem
            A_.conservativeResize(A_.rows() + meas_dim, A_.cols());
            b_.conservativeResize(b_.size() + meas_dim);
            A_nodes_.conservativeResize(constraints_.size(), nNodes());

            // ADD MEASUREMENTS
            for (unsigned int j = 0; j < idxs.size(); j++)
            {
                assert((unsigned int )(acc_node_permutation_.indices()(idxs.at(j))) == nodeOrder(idxs.at(j)));
                assert(jacobians.at(j).cols() == nodeDim(idxs.at(j)));
                assert(jacobians.at(j).rows() == meas_dim);

                addSparseBlock(jacobians.at(j), A_, A_.rows() - meas_dim, nodeLocation(idxs.at(j)));

                A_nodes_.coeffRef(A_nodes_.rows() - 1, nodeOrder(idxs.at(j))) = 1;
            }

            // error
            b_.tail(meas_dim) = error;

            time_managing_ += ((double)clock() - t_managing_) / CLOCKS_PER_SEC;
        }

        void ordering(const int & _first_ordered_idx)
        {
            std::cout << "ordering from idx " << _first_ordered_idx << std::endl;
            t_ordering_ = clock();

            // full problem ordering
            if (_first_ordered_idx == -1)
            {
                // ordering ordering constraints
                node_ordering_restrictions_.resize(nNodes());
                node_ordering_restrictions_ = A_nodes_.bottomRows(1).transpose();

                // computing nodes partial ordering_
                A_nodes_.makeCompressed();
                Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> incr_permutation_nodes(nNodes());
                orderer_(A_nodes_, incr_permutation_nodes, node_ordering_restrictions_.data());

                // node ordering to variable ordering
                Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> incr_permutation(A_.cols());
                nodePermutation2VariablesPermutation(incr_permutation_nodes, incr_permutation);

                // apply partial_ordering orderings
                A_nodes_ = (A_nodes_ * incr_permutation_nodes.transpose()).sparseView();
                A_ = (A_ * incr_permutation.transpose()).sparseView();

                // ACCUMULATING PERMUTATIONS
                accumulatePermutation(incr_permutation_nodes);
            }

            // partial ordering
            else
            {
                unsigned int first_ordered_node = nodeOrder(_first_ordered_idx); //nodes_.at(_first_ordered_idx).order;
                unsigned int ordered_nodes = nNodes() - first_ordered_node;

                if (ordered_nodes > 2) // only reordering when involved nodes in the measurement are not the two last ones
                {
                    // SUBPROBLEM ORDERING (from first node variable to last one)
                    //std::cout << "ordering partial_ordering problem: " << _first_ordered_node << " to "<< n_nodes_ - 1 << std::endl;
                    Eigen::SparseMatrix<int> sub_A_nodes_ = A_nodes_.rightCols(ordered_nodes);

                    // _partial_ordering ordering_ constraints
                    node_ordering_restrictions_.resize(ordered_nodes);
                    node_ordering_restrictions_ = sub_A_nodes_.bottomRows(1).transpose();

                    // computing nodes partial ordering_
                    sub_A_nodes_.makeCompressed();
                    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> partial_permutation_nodes(ordered_nodes);
                    orderer_(sub_A_nodes_, partial_permutation_nodes, node_ordering_restrictions_.data());

                    // node ordering to variable ordering
                    unsigned int ordered_variables = A_.cols() - nodeLocation(_first_ordered_idx); //nodes_.at(_first_ordered_idx).location;
//                    std::cout << "first_ordered_node " << first_ordered_node << std::endl;
//                    std::cout << "A_.cols() " << A_.cols() << std::endl;
//                    std::cout << "nodes_.at(_first_ordered_idx).location " << nodes_.at(_first_ordered_idx).location << std::endl;
//                    std::cout << "ordered_variables " << ordered_variables << std::endl;
                    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> partial_permutation(ordered_variables);
                    nodePermutation2VariablesPermutation(partial_permutation_nodes, partial_permutation);

                    // apply partial_ordering orderings
                    A_nodes_.rightCols(ordered_nodes) = (A_nodes_.rightCols(ordered_nodes)
                            * partial_permutation_nodes.transpose()).sparseView();
                    A_.rightCols(ordered_variables) =
                            (A_.rightCols(ordered_variables) * partial_permutation.transpose()).sparseView();
                    R_.rightCols(ordered_variables) =
                            (R_.rightCols(ordered_variables) * partial_permutation.transpose()).sparseView();

                    // ACCUMULATING PERMUTATIONS
                    accumulatePermutation(partial_permutation_nodes);
                }
            }
            time_ordering_ += ((double)clock() - t_ordering_) / CLOCKS_PER_SEC;
        }

        unsigned int findFirstOrderedNode()
        {
            unsigned int first_ordered_node = nNodes();
            unsigned int first_ordered_idx;
            for (unsigned int i = 0; i < n_new_constraints_; i++)
            {
                ConstraintBase* ct_ptr = constraints_.at(constraints_.size() - 1 - i);
                std::cout << "constraint: " << i << " id: " << constraints_.at(constraints_.size() - 1 - i)->nodeId()
                        << std::endl;
                for (unsigned int j = 0; j < ct_ptr->getStatePtrVector().size(); j++)
                {
                    if (!ct_ptr->getStatePtrVector().at(j)->isFixed())
                    {
                        unsigned int idx = id_2_idx_[ct_ptr->getStatePtrVector().at(j)->getPtr()];
                        //std::cout << "estimated idx " << idx << std::endl;
                        //std::cout << "node_order(idx) " << node_order(idx) << std::endl;
                        //std::cout << "first_ordered_node " << first_ordered_node << std::endl;
                        if (first_ordered_node > nodeOrder(idx))                        //nodes_.at(idx).order)
                        {
                            first_ordered_idx = idx;
                            //std::cout << "first_ordered_idx " << first_ordered_idx << std::endl;
                            first_ordered_node = nodeOrder(idx);
                            //std::cout << "first_ordered_node " << first_ordered_node << std::endl;
                        }
                    }
                }
            }
            //std::cout << "found first_ordered_node " << first_ordered_node << std::endl;
            //std::cout << "found first_ordered_idx " << first_ordered_idx << std::endl;

            return first_ordered_idx;
        }

        bool solve(const unsigned int mode)
        {
            if (n_new_constraints_ == 0)
                return 1;

            std::cout << "solving mode " << mode << std::endl;

            bool batch, order;
            unsigned int first_ordered_idx;

            switch (mode)
            {
                case 0:
                {
                    batch = true;
                    order = false;
                    break;
                }
                case 1:
                {
                    batch = true;
                    order = (nNodes() > 1);
                    break;
                }
                case 2:
                {
                    first_ordered_idx = findFirstOrderedNode();
                    batch = (nodeOrder(first_ordered_idx) == 0);
                    order = (nNodes() > 1);
                }
            }

            // BATCH
            if (batch)
            {
                // REORDER
                if (order)
                    ordering(-1);

                //printProblem();

                // SOLVE
                t_solving_ = clock();
                A_.makeCompressed();
                solver_.compute(A_);
                if (solver_.info() != Eigen::Success)
                {
                    std::cout << "decomposition failed" << std::endl;
                    return 0;
                }
                x_incr_ = solver_.solve(-b_);
                R_ = solver_.matrixR();
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.cols(), R_.cols()) * R_ << std::endl;
                time_solving_ += ((double)clock() - t_solving_) / CLOCKS_PER_SEC;
            }
            // INCREMENTAL
            else
            {
                // REORDER SUBPROBLEM
                ordering(first_ordered_idx);
                //printProblem();

                // SOLVE ORDERED SUBPROBLEM
                t_solving_ = clock();
                A_nodes_.makeCompressed();
                A_.makeCompressed();

                // finding measurements block
                Eigen::SparseMatrix<int> measurements_to_initial = A_nodes_.col(nodeOrder(first_ordered_idx));
                //std::cout << "measurements_to_initial " << measurements_to_initial << std::endl;
                //std::cout << "measurements_to_initial.innerIndexPtr()[measurements_to_initial.outerIndexPtr()[0]] " << measurements_to_initial.innerIndexPtr()[measurements_to_initial.outerIndexPtr()[0]] << std::endl;
                unsigned int first_ordered_measurement =
                        measurements_to_initial.innerIndexPtr()[measurements_to_initial.outerIndexPtr()[0]];
                unsigned int ordered_measurements = A_.rows() - constraint_locations_.at(first_ordered_measurement);
                unsigned int ordered_variables = A_.cols() - nodeLocation(first_ordered_idx); //nodes_.at(first_ordered_idx).location;
                unsigned int unordered_variables = nodeLocation(first_ordered_idx); //nodes_.at(first_ordered_idx).location;

                Eigen::SparseMatrix<double> A_partial = A_.bottomRightCorner(ordered_measurements, ordered_variables);
                solver_.compute(A_partial);
                if (solver_.info() != Eigen::Success)
                {
                    std::cout << "decomposition failed" << std::endl;
                    return 0;
                }
                //std::cout << "R new" << std::endl << MatrixXd::Identity(A_partial.cols(), A_partial.cols()) * solver_.matrixR() << std::endl;
                x_incr_.tail(ordered_variables) = solver_.solve(-b_.tail(ordered_measurements));

                // store new part of R
                eraseSparseBlock(R_, unordered_variables, unordered_variables, ordered_variables, ordered_variables);
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.rows(), R_.rows()) * R_ << std::endl;
                addSparseBlock(solver_.matrixR(), R_, unordered_variables, unordered_variables);
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.rows(), R_.rows()) * R_ << std::endl;
                R_.makeCompressed();

                // solving not ordered subproblem
                if (unordered_variables > 0)
                {
                    //std::cout << "--------------------- solving unordered part" << std::endl;
                    Eigen::SparseMatrix<double> R1 = R_.topLeftCorner(unordered_variables, unordered_variables);
                    //std::cout << "R1" << std::endl << MatrixXd::Identity(R1.rows(), R1.rows()) * R1 << std::endl;
                    Eigen::SparseMatrix<double> R2 = R_.topRightCorner(unordered_variables, ordered_variables);
                    //std::cout << "R2" << std::endl << MatrixXd::Identity(R2.rows(), R2.rows()) * R2 << std::endl;
                    solver_.compute(R1);
                    if (solver_.info() != Eigen::Success)
                    {
                        std::cout << "decomposition failed" << std::endl;
                        return 0;
                    }
                    x_incr_.head(unordered_variables) = solver_.solve(
                            -b_.head(unordered_variables) + R2 * x_incr_.tail(ordered_variables));
                }

            }
            // UPDATE X VALUE
            for (unsigned int i = 0; i < nodes_.size(); i++)
            {
                Eigen::Map < Eigen::VectorXs > x_i(nodes_.at(i)->getPtr(), nodes_.at(i)->getSize());
                x_i += x_incr_.segment(nodeLocation(i), nodes_.at(i)->getSize());
            }
            // Zero the error
            b_.setZero();

            time_solving_ += ((double)clock() - t_solving_) / CLOCKS_PER_SEC;
            n_new_constraints_ = 0;
            return 1;
        }

        void nodePermutation2VariablesPermutation(const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> &_perm_nodes,
                                                  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> &perm_variables)
        {
            //std::cout << "perm_nodes: " << _perm_nodes.indices().transpose() << std::endl;
            nodePermutation2nodeLocations(_perm_nodes, node_locations_);
            //std::cout << "locations: " << locations.transpose() << std::endl;
            //std::cout << "perm_variables: " << perm_variables.indices().transpose() << std::endl;

            unsigned int last_idx = 0;
            for (unsigned int i = 0; i < node_locations_.size(); i++)
            {
                perm_variables.indices().segment(last_idx, nodeDim(i)) = Eigen::VectorXi::LinSpaced(
                        nodeDim(i), node_locations_(i), node_locations_(i) + nodeDim(i) - 1);
                last_idx += nodeDim(i);
                //std::cout << i << " perm_variables: " << perm_variables.indices().transpose() << std::endl;
            }
            //std::cout << "perm_variables: " << perm_variables.indices().transpose() << std::endl;
        }

        void nodePermutation2nodeLocations(const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> &_perm_nodes,
                                           Eigen::ArrayXi& locations)
        {
            locations = _perm_nodes.indices().array();

            for (unsigned int i = 0; i < locations.size(); i++)
                locations = (locations > locations(i)).select(locations + nodeDim(i) - 1, locations);
        }

        void augmentPermutation(Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> &perm, unsigned int new_size)
        {
            unsigned int old_size = perm.indices().size();
            unsigned int dim = new_size - old_size;

            Eigen::VectorXi new_indices(new_size);
            new_indices.head(old_size) = perm.indices();
            new_indices.tail(dim) = Eigen::ArrayXi::LinSpaced(dim, old_size, new_size - 1);
            perm.resize(new_size);
            perm.indices() = new_indices;
            std::cout << "permutation augmented" << std::endl;

            // resize and update locations
            node_locations_.conservativeResize(node_locations_.size() + 1);
            node_locations_(node_locations_.size() - 1) = x_incr_.size();
            std::cout << "node_locations_ augmented" << std::endl;
        }

        void accumulatePermutation(const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> &perm)
        {
            //std::cout << std::endl << "old acc_permutation_nodes_ " << acc_permutation_nodes_.indices().transpose() << std::endl;
            //std::cout << "incr perm " << perm.indices().transpose() << std::endl;

            // acumulate permutation
            if (perm.size() == acc_node_permutation_.size()) //full permutation
                acc_node_permutation_ = perm * acc_node_permutation_;
            else //partial permutation
            {
                Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> incr_permutation_nodes(
                        Eigen::VectorXi::LinSpaced(nNodes(), 0, nNodes() - 1)); // identity permutation
                incr_permutation_nodes.indices().tail(perm.size()) = perm.indices()
                        + Eigen::VectorXi::Constant(perm.size(), nNodes() - perm.size());
                //std::cout << "incr perm " << incr_permutation_nodes.indices().transpose() << std::endl;
                acc_node_permutation_ = incr_permutation_nodes * acc_node_permutation_;
            }
            //std::cout << "new acc_permutation_nodes_ " << acc_permutation_nodes_.indices().transpose() << std::endl;

            // update nodes orders and locations
            nodePermutation2nodeLocations(acc_node_permutation_, node_locations_);
        }

        unsigned int nodeDim(const unsigned int _idx)
        {
            assert(_idx < nNodes());
            return nodes_.at(_idx)->getSize();
        }

        unsigned int nodeOrder(const unsigned int _idx)
        {
            assert(_idx < nNodes());
            assert(_idx < acc_node_permutation_.indices().size());
            return acc_node_permutation_.indices()(_idx);
        }

        unsigned int nodeLocation(const unsigned int _idx)
        {
            assert(_idx < nNodes());
            assert(_idx < node_locations_.size());
            return node_locations_(_idx);
        }

        unsigned int nNodes()
        {
            return nodes_.size();
        }

        CostFunctionBase* createCostFunction(ConstraintBase* _corrPtr)
        {
            //std::cout << "adding ctr " << _corrPtr->nodeId() << std::endl;
            //_corrPtr->print();

            switch (_corrPtr->getType())
            {
                case CTR_GPS_FIX_2D:
                {
                    ConstraintGPS2D* specific_ptr = (ConstraintGPS2D*)(_corrPtr);
                    return (CostFunctionBase*)(new CostFunctionSparse<ConstraintGPS2D, specific_ptr->measurementSize,
                            specific_ptr->block0Size, specific_ptr->block1Size, specific_ptr->block2Size,
                            specific_ptr->block3Size, specific_ptr->block4Size, specific_ptr->block5Size,
                            specific_ptr->block6Size, specific_ptr->block7Size, specific_ptr->block8Size,
                            specific_ptr->block9Size>(specific_ptr));
                    break;
                }
                case CTR_ODOM_2D:
                {
                    ConstraintOdom2D* specific_ptr = (ConstraintOdom2D*)(_corrPtr);
                    return (CostFunctionBase*)new CostFunctionSparse<ConstraintOdom2D, specific_ptr->measurementSize,
                            specific_ptr->block0Size, specific_ptr->block1Size, specific_ptr->block2Size,
                            specific_ptr->block3Size, specific_ptr->block4Size, specific_ptr->block5Size,
                            specific_ptr->block6Size, specific_ptr->block7Size, specific_ptr->block8Size,
                            specific_ptr->block9Size>(specific_ptr);
                    break;
                }
                case CTR_CORNER_2D:
                {
                    ConstraintCorner2D* specific_ptr = (ConstraintCorner2D*)(_corrPtr);
                    return (CostFunctionBase*)new CostFunctionSparse<ConstraintCorner2D, specific_ptr->measurementSize,
                            specific_ptr->block0Size, specific_ptr->block1Size, specific_ptr->block2Size,
                            specific_ptr->block3Size, specific_ptr->block4Size, specific_ptr->block5Size,
                            specific_ptr->block6Size, specific_ptr->block7Size, specific_ptr->block8Size,
                            specific_ptr->block9Size>(specific_ptr);
                    break;
                }
                default:
                    std::cout << "Unknown constraint type! Please add it in the CeresWrapper::createCostFunction()"
                            << std::endl;

                    return nullptr;
            }
        }

        void printResults()
        {
            std::cout << " solved in " << time_solving_ * 1e3 << " ms | " << R_.nonZeros() << " nonzeros in R"
                    << std::endl;
            std::cout << "x = " << x_incr_.transpose() << std::endl;
        }

        void printProblem()
        {
            std::cout << std::endl << "A_nodes_: " << std::endl
                    << Eigen::MatrixXi::Identity(A_nodes_.rows(), A_nodes_.rows()) * A_nodes_ << std::endl << std::endl;
            //std::cout << "A_: " << std::endl << MatrixXd::Identity(A_.rows(), A_.rows()) * A_  << std::endl << std::endl;
            std::cout << "b_: " << std::endl << b_.transpose() << std::endl << std::endl;
        }
};

} // namespace wolf

#endif /* TRUNK_SRC_SOLVER_QR_SOLVER_H_ */
