/*
 * test_iQR_wolf.cpp
 *
 *  Created on: Jun 17, 2015
 *      Author: jvallve
 */

//std includes
#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

// eigen includes
#include <eigen3/Eigen/OrderingMethods>
#include <eigen3/Eigen/SparseQR>
#include <Eigen/SPQRSupport>

//Wolf includes
#include "state_base.h"
#include "constraint_base.h"
#include "wolf_manager.h"

// wolf solver
#include "solver/ccolamd_ordering.h"
#include "solver/cost_function_base.h"
#include "solver/cost_function_sparse.h"

//C includes for sleep, time and main args
#include "unistd.h"

//faramotics includes
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

//laser_scan_utils
#include "iri-algorithms/laser_scan_utils/corner_detector.h"
#include "iri-algorithms/laser_scan_utils/entities.h"

using namespace Eigen;

//function travel around
void motionCampus(unsigned int ii, Cpose3d & pose, double& displacement_, double& rotation_)
{
    if (ii <= 120)
    {
        displacement_ = 0.1;
        rotation_ = 0;
    }
    else if ((ii > 120) && (ii <= 170))
    {
        displacement_ = 0.2;
        rotation_ = 1.8 * M_PI / 180;
    }
    else if ((ii > 170) && (ii <= 220))
    {
        displacement_ = 0;
        rotation_ = -1.8 * M_PI / 180;
    }
    else if ((ii > 220) && (ii <= 310))
    {
        displacement_ = 0.1;
        rotation_ = 0;
    }
    else if ((ii > 310) && (ii <= 487))
    {
        displacement_ = 0.1;
        rotation_ = -1. * M_PI / 180;
    }
    else if ((ii > 487) && (ii <= 600))
    {
        displacement_ = 0.2;
        rotation_ = 0;
    }
    else if ((ii > 600) && (ii <= 700))
    {
        displacement_ = 0.1;
        rotation_ = -1. * M_PI / 180;
    }
    else if ((ii > 700) && (ii <= 780))
    {
        displacement_ = 0;
        rotation_ = -1. * M_PI / 180;
    }
    else
    {
        displacement_ = 0.3;
        rotation_ = 0.0 * M_PI / 180;
    }

    pose.moveForward(displacement_);
    pose.rt.setEuler(pose.rt.head() + rotation_, pose.rt.pitch(), pose.rt.roll());
}

class block_pruning
{
    public:
        int col, row, Nrows, Ncols;
        block_pruning(int _col, int _row, int _Nrows, int _Ncols) :
                col(_col),
                row(_row),
                Nrows(_Nrows),
                Ncols(_Ncols)
        {
            //
        }
        bool operator()(int i, int j, double) const
        {
            return (i < row || i > row + Nrows-1) || (j < col || j > col + Ncols-1);
        }
};

void erase_sparse_block(SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col, const unsigned int& Nrows, const unsigned int& Ncols)
{
    // prune all non-zero elements that not satisfy the 'keep' operand
    // elements that are not in the block rows or are not in the block columns should be kept
    //original.prune([](int i, int j, double) { return (i < row || i > row + Nrows-1) || (j < col || j > col + Ncols-1); });

    block_pruning bp(row, col, Nrows, Ncols);
    original.prune(bp);
}

void add_sparse_block(const MatrixXd& ins, SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col)
{
  for (uint r=0; r<ins.rows(); ++r)
      for (uint c = 0; c < ins.cols(); c++)
          if (ins(r,c) != 0)
              original.coeffRef(r + row, c + col) += ins(r,c);
}

struct measurement
{
    public:
        std::vector<int> nodes_idx;
        VectorXd error;
        int dim;
        int location;

        measurement(const std::vector<int> & _idxs, const VectorXd &_error, const int _meas_dim) :
            nodes_idx(_idxs),
            error(_error),
            dim(_meas_dim),
            location(0)
        {

        }
};

class SolverQR
{
    protected:
        std::string name_;
        SparseQR < SparseMatrix<double>, NaturalOrdering<int>> solver_;
        SparseMatrix<double> A_, R_;
        VectorXd b_, x_incr_;
        int n_measurements;
        int n_nodes_;
        //std::vector<node> nodes_;
        std::vector<StateBase*> states_;
        std::vector<measurement> measurements_;
        std::vector<ConstraintBase*> constraints_;
        std::vector<CostFunctionBase*> cost_functions_;

        // ordering
        SparseMatrix<int> A_nodes_;
        PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_nodes_;
        std::map<int, int> id_2_idx_;

        CCOLAMDOrdering<int> ordering_;
        VectorXi nodes_ordering_constraints_;
        ArrayXi node_locations_;
        int n_new_measurements_;

        // time
        clock_t t_ordering_, t_solving_, t_managing_;
        double time_ordering_, time_solving_, time_managing_;



    public:
        SolverQR(const std::string &_name) :
            name_(_name),
            A_(0,0),
            R_(0,0), 
            n_measurements(0),
            n_nodes_(0),
            A_nodes_(0,0),
            acc_permutation_nodes_(0),
            n_new_measurements_(0),
            time_ordering_(0),
            time_solving_(0),
            time_managing_(0)
        {
            node_locations_.resize(0);
        }

        virtual ~SolverQR()
        {
            
        }

        void update(WolfProblem* _problem_ptr)
        {
            // IF REALLOCATION OF STATE, REMOVE EVERYTHING AND BUILD THE PROBLEM AGAIN
            if (_problem_ptr->isReallocated())
            {
                // TODO
//                // Remove all parameter blocks (residual blocks will be also removed)
//                removeAllStateUnits();
//
//                // Add all parameter blocks
//                for(auto state_unit_it = _problem_ptr->getStateListPtr()->begin(); state_unit_it!=_problem_ptr->getStateListPtr()->end(); state_unit_it++)
//                    addStateUnit(*state_unit_it);
//
//                // Add all residual blocks
//                ConstraintBaseList ctr_list;
//                _problem_ptr->getTrajectoryPtr()->getConstraintList(ctr_list);
//                for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
//                    addConstraint(*ctr_it);
//
//                // set the wolf problem reallocation flag to false
//                _problem_ptr->reallocationDone();
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
                    // TODO
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

        void addStateUnit(StateBase* _state_ptr)
        {
            std::cout << "adding state unit " << _state_ptr->nodeId() << std::endl;
            if (_state_ptr->getStateStatus() == ST_ESTIMATED)
            {

                t_managing_ = clock();
                unsigned int node_dim = _state_ptr->getStateSize();
                int node_idx= _state_ptr->nodeId();

                n_nodes_++;
                states_.push_back(_state_ptr);
                id_2_idx_[_state_ptr->nodeId()] = states_.size()-1;

                std::cout << "idx " << id_2_idx_[_state_ptr->nodeId()] << std::endl;

                //nodes_.push_back(node(node_idx, node_dim, x_incr_.size(), n_nodes_-1));

                // Resize accumulated permutations
                augment_permutation(acc_permutation_nodes_, n_nodes_);

                // Resize state
                x_incr_.conservativeResize(x_incr_.size() + node_dim);

                // Resize problem
                A_.conservativeResize(A_.rows(), A_.cols() + node_dim);
                R_.conservativeResize(R_.cols() + node_dim, R_.cols() + node_dim);
                //A_nodes_.conservativeResize(n_measurements, n_nodes); // not necessary

                time_managing_ += ((double) clock() - t_managing_) / CLOCKS_PER_SEC;
            }
            _state_ptr->setPendingStatus(NOT_PENDING);
        }

        void updateStateUnitStatus(StateBase* _state_ptr)
        {
            //TODO
        }

        void addConstraint(ConstraintBase* _constraint_ptr)
        {
            std::cout << "adding constraint " << _constraint_ptr->nodeId() << std::endl;
            t_managing_ = clock();

            constraints_.push_back(_constraint_ptr);
            cost_functions_.push_back(createCostFunction(_constraint_ptr));

            int meas_dim = _constraint_ptr->getSize();

            std::vector<MatrixXs> jacobians(_constraint_ptr->getStatePtrVector().size());
            VectorXs error(_constraint_ptr->getSize());

            cost_functions_.back()->evaluateResidualJacobians();
            cost_functions_.back()->getResidual(error);
            cost_functions_.back()->getJacobians(jacobians);

            std::vector<int> idxs;
            for (int i = 0; i < _constraint_ptr->getStatePtrVector().size(); i++)
                if (_constraint_ptr->getStatePtrVector().at(i)->getStateStatus() == ST_ESTIMATED)
                    idxs.push_back(id_2_idx_[_constraint_ptr->getStatePtrVector().at(i)->nodeId()]);

            measurement _meas(idxs, error, meas_dim);

            n_measurements++;
            n_new_measurements_++;
            measurements_.push_back(_meas);
            measurements_.back().location = A_.rows();

            // Resize problem
            A_.conservativeResize(A_.rows() + meas_dim, A_.cols());
            b_.conservativeResize(b_.size() + meas_dim);
            A_nodes_.conservativeResize(n_measurements,n_nodes_);

            // ADD MEASUREMENTS
            for (unsigned int j = 0; j < idxs.size(); j++)
            {
                assert(acc_permutation_nodes_.indices()(idxs.at(j)) == node_order(idxs.at(j)));//nodes_.at(idxs.at(j)).order);

                //int ordered_node = nodes_.at(idxs.at(j)).order;

                add_sparse_block(jacobians.at(j), A_, A_.rows()-meas_dim, node_location(idxs.at(j)));//nodes_.at(idxs.at(j)).location);

                A_nodes_.coeffRef(A_nodes_.rows()-1, node_order(idxs.at(j))) = 1;

                assert(jacobians.at(j).cols() == node_dim(idxs.at(j)));//nodes_.at(idxs.at(j)).dim);
                assert(jacobians.at(j).rows() == meas_dim);
            }

            // error
            b_.tail(meas_dim) = error;

            _constraint_ptr->setPendingStatus(NOT_PENDING);

            time_managing_ += ((double) clock() - t_managing_) / CLOCKS_PER_SEC;
        }

        void ordering(const int & _first_ordered_idx)
        {
            std::cout << "ordering from idx " << _first_ordered_idx << std::endl;
            t_ordering_ = clock();

            // full problem ordering
            if (_first_ordered_idx == -1)
            {
                // ordering ordering constraints
                nodes_ordering_constraints_.resize(n_nodes_);
                nodes_ordering_constraints_ = A_nodes_.bottomRows(1).transpose();

                // computing nodes partial ordering_
                A_nodes_.makeCompressed();
                PermutationMatrix<Dynamic, Dynamic, int> incr_permutation_nodes(n_nodes_);
                ordering_(A_nodes_, incr_permutation_nodes, nodes_ordering_constraints_.data());

                // node ordering to variable ordering
                PermutationMatrix<Dynamic, Dynamic, int> incr_permutation(A_.cols());
                permutation_2_block_permutation(incr_permutation_nodes, incr_permutation);

                // apply partial_ordering orderings
                A_nodes_ = (A_nodes_ * incr_permutation_nodes.transpose()).sparseView();
                A_ = (A_ * incr_permutation.transpose()).sparseView();

                // ACCUMULATING PERMUTATIONS
                accumulate_permutation(incr_permutation_nodes);
            }

            // partial ordering
            else
            {
                int first_ordered_node = node_order(_first_ordered_idx);//nodes_.at(_first_ordered_idx).order;
                int ordered_nodes = n_nodes_ - first_ordered_node;
                int unordered_nodes = n_nodes_ - ordered_nodes;
                if (ordered_nodes > 2) // only reordering when involved nodes in the measurement are not the two last ones
                {
                    // SUBPROBLEM ORDERING (from first node variable to last one)
                    //std::cout << "ordering partial_ordering problem: " << _first_ordered_node << " to "<< n_nodes_ - 1 << std::endl;
                    SparseMatrix<int> sub_A_nodes_ = A_nodes_.rightCols(ordered_nodes);

                    // _partial_ordering ordering_ constraints
                    nodes_ordering_constraints_.resize(ordered_nodes);
                    nodes_ordering_constraints_ = sub_A_nodes_.bottomRows(1).transpose();

                    // computing nodes partial ordering_
                    sub_A_nodes_.makeCompressed();
                    PermutationMatrix<Dynamic, Dynamic, int> partial_permutation_nodes(ordered_nodes);
                    ordering_(sub_A_nodes_, partial_permutation_nodes, nodes_ordering_constraints_.data());

                    // node ordering to variable ordering
                    int ordered_variables = A_.cols() - node_location(_first_ordered_idx);//nodes_.at(_first_ordered_idx).location;
//                    std::cout << "first_ordered_node " << first_ordered_node << std::endl;
//                    std::cout << "A_.cols() " << A_.cols() << std::endl;
//                    std::cout << "nodes_.at(_first_ordered_idx).location " << nodes_.at(_first_ordered_idx).location << std::endl;
//                    std::cout << "ordered_variables " << ordered_variables << std::endl;
                    PermutationMatrix<Dynamic, Dynamic, int> partial_permutation(ordered_variables);
                    permutation_2_block_permutation(partial_permutation_nodes, partial_permutation);

                    // apply partial_ordering orderings
                    A_nodes_.rightCols(ordered_nodes) = (A_nodes_.rightCols(ordered_nodes) * partial_permutation_nodes.transpose()).sparseView();
                    A_.rightCols(ordered_variables) = (A_.rightCols(ordered_variables) * partial_permutation.transpose()).sparseView();
                    R_.rightCols(ordered_variables) = (R_.rightCols(ordered_variables) * partial_permutation.transpose()).sparseView();

                    // ACCUMULATING PERMUTATIONS
                    accumulate_permutation(partial_permutation_nodes);
                }
            }
            time_ordering_ += ((double) clock() - t_ordering_) / CLOCKS_PER_SEC;
        }

        bool solve(const int mode)
        {
            if (n_new_measurements_ == 0)
                return 1;

            std::cout << "solving mode " << mode << std::endl;

            int first_ordered_node = n_nodes_;
            int first_ordered_idx;
            for (int i = 0; i < n_new_measurements_; i++)
            {
                ConstraintBase* ct_ptr = constraints_.at(constraints_.size()-1-i);
                std::cout << "constraint: " << i << " id: " << constraints_.at(constraints_.size()-1-i)->nodeId() << std::endl;
                for (int j = 0; j < ct_ptr->getStatePtrVector().size(); j++)
                {
                    if (ct_ptr->getStatePtrVector().at(j)->getStateStatus() == ST_ESTIMATED)
                    {
                        int idx = id_2_idx_[ct_ptr->getStatePtrVector().at(j)->nodeId()];
                        std::cout << "estimated idx " << idx << std::endl;
                        std::cout << "node_order(idx) " << node_order(idx) << std::endl;
                        std::cout << "first_ordered_node " << first_ordered_node << std::endl;
                        if (first_ordered_node > node_order(idx))//nodes_.at(idx).order)
                        {
                            first_ordered_idx = idx;
                            std::cout << "first_ordered_idx " << first_ordered_idx << std::endl;
                            first_ordered_node = node_order(idx);
                            std::cout << "first_ordered_node " << first_ordered_node << std::endl;
                        }
                    }
                }
            }
            std::cout << "first_ordered_node " << first_ordered_node << std::endl;
            std::cout << "first_ordered_idx " << first_ordered_idx << std::endl;

            bool batch = (mode !=2 || first_ordered_node == 0);
            bool order = (mode !=0 && n_nodes_ > 1);

            // BATCH
            if (batch)
            {
                // REORDER
                if (order)
                    ordering(-1);

                //print_problem();

                // SOLVE
                t_solving_ = clock();
                A_.makeCompressed();
                solver_.compute(A_);
                if (solver_.info() != Success)
                {
                    std::cout << "decomposition failed" << std::endl;
                    return 0;
                }
                x_incr_ = solver_.solve(b_);
                R_ = solver_.matrixR();
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.cols(), R_.cols()) * R_ << std::endl;
                time_solving_ += ((double) clock() - t_solving_) / CLOCKS_PER_SEC;
            }
            // INCREMENTAL
            else
            {
                // REORDER SUBPROBLEM
                ordering(first_ordered_idx);
                //print_problem();

                // SOLVE ORDERED SUBPROBLEM
                t_solving_= clock();
                A_nodes_.makeCompressed();
                A_.makeCompressed();

                // finding measurements block
                SparseMatrix<int> measurements_to_initial = A_nodes_.col(first_ordered_node);
        //        std::cout << "measurements_to_initial " << measurements_to_initial << std::endl;
        //        std::cout << "measurements_to_initial.innerIndexPtr()[measurements_to_initial.outerIndexPtr()[0]] " << measurements_to_initial.innerIndexPtr()[measurements_to_initial.outerIndexPtr()[0]] << std::endl;
                int first_ordered_measurement = measurements_to_initial.innerIndexPtr()[measurements_to_initial.outerIndexPtr()[0]];
                int ordered_measurements = A_.rows() - measurements_.at(first_ordered_measurement).location;
                int ordered_variables = A_.cols() - node_location(first_ordered_idx);//nodes_.at(first_ordered_idx).location;
                int unordered_variables = node_location(first_ordered_idx);//nodes_.at(first_ordered_idx).location;

                SparseMatrix<double> A_partial = A_.bottomRightCorner(ordered_measurements, ordered_variables);
                solver_.compute(A_partial);
                if (solver_.info() != Success)
                {
                    std::cout << "decomposition failed" << std::endl;
                    return 0;
                }
                //std::cout << "R new" << std::endl << MatrixXd::Identity(A_partial.cols(), A_partial.cols()) * solver_.matrixR() << std::endl;
                x_incr_.tail(ordered_variables) = solver_.solve(b_.tail(ordered_measurements));

                // store new part of R
                erase_sparse_block(R_, unordered_variables, unordered_variables, ordered_variables, ordered_variables);
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.rows(), R_.rows()) * R_ << std::endl;
                add_sparse_block(solver_.matrixR(), R_, unordered_variables, unordered_variables);
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.rows(), R_.rows()) * R_ << std::endl;
                R_.makeCompressed();

                // solving not ordered subproblem
                if (unordered_variables > 0)
                {
                    //std::cout << "--------------------- solving unordered part" << std::endl;
                    SparseMatrix<double> R1 = R_.topLeftCorner(unordered_variables, unordered_variables);
                    //std::cout << "R1" << std::endl << MatrixXd::Identity(R1.rows(), R1.rows()) * R1 << std::endl;
                    SparseMatrix<double> R2 = R_.topRightCorner(unordered_variables, ordered_variables);
                    //std::cout << "R2" << std::endl << MatrixXd::Identity(R2.rows(), R2.rows()) * R2 << std::endl;
                    solver_.compute(R1);
                    if (solver_.info() != Success)
                    {
                        std::cout << "decomposition failed" << std::endl;
                        return 0;
                    }
                    x_incr_.head(unordered_variables) = solver_.solve(b_.head(unordered_variables) - R2 * x_incr_.tail(ordered_variables));
                }

            }
//            // UNDO ORDERING FOR RESULT
//            PermutationMatrix<Dynamic, Dynamic, int> acc_permutation(A_.cols());
//            permutation_2_block_permutation(acc_permutation_nodes_, acc_permutation); // TODO via pointers
//            x_incr_ = acc_permutation.inverse() * x_incr_;

            // UPDATE X VALUE
            for (unsigned int i = 0; i<states_.size(); i++)
            {
                Map<VectorXs> x_i(states_.at(i)->getPtr(), states_.at(i)->getStateSize());
                x_i += x_incr_.segment(node_location(i), states_.at(i)->getStateSize());
            }



            time_solving_ += ((double) clock() - t_solving_) / CLOCKS_PER_SEC;
            n_new_measurements_ = 0;
            return 1;
        }


        void permutation_2_block_permutation(const PermutationMatrix<Dynamic, Dynamic, int> &_perm_nodes, PermutationMatrix<Dynamic, Dynamic, int> &perm_variables)
        {
            //std::cout << "perm_nodes: " << _perm_nodes.indices().transpose() << std::endl;
            perm_nodes_2_locations(_perm_nodes, node_locations_);
            //std::cout << "locations: " << locations.transpose() << std::endl;
            //std::cout << "perm_variables: " << perm_variables.indices().transpose() << std::endl;

            int last_idx = 0;
            for (unsigned int i = 0; i<node_locations_.size(); i++)
            {
                perm_variables.indices().segment(last_idx, node_dim(i)) = VectorXi::LinSpaced(node_dim(i), node_locations_(i), node_locations_(i)+node_dim(i)-1);
                last_idx += node_dim(i);
                //std::cout << i << " perm_variables: " << perm_variables.indices().transpose() << std::endl;
            }
            //std::cout << "perm_variables: " << perm_variables.indices().transpose() << std::endl;
        }

        void perm_nodes_2_locations(const PermutationMatrix<Dynamic, Dynamic, int> &_perm_nodes, ArrayXi& locations)
        {
            locations = _perm_nodes.indices().array();

            for (unsigned int i = 0; i<locations.size(); i++)
                locations = (locations > locations(i)).select(locations + node_dim(i)-1, locations);
        }

        void augment_permutation(PermutationMatrix<Dynamic, Dynamic, int> &perm, const int new_size)
        {
            int old_size = perm.indices().size();
            int dim = new_size - old_size;
            VectorXi new_indices(new_size);
            new_indices.head(old_size)= perm.indices();
            new_indices.tail(dim) = ArrayXi::LinSpaced(dim, old_size, new_size-1);
            perm.resize(new_size);
            perm.indices() = new_indices;
            std::cout << "permutation augmented" << std::endl;

            // resize and update locations
            node_locations_.conservativeResize(node_locations_.size() + 1);
            node_locations_(node_locations_.size()-1) = x_incr_.size();
            std::cout << "node_locations_ augmented" << std::endl;
        }

        void accumulate_permutation(const PermutationMatrix<Dynamic, Dynamic, int> &perm)
        {
            print_name();
            //std::cout << std::endl << "old acc_permutation_nodes_ " << acc_permutation_nodes_.indices().transpose() << std::endl;
            //std::cout << "incr perm " << perm.indices().transpose() << std::endl;

            // acumulate permutation
            if (perm.size() == acc_permutation_nodes_.size()) //full permutation
                acc_permutation_nodes_ = perm * acc_permutation_nodes_;
            else //partial permutation
            {
                PermutationMatrix<Dynamic, Dynamic, int> incr_permutation_nodes(VectorXi::LinSpaced(n_nodes_, 0, n_nodes_ - 1)); // identity permutation
                incr_permutation_nodes.indices().tail(perm.size()) = perm.indices() + VectorXi::Constant(perm.size(), n_nodes_ - perm.size());
                //std::cout << "incr perm " << incr_permutation_nodes.indices().transpose() << std::endl;
                acc_permutation_nodes_ = incr_permutation_nodes * acc_permutation_nodes_;
            }
            //std::cout << "new acc_permutation_nodes_ " << acc_permutation_nodes_.indices().transpose() << std::endl;

            // update nodes orders and locations
            perm_nodes_2_locations(acc_permutation_nodes_, node_locations_);

//            for (int i = 0; i < nodes_.size(); i++)
//            {
//                nodes_.at(i).order = acc_permutation_nodes_.indices()(i);
//                nodes_.at(i).location = node_locations_(i);
//            }
        }

        int node_dim(const int _idx)
        {
            return states_.at(_idx)->getStateSize();
        }

        int node_order(const int _idx)
        {
            return acc_permutation_nodes_.indices()(_idx);
        }

        int node_location(const int _idx)
        {
            return node_locations_(_idx);
        }

        CostFunctionBase* createCostFunction(ConstraintBase* _corrPtr)
        {
            //std::cout << "adding ctr " << _corrPtr->nodeId() << std::endl;
            //_corrPtr->print();

            switch (_corrPtr->getConstraintType())
            {
                case CTR_GPS_FIX_2D:
                {
                    ConstraintGPS2D* specific_ptr = (ConstraintGPS2D*)(_corrPtr);
                    return (CostFunctionBase*)(new CostFunctionSparse<ConstraintGPS2D,
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
                                                               specific_ptr->block9Size>(specific_ptr));
                    break;
                }
                case CTR_ODOM_2D_COMPLEX_ANGLE:
                {
                    ConstraintOdom2DComplexAngle* specific_ptr = (ConstraintOdom2DComplexAngle*)(_corrPtr);
                    return (CostFunctionBase*)new CostFunctionSparse<ConstraintOdom2DComplexAngle,
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
                    return (CostFunctionBase*)new CostFunctionSparse<ConstraintOdom2DTheta,
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
                    return (CostFunctionBase*)new CostFunctionSparse<ConstraintCorner2DTheta,
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

        void print_name()
        {
            std::cout << name_;
        }

        void print_results()
        {
            print_name();
            std::cout << " solved in " << time_solving_*1e3 << " ms | " << R_.nonZeros() << " nonzeros in R"<< std::endl;
            std::cout << "x = " << x_incr_.transpose() << std::endl;
        }

        void print_problem()
        {
            print_name();
            std::cout << std::endl << "A_nodes_: " << std::endl << MatrixXi::Identity(A_nodes_.rows(), A_nodes_.rows()) * A_nodes_  << std::endl << std::endl;
            //std::cout << "A_: " << std::endl << MatrixXd::Identity(A_.rows(), A_.rows()) * A_  << std::endl << std::endl;
            std::cout << "b_: " << std::endl << b_.transpose() << std::endl << std::endl;
        }
};

//main
int main(int argc, char *argv[])
{
    // USER INPUT ============================================================================================
    if (argc != 2 || atoi(argv[1]) < 1 || atoi(argv[1]) > 1100)
    {
        std::cout << "Please call me with: [./test_ceres_manager NI], where:" << std::endl;
        std::cout << "     - NI is the number of iterations (0 < NI < 1100)" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }
    bool complex_angle = false;
    unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution

    // INITIALIZATION ============================================================================================
    // Problems
    SolverQR solver_ordered("FULL ORDERED");
    SolverQR solver_unordered("UNORDERED");
    SolverQR solver_ordered_partial("PARTIALLY ORDERED");

    //init random generators
    WolfScalar odom_std_factor = 0.1;
    WolfScalar gps_std = 1;
    std::default_random_engine generator(1);
    std::normal_distribution<WolfScalar> distribution_odom(0.0, odom_std_factor); //odometry noise
    std::normal_distribution<WolfScalar> distribution_gps(0.0, gps_std); //GPS noise

    std::ofstream log_file, landmark_file;  //output file

    // Faramotics stuff
    Cpose3d viewPoint, devicePose, laser1Pose, laser2Pose, estimated_vehicle_pose, estimated_laser_1_pose, estimated_laser_2_pose;
    vector < Cpose3d > devicePoses;
    vector<float> scan1, scan2;
    string modelFileName;

    //model and initial view point
    modelFileName = "/home/jvallve/iri-lab/faramotics/models/campusNordUPC.obj";
    //modelFileName = "/home/acoromin/dev/br/faramotics/models/campusNordUPC.obj";
    //modelFileName = "/home/andreu/dev/faramotics/models/campusNordUPC.obj";
    devicePose.setPose(2, 8, 0.2, 0, 0, 0);
    viewPoint.setPose(devicePose);
    viewPoint.moveForward(10);
    viewPoint.rt.setEuler(viewPoint.rt.head() + M_PI / 2, viewPoint.rt.pitch() + 30. * M_PI / 180., viewPoint.rt.roll());
    viewPoint.moveForward(-15);
    //glut initialization
    faramotics::initGLUT(argc, argv);

    //create a viewer for the 3D model and scan points
    CdynamicSceneRender* myRender = new CdynamicSceneRender(1200, 700, 90 * M_PI / 180, 90 * 700.0 * M_PI / (1200.0 * 180.0), 0.2, 100);
    myRender->loadAssimpModel(modelFileName, true); //with wireframe
    //create scanner and load 3D model
    CrangeScan2D* myScanner = new CrangeScan2D(HOKUYO_UTM30LX_180DEG);  //HOKUYO_UTM30LX_180DEG or LEUZE_RS4
    myScanner->loadAssimpModel(modelFileName);

    //variables
    Eigen::Vector3s odom_reading;
    Eigen::Vector2s gps_fix_reading;
    Eigen::VectorXs pose_odom(3); //current odometry integred pose
    Eigen::VectorXs ground_truth(n_execution * 3); //all true poses
    Eigen::VectorXs odom_trajectory(n_execution * 3); //open loop trajectory
    Eigen::VectorXs mean_times = Eigen::VectorXs::Zero(7);
    clock_t t1, t2;

    // Wolf manager initialization
    Eigen::Vector3s odom_pose = Eigen::Vector3s::Zero();
    Eigen::Vector3s gps_pose = Eigen::Vector3s::Zero();
    Eigen::Vector4s laser_1_pose, laser_2_pose; //xyz + theta
    laser_1_pose << 1.2, 0, 0, 0; //laser 1
    laser_2_pose << -1.2, 0, 0, M_PI; //laser 2
    SensorOdom2D odom_sensor(new StatePoint3D(odom_pose.data()), new StateTheta(&odom_pose(2)), odom_std_factor, odom_std_factor);
    SensorGPSFix gps_sensor(new StatePoint3D(gps_pose.data()), new StateTheta(&gps_pose(2)), gps_std);
    SensorLaser2D laser_1_sensor(new StatePoint3D(laser_1_pose.data()), new StateTheta(&laser_1_pose(3)));
    SensorLaser2D laser_2_sensor(new StatePoint3D(laser_2_pose.data()), new StateTheta(&laser_2_pose(3)));

    // Initial pose
    pose_odom << 2, 8, 0;
    ground_truth.head(3) = pose_odom;
    odom_trajectory.head(3) = pose_odom;

    WolfManager<StatePoint2D, StateTheta>* wolf_manager = new WolfManager<StatePoint2D, StateTheta>(1e3, &odom_sensor, pose_odom, Eigen::Matrix3s::Identity() * 0.01, n_execution*10, 0.01);

    std::cout << "STARTING INCREMENTAL QR TEST" << std::endl << std::endl;
    std::cout << "\n ========= 2D Robot with odometry and 2 LIDARs ===========\n";
    // START TRAJECTORY ============================================================================================
    for (unsigned int step = 1; step < n_execution; step++)
    {
        //get init time
        t2 = clock();

        // ROBOT MOVEMENT ---------------------------
        //std::cout << "ROBOT MOVEMENT..." << std::endl;
        // moves the device position
        t1 = clock();
        motionCampus(step, devicePose, odom_reading(0), odom_reading(2));
        odom_reading(1) = 0;
        devicePoses.push_back(devicePose);

        // SENSOR DATA ---------------------------
        //std::cout << "SENSOR DATA..." << std::endl;
        // store groundtruth
        ground_truth.segment(step * 3, 3) << devicePose.pt(0), devicePose.pt(1), devicePose.rt.head();

        // compute odometry
        odom_reading(0) += distribution_odom(generator) * (odom_reading(0) == 0 ? 1e-6 : odom_reading(0));
        odom_reading(1) += distribution_odom(generator) * 1e-6;
        odom_reading(2) += distribution_odom(generator) * (odom_reading(2) == 0 ? 1e-6 : odom_reading(2));

        // odometry integration
        pose_odom(0) = pose_odom(0) + odom_reading(0) * cos(pose_odom(2)) - odom_reading(1) * sin(pose_odom(2));
        pose_odom(1) = pose_odom(1) + odom_reading(0) * sin(pose_odom(2)) + odom_reading(1) * cos(pose_odom(2));
        pose_odom(2) = pose_odom(2) + odom_reading(1);
        odom_trajectory.segment(step * 3, 3) = pose_odom;

        // compute GPS
        gps_fix_reading << devicePose.pt(0), devicePose.pt(1);
        gps_fix_reading(0) += distribution_gps(generator);
        gps_fix_reading(1) += distribution_gps(generator);

        //compute scans
        scan1.clear();
        scan2.clear();
        // scan 1
        laser1Pose.setPose(devicePose);
        laser1Pose.moveForward(laser_1_pose(0));
        myScanner->computeScan(laser1Pose, scan1);
        // scan 2
        laser2Pose.setPose(devicePose);
        laser2Pose.moveForward(laser_2_pose(0));
        laser2Pose.rt.setEuler(laser2Pose.rt.head() + M_PI, laser2Pose.rt.pitch(), laser2Pose.rt.roll());
        myScanner->computeScan(laser2Pose, scan2);

        mean_times(0) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // ADD CAPTURES ---------------------------
        //std::cout << "ADD CAPTURES..." << std::endl;
        // adding new sensor captures
        wolf_manager->addCapture(new CaptureOdom2D(TimeStamp(), &odom_sensor, odom_reading));       //, odom_std_factor * Eigen::MatrixXs::Identity(2,2)));
        wolf_manager->addCapture(new CaptureGPSFix(TimeStamp(), &gps_sensor, gps_fix_reading, gps_std * Eigen::MatrixXs::Identity(3,3)));
        //wolf_manager->addCapture(new CaptureLaser2D(TimeStamp(), &laser_1_sensor, scan1));
        //wolf_manager->addCapture(new CaptureLaser2D(TimeStamp(), &laser_2_sensor, scan2));
        // updating problem
        wolf_manager->update();

        // UPDATING SOLVER ---------------------------
        //std::cout << "UPDATING..." << std::endl;
        // update state units and constraints in ceres
        solver_unordered.update(wolf_manager->getProblemPtr());

        // PRINT PROBLEM
        solver_unordered.print_problem();

        // SOLVE OPTIMIZATION ---------------------------
        //std::cout << "SOLVING..." << std::endl;
        solver_unordered.solve(0);

        std::cout << "========================= RESULTS " << step << ":" << std::endl;
        solver_unordered.print_results();

        // COMPUTE COVARIANCES ---------------------------
        //std::cout << "COMPUTING COVARIANCES..." << std::endl;
        // TODO

        // DRAWING STUFF ---------------------------
        // draw detected corners
        std::list < laserscanutils::Corner > corner_list;
        std::vector<double> corner_vector;
        CaptureLaser2D last_scan(TimeStamp(), &laser_1_sensor, scan1);
        last_scan.extractCorners(corner_list);
        for (std::list<laserscanutils::Corner>::iterator corner_it = corner_list.begin(); corner_it != corner_list.end(); corner_it++)
        {
            corner_vector.push_back(corner_it->pt_(0));
            corner_vector.push_back(corner_it->pt_(1));
        }
        myRender->drawCorners(laser1Pose, corner_vector);

        // draw landmarks
        std::vector<double> landmark_vector;
        for (auto landmark_it = wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++)
        {
            WolfScalar* position_ptr = (*landmark_it)->getPPtr()->getPtr();
            landmark_vector.push_back(*position_ptr); //x
            landmark_vector.push_back(*(position_ptr + 1)); //y
            landmark_vector.push_back(0.2); //z
        }
        myRender->drawLandmarks(landmark_vector);

        // draw localization and sensors
        estimated_vehicle_pose.setPose(wolf_manager->getVehiclePose()(0), wolf_manager->getVehiclePose()(1), 0.2, wolf_manager->getVehiclePose()(2), 0, 0);
        estimated_laser_1_pose.setPose(estimated_vehicle_pose);
        estimated_laser_1_pose.moveForward(laser_1_pose(0));
        estimated_laser_2_pose.setPose(estimated_vehicle_pose);
        estimated_laser_2_pose.moveForward(laser_2_pose(0));
        estimated_laser_2_pose.rt.setEuler(estimated_laser_2_pose.rt.head() + M_PI, estimated_laser_2_pose.rt.pitch(), estimated_laser_2_pose.rt.roll());
        myRender->drawPoseAxisVector( { estimated_vehicle_pose, estimated_laser_1_pose, estimated_laser_2_pose });

        //Set view point and render the scene
        //locate visualization view point, somewhere behind the device
        myRender->setViewPoint(viewPoint);
        myRender->drawPoseAxis(devicePose);
        myRender->drawScan(laser1Pose, scan1, 180. * M_PI / 180., 90. * M_PI / 180.); //draw scan
        myRender->render();

        // TIME MANAGEMENT ---------------------------
        double dt = ((double) clock() - t2) / CLOCKS_PER_SEC;
        mean_times(6) += dt;
        if (dt < 0.1)
            usleep(100000 - 1e6 * dt);

//      std::cout << "\nTree after step..." << std::endl;
//      wolf_manager->getProblemPtr()->print();
    }

    // DISPLAY RESULTS ============================================================================================
    mean_times /= n_execution;
    std::cout << "\nSIMULATION AVERAGE LOOP DURATION [s]" << std::endl;
    std::cout << "  data generation:    " << mean_times(0) << std::endl;
    std::cout << "  wolf managing:      " << mean_times(1) << std::endl;
    std::cout << "  ceres managing:     " << mean_times(2) << std::endl;
    std::cout << "  ceres optimization: " << mean_times(3) << std::endl;
    std::cout << "  ceres covariance:   " << mean_times(4) << std::endl;
    std::cout << "  results drawing:    " << mean_times(5) << std::endl;
    std::cout << "  loop time:          " << mean_times(6) << std::endl;

//  std::cout << "\nTree before deleting..." << std::endl;
//  wolf_manager->getProblemPtr()->print();

    // Draw Final result -------------------------
    std::vector<double> landmark_vector;
    for (auto landmark_it = wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++)
    {
        WolfScalar* position_ptr = (*landmark_it)->getPPtr()->getPtr();
        landmark_vector.push_back(*position_ptr); //x
        landmark_vector.push_back(*(position_ptr + 1)); //y
        landmark_vector.push_back(0.2); //z
    }
    myRender->drawLandmarks(landmark_vector);
//  viewPoint.setPose(devicePoses.front());
//  viewPoint.moveForward(10);
//  viewPoint.rt.setEuler( viewPoint.rt.head()+M_PI/4, viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
//  viewPoint.moveForward(-10);
    myRender->setViewPoint(viewPoint);
    myRender->render();

    // Print Final result in a file -------------------------
    // Vehicle poses
    int i = 0;
    Eigen::VectorXs state_poses(n_execution * 3);
    for (auto frame_it = wolf_manager->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()->begin(); frame_it != wolf_manager->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()->end(); frame_it++)
    {
        if (complex_angle)
            state_poses.segment(i, 3) << *(*frame_it)->getPPtr()->getPtr(), *((*frame_it)->getPPtr()->getPtr() + 1), atan2(*(*frame_it)->getOPtr()->getPtr(), *((*frame_it)->getOPtr()->getPtr() + 1));
        else
            state_poses.segment(i, 3) << *(*frame_it)->getPPtr()->getPtr(), *((*frame_it)->getPPtr()->getPtr() + 1), *(*frame_it)->getOPtr()->getPtr();
        i += 3;
    }

    // Landmarks
    i = 0;
    Eigen::VectorXs landmarks(wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->size() * 2);
    for (auto landmark_it = wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++)
    {
        Eigen::Map<Eigen::Vector2s> landmark((*landmark_it)->getPPtr()->getPtr());
        landmarks.segment(i, 2) = landmark;
        i += 2;
    }

    // Print log files
    std::string filepath = getenv("HOME") + (complex_angle ? std::string("/Desktop/log_file_3.txt") : std::string("/Desktop/log_file_2.txt"));
    log_file.open(filepath, std::ofstream::out); //open log file

    if (log_file.is_open())
    {
        log_file << 0 << std::endl;
        for (unsigned int ii = 0; ii < n_execution; ii++)
            log_file << state_poses.segment(ii * 3, 3).transpose() << "\t" << ground_truth.segment(ii * 3, 3).transpose() << "\t" << (state_poses.segment(ii * 3, 3) - ground_truth.segment(ii * 3, 3)).transpose() << "\t" << odom_trajectory.segment(ii * 3, 3).transpose() << std::endl;
        log_file.close(); //close log file
        std::cout << std::endl << "Result file " << filepath << std::endl;
    }
    else
        std::cout << std::endl << "Failed to write the log file " << filepath << std::endl;

    std::string filepath2 = getenv("HOME") + (complex_angle ? std::string("/Desktop/landmarks_file_3.txt") : std::string("/Desktop/landmarks_file_2.txt"));
    landmark_file.open(filepath2, std::ofstream::out); //open log file

    if (landmark_file.is_open())
    {
        for (unsigned int ii = 0; ii < landmarks.size(); ii += 2)
            landmark_file << landmarks.segment(ii, 2).transpose() << std::endl;
        landmark_file.close(); //close log file
        std::cout << std::endl << "Landmark file " << filepath << std::endl;
    }
    else
        std::cout << std::endl << "Failed to write the landmark file " << filepath << std::endl;

    std::cout << "Press any key for ending... " << std::endl << std::endl;
    std::getchar();

    delete myRender;
    delete myScanner;
    delete wolf_manager;
    std::cout << "wolf deleted" << std::endl;

    std::cout << " ========= END ===========" << std::endl << std::endl;

    //exit
    return 0;
}






