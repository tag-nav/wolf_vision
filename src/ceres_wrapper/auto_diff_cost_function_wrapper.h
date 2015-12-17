#ifndef TRUNK_SRC_SOLVER_AUTODIFF_COST_FUNCTION_WRAPPER_H_
#define TRUNK_SRC_SOLVER_AUTODIFF_COST_FUNCTION_WRAPPER_H_

// WOLF
#include "../wolf.h"

// CERES
#include "ceres/jet.h"
#include "ceres/sized_cost_function.h"

template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE, unsigned int BLOCK_6_SIZE, unsigned int BLOCK_7_SIZE, unsigned int BLOCK_8_SIZE, unsigned int BLOCK_9_SIZE>
class AutoDiffCostFunctionWrapper : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                                             BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                                                             BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE + BLOCK_6_SIZE + BLOCK_7_SIZE + BLOCK_8_SIZE + BLOCK_9_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                                     BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(10),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                          BLOCK_5_SIZE, BLOCK_6_SIZE, BLOCK_7_SIZE, BLOCK_8_SIZE, BLOCK_9_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE+BLOCK_7_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE+BLOCK_7_SIZE+BLOCK_8_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
            {
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4],
                                         parameters[5], parameters[6], parameters[7], parameters[8], parameters[9], residuals);
            }
            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet jets_2[BLOCK_2_SIZE];
                WolfJet jets_3[BLOCK_3_SIZE];
                WolfJet jets_4[BLOCK_4_SIZE];
                WolfJet jets_5[BLOCK_5_SIZE];
                WolfJet jets_6[BLOCK_6_SIZE];
                WolfJet jets_7[BLOCK_7_SIZE];
                WolfJet jets_8[BLOCK_8_SIZE];
                WolfJet jets_9[BLOCK_9_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    jets_2[i] = WolfJet(parameters[2][i], last_jet_idx++);
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    jets_3[i] = WolfJet(parameters[3][i], last_jet_idx++);
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    jets_4[i] = WolfJet(parameters[4][i], last_jet_idx++);
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    jets_5[i] = WolfJet(parameters[5][i], last_jet_idx++);
                for (i = 0; i < BLOCK_6_SIZE; i++)
                    jets_6[i] = WolfJet(parameters[6][i], last_jet_idx++);
                for (i = 0; i < BLOCK_7_SIZE; i++)
                    jets_7[i] = WolfJet(parameters[7][i], last_jet_idx++);
                for (i = 0; i < BLOCK_8_SIZE; i++)
                    jets_8[i] = WolfJet(parameters[8][i], last_jet_idx++);
                for (i = 0; i < BLOCK_9_SIZE; i++)
                    jets_9[i] = WolfJet(parameters[9][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, jets_2, jets_3, jets_4, jets_5, jets_6, jets_7, jets_8, jets_9, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<this->n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 9 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE, unsigned int BLOCK_6_SIZE, unsigned int BLOCK_7_SIZE, unsigned int BLOCK_8_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  BLOCK_5_SIZE, BLOCK_6_SIZE, BLOCK_7_SIZE, BLOCK_8_SIZE, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE + BLOCK_6_SIZE + BLOCK_7_SIZE + BLOCK_8_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                                     BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(9),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                          BLOCK_5_SIZE, BLOCK_6_SIZE, BLOCK_7_SIZE, BLOCK_8_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE+BLOCK_7_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4],
                                         parameters[5], parameters[6], parameters[7], parameters[8], residuals);

            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet jets_2[BLOCK_2_SIZE];
                WolfJet jets_3[BLOCK_3_SIZE];
                WolfJet jets_4[BLOCK_4_SIZE];
                WolfJet jets_5[BLOCK_5_SIZE];
                WolfJet jets_6[BLOCK_6_SIZE];
                WolfJet jets_7[BLOCK_7_SIZE];
                WolfJet jets_8[BLOCK_8_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    jets_2[i] = WolfJet(parameters[2][i], last_jet_idx++);
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    jets_3[i] = WolfJet(parameters[3][i], last_jet_idx++);
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    jets_4[i] = WolfJet(parameters[4][i], last_jet_idx++);
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    jets_5[i] = WolfJet(parameters[5][i], last_jet_idx++);
                for (i = 0; i < BLOCK_6_SIZE; i++)
                    jets_6[i] = WolfJet(parameters[6][i], last_jet_idx++);
                for (i = 0; i < BLOCK_7_SIZE; i++)
                    jets_7[i] = WolfJet(parameters[7][i], last_jet_idx++);
                for (i = 0; i < BLOCK_8_SIZE; i++)
                    jets_8[i] = WolfJet(parameters[8][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, jets_2, jets_3, jets_4, jets_5, jets_6, jets_7, jets_8, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 8 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE, unsigned int BLOCK_6_SIZE, unsigned int BLOCK_7_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  BLOCK_5_SIZE, BLOCK_6_SIZE, BLOCK_7_SIZE, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,0,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE + BLOCK_6_SIZE + BLOCK_7_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                                     BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(8),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                          BLOCK_5_SIZE, BLOCK_6_SIZE, BLOCK_7_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4],
                                         parameters[5], parameters[6], parameters[7], residuals);

            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet jets_2[BLOCK_2_SIZE];
                WolfJet jets_3[BLOCK_3_SIZE];
                WolfJet jets_4[BLOCK_4_SIZE];
                WolfJet jets_5[BLOCK_5_SIZE];
                WolfJet jets_6[BLOCK_6_SIZE];
                WolfJet jets_7[BLOCK_7_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    jets_2[i] = WolfJet(parameters[2][i], last_jet_idx++);
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    jets_3[i] = WolfJet(parameters[3][i], last_jet_idx++);
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    jets_4[i] = WolfJet(parameters[4][i], last_jet_idx++);
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    jets_5[i] = WolfJet(parameters[5][i], last_jet_idx++);
                for (i = 0; i < BLOCK_6_SIZE; i++)
                    jets_6[i] = WolfJet(parameters[6][i], last_jet_idx++);
                for (i = 0; i < BLOCK_7_SIZE; i++)
                    jets_7[i] = WolfJet(parameters[7][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, jets_2, jets_3, jets_4, jets_5, jets_6, jets_7, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 7 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE, unsigned int BLOCK_6_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  BLOCK_5_SIZE, BLOCK_6_SIZE, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               BLOCK_5_SIZE,BLOCK_6_SIZE,0,0,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE + BLOCK_6_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                                     BLOCK_5_SIZE,BLOCK_6_SIZE,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(7),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                          BLOCK_5_SIZE, BLOCK_6_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4],
                                         parameters[5], parameters[6], residuals);

            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet jets_2[BLOCK_2_SIZE];
                WolfJet jets_3[BLOCK_3_SIZE];
                WolfJet jets_4[BLOCK_4_SIZE];
                WolfJet jets_5[BLOCK_5_SIZE];
                WolfJet jets_6[BLOCK_6_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    jets_2[i] = WolfJet(parameters[2][i], last_jet_idx++);
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    jets_3[i] = WolfJet(parameters[3][i], last_jet_idx++);
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    jets_4[i] = WolfJet(parameters[4][i], last_jet_idx++);
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    jets_5[i] = WolfJet(parameters[5][i], last_jet_idx++);
                for (i = 0; i < BLOCK_6_SIZE; i++)
                    jets_6[i] = WolfJet(parameters[6][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, jets_2, jets_3, jets_4, jets_5, jets_6, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 6 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  BLOCK_5_SIZE, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               BLOCK_5_SIZE,0,0,0,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                                     BLOCK_5_SIZE,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(6),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                          BLOCK_5_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4],
                                         parameters[5], residuals);

            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet jets_2[BLOCK_2_SIZE];
                WolfJet jets_3[BLOCK_3_SIZE];
                WolfJet jets_4[BLOCK_4_SIZE];
                WolfJet jets_5[BLOCK_5_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    jets_2[i] = WolfJet(parameters[2][i], last_jet_idx++);
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    jets_3[i] = WolfJet(parameters[3][i], last_jet_idx++);
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    jets_4[i] = WolfJet(parameters[4][i], last_jet_idx++);
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    jets_5[i] = WolfJet(parameters[5][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, jets_2, jets_3, jets_4, jets_5, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 5 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               0,0,0,0,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(5),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4],
                                         residuals);

            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet jets_2[BLOCK_2_SIZE];
                WolfJet jets_3[BLOCK_3_SIZE];
                WolfJet jets_4[BLOCK_4_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    jets_2[i] = WolfJet(parameters[2][i], last_jet_idx++);
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    jets_3[i] = WolfJet(parameters[3][i], last_jet_idx++);
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    jets_4[i] = WolfJet(parameters[4][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, jets_2, jets_3, jets_4, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 4 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, 0,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,0,
                               0,0,0,0,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,0,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(4),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], parameters[3], residuals);
            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet jets_2[BLOCK_2_SIZE];
                WolfJet jets_3[BLOCK_3_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    jets_2[i] = WolfJet(parameters[2][i], last_jet_idx++);
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    jets_3[i] = WolfJet(parameters[3][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, jets_2, jets_3, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 3 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, 0, 0,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,0,0,
                               0,0,0,0,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,0,0,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(3),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], residuals);

            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet jets_2[BLOCK_2_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    jets_2[i] = WolfJet(parameters[2][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, jets_2, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 2 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, 0, 0, 0,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,0,0,0,
                               0,0,0,0,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE + BLOCK_1_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,0,0,0,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(2),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], residuals);

            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet jets_1[BLOCK_1_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    jets_1[i] = WolfJet(parameters[1][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, jets_1, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 1 BLOCK
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE>
class AutoDiffCostFunctionWrapper<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,0,0,0,0,
                               0,0,0,0,0>
{
    typedef ceres::Jet<WolfScalar, BLOCK_0_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;

    public:

        AutoDiffCostFunctionWrapper(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,0,0,0,0,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(1),
            block_sizes_({BLOCK_0_SIZE}),
            jacobian_locations_({0})
        {

        };

        virtual ~AutoDiffCostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], residuals);
            // also compute jacobians
            else
            {
                // create jets
                WolfJet jets_0[BLOCK_0_SIZE];
                WolfJet residuals_jets[MEASUREMENT_SIZE];

                // initialize jets
                unsigned int i, last_jet_idx = 0;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    jets_0[i] = WolfJet(parameters[0][i], last_jet_idx++);

                // call functor
                (*this->constraint_ptr_)(jets_0, residuals_jets);

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = residuals_jets[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy(residuals_jets[row].v.data() + jacobian_locations_.at(i),
                                      residuals_jets[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};


#endif /* TRUNK_SRC_SOLVER_AUTODIFF_COST_FUNCTION_WRAPPER_H_ */
