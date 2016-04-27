#ifndef TRUNK_SRC_AUTODIFF_COST_FUNCTION_WRAPPER_BASE_H_
#define TRUNK_SRC_AUTODIFF_COST_FUNCTION_WRAPPER_BASE_H_

// WOLF
#include "../wolf.h"

// CERES
#include "ceres/jet.h"
#include "ceres/sized_cost_function.h"

// GENERAL
#include <array>

namespace wolf {

template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE, unsigned int BLOCK_6_SIZE, unsigned int BLOCK_7_SIZE, unsigned int BLOCK_8_SIZE, unsigned int BLOCK_9_SIZE>
class AutoDiffCostFunctionWrapperBase : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                                             BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                                                             BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE + BLOCK_6_SIZE + BLOCK_7_SIZE + BLOCK_8_SIZE + BLOCK_9_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, BLOCK_2_SIZE>* jets_2_;
        std::array<WolfJet, BLOCK_3_SIZE>* jets_3_;
        std::array<WolfJet, BLOCK_4_SIZE>* jets_4_;
        std::array<WolfJet, BLOCK_5_SIZE>* jets_5_;
        std::array<WolfJet, BLOCK_6_SIZE>* jets_6_;
        std::array<WolfJet, BLOCK_7_SIZE>* jets_7_;
        std::array<WolfJet, BLOCK_8_SIZE>* jets_8_;
        std::array<WolfJet, BLOCK_9_SIZE>* jets_9_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
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
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE+BLOCK_7_SIZE+BLOCK_8_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            jets_2_(new std::array<WolfJet, BLOCK_2_SIZE>),
            jets_3_(new std::array<WolfJet, BLOCK_3_SIZE>),
            jets_4_(new std::array<WolfJet, BLOCK_4_SIZE>),
            jets_5_(new std::array<WolfJet, BLOCK_5_SIZE>),
            jets_6_(new std::array<WolfJet, BLOCK_6_SIZE>),
            jets_7_(new std::array<WolfJet, BLOCK_7_SIZE>),
            jets_8_(new std::array<WolfJet, BLOCK_8_SIZE>),
            jets_9_(new std::array<WolfJet, BLOCK_9_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_2_SIZE; i++)
                (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_3_SIZE; i++)
                (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_4_SIZE; i++)
                (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_5_SIZE; i++)
                (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_6_SIZE; i++)
                (*jets_6_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_7_SIZE; i++)
                (*jets_7_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_8_SIZE; i++)
                (*jets_8_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_9_SIZE; i++)
                (*jets_9_)[i] = WolfJet(0, last_jet_idx++);

        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete jets_2_;
            delete jets_3_;
            delete jets_4_;
            delete jets_5_;
            delete jets_6_;
            delete jets_7_;
            delete jets_8_;
            delete jets_9_;
            delete residuals_jets_;
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
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    (*jets_2_)[i].a = parameters[2][i];
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    (*jets_3_)[i].a = parameters[3][i];
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    (*jets_4_)[i].a = parameters[4][i];
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    (*jets_5_)[i].a = parameters[5][i];
                for (i = 0; i < BLOCK_6_SIZE; i++)
                    (*jets_6_)[i].a = parameters[6][i];
                for (i = 0; i < BLOCK_7_SIZE; i++)
                    (*jets_7_)[i].a = parameters[7][i];
                for (i = 0; i < BLOCK_8_SIZE; i++)
                    (*jets_8_)[i].a = parameters[8][i];
                for (i = 0; i < BLOCK_9_SIZE; i++)
                    (*jets_9_)[i].a = parameters[9][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), jets_2_->data(), jets_3_->data(), jets_4_->data(),
                                         jets_5_->data(), jets_6_->data(), jets_7_->data(), jets_8_->data(), jets_9_->data(),
                                         residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<this->n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 9 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE, unsigned int BLOCK_6_SIZE, unsigned int BLOCK_7_SIZE, unsigned int BLOCK_8_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  BLOCK_5_SIZE, BLOCK_6_SIZE, BLOCK_7_SIZE, BLOCK_8_SIZE, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE + BLOCK_6_SIZE + BLOCK_7_SIZE + BLOCK_8_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, BLOCK_2_SIZE>* jets_2_;
        std::array<WolfJet, BLOCK_3_SIZE>* jets_3_;
        std::array<WolfJet, BLOCK_4_SIZE>* jets_4_;
        std::array<WolfJet, BLOCK_5_SIZE>* jets_5_;
        std::array<WolfJet, BLOCK_6_SIZE>* jets_6_;
        std::array<WolfJet, BLOCK_7_SIZE>* jets_7_;
        std::array<WolfJet, BLOCK_8_SIZE>* jets_8_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
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
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE+BLOCK_7_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            jets_2_(new std::array<WolfJet, BLOCK_2_SIZE>),
            jets_3_(new std::array<WolfJet, BLOCK_3_SIZE>),
            jets_4_(new std::array<WolfJet, BLOCK_4_SIZE>),
            jets_5_(new std::array<WolfJet, BLOCK_5_SIZE>),
            jets_6_(new std::array<WolfJet, BLOCK_6_SIZE>),
            jets_7_(new std::array<WolfJet, BLOCK_7_SIZE>),
            jets_8_(new std::array<WolfJet, BLOCK_8_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_2_SIZE; i++)
                (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_3_SIZE; i++)
                (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_4_SIZE; i++)
                (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_5_SIZE; i++)
                (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_6_SIZE; i++)
                (*jets_6_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_7_SIZE; i++)
                (*jets_7_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_8_SIZE; i++)
                (*jets_8_)[i] = WolfJet(0, last_jet_idx++);
        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete jets_2_;
            delete jets_3_;
            delete jets_4_;
            delete jets_5_;
            delete jets_6_;
            delete jets_7_;
            delete jets_8_;
            delete residuals_jets_;
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
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    (*jets_2_)[i].a = parameters[2][i];
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    (*jets_3_)[i].a = parameters[3][i];
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    (*jets_4_)[i].a = parameters[4][i];
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    (*jets_5_)[i].a = parameters[5][i];
                for (i = 0; i < BLOCK_6_SIZE; i++)
                    (*jets_6_)[i].a = parameters[6][i];
                for (i = 0; i < BLOCK_7_SIZE; i++)
                    (*jets_7_)[i].a = parameters[7][i];
                for (i = 0; i < BLOCK_8_SIZE; i++)
                    (*jets_8_)[i].a = parameters[8][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), jets_2_->data(), jets_3_->data(), jets_4_->data(),
                                         jets_5_->data(), jets_6_->data(), jets_7_->data(), jets_8_->data(),
                                         residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<this->n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 8 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE, unsigned int BLOCK_6_SIZE, unsigned int BLOCK_7_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  BLOCK_5_SIZE, BLOCK_6_SIZE, BLOCK_7_SIZE, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,0,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE + BLOCK_6_SIZE + BLOCK_7_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, BLOCK_2_SIZE>* jets_2_;
        std::array<WolfJet, BLOCK_3_SIZE>* jets_3_;
        std::array<WolfJet, BLOCK_4_SIZE>* jets_4_;
        std::array<WolfJet, BLOCK_5_SIZE>* jets_5_;
        std::array<WolfJet, BLOCK_6_SIZE>* jets_6_;
        std::array<WolfJet, BLOCK_7_SIZE>* jets_7_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
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
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE+BLOCK_6_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            jets_2_(new std::array<WolfJet, BLOCK_2_SIZE>),
            jets_3_(new std::array<WolfJet, BLOCK_3_SIZE>),
            jets_4_(new std::array<WolfJet, BLOCK_4_SIZE>),
            jets_5_(new std::array<WolfJet, BLOCK_5_SIZE>),
            jets_6_(new std::array<WolfJet, BLOCK_6_SIZE>),
            jets_7_(new std::array<WolfJet, BLOCK_7_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_2_SIZE; i++)
                (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_3_SIZE; i++)
                (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_4_SIZE; i++)
                (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_5_SIZE; i++)
                (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_6_SIZE; i++)
                (*jets_6_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_7_SIZE; i++)
                (*jets_7_)[i] = WolfJet(0, last_jet_idx++);
        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete jets_2_;
            delete jets_3_;
            delete jets_4_;
            delete jets_5_;
            delete jets_6_;
            delete jets_7_;
            delete residuals_jets_;
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
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    (*jets_2_)[i].a = parameters[2][i];
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    (*jets_3_)[i].a = parameters[3][i];
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    (*jets_4_)[i].a = parameters[4][i];
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    (*jets_5_)[i].a = parameters[5][i];
                for (i = 0; i < BLOCK_6_SIZE; i++)
                    (*jets_6_)[i].a = parameters[6][i];
                for (i = 0; i < BLOCK_7_SIZE; i++)
                    (*jets_7_)[i].a = parameters[7][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), jets_2_->data(), jets_3_->data(), jets_4_->data(),
                                         jets_5_->data(), jets_6_->data(), jets_7_->data(),
                                         residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 7 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE, unsigned int BLOCK_6_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  BLOCK_5_SIZE, BLOCK_6_SIZE, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               BLOCK_5_SIZE,BLOCK_6_SIZE,0,0,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE + BLOCK_6_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, BLOCK_2_SIZE>* jets_2_;
        std::array<WolfJet, BLOCK_3_SIZE>* jets_3_;
        std::array<WolfJet, BLOCK_4_SIZE>* jets_4_;
        std::array<WolfJet, BLOCK_5_SIZE>* jets_5_;
        std::array<WolfJet, BLOCK_6_SIZE>* jets_6_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
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
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE+BLOCK_5_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            jets_2_(new std::array<WolfJet, BLOCK_2_SIZE>),
            jets_3_(new std::array<WolfJet, BLOCK_3_SIZE>),
            jets_4_(new std::array<WolfJet, BLOCK_4_SIZE>),
            jets_5_(new std::array<WolfJet, BLOCK_5_SIZE>),
            jets_6_(new std::array<WolfJet, BLOCK_6_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_2_SIZE; i++)
                (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_3_SIZE; i++)
                (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_4_SIZE; i++)
                (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_5_SIZE; i++)
                (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_6_SIZE; i++)
                (*jets_6_)[i] = WolfJet(0, last_jet_idx++);
        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete jets_2_;
            delete jets_3_;
            delete jets_4_;
            delete jets_5_;
            delete jets_6_;
            delete residuals_jets_;
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
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    (*jets_2_)[i].a = parameters[2][i];
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    (*jets_3_)[i].a = parameters[3][i];
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    (*jets_4_)[i].a = parameters[4][i];
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    (*jets_5_)[i].a = parameters[5][i];
                for (i = 0; i < BLOCK_6_SIZE; i++)
                    (*jets_6_)[i].a = parameters[6][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), jets_2_->data(), jets_3_->data(), jets_4_->data(),
                                         jets_5_->data(), jets_6_->data(),
                                         residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 6 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE,
          unsigned int BLOCK_5_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  BLOCK_5_SIZE, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               BLOCK_5_SIZE,0,0,0,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE +
                                   BLOCK_5_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, BLOCK_2_SIZE>* jets_2_;
        std::array<WolfJet, BLOCK_3_SIZE>* jets_3_;
        std::array<WolfJet, BLOCK_4_SIZE>* jets_4_;
        std::array<WolfJet, BLOCK_5_SIZE>* jets_5_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
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
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE+BLOCK_4_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            jets_2_(new std::array<WolfJet, BLOCK_2_SIZE>),
            jets_3_(new std::array<WolfJet, BLOCK_3_SIZE>),
            jets_4_(new std::array<WolfJet, BLOCK_4_SIZE>),
            jets_5_(new std::array<WolfJet, BLOCK_5_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_2_SIZE; i++)
                (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_3_SIZE; i++)
                (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_4_SIZE; i++)
                (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_5_SIZE; i++)
                (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete jets_2_;
            delete jets_3_;
            delete jets_4_;
            delete jets_5_;
            delete residuals_jets_;
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
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    (*jets_2_)[i].a = parameters[2][i];
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    (*jets_3_)[i].a = parameters[3][i];
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    (*jets_4_)[i].a = parameters[4][i];
                for (i = 0; i < BLOCK_5_SIZE; i++)
                    (*jets_5_)[i].a = parameters[5][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), jets_2_->data(), jets_3_->data(), jets_4_->data(),
                                         jets_5_->data(), residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 5 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE, unsigned int BLOCK_4_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, BLOCK_4_SIZE,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,
                               0,0,0,0,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE + BLOCK_4_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, BLOCK_2_SIZE>* jets_2_;
        std::array<WolfJet, BLOCK_3_SIZE>* jets_3_;
        std::array<WolfJet, BLOCK_4_SIZE>* jets_4_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
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
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE+BLOCK_3_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            jets_2_(new std::array<WolfJet, BLOCK_2_SIZE>),
            jets_3_(new std::array<WolfJet, BLOCK_3_SIZE>),
            jets_4_(new std::array<WolfJet, BLOCK_4_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_2_SIZE; i++)
                (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_3_SIZE; i++)
                (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_4_SIZE; i++)
                (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete jets_2_;
            delete jets_3_;
            delete jets_4_;
            delete residuals_jets_;
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
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    (*jets_2_)[i].a = parameters[2][i];
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    (*jets_3_)[i].a = parameters[3][i];
                for (i = 0; i < BLOCK_4_SIZE; i++)
                    (*jets_4_)[i].a = parameters[4][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), jets_2_->data(), jets_3_->data(), jets_4_->data(),
                                         residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 4 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE, unsigned int BLOCK_3_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE, 0,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,0,
                               0,0,0,0,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE + BLOCK_3_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, BLOCK_2_SIZE>* jets_2_;
        std::array<WolfJet, BLOCK_3_SIZE>* jets_3_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,0,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(4),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, BLOCK_3_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE+BLOCK_2_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            jets_2_(new std::array<WolfJet, BLOCK_2_SIZE>),
            jets_3_(new std::array<WolfJet, BLOCK_3_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_2_SIZE; i++)
                (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_3_SIZE; i++)
                (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete jets_2_;
            delete jets_3_;
            delete residuals_jets_;
        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], parameters[3], residuals);
            // also compute jacobians
            else
            {
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    (*jets_2_)[i].a = parameters[2][i];
                for (i = 0; i < BLOCK_3_SIZE; i++)
                    (*jets_3_)[i].a = parameters[3][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), jets_2_->data(), jets_3_->data(), residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 3 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE, unsigned int BLOCK_2_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE, 0, 0,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,0,0,
                               0,0,0,0,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE + BLOCK_2_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, BLOCK_2_SIZE>* jets_2_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,0,0,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(3),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE,
                                 BLOCK_0_SIZE+BLOCK_1_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            jets_2_(new std::array<WolfJet, BLOCK_2_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_2_SIZE; i++)
                (*jets_2_)[i] = WolfJet(0, last_jet_idx++);

        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete jets_2_;
            delete residuals_jets_;
        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], parameters[2], residuals);

            // also compute jacobians
            else
            {
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];
                for (i = 0; i < BLOCK_2_SIZE; i++)
                    (*jets_2_)[i].a = parameters[2][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), jets_2_->data(), residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 2 BLOCKS
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE, unsigned int BLOCK_1_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, BLOCK_1_SIZE, 0, 0, 0,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,BLOCK_1_SIZE,0,0,0,
                               0,0,0,0,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE + BLOCK_1_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, BLOCK_1_SIZE>* jets_1_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,BLOCK_1_SIZE,0,0,0,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(2),
            block_sizes_({BLOCK_0_SIZE, BLOCK_1_SIZE}),
            jacobian_locations_({0,
                                 BLOCK_0_SIZE}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            jets_1_(new std::array<WolfJet, BLOCK_1_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (i = 0; i < BLOCK_1_SIZE; i++)
                (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete jets_1_;
            delete residuals_jets_;
        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], parameters[1], residuals);

            // also compute jacobians
            else
            {
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];
                for (i = 0; i < BLOCK_1_SIZE; i++)
                    (*jets_1_)[i].a = parameters[1][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), jets_1_->data(), residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

// SPECIALIZATION 1 BLOCK
template <class ConstraintType, const unsigned int MEASUREMENT_SIZE,
          unsigned int BLOCK_0_SIZE>
class AutoDiffCostFunctionWrapperBase<ConstraintType, MEASUREMENT_SIZE,
                                  BLOCK_0_SIZE, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0>
    : public ceres::SizedCostFunction<MEASUREMENT_SIZE,
                               BLOCK_0_SIZE,0,0,0,0,
                               0,0,0,0,0>
{
    typedef ceres::Jet<Scalar, BLOCK_0_SIZE> WolfJet;

    protected:
        ConstraintType* constraint_ptr_;
        unsigned int n_blocks_;
        std::vector<unsigned int> block_sizes_, jacobian_locations_;
        std::array<WolfJet, BLOCK_0_SIZE>* jets_0_;
        std::array<WolfJet, MEASUREMENT_SIZE>* residuals_jets_;

    public:

        AutoDiffCostFunctionWrapperBase(ConstraintType* _constraint_ptr) :
            ceres::SizedCostFunction<MEASUREMENT_SIZE,
                                     BLOCK_0_SIZE,0,0,0,0,
                                     0,0,0,0,0>(),
            constraint_ptr_(_constraint_ptr),
            n_blocks_(1),
            block_sizes_({BLOCK_0_SIZE}),
            jacobian_locations_({0}),
            jets_0_(new std::array<WolfJet, BLOCK_0_SIZE>),
            residuals_jets_(new std::array<WolfJet, MEASUREMENT_SIZE>)
        {
            // initialize jets
            unsigned int i, last_jet_idx = 0;
            for (i = 0; i < BLOCK_0_SIZE; i++)
                (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
        };

        virtual ~AutoDiffCostFunctionWrapperBase()
        {
            delete jets_0_;
            delete residuals_jets_;
        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
                (*this->constraint_ptr_)(parameters[0], residuals);
            // also compute jacobians
            else
            {
                // update jets real part
                unsigned int i;
                for (i = 0; i < BLOCK_0_SIZE; i++)
                    (*jets_0_)[i].a = parameters[0][i];

                // call functor
                (*this->constraint_ptr_)(jets_0_->data(), residuals_jets_->data());

                // fill the residual array
                for (i = 0; i < MEASUREMENT_SIZE; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (i = 0; i<n_blocks_; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < MEASUREMENT_SIZE; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + block_sizes_.at(i),
                                      jacobians[i] + row * block_sizes_.at(i));
            }
            return true;
        }
};

} // namespace wolf

#endif /* TRUNK_SRC_AUTODIFF_COST_FUNCTION_WRAPPER_BASE_H_ */
