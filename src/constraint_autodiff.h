
#ifndef CONSTRAINT_AUTODIFF_H_
#define CONSTRAINT_AUTODIFF_H_

//Wolf includes
#include "constraint_base.h"
#include "state_block.h"

// CERES
#include "ceres/jet.h"

// GENERAL
#include <array>

namespace wolf {

//template class ConstraintAutodiff
template <class CtrT, unsigned int RES, unsigned int B0, unsigned int B1 = 0, unsigned int B2 = 0, unsigned int B3 = 0, unsigned int B4 = 0, unsigned int B5 = 0, unsigned int B6 = 0, unsigned int B7 = 0, unsigned int B8 = 0, unsigned int B9 = 0>
class ConstraintAutodiff: public ConstraintBase
{
    public:

        static const unsigned int residualSize = RES;
        static const unsigned int block0Size = B0;
        static const unsigned int block1Size = B1;
        static const unsigned int block2Size = B2;
        static const unsigned int block3Size = B3;
        static const unsigned int block4Size = B4;
        static const unsigned int block5Size = B5;
        static const unsigned int block6Size = B6;
        static const unsigned int block7Size = B7;
        static const unsigned int block8Size = B8;
        static const unsigned int block9Size = B9;
        static const unsigned int n_blocks = 10;

        static const std::vector<unsigned int> state_block_sizes_;

        typedef ceres::Jet<Scalar, B0 + B1 + B2 + B3 + B4 +
                                   B5 + B6 + B7 + B8 + B9> WolfJet;

    protected:

        std::vector<StateBlockPtr> state_ptrs_;

        static const std::vector<unsigned int> jacobian_locations_;
        std::array<WolfJet, RES>* residuals_jets_;
        std::array<WolfJet, B0>* jets_0_;
        std::array<WolfJet, B1>* jets_1_;
        std::array<WolfJet, B2>* jets_2_;
        std::array<WolfJet, B3>* jets_3_;
        std::array<WolfJet, B4>* jets_4_;
        std::array<WolfJet, B5>* jets_5_;
        std::array<WolfJet, B6>* jets_6_;
        std::array<WolfJet, B7>* jets_7_;
        std::array<WolfJet, B8>* jets_8_;
        std::array<WolfJet, B9>* jets_9_;

    public:
        /** \brief Constructor valid for all categories (ABSOLUTE, FRAME, FEATURE, LANDMARK)
         **/
        ConstraintAutodiff(ConstraintType _tp,
                           const FrameBasePtr& _frame_other_ptr,
                           const CaptureBasePtr& _capture_other_ptr,
                           const FeatureBasePtr& _feature_other_ptr,
                           const LandmarkBasePtr& _landmark_other_ptr,
                           const ProcessorBasePtr& _processor_ptr,
                           bool _apply_loss_function,
                           ConstraintStatus _status,
                           StateBlockPtr _state0Ptr,
                           StateBlockPtr _state1Ptr,
                           StateBlockPtr _state2Ptr,
                           StateBlockPtr _state3Ptr,
                           StateBlockPtr _state4Ptr,
                           StateBlockPtr _state5Ptr,
                           StateBlockPtr _state6Ptr,
                           StateBlockPtr _state7Ptr,
                           StateBlockPtr _state8Ptr,
                           StateBlockPtr _state9Ptr) :
            ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
            state_ptrs_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            residuals_jets_(new std::array<WolfJet, RES>),
            jets_0_(new std::array<WolfJet, B0>),
            jets_1_(new std::array<WolfJet, B1>),
            jets_2_(new std::array<WolfJet, B2>),
            jets_3_(new std::array<WolfJet, B3>),
            jets_4_(new std::array<WolfJet, B4>),
            jets_5_(new std::array<WolfJet, B5>),
            jets_6_(new std::array<WolfJet, B6>),
            jets_7_(new std::array<WolfJet, B7>),
            jets_8_(new std::array<WolfJet, B8>),
            jets_9_(new std::array<WolfJet, B9>)
        {
            // initialize jets
            unsigned int last_jet_idx = 0;
            for (auto i = 0; i < B0; i++)
               (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B1; i++)
               (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B2; i++)
               (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B3; i++)
               (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B4; i++)
               (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B5; i++)
               (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B6; i++)
               (*jets_6_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B7; i++)
               (*jets_7_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B8; i++)
               (*jets_8_)[i] = WolfJet(0, last_jet_idx++);
            for (auto i = 0; i < B9; i++)
                (*jets_9_)[i] = WolfJet(0, last_jet_idx++);
        }

        virtual ~ConstraintAutodiff()
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
        }

        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

        /** \brief Returns the residual and jacobians given the state values
         *
         * Returns the residual and jacobians given the state values
         *
         **/
        virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // only residuals
            if (jacobians == nullptr)
            {
                (*static_cast<CtrT const*>(this))(parameters[0],
                                                  parameters[1],
                                                  parameters[2],
                                                  parameters[3],
                                                  parameters[4],
                                                  parameters[5],
                                                  parameters[6],
                                                  parameters[7],
                                                  parameters[8],
                                                  parameters[9],
                                                  residuals);
            }
            // also compute jacobians
            else
            {
                // update jets real part
                std::vector<double const*> param_vec;
                param_vec.assign(parameters,parameters+n_blocks);
                updateJetsRealPart(param_vec);

                // call functor
                (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                  jets_1_->data(),
                                                  jets_2_->data(),
                                                  jets_3_->data(),
                                                  jets_4_->data(),
                                                  jets_5_->data(),
                                                  jets_6_->data(),
                                                  jets_7_->data(),
                                                  jets_8_->data(),
                                                  jets_9_->data(),
                                                  residuals_jets_->data());

                // fill the residual array
                for (auto i = 0; i < RES; i++)
                    residuals[i] = (*residuals_jets_)[i].a;

                // fill the jacobian matrices
                for (auto i = 0; i<n_blocks; i++)
                    if (jacobians[i] != nullptr)
                        for (unsigned int row = 0; row < RES; row++)
                            std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                      (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                      jacobians[i] + row * state_block_sizes_.at(i));
            }
            return true;
        }

        /** \brief Updates all jets real part with values of parameters
         *
         **/
        void updateJetsRealPart(const std::vector<double const*>& parameters) const
        {
            // update jets real part
            for (auto i = 0; i < B0; i++)
                (*jets_0_)[i].a = parameters[0][i];
            for (auto i = 0; i < B1; i++)
                (*jets_1_)[i].a = parameters[1][i];
            for (auto i = 0; i < B2; i++)
                (*jets_2_)[i].a = parameters[2][i];
            for (auto i = 0; i < B3; i++)
                (*jets_3_)[i].a = parameters[3][i];
            for (auto i = 0; i < B4; i++)
                (*jets_4_)[i].a = parameters[4][i];
            for (auto i = 0; i < B5; i++)
                (*jets_5_)[i].a = parameters[5][i];
            for (auto i = 0; i < B6; i++)
                (*jets_6_)[i].a = parameters[6][i];
            for (auto i = 0; i < B7; i++)
                (*jets_7_)[i].a = parameters[7][i];
            for (auto i = 0; i < B8; i++)
                (*jets_8_)[i].a = parameters[8][i];
            for (auto i = 0; i < B9; i++)
                (*jets_9_)[i].a = parameters[9][i];
        }

        /** \brief Returns a vector of Jacobian matrix corresponding to each state block evaluated in the point provided in _states_ptr
         *
         **/
        virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
        {
            assert(residual_.size() == RES);
            jacobians_.clear();

            assert(_states_ptr.size() == n_blocks);

            // init jacobian
            for(auto i = 0; i < n_blocks; ++i)
            {
               Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
               jacobians_.push_back(Ji);
            }

            // update jets real part
            updateJetsRealPart(_states_ptr);

            // call functor
            (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                              jets_1_->data(),
                                              jets_2_->data(),
                                              jets_3_->data(),
                                              jets_4_->data(),
                                              jets_5_->data(),
                                              jets_6_->data(),
                                              jets_7_->data(),
                                              jets_8_->data(),
                                              jets_9_->data(),
                                              residuals_jets_->data());

            // fill the residual vector
            for (auto i = 0; i < RES; i++)
                residual_(i) = (*residuals_jets_)[i].a;

            // fill the jacobian matrices
            for (auto i = 0; i<n_blocks; i++)
                if (!state_ptrs_[i]->isFixed())
                    for (unsigned int row = 0; row < RES; row++)
                        std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                  (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                  jacobians_[i].data() + row * state_block_sizes_.at(i));

           // print jacobian matrices
//           for (auto i = 0; i < n_blocks; i++)
//               std::cout << jacobians_[i] << std::endl << std::endl;
        }

        /** \brief Returns a vector of pointers to the state blocks
         *
         * Returns a vector of pointers to the state blocks in which this constraint depends
         *
         **/
        virtual std::vector<Scalar*> getStateScalarPtrVector() const
        {
            return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                         state_ptrs_[1]->getPtr(),
                                         state_ptrs_[2]->getPtr(),
                                         state_ptrs_[3]->getPtr(),
                                         state_ptrs_[4]->getPtr(),
                                         state_ptrs_[5]->getPtr(),
                                         state_ptrs_[6]->getPtr(),
                                         state_ptrs_[7]->getPtr(),
                                         state_ptrs_[8]->getPtr(),
                                         state_ptrs_[9]->getPtr()
                                         });
        }

        /** \brief Returns a vector of pointers to the states
         *
         * Returns a vector of pointers to the state in which this constraint depends
         *
         **/
        virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
        {
            return state_ptrs_;
        }

        /** \brief Returns a vector of the states sizes
         *
         **/
        virtual std::vector<unsigned int> getStateSizes() const
        {
            return state_block_sizes_;
        }

        /** \brief Returns the residual size
         *
         * Returns the residual size
         *
         **/
        virtual unsigned int getSize() const
        {
            return RES;
        }
};


////////////////// SPECIALIZATION 9 BLOCKS ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6,unsigned int B7,unsigned int B8>
class ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,B7,B8,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = B1;
       static const unsigned int block2Size = B2;
       static const unsigned int block3Size = B3;
       static const unsigned int block4Size = B4;
       static const unsigned int block5Size = B5;
       static const unsigned int block6Size = B6;
       static const unsigned int block7Size = B7;
       static const unsigned int block8Size = B8;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 9;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0 + B1 + B2 + B3 + B4 +
                                  B5 + B6 + B7 + B8> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;
       std::array<WolfJet, B1>* jets_1_;
       std::array<WolfJet, B2>* jets_2_;
       std::array<WolfJet, B3>* jets_3_;
       std::array<WolfJet, B4>* jets_4_;
       std::array<WolfJet, B5>* jets_5_;
       std::array<WolfJet, B6>* jets_6_;
       std::array<WolfJet, B7>* jets_7_;
       std::array<WolfJet, B8>* jets_8_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr,
                          StateBlockPtr _state1Ptr,
                          StateBlockPtr _state2Ptr,
                          StateBlockPtr _state3Ptr,
                          StateBlockPtr _state4Ptr,
                          StateBlockPtr _state5Ptr,
                          StateBlockPtr _state6Ptr,
                          StateBlockPtr _state7Ptr,
                          StateBlockPtr _state8Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>),
           jets_1_(new std::array<WolfJet, B1>),
           jets_2_(new std::array<WolfJet, B2>),
           jets_3_(new std::array<WolfJet, B3>),
           jets_4_(new std::array<WolfJet, B4>),
           jets_5_(new std::array<WolfJet, B5>),
           jets_6_(new std::array<WolfJet, B6>),
           jets_7_(new std::array<WolfJet, B7>),
           jets_8_(new std::array<WolfJet, B8>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B1; i++)
              (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B2; i++)
              (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B3; i++)
              (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B4; i++)
              (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B5; i++)
              (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B6; i++)
              (*jets_6_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B7; i++)
              (*jets_7_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B8; i++)
              (*jets_8_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
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
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                        parameters[1],
                                                        parameters[2],
                                                        parameters[3],
                                                        parameters[4],
                                                        parameters[5],
                                                        parameters[6],
                                                        parameters[7],
                                                        parameters[8],
                                                        residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                        jets_1_->data(),
                                                        jets_2_->data(),
                                                        jets_3_->data(),
                                                        jets_4_->data(),
                                                        jets_5_->data(),
                                                        jets_6_->data(),
                                                        jets_7_->data(),
                                                        jets_8_->data(),
                                                        residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
           for (auto i = 0; i < B1; i++)
               (*jets_1_)[i].a = parameters[1][i];
           for (auto i = 0; i < B2; i++)
               (*jets_2_)[i].a = parameters[2][i];
           for (auto i = 0; i < B3; i++)
               (*jets_3_)[i].a = parameters[3][i];
           for (auto i = 0; i < B4; i++)
               (*jets_4_)[i].a = parameters[4][i];
           for (auto i = 0; i < B5; i++)
               (*jets_5_)[i].a = parameters[5][i];
           for (auto i = 0; i < B6; i++)
               (*jets_6_)[i].a = parameters[6][i];
           for (auto i = 0; i < B7; i++)
               (*jets_7_)[i].a = parameters[7][i];
           for (auto i = 0; i < B8; i++)
               (*jets_8_)[i].a = parameters[8][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             jets_1_->data(),
                                             jets_2_->data(),
                                             jets_3_->data(),
                                             jets_4_->data(),
                                             jets_5_->data(),
                                             jets_6_->data(),
                                             jets_7_->data(),
                                             jets_8_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));

          // print jacobian matrices
//           for (auto i = 0; i < n_blocks; i++)
//               std::cout << jacobians_[i] << std::endl << std::endl;
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                        state_ptrs_[1]->getPtr(),
                                        state_ptrs_[2]->getPtr(),
                                        state_ptrs_[3]->getPtr(),
                                        state_ptrs_[4]->getPtr(),
                                        state_ptrs_[5]->getPtr(),
                                        state_ptrs_[6]->getPtr(),
                                        state_ptrs_[7]->getPtr(),
                                        state_ptrs_[8]->getPtr()
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////// SPECIALIZATION 8 BLOCKS ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6,unsigned int B7>
class ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,B7,0,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = B1;
       static const unsigned int block2Size = B2;
       static const unsigned int block3Size = B3;
       static const unsigned int block4Size = B4;
       static const unsigned int block5Size = B5;
       static const unsigned int block6Size = B6;
       static const unsigned int block7Size = B7;
       static const unsigned int block8Size = 0;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 8;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0 + B1 + B2 + B3 + B4 +
                                  B5 + B6 + B7> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;
       std::array<WolfJet, B1>* jets_1_;
       std::array<WolfJet, B2>* jets_2_;
       std::array<WolfJet, B3>* jets_3_;
       std::array<WolfJet, B4>* jets_4_;
       std::array<WolfJet, B5>* jets_5_;
       std::array<WolfJet, B6>* jets_6_;
       std::array<WolfJet, B7>* jets_7_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr,
                          StateBlockPtr _state1Ptr,
                          StateBlockPtr _state2Ptr,
                          StateBlockPtr _state3Ptr,
                          StateBlockPtr _state4Ptr,
                          StateBlockPtr _state5Ptr,
                          StateBlockPtr _state6Ptr,
                          StateBlockPtr _state7Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>),
           jets_1_(new std::array<WolfJet, B1>),
           jets_2_(new std::array<WolfJet, B2>),
           jets_3_(new std::array<WolfJet, B3>),
           jets_4_(new std::array<WolfJet, B4>),
           jets_5_(new std::array<WolfJet, B5>),
           jets_6_(new std::array<WolfJet, B6>),
           jets_7_(new std::array<WolfJet, B7>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B1; i++)
              (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B2; i++)
              (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B3; i++)
              (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B4; i++)
              (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B5; i++)
              (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B6; i++)
              (*jets_6_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B7; i++)
              (*jets_7_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
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
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                        parameters[1],
                                                        parameters[2],
                                                        parameters[3],
                                                        parameters[4],
                                                        parameters[5],
                                                        parameters[6],
                                                        parameters[7],
                                                        residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                        jets_1_->data(),
                                                        jets_2_->data(),
                                                        jets_3_->data(),
                                                        jets_4_->data(),
                                                        jets_5_->data(),
                                                        jets_6_->data(),
                                                        jets_7_->data(),
                                                        residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
           for (auto i = 0; i < B1; i++)
               (*jets_1_)[i].a = parameters[1][i];
           for (auto i = 0; i < B2; i++)
               (*jets_2_)[i].a = parameters[2][i];
           for (auto i = 0; i < B3; i++)
               (*jets_3_)[i].a = parameters[3][i];
           for (auto i = 0; i < B4; i++)
               (*jets_4_)[i].a = parameters[4][i];
           for (auto i = 0; i < B5; i++)
               (*jets_5_)[i].a = parameters[5][i];
           for (auto i = 0; i < B6; i++)
               (*jets_6_)[i].a = parameters[6][i];
           for (auto i = 0; i < B7; i++)
               (*jets_7_)[i].a = parameters[7][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             jets_1_->data(),
                                             jets_2_->data(),
                                             jets_3_->data(),
                                             jets_4_->data(),
                                             jets_5_->data(),
                                             jets_6_->data(),
                                             jets_7_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));

          // print jacobian matrices
//           for (auto i = 0; i < n_blocks; i++)
//               std::cout << jacobians_[i] << std::endl << std::endl;
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                        state_ptrs_[1]->getPtr(),
                                        state_ptrs_[2]->getPtr(),
                                        state_ptrs_[3]->getPtr(),
                                        state_ptrs_[4]->getPtr(),
                                        state_ptrs_[5]->getPtr(),
                                        state_ptrs_[6]->getPtr(),
                                        state_ptrs_[7]->getPtr()
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////// SPECIALIZATION 7 BLOCKS ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6>
class ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,0,0,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = B1;
       static const unsigned int block2Size = B2;
       static const unsigned int block3Size = B3;
       static const unsigned int block4Size = B4;
       static const unsigned int block5Size = B5;
       static const unsigned int block6Size = B6;
       static const unsigned int block7Size = 0;
       static const unsigned int block8Size = 0;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 7;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0 + B1 + B2 + B3 + B4 +
                                  B5 + B6> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;
       std::array<WolfJet, B1>* jets_1_;
       std::array<WolfJet, B2>* jets_2_;
       std::array<WolfJet, B3>* jets_3_;
       std::array<WolfJet, B4>* jets_4_;
       std::array<WolfJet, B5>* jets_5_;
       std::array<WolfJet, B6>* jets_6_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr,
                          StateBlockPtr _state1Ptr,
                          StateBlockPtr _state2Ptr,
                          StateBlockPtr _state3Ptr,
                          StateBlockPtr _state4Ptr,
                          StateBlockPtr _state5Ptr,
                          StateBlockPtr _state6Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>),
           jets_1_(new std::array<WolfJet, B1>),
           jets_2_(new std::array<WolfJet, B2>),
           jets_3_(new std::array<WolfJet, B3>),
           jets_4_(new std::array<WolfJet, B4>),
           jets_5_(new std::array<WolfJet, B5>),
           jets_6_(new std::array<WolfJet, B6>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B1; i++)
              (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B2; i++)
              (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B3; i++)
              (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B4; i++)
              (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B5; i++)
              (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B6; i++)
              (*jets_6_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
       {
           delete jets_0_;
           delete jets_1_;
           delete jets_2_;
           delete jets_3_;
           delete jets_4_;
           delete jets_5_;
           delete jets_6_;
           delete residuals_jets_;
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                        parameters[1],
                                                        parameters[2],
                                                        parameters[3],
                                                        parameters[4],
                                                        parameters[5],
                                                        parameters[6],
                                                        residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                        jets_1_->data(),
                                                        jets_2_->data(),
                                                        jets_3_->data(),
                                                        jets_4_->data(),
                                                        jets_5_->data(),
                                                        jets_6_->data(),
                                                        residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
           for (auto i = 0; i < B1; i++)
               (*jets_1_)[i].a = parameters[1][i];
           for (auto i = 0; i < B2; i++)
               (*jets_2_)[i].a = parameters[2][i];
           for (auto i = 0; i < B3; i++)
               (*jets_3_)[i].a = parameters[3][i];
           for (auto i = 0; i < B4; i++)
               (*jets_4_)[i].a = parameters[4][i];
           for (auto i = 0; i < B5; i++)
               (*jets_5_)[i].a = parameters[5][i];
           for (auto i = 0; i < B6; i++)
               (*jets_6_)[i].a = parameters[6][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             jets_1_->data(),
                                             jets_2_->data(),
                                             jets_3_->data(),
                                             jets_4_->data(),
                                             jets_5_->data(),
                                             jets_6_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));

          // print jacobian matrices
//           for (auto i = 0; i < n_blocks; i++)
//               std::cout << jacobians_[i] << std::endl << std::endl;
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                        state_ptrs_[1]->getPtr(),
                                        state_ptrs_[2]->getPtr(),
                                        state_ptrs_[3]->getPtr(),
                                        state_ptrs_[4]->getPtr(),
                                        state_ptrs_[5]->getPtr(),
                                        state_ptrs_[6]->getPtr()
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////// SPECIALIZATION 6 BLOCKS ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5>
class ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,0,0,0,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = B1;
       static const unsigned int block2Size = B2;
       static const unsigned int block3Size = B3;
       static const unsigned int block4Size = B4;
       static const unsigned int block5Size = B5;
       static const unsigned int block6Size = 0;
       static const unsigned int block7Size = 0;
       static const unsigned int block8Size = 0;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 6;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0 + B1 + B2 + B3 + B4 +
                                  B5> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;
       std::array<WolfJet, B1>* jets_1_;
       std::array<WolfJet, B2>* jets_2_;
       std::array<WolfJet, B3>* jets_3_;
       std::array<WolfJet, B4>* jets_4_;
       std::array<WolfJet, B5>* jets_5_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr,
                          StateBlockPtr _state1Ptr,
                          StateBlockPtr _state2Ptr,
                          StateBlockPtr _state3Ptr,
                          StateBlockPtr _state4Ptr,
                          StateBlockPtr _state5Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>),
           jets_1_(new std::array<WolfJet, B1>),
           jets_2_(new std::array<WolfJet, B2>),
           jets_3_(new std::array<WolfJet, B3>),
           jets_4_(new std::array<WolfJet, B4>),
           jets_5_(new std::array<WolfJet, B5>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B1; i++)
              (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B2; i++)
              (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B3; i++)
              (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B4; i++)
              (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B5; i++)
              (*jets_5_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
       {
           delete jets_0_;
           delete jets_1_;
           delete jets_2_;
           delete jets_3_;
           delete jets_4_;
           delete jets_5_;
           delete residuals_jets_;
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                        parameters[1],
                                                        parameters[2],
                                                        parameters[3],
                                                        parameters[4],
                                                        parameters[5],
                                                        residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                        jets_1_->data(),
                                                        jets_2_->data(),
                                                        jets_3_->data(),
                                                        jets_4_->data(),
                                                        jets_5_->data(),
                                                        residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
           for (auto i = 0; i < B1; i++)
               (*jets_1_)[i].a = parameters[1][i];
           for (auto i = 0; i < B2; i++)
               (*jets_2_)[i].a = parameters[2][i];
           for (auto i = 0; i < B3; i++)
               (*jets_3_)[i].a = parameters[3][i];
           for (auto i = 0; i < B4; i++)
               (*jets_4_)[i].a = parameters[4][i];
           for (auto i = 0; i < B5; i++)
               (*jets_5_)[i].a = parameters[5][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             jets_1_->data(),
                                             jets_2_->data(),
                                             jets_3_->data(),
                                             jets_4_->data(),
                                             jets_5_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                        state_ptrs_[1]->getPtr(),
                                        state_ptrs_[2]->getPtr(),
                                        state_ptrs_[3]->getPtr(),
                                        state_ptrs_[4]->getPtr(),
                                        state_ptrs_[5]->getPtr()
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////// SPECIALIZATION 5 BLOCKS ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4>
class ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,0,0,0,0,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = B1;
       static const unsigned int block2Size = B2;
       static const unsigned int block3Size = B3;
       static const unsigned int block4Size = B4;
       static const unsigned int block5Size = 0;
       static const unsigned int block6Size = 0;
       static const unsigned int block7Size = 0;
       static const unsigned int block8Size = 0;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 5;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0 + B1 + B2 + B3 + B4> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;
       std::array<WolfJet, B1>* jets_1_;
       std::array<WolfJet, B2>* jets_2_;
       std::array<WolfJet, B3>* jets_3_;
       std::array<WolfJet, B4>* jets_4_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr,
                          StateBlockPtr _state1Ptr,
                          StateBlockPtr _state2Ptr,
                          StateBlockPtr _state3Ptr,
                          StateBlockPtr _state4Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>),
           jets_1_(new std::array<WolfJet, B1>),
           jets_2_(new std::array<WolfJet, B2>),
           jets_3_(new std::array<WolfJet, B3>),
           jets_4_(new std::array<WolfJet, B4>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B1; i++)
              (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B2; i++)
              (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B3; i++)
              (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B4; i++)
              (*jets_4_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
       {
           delete jets_0_;
           delete jets_1_;
           delete jets_2_;
           delete jets_3_;
           delete jets_4_;
           delete residuals_jets_;
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                        parameters[1],
                                                        parameters[2],
                                                        parameters[3],
                                                        parameters[4],
                                                        residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                        jets_1_->data(),
                                                        jets_2_->data(),
                                                        jets_3_->data(),
                                                        jets_4_->data(),
                                                        residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
           for (auto i = 0; i < B1; i++)
               (*jets_1_)[i].a = parameters[1][i];
           for (auto i = 0; i < B2; i++)
               (*jets_2_)[i].a = parameters[2][i];
           for (auto i = 0; i < B3; i++)
               (*jets_3_)[i].a = parameters[3][i];
           for (auto i = 0; i < B4; i++)
               (*jets_4_)[i].a = parameters[4][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             jets_1_->data(),
                                             jets_2_->data(),
                                             jets_3_->data(),
                                             jets_4_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                        state_ptrs_[1]->getPtr(),
                                        state_ptrs_[2]->getPtr(),
                                        state_ptrs_[3]->getPtr(),
                                        state_ptrs_[4]->getPtr(),
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////// SPECIALIZATION 4 BLOCKS ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3>
class ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,0,0,0,0,0,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = B1;
       static const unsigned int block2Size = B2;
       static const unsigned int block3Size = B3;
       static const unsigned int block4Size = 0;
       static const unsigned int block5Size = 0;
       static const unsigned int block6Size = 0;
       static const unsigned int block7Size = 0;
       static const unsigned int block8Size = 0;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 4;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0 + B1 + B2 + B3> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;
       std::array<WolfJet, B1>* jets_1_;
       std::array<WolfJet, B2>* jets_2_;
       std::array<WolfJet, B3>* jets_3_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr,
                          StateBlockPtr _state1Ptr,
                          StateBlockPtr _state2Ptr,
                          StateBlockPtr _state3Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>),
           jets_1_(new std::array<WolfJet, B1>),
           jets_2_(new std::array<WolfJet, B2>),
           jets_3_(new std::array<WolfJet, B3>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B1; i++)
              (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B2; i++)
              (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B3; i++)
              (*jets_3_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
       {
           delete jets_0_;
           delete jets_1_;
           delete jets_2_;
           delete jets_3_;
           delete residuals_jets_;
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                 parameters[1],
                                                 parameters[2],
                                                 parameters[3],
                                                 residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                 jets_1_->data(),
                                                 jets_2_->data(),
                                                 jets_3_->data(),
                                                 residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
           for (auto i = 0; i < B1; i++)
               (*jets_1_)[i].a = parameters[1][i];
           for (auto i = 0; i < B2; i++)
               (*jets_2_)[i].a = parameters[2][i];
           for (auto i = 0; i < B3; i++)
               (*jets_3_)[i].a = parameters[3][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             jets_1_->data(),
                                             jets_2_->data(),
                                             jets_3_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));

          // print jacobian matrices
//           for (auto i = 0; i < n_blocks; i++)
//               std::cout << jacobians_[i] << std::endl << std::endl;
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                        state_ptrs_[1]->getPtr(),
                                        state_ptrs_[2]->getPtr(),
                                        state_ptrs_[3]->getPtr()
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////// SPECIALIZATION 3 BLOCKS ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2>
class ConstraintAutodiff<CtrT,RES,B0,B1,B2,0,0,0,0,0,0,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = B1;
       static const unsigned int block2Size = B2;
       static const unsigned int block3Size = 0;
       static const unsigned int block4Size = 0;
       static const unsigned int block5Size = 0;
       static const unsigned int block6Size = 0;
       static const unsigned int block7Size = 0;
       static const unsigned int block8Size = 0;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 3;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0 + B1 + B2> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;
       std::array<WolfJet, B1>* jets_1_;
       std::array<WolfJet, B2>* jets_2_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr,
                          StateBlockPtr _state1Ptr,
                          StateBlockPtr _state2Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr,_state1Ptr,_state2Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>),
           jets_1_(new std::array<WolfJet, B1>),
           jets_2_(new std::array<WolfJet, B2>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B1; i++)
              (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B2; i++)
              (*jets_2_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
       {
           delete jets_0_;
           delete jets_1_;
           delete jets_2_;
           delete residuals_jets_;
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                        parameters[1],
                                                        parameters[2],
                                                        residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                        jets_1_->data(),
                                                        jets_2_->data(),
                                                        residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
           for (auto i = 0; i < B1; i++)
               (*jets_1_)[i].a = parameters[1][i];
           for (auto i = 0; i < B2; i++)
               (*jets_2_)[i].a = parameters[2][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             jets_1_->data(),
                                             jets_2_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));

          // print jacobian matrices
//           for (auto i = 0; i < n_blocks; i++)
//               std::cout << jacobians_[i] << std::endl << std::endl;
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                        state_ptrs_[1]->getPtr(),
                                        state_ptrs_[2]->getPtr()
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////// SPECIALIZATION 2 BLOCKS ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1>
class ConstraintAutodiff<CtrT,RES,B0,B1,0,0,0,0,0,0,0,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = B1;
       static const unsigned int block2Size = 0;
       static const unsigned int block3Size = 0;
       static const unsigned int block4Size = 0;
       static const unsigned int block5Size = 0;
       static const unsigned int block6Size = 0;
       static const unsigned int block7Size = 0;
       static const unsigned int block8Size = 0;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 2;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0 + B1> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;
       std::array<WolfJet, B1>* jets_1_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr,
                          StateBlockPtr _state1Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr,_state1Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>),
           jets_1_(new std::array<WolfJet, B1>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           for (auto i = 0; i < B1; i++)
              (*jets_1_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
       {
           delete jets_0_;
           delete jets_1_;
           delete residuals_jets_;
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                 parameters[1],
                                                 residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                 jets_1_->data(),
                                                 residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
           for (auto i = 0; i < B1; i++)
               (*jets_1_)[i].a = parameters[1][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             jets_1_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));

          // print jacobian matrices
//           for (auto i = 0; i < n_blocks; i++)
//               std::cout << jacobians_[i] << std::endl << std::endl;
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr(),
                                        state_ptrs_[1]->getPtr()
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////// SPECIALIZATION 1 BLOCK ////////////////////////////////////////////////////////////////////////

template <class CtrT,unsigned int RES,unsigned int B0>
class ConstraintAutodiff<CtrT,RES,B0,0,0,0,0,0,0,0,0,0> : public ConstraintBase
{
   public:

       static const unsigned int residualSize = RES;
       static const unsigned int block0Size = B0;
       static const unsigned int block1Size = 0;
       static const unsigned int block2Size = 0;
       static const unsigned int block3Size = 0;
       static const unsigned int block4Size = 0;
       static const unsigned int block5Size = 0;
       static const unsigned int block6Size = 0;
       static const unsigned int block7Size = 0;
       static const unsigned int block8Size = 0;
       static const unsigned int block9Size = 0;
       static const unsigned int n_blocks = 1;

       static const std::vector<unsigned int> state_block_sizes_;

       typedef ceres::Jet<Scalar, B0> WolfJet;

   protected:

       std::vector<StateBlockPtr> state_ptrs_;

       static const std::vector<unsigned int> jacobian_locations_;
       std::array<WolfJet, RES>* residuals_jets_;

       std::array<WolfJet, B0>* jets_0_;

   public:

       ConstraintAutodiff(ConstraintType _tp,
                          const FrameBasePtr& _frame_other_ptr,
                          const CaptureBasePtr& _capture_other_ptr,
                          const FeatureBasePtr& _feature_other_ptr,
                          const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status,
                          StateBlockPtr _state0Ptr) :
           ConstraintBase(_tp, _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
           state_ptrs_({_state0Ptr}),
           residuals_jets_(new std::array<WolfJet, RES>),
           jets_0_(new std::array<WolfJet, B0>)
       {
           // initialize jets
           unsigned int last_jet_idx = 0;
           for (auto i = 0; i < B0; i++)
              (*jets_0_)[i] = WolfJet(0, last_jet_idx++);
           state_ptrs_.resize(n_blocks);
       }

       virtual ~ConstraintAutodiff()
       {
           delete jets_0_;
           delete residuals_jets_;
       }

       virtual JacobianMethod getJacobianMethod() const
       {
           return JAC_AUTO;
       }

       virtual bool evaluate(double const* const* parameters, double* residuals, double** jacobians) const
       {
           // only residuals
           if (jacobians == nullptr)
           {
               (*static_cast<CtrT const*>(this))(parameters[0],
                                                 residuals);
           }
           // also compute jacobians
           else
           {
               // update jets real part
               std::vector<double const*> param_vec;
               param_vec.assign(parameters,parameters+n_blocks);
               updateJetsRealPart(param_vec);

               // call functor
               (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                                 residuals_jets_->data());

               // fill the residual array
               for (auto i = 0; i < RES; i++)
                   residuals[i] = (*residuals_jets_)[i].a;

               // fill the jacobian matrices
               for (auto i = 0; i<n_blocks; i++)
                   if (jacobians[i] != nullptr)
                       for (unsigned int row = 0; row < RES; row++)
                           std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                     (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                     jacobians[i] + row * state_block_sizes_.at(i));
           }
           return true;
       }

       void updateJetsRealPart(const std::vector<double const*>& parameters) const
       {
           // update jets real part
           for (auto i = 0; i < B0; i++)
               (*jets_0_)[i].a = parameters[0][i];
       }

       virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const
       {
           assert(residual_.size() == RES);
           jacobians_.clear();

           assert(_states_ptr.size() == n_blocks);

           // init jacobian
           for(auto i = 0; i < n_blocks; ++i)
           {
              Eigen::MatrixXs Ji = Eigen::MatrixXs::Zero(RES, state_block_sizes_[i]);
              jacobians_.push_back(Ji);
           }

           // update jets real part
           updateJetsRealPart(_states_ptr);

           // call functor
           (*static_cast<CtrT const*>(this))(jets_0_->data(),
                                             residuals_jets_->data());

           // fill the residual vector
           for (auto i = 0; i < RES; i++)
               residual_(i) = (*residuals_jets_)[i].a;

           // fill the jacobian matrices
           for (auto i = 0; i<n_blocks; i++)
               if (!state_ptrs_[i]->isFixed())
                   for (unsigned int row = 0; row < RES; row++)
                       std::copy((*residuals_jets_)[row].v.data() + jacobian_locations_.at(i),
                                 (*residuals_jets_)[row].v.data() + jacobian_locations_.at(i) + state_block_sizes_.at(i),
                                 jacobians_[i].data() + row * state_block_sizes_.at(i));

          // print jacobian matrices
//           for (auto i = 0; i < n_blocks; i++)
//               std::cout << jacobians_[i] << std::endl << std::endl;
       }

       virtual std::vector<Scalar*> getStateScalarPtrVector() const
       {
           return std::vector<Scalar*>({state_ptrs_[0]->getPtr()
                                        });
       }

       virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const
       {
           return state_ptrs_;
       }

       virtual std::vector<unsigned int> getStateSizes() const
       {
           return state_block_sizes_;
       }

       virtual unsigned int getSize() const
       {
           return RES;
       }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          STATIC CONST VECTORS INITIALIZATION
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// state_block_sizes_
// 10 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6,unsigned int B7,unsigned int B8,unsigned int B9>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,B7,B8,B9>::state_block_sizes_ = {B0,B1,B2,B3,B4,B5,B6,B7,B8,B9};
// 9 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6,unsigned int B7,unsigned int B8>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,B7,B8,0>::state_block_sizes_ = {B0,B1,B2,B3,B4,B5,B6,B7,B8};
// 8 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6,unsigned int B7>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,B7,0,0>::state_block_sizes_ = {B0,B1,B2,B3,B4,B5,B6,B7};
// 7 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,0,0,0>::state_block_sizes_ = {B0,B1,B2,B3,B4,B5,B6};
// 6 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,0,0,0,0>::state_block_sizes_ = {B0,B1,B2,B3,B4,B5};
// 5 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,0,0,0,0,0>::state_block_sizes_ = {B0,B1,B2,B3,B4};
// 4 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,0,0,0,0,0,0>::state_block_sizes_ = {B0,B1,B2,B3};
// 3 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,0,0,0,0,0,0,0>::state_block_sizes_ = {B0,B1,B2};
// 2 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,0,0,0,0,0,0,0,0>::state_block_sizes_ = {B0,B1};
// 1 BLOCK
template <class CtrT,unsigned int RES,unsigned int B0>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,0,0,0,0,0,0,0,0,0>::state_block_sizes_ = {B0};

// jacobian_locations_
// 10 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6,unsigned int B7,unsigned int B8,unsigned int B9>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,B7,B8,B9>::jacobian_locations_ = {0,
                                                                                                                   B0,
                                                                                                                   B0+B1,
                                                                                                                   B0+B1+B2,
                                                                                                                   B0+B1+B2+B3,
                                                                                                                   B0+B1+B2+B3+B4,
                                                                                                                   B0+B1+B2+B3+B4+B5,
                                                                                                                   B0+B1+B2+B3+B4+B5+B6,
                                                                                                                   B0+B1+B2+B3+B4+B5+B6+B7,
                                                                                                                   B0+B1+B2+B3+B4+B5+B6+B7+B8};
// 9 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6,unsigned int B7,unsigned int B8>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,B7,B8,0>::jacobian_locations_ = {0,
                                                                                                                  B0,
                                                                                                                  B0+B1,
                                                                                                                  B0+B1+B2,
                                                                                                                  B0+B1+B2+B3,
                                                                                                                  B0+B1+B2+B3+B4,
                                                                                                                  B0+B1+B2+B3+B4+B5,
                                                                                                                  B0+B1+B2+B3+B4+B5+B6,
                                                                                                                  B0+B1+B2+B3+B4+B5+B6+B7};
// 8 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6,unsigned int B7>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,B7,0,0>::jacobian_locations_ = {0,
                                                                                                                 B0,
                                                                                                                 B0+B1,
                                                                                                                 B0+B1+B2,
                                                                                                                 B0+B1+B2+B3,
                                                                                                                 B0+B1+B2+B3+B4,
                                                                                                                 B0+B1+B2+B3+B4+B5,
                                                                                                                 B0+B1+B2+B3+B4+B5+B6};
// 7 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5,unsigned int B6>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,B6,0,0,0>::jacobian_locations_ = {0,
                                                                                                                B0,
                                                                                                                B0+B1,
                                                                                                                B0+B1+B2,
                                                                                                                B0+B1+B2+B3,
                                                                                                                B0+B1+B2+B3+B4,
                                                                                                                B0+B1+B2+B3+B4+B5};
// 6 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4,unsigned int B5>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,B5,0,0,0,0>::jacobian_locations_ = {0,
                                                                                                               B0,
                                                                                                               B0+B1,
                                                                                                               B0+B1+B2,
                                                                                                               B0+B1+B2+B3,
                                                                                                               B0+B1+B2+B3+B4};
// 5 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3,unsigned int B4>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,B4,0,0,0,0,0>::jacobian_locations_ = {0,
                                                                                                              B0,
                                                                                                              B0+B1,
                                                                                                              B0+B1+B2,
                                                                                                              B0+B1+B2+B3};
// 4 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2,unsigned int B3>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,B3,0,0,0,0,0,0>::jacobian_locations_ = {0,
                                                                                                             B0,
                                                                                                             B0+B1,
                                                                                                             B0+B1+B2};
// 3 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1,unsigned int B2>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,B2,0,0,0,0,0,0,0>::jacobian_locations_ = {0,
                                                                                                            B0,
                                                                                                            B0+B1};
// 2 BLOCKS
template <class CtrT,unsigned int RES,unsigned int B0,unsigned int B1>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,B1,0,0,0,0,0,0,0,0>::jacobian_locations_ = {0,
                                                                                                           B0};
// 1 BLOCK
template <class CtrT,unsigned int RES,unsigned int B0>
const std::vector<unsigned int> ConstraintAutodiff<CtrT,RES,B0,0,0,0,0,0,0,0,0,0>::jacobian_locations_ = {0};

} // namespace wolf

#endif
