#ifndef FACTOR_FEATURE_EPIPOLAR_H
#define FACTOR_FEATURE_EPIPOLAR_H

#include "core/factor/factor_base.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(FactorFeatureEpipolar);

class FactorFeatureEpipolar : public FactorBase
{
    public:
        FactorFeatureEpipolar(const FeatureBasePtr& _feature_ptr,
                           const FeatureBasePtr& _feature_other_ptr,
                           const ProcessorBasePtr& _processor_ptr = nullptr,
                           bool _apply_loss_function = false,
                           FactorStatus _status = FAC_ACTIVE);

        virtual ~FactorFeatureEpipolar() = default;

        /** \brief Evaluate the factor given the input parameters and returning the residuals and jacobians
        **/
        virtual bool evaluate(Scalar const* const* parameters, Scalar* residuals, Scalar** jacobians) const override {return true;};

        /** Returns a residual vector and a vector of Jacobian matrix corresponding to each state block evaluated in the point provided in _states_ptr
         **/
        virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& residual_, std::vector<Eigen::MatrixXs>& jacobians_) const override {};

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const override {return JAC_ANALYTIC;}

        /** \brief Returns a vector of pointers to the states in which this factor depends
         **/
        virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const override {return std::vector<StateBlockPtr>(0);}

        /** \brief Returns the factor residual size
         **/
        virtual unsigned int getSize() const override {return 0;}

        /** \brief Returns the factor states sizes
         **/
        virtual std::vector<unsigned int> getStateSizes() const override {return std::vector<unsigned int>({1});}

    public:
        static FactorBasePtr create(const FeatureBasePtr& _feature_ptr,
                                              const NodeBasePtr& _correspondant_ptr,
                                              const ProcessorBasePtr& _processor_ptr = nullptr);

};

inline FactorFeatureEpipolar::FactorFeatureEpipolar(const FeatureBasePtr& /*_feature_ptr*/, const FeatureBasePtr& _feature_other_ptr,
                                              const ProcessorBasePtr& _processor_ptr,
                                              bool _apply_loss_function, FactorStatus _status) :
        FactorBase("FEATURE DUMMY", nullptr, nullptr, _feature_other_ptr, nullptr, _processor_ptr, _apply_loss_function, _status)
{
    //
}

inline FactorBasePtr FactorFeatureEpipolar::create(const FeatureBasePtr& _feature_ptr, const NodeBasePtr& _correspondant_ptr,
                                                          const ProcessorBasePtr& _processor_ptr)
{
    return std::make_shared<FactorFeatureEpipolar>(_feature_ptr, std::static_pointer_cast<FeatureBase>(_correspondant_ptr), _processor_ptr);
}

} // namespace wolf

#endif // FACTOR_FEATURE_EPIPOLAR_H
