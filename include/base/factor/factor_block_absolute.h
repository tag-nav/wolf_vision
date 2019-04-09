/**
 * \file factor_block_absolute.h
 *
 *  Created on: Dec 15, 2017
 *      \author: AtDinesh
 */

#ifndef FACTOR_BLOCK_ABSOLUTE_H_
#define FACTOR_BLOCK_ABSOLUTE_H_

//Wolf includes
#include "base/factor/factor_analytic.h"
#include "base/factor/factor_autodiff.h"
#include "base/frame_base.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(FactorBlockAbsolute);

//class
class FactorBlockAbsolute : public FactorAnalytic
{
    private:
        SizeEigen sb_size_;              // the size of the state block
        SizeEigen sb_constrained_start_; // the index of the first state element that is constrained
        SizeEigen sb_constrained_size_;  // the size of the state segment that is constrained
        Eigen::MatrixXs J_;              // Jacobian

    public:

        /** \brief Constructor
         *
         * \param _sb_ptr the constrained state block
         * \param _start_idx (default=0) the index of the first state element that is constrained
         * \param _start_idx (default=-1) the size of the state segment that is constrained, -1 = all the
         *
         */
        FactorBlockAbsolute(StateBlockPtr _sb_ptr,
                            unsigned int _start_idx = 0,
                            int _size = -1,
                            bool _apply_loss_function = false,
                            FactorStatus _status = FAC_ACTIVE) :
            FactorAnalytic("BLOCK ABS",
                           _apply_loss_function,
                           _status,
                           _sb_ptr),
            sb_size_(_sb_ptr->getSize()),
            sb_constrained_start_(_start_idx),
            sb_constrained_size_(_size == -1 ? sb_size_ : _size)
        {
            assert(sb_constrained_size_+sb_constrained_start_ <= sb_size_);

            // precompute Jacobian (Identity)
            if (sb_constrained_start_ == 0)
                J_ = Eigen::MatrixXs::Identity(sb_constrained_size_, sb_size_);
            else
            {
                J_ = Eigen::MatrixXs::Zero(sb_constrained_size_,sb_size_);
                J_.middleCols(sb_constrained_start_,sb_constrained_size_) = Eigen::MatrixXs::Identity(sb_constrained_size_, sb_constrained_size_);
            }
        }

        virtual ~FactorBlockAbsolute() = default;

        /** \brief Returns the residual evaluated in the states provided
         *
         * Returns the residual evaluated in the states provided in std::vector of mapped Eigen::VectorXs
         *
         **/
        virtual Eigen::VectorXs evaluateResiduals(const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector) const;

        /** \brief Returns the normalized jacobians evaluated in the states
         *
         * Returns the normalized (by the measurement noise sqrt information) jacobians evaluated in the states provided in std::vector of mapped Eigen::VectorXs.
         * IMPORTANT: only fill the jacobians of the state blocks specified in _compute_jacobian.
         *
         * \param _st_vector is a vector containing the mapped eigen vectors of all state blocks involved in the factor
         * \param jacobians is an output vector of mapped eigen matrices that sould contain the jacobians w.r.t each state block
         * \param _compute_jacobian is a vector that specifies whether the ith jacobian sould be computed or not
         *
         **/
        virtual void evaluateJacobians(const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector,
                                       std::vector<Eigen::Map<Eigen::MatrixXs> >& jacobians,
                                       const std::vector<bool>& _compute_jacobian) const;

        /** \brief Returns the pure jacobians (without measurement noise) evaluated in the state blocks values
         *
         * Returns the pure jacobians (without measurement noise) evaluated in the current state blocks values
         *
         * \param jacobians is an output vector of mapped eigen matrices that sould contain the jacobians w.r.t each state block
         *
         **/
        virtual void evaluatePureJacobians(std::vector<Eigen::MatrixXs>& jacobians) const;

        /** \brief Returns the factor residual size
         **/
        virtual unsigned int getSize() const;
};

inline Eigen::VectorXs FactorBlockAbsolute::evaluateResiduals(const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector) const
{
    assert(_st_vector.size() == 1 && "Wrong number of state blocks!");
    assert(_st_vector.front().size() >= getMeasurement().size() && "Wrong StateBlock size");

    // residual
    if (sb_constrained_size_ == _st_vector.front().size())
        return getMeasurementSquareRootInformationUpper() * (_st_vector.front() - getMeasurement());
    else
        return getMeasurementSquareRootInformationUpper() * (_st_vector.front().segment(sb_constrained_start_,sb_constrained_size_) - getMeasurement());
}

inline void FactorBlockAbsolute::evaluateJacobians(
        const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector,
        std::vector<Eigen::Map<Eigen::MatrixXs> >& jacobians, const std::vector<bool>& _compute_jacobian) const
{
    assert(_st_vector.size() == 1 && "Wrong number of state blocks!");
    assert(jacobians.size() == 1 && "Wrong number of jacobians!");
    assert(_compute_jacobian.size() == 1 && "Wrong number of _compute_jacobian booleans!");
    assert(_st_vector.front().size() == sb_size_ && "Wrong StateBlock size");
    assert(_st_vector.front().size() >= getMeasurement().size() && "StateBlock size and measurement size should match");

    // normalized jacobian
    if (_compute_jacobian.front())
        jacobians.front() = getMeasurementSquareRootInformationUpper() * J_;
}

inline void FactorBlockAbsolute::evaluatePureJacobians(std::vector<Eigen::MatrixXs>& jacobians) const
{
    assert(jacobians.size() == 1 && "Wrong number of jacobians!");

    jacobians.front() = J_;
}

inline unsigned int FactorBlockAbsolute::getSize() const
{
    return sb_constrained_size_;
}

} // namespace wolf

#endif
