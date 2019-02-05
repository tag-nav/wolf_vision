/**
 * \file constraint_block_absolute.h
 *
 *  Created on: Dec 15, 2017
 *      \author: AtDinesh
 */

#ifndef CONSTRAINT_BLOCK_ABSOLUTE_H_
#define CONSTRAINT_BLOCK_ABSOLUTE_H_

//Wolf includes
#include "constraint_analytic.h"
#include "frame_base.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintBlockAbsolute);

//class
class ConstraintBlockAbsolute : public ConstraintAnalytic
{
    private:
        SizeEigen sb_constrained_start_; // the index of the first state element that is constrained
        SizeEigen sb_constrained_size_;  // the size of the state segment that is constrained

    public:

        /** \brief Constructor
         *
         * \param _sb_ptr the constrained state block
         * \param _start_idx (default=0) the index of the first state element that is constrained
         * \param _start_idx (default=-1) the size of the state segment that is constrained, -1 = all the
         *
         */
        ConstraintBlockAbsolute(StateBlockPtr _sb_ptr, unsigned int _start_idx = 0, int _size = -1, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAnalytic("BLOCK ABS", _apply_loss_function, _status, _sb_ptr),
            sb_constrained_start_(_start_idx),
            sb_constrained_size_(_size == -1 ? _sb_ptr->getSize() : _size)
        {
            assert(sb_constrained_size_-sb_constrained_start_ == _sb_ptr->getSize());
        }

        virtual ~ConstraintBlockAbsolute() = default;

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
         * \param _st_vector is a vector containing the mapped eigen vectors of all state blocks involved in the constraint
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

        /** \brief Returns the constraint residual size
         **/
        virtual unsigned int getSize() const;
};

inline Eigen::VectorXs ConstraintBlockAbsolute::evaluateResiduals(const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector) const
{
    assert(_st_vector.size() == 1 && "Wrong number of state blocks!");
    assert(_st_vector.front().size() >= getMeasurement().size() && "Wrong StateBlock size");

    // residual
    if (sb_constrained_size_ == _st_vector.front().size())
        return getMeasurementSquareRootInformationUpper() * (_st_vector.front() - getMeasurement());
    else
        return getMeasurementSquareRootInformationUpper() * (_st_vector.front().segment(sb_constrained_start_,sb_constrained_size_) - getMeasurement());
}

inline void ConstraintBlockAbsolute::evaluateJacobians(
        const std::vector<Eigen::Map<const Eigen::VectorXs> >& _st_vector,
        std::vector<Eigen::Map<Eigen::MatrixXs> >& jacobians, const std::vector<bool>& _compute_jacobian) const
{
    assert(_st_vector.size() == 1 && "Wrong number of state blocks!");
    assert(jacobians.size() == 1 && "Wrong number of jacobians!");
    assert(_compute_jacobian.size() == 1 && "Wrong number of _compute_jacobian booleans!");
    assert(_st_vector.front().size() >= getMeasurement().size() && "StateBlock size and measurement size should match");


    // jacobian is identity -> returning sqrt information right (or upper)
    if (_compute_jacobian.front())
    {
        // whole state constrained
        if (sb_constrained_size_ == _st_vector.front().size())
            jacobians.front() = getMeasurementSquareRootInformationUpper();
        // segment of state constrained
        else
        {
            jacobians.front() = Eigen::MatrixXs::Zero(sb_constrained_size_,_st_vector.front().size());
            jacobians.front().middleCols(sb_constrained_start_,sb_constrained_size_) = getMeasurementSquareRootInformationUpper();
        }
    }
}

inline void ConstraintBlockAbsolute::evaluatePureJacobians(std::vector<Eigen::MatrixXs>& jacobians) const
{
    assert(jacobians.size() == 1 && "Wrong number of jacobians!");

    // jacobian is identity
    jacobians.front() = Eigen::MatrixXs::Identity(getMeasurement().size(), getMeasurement().size());
}

inline unsigned int ConstraintBlockAbsolute::getSize() const
{
    return sb_constrained_size_;
}

} // namespace wolf

#endif
