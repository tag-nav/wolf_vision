#ifndef CONSTRAINT_BASE_H_
#define CONSTRAINT_BASE_H_

// Forward declarations for node templates
namespace wolf{
class FeatureBase;
}

//Wolf includes
#include "base/wolf.h"
#include "base/node_base.h"

//std includes

namespace wolf {

/** \brief Enumeration of constraint status
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    CTR_INACTIVE = 0,   ///< Constraint established with a frame (odometry).
    CTR_ACTIVE = 1      ///< Constraint established with absolute reference.
} ConstraintStatus;

/** \brief Enumeration of jacobian computation method
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    JAC_AUTO = 1,   ///< Auto differentiation (AutoDiffCostFunctionWrapper or ceres::NumericDiffCostFunction).
    JAC_NUMERIC,    ///< Numeric differentiation (ceres::NumericDiffCostFunction).
    JAC_ANALYTIC    ///< Analytic jacobians.
} JacobianMethod;

//class ConstraintBase
class ConstraintBase : public NodeBase, public std::enable_shared_from_this<ConstraintBase>
{
    private:
        FeatureBaseWPtr feature_ptr_;                    ///< FeatureBase pointer (upper node)

        static unsigned int constraint_id_count_;

    protected:
        unsigned int constraint_id_;
        ConstraintStatus status_;                       ///< status of constraint (types defined at wolf.h)
        bool apply_loss_function_;                      ///< flag for applying loss function to this constraint
        FrameBaseWPtr frame_other_ptr_;                 ///< FrameBase pointer (for category CTR_FRAME)
        CaptureBaseWPtr capture_other_ptr_;             ///< CaptureBase pointer
        FeatureBaseWPtr feature_other_ptr_;             ///< FeatureBase pointer (for category CTR_FEATURE)
        LandmarkBaseWPtr landmark_other_ptr_;           ///< LandmarkBase pointer (for category CTR_LANDMARK)
        ProcessorBaseWPtr processor_ptr_;               ///< ProcessorBase pointer

    public:

        /** \brief Constructor of category CTR_ABSOLUTE
         **/
        ConstraintBase(const std::string&  _tp,
                       bool _apply_loss_function = false,
                       ConstraintStatus _status = CTR_ACTIVE);

        /** \brief Constructor valid for all categories (FRAME, FEATURE, LANDMARK)
         **/
        ConstraintBase(const std::string&  _tp,
                       const FrameBasePtr& _frame_other_ptr,
                       const CaptureBasePtr& _capture_other_ptr,
                       const FeatureBasePtr& _feature_other_ptr,
                       const LandmarkBasePtr& _landmark_other_ptr,
                       const ProcessorBasePtr& _processor_ptr = nullptr,
                       bool _apply_loss_function = false,
                       ConstraintStatus _status = CTR_ACTIVE);

        virtual ~ConstraintBase() = default;

        virtual void remove();

        unsigned int id() const;

        /** \brief Evaluate the constraint given the input parameters and returning the residuals and jacobians
        **/
        virtual bool evaluate(Scalar const* const* _parameters, Scalar* _residuals, Scalar** _jacobians) const = 0;

        /** Returns a vector of Jacobian matrix corresponding to each state block evaluated in the point provided in _states_ptr and the residual vector
         **/
        virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& _residual, std::vector<Eigen::MatrixXs>& _jacobians) const = 0;

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const = 0;

        /** \brief Returns a vector of pointers to the states in which this constraint depends
         **/
        virtual std::vector<StateBlockPtr> getStateBlockPtrVector() const = 0;

        /** \brief Returns a vector of the states sizes
         **/
        virtual std::vector<unsigned int> getStateSizes() const = 0;

        /** \brief Returns a reference to the feature measurement
         **/
        virtual const Eigen::VectorXs& getMeasurement() const;

        /** \brief Returns a reference to the feature measurement covariance
         **/
        virtual const Eigen::MatrixXs& getMeasurementCovariance() const;

        /** \brief Returns a reference to the feature measurement square root information
         **/
        virtual const Eigen::MatrixXs& getMeasurementSquareRootInformationUpper() const;

        /** \brief Returns a pointer to the feature constrained from
         **/
        FeatureBasePtr getFeaturePtr() const;
        void setFeaturePtr(const FeatureBasePtr _ft_ptr){feature_ptr_ = _ft_ptr;}

        /** \brief Returns a pointer to its capture
         **/
        CaptureBasePtr getCapturePtr() const;

        /** \brief Returns the constraint residual size
         **/
        virtual unsigned int getSize() const = 0;

        /** \brief Gets the status
         */
        ConstraintStatus getStatus() const;

        /** \brief Sets the status
         */
        void setStatus(ConstraintStatus _status);

        /** \brief Gets the apply loss function flag
         */
        bool getApplyLossFunction();

        /** \brief Gets the apply loss function flag
         */
        void setApplyLossFunction(const bool _apply);

        /** \brief Returns a pointer to the frame constrained to
         **/
        FrameBasePtr getFrameOtherPtr() const       { return frame_other_ptr_.lock(); }
        void setFrameOtherPtr(FrameBasePtr _frm_o)  { frame_other_ptr_ = _frm_o; }

        /** \brief Returns a pointer to the frame constrained to
         **/
        CaptureBasePtr getCaptureOtherPtr() const       { return capture_other_ptr_.lock(); }
        void setCaptureOtherPtr(CaptureBasePtr _cap_o)  { capture_other_ptr_ = _cap_o; }

        /** \brief Returns a pointer to the feature constrained to
         **/
        FeatureBasePtr getFeatureOtherPtr() const       { return feature_other_ptr_.lock(); }
        void setFeatureOtherPtr(FeatureBasePtr _ftr_o)  { feature_other_ptr_ = _ftr_o; }

        /** \brief Returns a pointer to the landmark constrained to
         **/
        LandmarkBasePtr getLandmarkOtherPtr() const     { return landmark_other_ptr_.lock(); }
        void setLandmarkOtherPtr(LandmarkBasePtr _lmk_o){ landmark_other_ptr_ = _lmk_o; }

        /**
         * @brief getProcessor
         * @return
         */
        ProcessorBasePtr getProcessor() const;

        /**
         * @brief setProcessor
         * @param _processor_ptr
         */
        void setProcessor(const ProcessorBasePtr& _processor_ptr);

//    protected:
//        template<typename D>
//        void print(const std::string& name, const Eigen::MatrixBase<D>& mat) const; // Do nothing if input Scalar type is ceres::Jet
//        template<int R, int C>
//        void print(const std::string& name, const Eigen::Matrix<Scalar, R, C>& mat) const; // Normal print if Scalar type is wolf::Scalar
};

}

// IMPLEMENTATION //

#include "base/problem.h"
#include "base/frame_base.h"
#include "base/feature/feature_base.h"
#include "base/sensor/sensor_base.h"
#include "base/landmark/landmark_base.h"

namespace wolf{

//template<typename D>
//inline void ConstraintBase::print(const std::string& name, const Eigen::MatrixBase<D>& mat) const {} // Do nothing if input Scalar type is ceres::Jet
//template<int R, int C>
//inline void ConstraintBase::print(const std::string& name, const Eigen::Matrix<Scalar, R, C>& mat) const // Normal print if Scalar type is wolf::Scalar
//{
//    if (mat.cols() == 1)
//    {
//        WOLF_TRACE(name, ": ", mat.transpose());
//    }
//    else if (mat.rows() == 1)
//    {
//        WOLF_TRACE(name, ": ", mat);
//    }
//    else
//    {
//        WOLF_TRACE(name, ":\n", mat);
//    }
//}

inline unsigned int ConstraintBase::id() const
{
    return constraint_id_;
}

inline FeatureBasePtr ConstraintBase::getFeaturePtr() const
{
    return feature_ptr_.lock();
}

inline ConstraintStatus ConstraintBase::getStatus() const
{
    return status_;
}

inline bool ConstraintBase::getApplyLossFunction()
{
    return apply_loss_function_;
}

inline ProcessorBasePtr ConstraintBase::getProcessor() const
{
  return processor_ptr_.lock();
}

inline void ConstraintBase::setProcessor(const ProcessorBasePtr& _processor_ptr)
{
  processor_ptr_ = _processor_ptr;
}

} // namespace wolf
#endif
