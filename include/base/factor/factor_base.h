#ifndef FACTOR_BASE_H_
#define FACTOR_BASE_H_

// Forward declarations for node templates
namespace wolf{
class FeatureBase;
}

//Wolf includes
#include "base/wolf.h"
#include "base/node_base.h"

//std includes

namespace wolf {

/** \brief Enumeration of factor status
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    CTR_INACTIVE = 0,   ///< Factor established with a frame (odometry).
    CTR_ACTIVE = 1      ///< Factor established with absolute reference.
} FactorStatus;

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

//class FactorBase
class FactorBase : public NodeBase, public std::enable_shared_from_this<FactorBase>
{
    private:
        FeatureBaseWPtr feature_ptr_;                    ///< FeatureBase pointer (upper node)

        static unsigned int factor_id_count_;

    protected:
        unsigned int factor_id_;
        FactorStatus status_;                       ///< status of factor (types defined at wolf.h)
        bool apply_loss_function_;                      ///< flag for applying loss function to this factor
        FrameBaseWPtr frame_other_ptr_;                 ///< FrameBase pointer (for category CTR_FRAME)
        CaptureBaseWPtr capture_other_ptr_;             ///< CaptureBase pointer
        FeatureBaseWPtr feature_other_ptr_;             ///< FeatureBase pointer (for category CTR_FEATURE)
        LandmarkBaseWPtr landmark_other_ptr_;           ///< LandmarkBase pointer (for category CTR_LANDMARK)
        ProcessorBaseWPtr processor_ptr_;               ///< ProcessorBase pointer

    public:

        /** \brief Constructor of category CTR_ABSOLUTE
         **/
        FactorBase(const std::string&  _tp,
                       bool _apply_loss_function = false,
                       FactorStatus _status = CTR_ACTIVE);

        /** \brief Constructor valid for all categories (FRAME, FEATURE, LANDMARK)
         **/
        FactorBase(const std::string&  _tp,
                       const FrameBasePtr& _frame_other_ptr,
                       const CaptureBasePtr& _capture_other_ptr,
                       const FeatureBasePtr& _feature_other_ptr,
                       const LandmarkBasePtr& _landmark_other_ptr,
                       const ProcessorBasePtr& _processor_ptr = nullptr,
                       bool _apply_loss_function = false,
                       FactorStatus _status = CTR_ACTIVE);

        virtual ~FactorBase() = default;

        virtual void remove();

        unsigned int id() const;

        /** \brief Evaluate the factor given the input parameters and returning the residuals and jacobians
        **/
        virtual bool evaluate(Scalar const* const* _parameters, Scalar* _residuals, Scalar** _jacobians) const = 0;

        /** Returns a vector of Jacobian matrix corresponding to each state block evaluated in the point provided in _states_ptr and the residual vector
         **/
        virtual void evaluate(const std::vector<const Scalar*>& _states_ptr, Eigen::VectorXs& _residual, std::vector<Eigen::MatrixXs>& _jacobians) const = 0;

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const = 0;

        /** \brief Returns a vector of pointers to the states in which this factor depends
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
        FeatureBasePtr getFeature() const;
        void setFeaturePtr(const FeatureBasePtr _ft_ptr){feature_ptr_ = _ft_ptr;}

        /** \brief Returns a pointer to its capture
         **/
        CaptureBasePtr getCapture() const;

        /** \brief Returns the factor residual size
         **/
        virtual unsigned int getSize() const = 0;

        /** \brief Gets the status
         */
        FactorStatus getStatus() const;

        /** \brief Sets the status
         */
        void setStatus(FactorStatus _status);

        /** \brief Gets the apply loss function flag
         */
        bool getApplyLossFunction();

        /** \brief Gets the apply loss function flag
         */
        void setApplyLossFunction(const bool _apply);

        /** \brief Returns a pointer to the frame constrained to
         **/
        FrameBasePtr getFrameOther() const       { return frame_other_ptr_.lock(); }
        void setFrameOtherPtr(FrameBasePtr _frm_o)  { frame_other_ptr_ = _frm_o; }

        /** \brief Returns a pointer to the frame constrained to
         **/
        CaptureBasePtr getCaptureOther() const       { return capture_other_ptr_.lock(); }
        void setCaptureOtherPtr(CaptureBasePtr _cap_o)  { capture_other_ptr_ = _cap_o; }

        /** \brief Returns a pointer to the feature constrained to
         **/
        FeatureBasePtr getFeatureOther() const       { return feature_other_ptr_.lock(); }
        void setFeatureOtherPtr(FeatureBasePtr _ftr_o)  { feature_other_ptr_ = _ftr_o; }

        /** \brief Returns a pointer to the landmark constrained to
         **/
        LandmarkBasePtr getLandmarkOther() const     { return landmark_other_ptr_.lock(); }
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

        template<typename otherType>
        void link(otherType);
        template<typename otherType, typename classType, typename... T>
        static std::shared_ptr<FactorBase> emplace(otherType _oth_ptr, T&&... all);

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
//inline void FactorBase::print(const std::string& name, const Eigen::MatrixBase<D>& mat) const {} // Do nothing if input Scalar type is ceres::Jet
//template<int R, int C>
//inline void FactorBase::print(const std::string& name, const Eigen::Matrix<Scalar, R, C>& mat) const // Normal print if Scalar type is wolf::Scalar
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

template<typename OtherPtr, typename classType, typename... T>
std::shared_ptr<FactorBase> FactorBase::emplace(OtherPtr _oth_ptr, T&&... all)
{
    FactorBasePtr ctr = std::make_shared<classType>(std::forward<T>(all)...);
    ctr->link(_oth_ptr);
    return ctr;
}

template<typename otherType>
void FactorBase::link(otherType _oth_ptr)
{
    std::cout << "Linking FactorBase" << std::endl;
    // _oth_ptr->addConstrainedBy(shared_from_this());
    _oth_ptr->addFactor(shared_from_this());
    auto frame_other = this->frame_other_ptr_.lock();
    if(frame_other != nullptr) frame_other->addConstrainedBy(shared_from_this());
    auto capture_other = this->capture_other_ptr_.lock();
    if(capture_other != nullptr) capture_other->addConstrainedBy(shared_from_this());
    auto feature_other = this->feature_other_ptr_.lock();
    if(feature_other != nullptr) feature_other->addConstrainedBy(shared_from_this());
    auto landmark_other = this->landmark_other_ptr_.lock();
    if(landmark_other != nullptr) landmark_other->addConstrainedBy(shared_from_this());
}

inline unsigned int FactorBase::id() const
{
    return factor_id_;
}

inline FeatureBasePtr FactorBase::getFeature() const
{
    return feature_ptr_.lock();
}

inline FactorStatus FactorBase::getStatus() const
{
    return status_;
}

inline bool FactorBase::getApplyLossFunction()
{
    return apply_loss_function_;
}

inline ProcessorBasePtr FactorBase::getProcessor() const
{
  return processor_ptr_.lock();
}

inline void FactorBase::setProcessor(const ProcessorBasePtr& _processor_ptr)
{
  processor_ptr_ = _processor_ptr;
}

} // namespace wolf
#endif
