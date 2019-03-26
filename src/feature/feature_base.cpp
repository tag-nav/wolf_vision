#include "base/feature/feature_base.h"
#include "base/factor/factor_base.h"
#include "base/capture/capture_base.h"

namespace wolf {

unsigned int FeatureBase::feature_id_count_ = 0;

FeatureBase::FeatureBase(const std::string& _type, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	NodeBase("FEATURE", _type),
    capture_ptr_(),
    feature_id_(++feature_id_count_),
    track_id_(0),
    landmark_id_(0),
	measurement_(_measurement)
{
    setMeasurementCovariance(_meas_covariance);
//    std::cout << "constructed      +f" << id() << std::endl;
}

FeatureBase::~FeatureBase()
{
//    std::cout << "destructed       -f" << id() << std::endl;
}

void FeatureBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        FeatureBasePtr this_f = shared_from_this(); // keep this alive while removing it

        // remove from upstream
        CaptureBasePtr C = capture_ptr_.lock();
        if (C)
        {
            C->getFeatureList().remove(this_f); // remove from upstream
            if (C->getFeatureList().empty())
                C->remove(); // remove upstream
        }

        // remove downstream
        while (!factor_list_.empty())
        {
            factor_list_.front()->remove(); // remove downstream
        }
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove(); // remove constrained
        }
    }
}

FactorBasePtr FeatureBase::addFactor(FactorBasePtr _co_ptr)
{
    factor_list_.push_back(_co_ptr);
    _co_ptr->setFeaturePtr(shared_from_this());
    _co_ptr->setProblem(getProblem());
    // add factor to be added in solver
    if (getProblem() != nullptr)
    {
        if (_co_ptr->getStatus() == CTR_ACTIVE)
            getProblem()->addFactor(_co_ptr);
    }
    else
        WOLF_TRACE("WARNING: ADDING CONSTRAINT ", _co_ptr->id(), " TO FEATURE ", this->id(), " NOT CONNECTED WITH PROBLEM.");
    return _co_ptr;
}

FrameBasePtr FeatureBase::getFrame() const
{
    return capture_ptr_.lock()->getFrame();
}

FactorBasePtr FeatureBase::addConstrainedBy(FactorBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
    _ctr_ptr->setFeatureOtherPtr(shared_from_this());
    return _ctr_ptr;
}

FactorBasePtrList& FeatureBase::getFactorList()
{
    return factor_list_;
}

void FeatureBase::getFactorList(FactorBasePtrList & _ctr_list)
{
    _ctr_list.insert(_ctr_list.end(), factor_list_.begin(), factor_list_.end());
}

void FeatureBase::setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov)
{
    WOLF_ASSERT_COVARIANCE_MATRIX(_meas_cov);

    // set (ensuring symmetry)
    measurement_covariance_ = _meas_cov.selfadjointView<Eigen::Upper>();

	// compute square root information upper matrix
	measurement_sqrt_information_upper_ = computeSqrtUpper(measurement_covariance_.inverse());
}

void FeatureBase::setMeasurementInformation(const Eigen::MatrixXs & _meas_info)
{
    WOLF_ASSERT_INFORMATION_MATRIX(_meas_info);

    // set (ensuring symmetry)
    measurement_covariance_ = _meas_info.inverse().selfadjointView<Eigen::Upper>();

    // Avoid singular covariance
    makePosDef(measurement_covariance_);

    // compute square root information upper matrix
    measurement_sqrt_information_upper_ = computeSqrtUpper(_meas_info);
}

void FeatureBase::setProblem(ProblemPtr _problem)
{
    NodeBase::setProblem(_problem);
    for (auto ctr : factor_list_)
        ctr->setProblem(_problem);
}

Eigen::MatrixXs FeatureBase::computeSqrtUpper(const Eigen::MatrixXs & _info) const
{
    // impose symmetry
    Eigen::MatrixXs info = _info.selfadjointView<Eigen::Upper>();

    // Normal Cholesky factorization
    Eigen::LLT<Eigen::MatrixXs> llt_of_info(info);
    Eigen::MatrixXs R = llt_of_info.matrixU();

    // Good factorization
    if (info.isApprox(R.transpose() * R, Constants::EPS))
        return R;

    // Not good factorization: SelfAdjointEigenSolver
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXs> es(info);
    Eigen::VectorXs eval = es.eigenvalues().real().cwiseMax(Constants::EPS);

    R = eval.cwiseSqrt().asDiagonal() * es.eigenvectors().real().transpose();

    return R;
}

} // namespace wolf
