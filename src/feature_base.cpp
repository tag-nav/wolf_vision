#include "feature_base.h"
#include "constraint_base.h"
#include "capture_base.h"

namespace wolf {

unsigned int FeatureBase::feature_id_count_ = 0;

FeatureBase::FeatureBase(const std::string& _type, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	NodeBase("FEATURE", _type),
    capture_ptr_(),
    is_removing_(false),
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
        while (!constraint_list_.empty())
        {
            constraint_list_.front()->remove(); // remove downstream
        }
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove(); // remove constrained
        }
    }
}

ConstraintBasePtr FeatureBase::addConstraint(ConstraintBasePtr _co_ptr)
{
    constraint_list_.push_back(_co_ptr);
    _co_ptr->setFeaturePtr(shared_from_this());
    _co_ptr->setProblem(getProblem());
    // add constraint to be added in solver
    if (getProblem() != nullptr)
        getProblem()->addConstraintPtr(_co_ptr);
    else
        WOLF_TRACE("WARNING: ADDING CONSTRAINT ", _co_ptr->id(), " TO FEATURE ", this->id(), " NOT CONNECTED WITH PROBLEM.");
    return _co_ptr;
}

FrameBasePtr FeatureBase::getFramePtr() const
{
    return capture_ptr_.lock()->getFramePtr();
}

ConstraintBasePtr FeatureBase::addConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
    _ctr_ptr->setFeatureOtherPtr(shared_from_this());
    return _ctr_ptr;
}

ConstraintBaseList& FeatureBase::getConstraintList()
{
    return constraint_list_;
}

void FeatureBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
    _ctr_list.insert(_ctr_list.end(), constraint_list_.begin(), constraint_list_.end());
}

void FeatureBase::setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov)
{
    // Check symmetry (soft)
    assert((_meas_cov - _meas_cov.transpose()).cwiseAbs().maxCoeff() < Constants::EPS && "Not symmetric measurement covariance");

    // set (ensuring strong symmetry)
    measurement_covariance_ = _meas_cov.selfadjointView<Eigen::Upper>();

    // Avoid singular covariance
    avoidSingleCovariance();

	// compute square root information upper matrix
	measurement_sqrt_information_upper_ = computeSqrtUpper(measurement_covariance_.inverse());
}

void FeatureBase::setMeasurementInformation(const Eigen::MatrixXs & _meas_info)
{
    assert(_meas_info.determinant() > Constants::EPS_SMALL && "Not positive definite measurement information");
    assert((_meas_info - _meas_info.transpose()).cwiseAbs().maxCoeff() < Constants::EPS && "Not symmetric measurement information");

    // set (ensuring strong symmetry)
    measurement_covariance_ = _meas_info.inverse().selfadjointView<Eigen::Upper>();

    // Avoid singular covariance
    avoidSingleCovariance();

    // compute square root information upper matrix
    measurement_sqrt_information_upper_ = computeSqrtUpper(_meas_info);
}

Eigen::MatrixXs FeatureBase::computeSqrtUpper(const Eigen::MatrixXs & _info) const
{
    assert(_info.determinant() > Constants::EPS_SMALL && "Matrix is not positive definite!");
    assert((_info - _info.transpose()).cwiseAbs().maxCoeff() < Constants::EPS && "Matrix is not symmetric!");

    // impose symmetry
    Eigen::MatrixXs info = ((_info + _info.transpose()) / 2);
    info = info.selfadjointView<Eigen::Upper>();

    // Normal Cholesky factorization
    Eigen::LLT<Eigen::MatrixXs> llt_of_info(info);
    Eigen::MatrixXs R = llt_of_info.matrixU();

    // Good factorization
    if ((R.transpose() * R - info).cwiseAbs().maxCoeff() < Constants::EPS)
        return R;

    // Not good factorization: SelfAdjointEigenSolver
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXs> es(info);
    Eigen::VectorXs eval = es.eigenvalues().real().cwiseMax(0);

    R = eval.cwiseSqrt().asDiagonal() * es.eigenvectors().real().transpose();

    return R;
}

void FeatureBase::avoidSingleCovariance()
{
    Scalar eps_scalar = 1e-10;
    while (measurement_covariance_.determinant() < Constants::EPS_SMALL && eps_scalar < 1e-3)
    {
        measurement_covariance_ += Eigen::MatrixXs::Identity(measurement_covariance_.rows(), measurement_covariance_.cols()) * eps_scalar; // avoid singular covariance
        eps_scalar*=10;
    }
    assert(measurement_covariance_.determinant() > Constants::EPS_SMALL && "Couldn't avoid singular covariance");
}

} // namespace wolf
