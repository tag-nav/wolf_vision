#include "feature_gps_pseudorange.h"

FeatureGPSPseudorange::FeatureGPSPseudorange(Eigen::Vector3s &_sat_position, WolfScalar _pseudorange, WolfScalar _covariance) :
        FeatureBase(Eigen::VectorXs::Constant(1,_pseudorange), Eigen::MatrixXs::Identity(1,1)*_covariance),
        sat_position_(_sat_position),
        pseudorange_(_pseudorange)
{
//    std::cout << "FeatureGPSPseudorange() " << std::setprecision(12)
//              << "   --pr=" << pseudorange_
//              << "\t--pos("  << sat_position_[0] << ", " << sat_position_[1] << ", " << sat_position_[2] << ")"
//              << "   --covariance =" << getMeasurementCovariance()
//              << std::endl;
}

FeatureGPSPseudorange::~FeatureGPSPseudorange()
{

}

WolfScalar FeatureGPSPseudorange::getPseudorange() const
{
    return pseudorange_;
}

const Eigen::Vector3s &FeatureGPSPseudorange::getSatPosition() const
{
    return sat_position_;
}