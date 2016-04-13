#include "processor_tracker_laser.h"

namespace wolf
{

void ProcessorTrackerLaser::extractCorners(CaptureLaser2D* _capture_laser_ptr,
                                                  std::list<FeatureCorner2D*> _corner_list)
{
    std::cout << "Extracting corners..." << std::endl;
    //variables
    std::list<laserscanutils::Corner> corners;
    //extract corners from range data
    laserscanutils::extractCorners(scan_params_, corner_alg_params_, _capture_laser_ptr->getRanges(), corners);
    //std::cout << corners.size() << " corners extracted" << std::endl;
    Eigen::Matrix4s measurement_cov;
    Eigen::Matrix3s R = Eigen::Matrix3s::Identity();
    Eigen::Vector4s measurement;
    for (auto corner : corners)
    {
        measurement.head(2) = corner.pt_.head(2);
        measurement(2) = corner.orientation_;
        measurement(3) = corner.aperture_;
        // TODO: maybe in line object?
        WolfScalar L1 = corner.line_1_.length();
        WolfScalar L2 = corner.line_2_.length();
        WolfScalar cov_angle_line1 = 12 * corner.line_1_.error_
                / (pow(L1, 2) * (pow(corner.line_1_.np_, 3) - pow(corner.line_1_.np_, 2)));
        WolfScalar cov_angle_line2 = 12 * corner.line_2_.error_
                / (pow(L2, 2) * (pow(corner.line_2_.np_, 3) - pow(corner.line_2_.np_, 2)));
        //init cov in corner coordinates
        measurement_cov << corner.line_1_.error_ + cov_angle_line1 * L1 * L1 / 4, 0, 0, 0, 0, corner.line_2_.error_
                + cov_angle_line2 * L2 * L2 / 4, 0, 0, 0, 0, cov_angle_line1 + cov_angle_line2, 0, 0, 0, 0, cov_angle_line1
                + cov_angle_line2;
        measurement_cov = 10 * measurement_cov;
        //std::cout << "New feature: " << meas.transpose() << std::endl;
        //std::cout << measurement_cov << std::endl;
        // Rotate covariance
        R.topLeftCorner<2, 2>() = Eigen::Rotation2Ds(corner.orientation_).matrix();
        measurement_cov.topLeftCorner<3, 3>() = R.transpose() * measurement_cov.topLeftCorner<3, 3>() * R;
        //std::cout << "rotated covariance: " << std::endl;
        //std::cout << measurement_cov << std::endl;
        _corner_list.push_back(new FeatureCorner2D(measurement, measurement_cov));
    }
}

}        //namespace wolf
