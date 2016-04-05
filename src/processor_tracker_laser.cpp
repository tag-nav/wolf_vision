#include "processor_tracker_laser.h"


ProcessorTrackerLaser::ProcessorTrackerLaser::ProcessorTrackerLaser() :
        ProcessorTracker(PRC_TRACKER_LIDAR, true),
        scan_params_(((SensorLaser2D*)(upperNodePtr()))->getScanParams()),
        corner_alg_params_(((SensorLaser2D*)(upperNodePtr()))->getCornerAlgParams())
{
}

ProcessorTrackerLaser::ProcessorTrackerLaser::~ProcessorTrackerLaser()
{

}

unsigned int ProcessorTrackerLaser::detectNewFeatures()
{
    // TO UNCOMMENT WHEN track() DOESN'T PROCESS THE WHOLE SCAN

    /*
    std::list <laserscanutils::Corner> corners(0);

    laserscanutils::extractCorners(scan_params_, corner_alg_params_, scan_last_->getRanges(), corners);

    // TODO: Change to laserscanutils
    Eigen::Matrix4s cov_mat;
    Eigen::Matrix3s R = Eigen::Matrix3s::Identity();
    Eigen::Vector4s meas;

    //for each corner in the list create a feature
    for (auto corner_it = corners.begin(); corner_it != corners.end(); corner_it++)
    {
        meas.head(2) = corner_it->pt_.head(2);
        meas(2) = corner_it->orientation_;
        meas(3) = corner_it->aperture_;

        // TODO: maybe in line object?
        WolfScalar L1 = corner_it->line_1_.length();
        WolfScalar L2 = corner_it->line_2_.length();
        WolfScalar cov_angle_line1 = 12 * corner_it->line_1_.error_ / (pow(L1,2) * (pow(corner_it->line_1_.np_,3)-pow(corner_it->line_1_.np_,2)));
        WolfScalar cov_angle_line2 = 12 * corner_it->line_2_.error_ / (pow(L2,2) * (pow(corner_it->line_2_.np_,3)-pow(corner_it->line_2_.np_,2)));


        //init cov in corner coordinates
        cov_mat << corner_it->line_1_.error_ + cov_angle_line1 * L1 * L1 / 4, 0, 0, 0,
                   0, corner_it->line_2_.error_ + cov_angle_line2 * L2 * L2 / 4, 0, 0,
                   0, 0, cov_angle_line1 + cov_angle_line2, 0,
                   0, 0, 0, cov_angle_line1 + cov_angle_line2;

        //std::cout << "New feature: " << meas.transpose() << std::endl;
        //std::cout << cov_mat << std::endl;

        // Rotate covariance
        R.topLeftCorner<2,2>() = Eigen::Rotation2D<WolfScalar>(corner_it->orientation_).matrix();
        cov_mat.topLeftCorner<3,3>() = R.transpose() * cov_mat.topLeftCorner<3,3>() * R;

        //std::cout << "rotated covariance: " << std::endl;
        //std::cout << cov_mat << std::endl;

        // TODO: discard repeated features!!

        addNewFeature(new FeatureCorner2D(meas, 10*cov_mat));
    }
    */
    return 0;
}

unsigned int ProcessorTrackerLaser::track(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out)
{
    std::list <laserscanutils::Corner> corners(0);

        laserscanutils::extractCorners(scan_params_, corner_alg_params_, scan_last_->getRanges(), corners);

        // TODO: Change to laserscanutils
        Eigen::Matrix4s cov_mat;
        Eigen::Matrix3s R = Eigen::Matrix3s::Identity();
        Eigen::Vector4s meas;

        //for each corner in the list create a feature
        for (auto corner_it = corners.begin(); corner_it != corners.end(); corner_it++)
        {
            meas.head(2) = corner_it->pt_.head(2);
            meas(2) = corner_it->orientation_;
            meas(3) = corner_it->aperture_;

            // TODO: maybe in line object?
            WolfScalar L1 = corner_it->line_1_.length();
            WolfScalar L2 = corner_it->line_2_.length();
            WolfScalar cov_angle_line1 = 12 * corner_it->line_1_.error_ / (pow(L1,2) * (pow(corner_it->line_1_.np_,3)-pow(corner_it->line_1_.np_,2)));
            WolfScalar cov_angle_line2 = 12 * corner_it->line_2_.error_ / (pow(L2,2) * (pow(corner_it->line_2_.np_,3)-pow(corner_it->line_2_.np_,2)));


            //init cov in corner coordinates
            cov_mat << corner_it->line_1_.error_ + cov_angle_line1 * L1 * L1 / 4, 0, 0, 0,
                       0, corner_it->line_2_.error_ + cov_angle_line2 * L2 * L2 / 4, 0, 0,
                       0, 0, cov_angle_line1 + cov_angle_line2, 0,
                       0, 0, 0, cov_angle_line1 + cov_angle_line2;

            //std::cout << "New feature: " << meas.transpose() << std::endl;
            //std::cout << cov_mat << std::endl;

            // Rotate covariance
            R.topLeftCorner<2,2>() = Eigen::Rotation2D<WolfScalar>(corner_it->orientation_).matrix();
            cov_mat.topLeftCorner<3,3>() = R.transpose() * cov_mat.topLeftCorner<3,3>() * R;

            //std::cout << "rotated covariance: " << std::endl;
            //std::cout << cov_mat << std::endl;

            // TODO: discard repeated features!!

            addNewFeature(new FeatureCorner2D(meas, 10*cov_mat));
        }

    return 0;
}

LandmarkBase* ProcessorTrackerLaser::createLandmark(FeatureBase* _feature_ptr)
{
    return nullptr;
}

ConstraintBase* ProcessorTrackerLaser::createConstraint(FeatureBase* _feature_ptr, NodeBase* _node_ptr)
{
    return nullptr;
}

bool ProcessorTrackerLaser::voteForKeyFrame()
{
    return false;
}
