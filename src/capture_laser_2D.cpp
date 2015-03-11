#include "capture_laser_2D.h"


// unsigned int CaptureLaser2D::segment_window_size = 8;//window size to extract segments
// double CaptureLaser2D::theta_min = 0.4; //minimum theta between consecutive segments to detect corner. PI/8=0.39
// double CaptureLaser2D::theta_max_parallel = 0.1; //maximum theta between consecutive segments to fuse them in a single line.
// double CaptureLaser2D::k_sigmas = 3.;//How many std_dev are tolerated to count that a point is supporting a line
// unsigned int CaptureLaser2D::max_beam_distance = 5;//max number of beams of distance between lines to consider corner or concatenation
// double CaptureLaser2D::max_distance = 0.5;//max distance between line ends to consider corner or concatenation

CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _ranges):
	CaptureBase(_ts, _sensor_ptr, _ranges),
	ranges_(data_.data(), _ranges.size()),
	intensities_(data_.data(), 0)
{
    laser_ptr_ = (SensorLaser2D*)sensor_ptr_;
}

CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _ranges, const Eigen::VectorXs& _intensities):
		CaptureBase(_ts, _sensor_ptr, _ranges),
		ranges_(data_.data(), _ranges.size()),
		intensities_(data_.data(), _intensities.size())
{
    laser_ptr_ = (SensorLaser2D*)sensor_ptr_;
}

CaptureLaser2D::~CaptureLaser2D()
{
    //
}

void CaptureLaser2D::processCapture()
{
    //variables
    //std::list<Eigen::Vector4s> corners;
    std::list<laserscanutils::Corner> corners;
    
    //extract corners from range data
    extractCorners(corners);
    //std::cout << corners.size() << " corners extracted" << std::endl;
    
    //generate a feature for each corner
    createFeatures(corners);
    //std::cout << getFeatureListPtr()->size() << " Features created" << std::endl;
    
    //Establish constraints for each feature
    establishConstraints();
    //std::cout << "Constraints created" << std::endl;
}

unsigned int CaptureLaser2D::extractCorners(std::list<laserscanutils::Corner> & _corner_list) const
{
    return laserscanutils::extractCorners(laser_ptr_->getScanParams(), laser_ptr_->getCornerAlgParams(), ranges_, _corner_list);
}

void CaptureLaser2D::createFeatures(std::list<laserscanutils::Corner> & _corner_list)
{
	// TODO: Sensor model
    Eigen::Matrix4s cov_mat;
    Eigen::Vector4s meas;
    
    //init constant cov
    cov_mat << 0.01, 0,    0,    0,
    		   0,    0.01, 0,    0,
			   0,    0,    0.01, 0,
			   0,    0,    0,    0.01;
    
    //for each corner in the list create a feature
    for (auto corner_it = _corner_list.begin(); corner_it != _corner_list.end(); corner_it ++)
    {
    	meas.head(2) = (*corner_it).pt_.head(2);
    	meas(2) = (*corner_it).orientation_;
    	meas(3) = (*corner_it).aperture_;
    	//TODO: add the rest of descriptors and struct atributes
        this->addFeature( (FeatureBase*)(new FeatureCorner2D( meas, cov_mat ) ) );
    }
}

void CaptureLaser2D::establishConstraints()
{
	// Global transformation TODO: Change by a function
	Eigen::Vector2s t_robot(*getFramePtr()->getPPtr()->getPtr(),*(getFramePtr()->getPPtr()->getPtr()+1));
	Eigen::Matrix2s R_robot = ((StateOrientation*)(getFramePtr()->getOPtr()))->getRotationMatrix().topLeftCorner<2,2>();
	WolfScalar& robot_orientation = *(getFramePtr()->getOPtr()->getPtr());

	// Sensor transformation
	Eigen::Vector2s t_sensor = getSensorPtr()->getSensorPosition()->head(2);
	Eigen::Matrix2s R_sensor = getSensorPtr()->getSensorRotation()->topLeftCorner<2,2>();

    //Brute force closest (xy and theta) landmark search //TODO: B&B
//    std::cout << "Brute force closest (xy and theta) landmark search: N features:" << getFeatureListPtr()->size() << std::endl;
//    std::cout << "N landmark:" << getTop()->getMapPtr()->getLandmarkListPtr()->size() << std::endl;
//    std::cout << "Vehicle transformation: " << std::endl;
//	std::cout << "t: " << t.transpose() << std::endl;
//	std::cout << "rot:" << R << std::endl;
    for (auto feature_it = getFeatureListPtr()->begin(); feature_it != getFeatureListPtr()->end(); feature_it++ )
	{
    	WolfScalar max_distance_matching2 = 0.5;
    	WolfScalar max_theta_matching = M_PI / 8;

		//Find the closest landmark to the feature
		LandmarkCorner2D* correspondent_landmark = nullptr;
		const Eigen::Vector2s& feature_position = (*feature_it)->getMeasurement().head(2);
    	const WolfScalar& feature_orientation = (*feature_it)->getMeasurement()(2);
    	const WolfScalar& feature_aperture = (*feature_it)->getMeasurement()(3);

    	Eigen::Vector2s feature_global_position = R_robot * (R_sensor * feature_position + t_sensor) + t_robot;
    	WolfScalar feature_global_orientation = feature_orientation + robot_orientation + atan2(R_sensor(1,0),R_sensor(0,0));
    	// fit in (-pi, pi]
    	feature_global_orientation = (feature_global_orientation > 0 ? fmod(feature_global_orientation+M_PI, 2 * M_PI)-M_PI : fmod(feature_global_orientation-M_PI, 2 * M_PI)+M_PI);
    	//feature_global_orientation(0) = 0;
    	double min_distance2 = max_distance_matching2;

    	std::cout << "Feature: " << (*feature_it)->nodeId() << std::endl;
//    	std::cout << "local position: " << feature_position.transpose() << " orientation:" << feature_orientation << std::endl;
    	std::cout << "global position: " << feature_global_position.transpose() <<
    				 " orientation: " << feature_global_orientation <<
    				 " aperture:" << feature_aperture << std::endl;
    	for (auto landmark_it = getTop()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != getTop()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++ )
		{
    		Eigen::Map<Eigen::Vector2s> landmark_position((*landmark_it)->getPPtr()->getPtr());
    		std::cout << "landmark: " << (*landmark_it)->nodeId() << " " << landmark_position.transpose();
    		WolfScalar& landmark_orientation = *((*landmark_it)->getOPtr()->getPtr());
    		std::cout << " orientation: " << landmark_orientation << std::endl;
    		const WolfScalar& landmark_aperture = (*landmark_it)->getDescriptor()(0);

    		WolfScalar distance2 = (landmark_position-feature_global_position).transpose() * (landmark_position-feature_global_position);
    		WolfScalar theta_distance = fabs(landmark_orientation-feature_global_orientation);

    		if (theta_distance > M_PI)
    			theta_distance -= 2 * M_PI;
			if (distance2 < min_distance2)
			{
				if (theta_distance < max_theta_matching)
				{
					std::cout << "Position & orientation near landmark found: " << (*landmark_it)->nodeId() << std::endl;
					std::cout << "global position: " << landmark_position.transpose() << " orientation: " << landmark_orientation << std::endl;

					correspondent_landmark = (LandmarkCorner2D*)(*landmark_it);
					min_distance2 = distance2;
				}
				else
				{
//					std::cout << "Feature: " << (*feature_it)->nodeId() << std::endl;
//					std::cout << "global position:" << feature_global_position.transpose() <<
//								 " orientation:" << feature_global_orientation <<
//								 " aperture:" << feature_aperture << std::endl;
//					std::cout << "Landmark with near position but wrong orientation: " << (*landmark_it)->nodeId() << std::endl;
//					std::cout << "global position:" << landmark_position.transpose() <<
//								 " orientation:" << landmark_orientation <<
//								 " aperture:" << landmark_aperture << std::endl;
				}
			}
		}
    	if (correspondent_landmark == nullptr)
    	{
    		std::cout << "No landmark found. Creating a new one..." << std::endl;
    		StateBase* new_landmark_state_position = new StatePoint2D(getTop()->getNewStatePtr());
    		getTop()->addState(new_landmark_state_position, feature_global_position);
    		StateBase* new_landmark_state_orientation = new StateTheta(getTop()->getNewStatePtr());
    		getTop()->addState(new_landmark_state_orientation, Eigen::Map<Eigen::Vector1s>(&feature_global_orientation,1));

    		correspondent_landmark = new LandmarkCorner2D(new_landmark_state_position, new_landmark_state_orientation, feature_aperture);
    		LandmarkBase* corr_landmark(correspondent_landmark);
    		getTop()->getMapPtr()->addLandmark(corr_landmark);

//    		std::cout << "Landmark created: " << getTop()->getMapPtr()->getLandmarkListPtr()->back()->nodeId() << std::endl;
//			std::cout << "global position: " << *corr_landmark->getPPtr()->getPtr() << " " << *(corr_landmark->getPPtr()->getPtr()+1) <<
//						 " orientation: " << *corr_landmark->getOPtr()->getPtr() <<
//						 " aperture:" << corr_landmark->getDescriptor()(0) <<  std::endl;
    	}
    	else
    		correspondent_landmark->hit();

//    	std::cout << "Creating new constraint: Landmark " << getTop()->getMapPtr()->getLandmarkListPtr()->back()->nodeId() << " & feature " << (*feature_it)->nodeId() << std::endl;

    	// Add constraint to the correspondent landmark
    	(*feature_it)->addConstraint(new ConstraintCorner2DTheta(*feature_it,
																 correspondent_landmark,
																 getFramePtr()->getPPtr(),//_robotPPtr,
																 getFramePtr()->getOPtr(),//_robotOPtr,
																 correspondent_landmark->getPPtr(), //_landmarkPPtr,
																 correspondent_landmark->getOPtr())); //_landmarkOPtr,
	}
}

Eigen::VectorXs CaptureLaser2D::computePrior() const
{
    return Eigen::Vector3s(1,2,3);
}

