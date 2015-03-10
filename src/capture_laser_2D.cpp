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
    std::list<Eigen::Vector4s> corners;
    
    //extract corners from range data
    extractCorners(corners);
    //std::cout << corners.size() << " corners extracted" << std::endl;
    
    //generate a feature for each corner
    //std::cout << "CaptureLaser2D::createFeatures..." << std::endl;
    createFeatures(corners);
    
    //Establish constraints for each feature
    //std::cout << "CaptureLaser2D::establishConstraints..." << std::endl;
    establishConstraints();
}

unsigned int CaptureLaser2D::extractCorners(std::list<Eigen::Vector4s> & _corner_list) const
{
    return laserscanutils::extractCorners(laser_ptr_->getScanParams(), laser_ptr_->getCornerAlgParams(), ranges_, _corner_list);
}

void CaptureLaser2D::createFeatures(std::list<Eigen::Vector4s> & _corner_list)
{
    Eigen::Matrix4s cov_mat;
    
    //init constant cov
    cov_mat << 0.01, 0,    0,    0,
    		   0,    0.01, 0,    0,
			   0,    0,    0.01, 0,
			   0,    0,    0,    0.01;
    
    //for each corner in the list create a feature
    for (auto corner_it = _corner_list.begin(); corner_it != _corner_list.end(); corner_it ++)
    {
    	(*corner_it)(2) = 0;
        this->addFeature( (FeatureBase*)(new FeatureCorner2D( (*corner_it), cov_mat ) ) );
    }
}

void CaptureLaser2D::establishConstraints()
{
	// Global transformation TODO: Change by a function
	Eigen::Vector2s t_robot(*getFramePtr()->getPPtr()->getPtr(),*(getFramePtr()->getPPtr()->getPtr()+1));
	WolfScalar o = *(getFramePtr()->getOPtr()->getPtr());
	Eigen::Matrix2s R_robot;
	if (getFramePtr()->getOPtr()->getStateType() == ST_THETA)
		R_robot << cos(o),-sin(o),
			 sin(o), cos(o);
    else
    {
    	//TODO
    }

	// Sensor transformation
	Eigen::Vector2s t_sensor = getSensorPtr()->getSensorPosition()->head(2);
	Eigen::Matrix2s R_sensor = getSensorPtr()->getSensorRotation()->topLeftCorner<2,2>().transpose();

    //Brute force closest (xy and theta) landmark search
//    std::cout << "Brute force closest (xy and theta) landmark search: N features:" << getFeatureListPtr()->size() << std::endl;
//    std::cout << "N landmark:" << getTop()->getMapPtr()->getLandmarkListPtr()->size() << std::endl;
//    std::cout << "Vehicle transformation: " << std::endl;
//	std::cout << "t: " << t.transpose() << std::endl;
//	std::cout << "rot:" << R << std::endl;
    for (auto feature_it = getFeatureListPtr()->begin(); feature_it != getFeatureListPtr()->end(); feature_it++ )
	{
		double max_distance_matching2 = 0.5; //TODO: max_distance_matching depending on localization and landmarks uncertainty
		double max_theta_matching = 0.2; //TODO: max_theta_matching depending on localization and landmarks uncertainty

		//Find the closest landmark to the feature
		LandmarkCorner2D* correspondent_landmark = nullptr;
    	Eigen::Map<Eigen::Vector2s> feature_position((*feature_it)->getMeasurementPtr()->data());
    	Eigen::Map<Eigen::Vector1s> feature_orientation = ((*feature_it)->getMeasurementPtr()->data()+2);

    	Eigen::Vector2s feature_global_position = R_robot * (R_sensor * feature_position + t_sensor) + t_robot;
    	Eigen::Vector1s feature_global_orientation;
    	feature_global_orientation(0) = feature_orientation(0) + o + atan2(R_sensor(1,0),R_sensor(0,0));
    	//feature_global_orientation(0) = 0;
    	double min_distance2 = max_distance_matching2;

//    	std::cout << "Feature: " << (*feature_it)->nodeId() << std::endl;
//    	std::cout << "local position: " << feature_position.transpose() << " orientation:" << feature_orientation << std::endl;
//    	std::cout << "global position:" << feature_global_position.transpose() << " orientation:" << feature_global_orientation << std::endl;
    	for (auto landmark_it = getTop()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != getTop()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++ )
		{
    		Eigen::Map<Eigen::Vector2s> landmark_position((*landmark_it)->getPPtr()->getPtr());
    		WolfScalar landmark_orientation = *((*landmark_it)->getOPtr()->getPtr());

    		WolfScalar distance2 = (landmark_position-feature_global_position).transpose() * (landmark_position-feature_global_position);
    		WolfScalar theta_distance = fabs(landmark_orientation-feature_global_orientation(0));

    		if (theta_distance > M_PI)
    			theta_distance -= 2 * M_PI;
			if (distance2 < min_distance2)
			{
				if (theta_distance < max_theta_matching)
				{
					//std::cout << "Position & orientation near landmark found: " << (*landmark_it)->nodeId() << std::endl;
					//std::cout << "global position:" << landmark_position.transpose() << " orientation:" << landmark_orientation << std::endl;

					correspondent_landmark = (LandmarkCorner2D*)(*landmark_it);
					min_distance2 = distance2;
				}
				else
				{
//					std::cout << "Feature: " << (*feature_it)->nodeId() << std::endl;
//					std::cout << "global position:" << feature_global_position.transpose() << " orientation:" << feature_global_orientation << std::endl;
//					std::cout << "Landmark with near position but wrong orientation: " << (*landmark_it)->nodeId() << std::endl;
//					std::cout << "global position:" << landmark_position.transpose() << " orientation:" << landmark_orientation << std::endl;
				}
			}
		}
    	if (correspondent_landmark == nullptr)
    	{
    		//std::cout << "No landmark found. Creating a new one..." << std::endl;
    		StateBase* new_landmark_state_position = new StatePoint2D(getTop()->getNewStatePtr());
    		getTop()->addState(new_landmark_state_position, feature_global_position);
    		StateBase* new_landmark_state_orientation = new StateTheta(getTop()->getNewStatePtr());
    		getTop()->addState(new_landmark_state_orientation, feature_global_orientation);

    		correspondent_landmark = new LandmarkCorner2D(new_landmark_state_position, new_landmark_state_orientation);
    		LandmarkBase* corr_landmark(correspondent_landmark);
    		getTop()->getMapPtr()->addLandmark(corr_landmark);

    		//std::cout << "Landmark created: " << getTop()->getMapPtr()->getLandmarkListPtr()->back()->nodeId() << std::endl;
			//std::cout << "global position: " << *corr_landmark->getPPtr()->getPtr() << " " << *(corr_landmark->getPPtr()->getPtr()+1) << " orientation:" << *corr_landmark->getOPtr()->getPtr() << std::endl;
    	}
    	else
    		correspondent_landmark->hit();

    	//std::cout << "Creating new constraint: Landmark " << getTop()->getMapPtr()->getLandmarkListPtr()->back()->nodeId() << " & feature " << (*feature_it)->nodeId() << std::endl;

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

