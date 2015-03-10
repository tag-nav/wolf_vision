#include "capture_odom_2D.h"

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _ts, SensorBase* _sensor_ptr) :
    CaptureRelative(_ts, _sensor_ptr)
{
    //
}

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureRelative(_ts, _sensor_ptr, _data)
{
	//
}

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureRelative(_ts, _sensor_ptr, _data, _data_covariance)
{
	//
}

CaptureOdom2D::~CaptureOdom2D()
{
	//std::cout << "Destroying GPS fix capture...\n";
}

inline void CaptureOdom2D::processCapture()
{
    //std::cout << "CaptureOdom2D::addFeature..." << std::endl;
	// ADD FEATURE
    addFeature(new FeatureOdom2D(data_,data_covariance_));

    //std::cout << "CaptureOdom2D::addConstraints..." << std::endl;
    // ADD CONSTRAINT
    addConstraints();
}

Eigen::VectorXs CaptureOdom2D::computePrior() const
{
	assert(up_node_ptr_ != nullptr && "This Capture is not linked to any frame");

	if (getFramePtr()->getOPtr()->getStateType() == ST_COMPLEX_ANGLE)
	{
		Eigen::VectorXs prior(4);
		Eigen::Map<Eigen::VectorXs> initial_pose(getFramePtr()->getPPtr()->getPtr(), 4);

		double prior_2 = initial_pose(2) * cos(data_(1)) - initial_pose(3) * sin(data_(1));
		double prior_3 = initial_pose(2) * sin(data_(1)) + initial_pose(3) * cos(data_(1));
		prior(0) = initial_pose(0) + data_(0) * prior_2;
		prior(1) = initial_pose(1) + data_(0) * prior_3;
		prior(2) = prior_2;
		prior(3) = prior_3;

		return prior;
	}
	else
	{
		Eigen::VectorXs prior(3);
		Eigen::Map<Eigen::VectorXs> initial_pose(getFramePtr()->getPPtr()->getPtr(), 3);

		prior(0) = initial_pose(0) + data_(0) * cos(initial_pose(2) + (data_(1)));
		prior(1) = initial_pose(1) + data_(0) * sin(initial_pose(2) + (data_(1)));
		prior(2) = initial_pose(2) + data_(1);

		return prior;
	}
}

void CaptureOdom2D::addConstraints()
{
	assert(getFramePtr()->getNextFrame() != nullptr && "Trying to add odometry constraint in the last frame (there is no next frame)");

	if (getFramePtr()->getOPtr()->getStateType() == ST_COMPLEX_ANGLE)
	{
		getFeatureListPtr()->front()->addConstraint(new ConstraintOdom2DComplexAngle(getFeatureListPtr()->front(),
																					 getFramePtr()->getPPtr(),
																					 getFramePtr()->getOPtr(),
																					 getFramePtr()->getNextFrame()->getPPtr(),
																					 getFramePtr()->getNextFrame()->getOPtr()));
	}
	else
	{
		getFeatureListPtr()->front()->addConstraint(new ConstraintOdom2DTheta(getFeatureListPtr()->front(),
																			  getFramePtr()->getPPtr(),
																			  getFramePtr()->getOPtr(),
																			  getFramePtr()->getNextFrame()->getPPtr(),
																			  getFramePtr()->getNextFrame()->getOPtr()));
	}
}

void CaptureOdom2D::integrateCapture(CaptureRelative* _new_capture)
{
	//std::cout << "Trying to integrate CaptureOdom2D" << std::endl;
	assert(dynamic_cast<CaptureOdom2D*>(_new_capture) && "Trying to integrate with a CaptureOdom2D a CaptureRelativePtr which is not CaptureOdom2D");
	data_(0) += _new_capture->getData()(0);
	data_(1) += _new_capture->getData()(1);
	data_covariance_ += _new_capture->getDataCovariance();
	//std::cout << "integrated!" << std::endl;
}

//void CaptureOdom2D::printSelf(unsigned int _ntabs, std::ostream & _ost) const
//{
//    NodeLinked::printSelf(_ntabs, _ost);
//    //printTabs(_ntabs);
//    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
//    //printNTabs(_ntabs);
//    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
//}



