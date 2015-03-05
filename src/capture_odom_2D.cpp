#include "capture_odom_2D.h"

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr) :
    CaptureRelative(_ts, _sensor_ptr)
{
    //
}

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureRelative(_ts, _sensor_ptr, _data)
{
	//
}

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
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
	// ADD FEATURE
    //FeatureBaseShPtr new_feature(new FeatureOdom2D(CaptureBasePtr(this),data_,data_covariance_));
    FeatureBaseShPtr new_feature(new FeatureOdom2D(data_,data_covariance_));
    addFeature(new_feature);

    // ADD CONSTRAINT
    addConstraints();
}

Eigen::VectorXs CaptureOdom2D::computePrior() const
{
	assert(up_node_ptr_ != nullptr && "This Capture is not linked to any frame");

	FrameBasePtr _previous_frame = getFramePtr()->getPreviousFrame();

	if (_previous_frame->getOPtr()->getStateType() == ST_COMPLEX_ANGLE)
	{
		Eigen::VectorXs pose_predicted(4);
		Eigen::Map<Eigen::VectorXs> previous_pose(_previous_frame->getPPtr()->getPtr(), 4);

		double new_pose_predicted_2 = previous_pose(2) * cos(data_(1)) - previous_pose(3) * sin(data_(1));
		double new_pose_predicted_3 = previous_pose(2) * sin(data_(1)) + previous_pose(3) * cos(data_(1));
		pose_predicted(0) = previous_pose(0) + data_(0) * new_pose_predicted_2;
		pose_predicted(1) = previous_pose(1) + data_(0) * new_pose_predicted_3;
		pose_predicted(2) = new_pose_predicted_2;
		pose_predicted(3) = new_pose_predicted_3;

		return pose_predicted;
	}
	else
	{
		Eigen::VectorXs pose_predicted(3);
		Eigen::Map<Eigen::VectorXs> previous_pose(_previous_frame->getPPtr()->getPtr(), 3);

		pose_predicted(0) = previous_pose(0) + data_(0) * cos(previous_pose(2) + (data_(1)));
		pose_predicted(1) = previous_pose(1) + data_(0) * sin(previous_pose(2) + (data_(1)));
		pose_predicted(2) = previous_pose(2) + data_(1);

		return pose_predicted;
	}
}

void CaptureOdom2D::addConstraints()
{
	if (getFramePtr()->getOPtr()->getStateType() == ST_COMPLEX_ANGLE)
	{
		ConstraintBaseShPtr odom_constraint(new ConstraintOdom2DComplexAngle(getFeatureListPtr()->front().get(),
																			 getFramePtr()->getPreviousFrame()->getPPtr(),
																			 getFramePtr()->getPreviousFrame()->getOPtr(),
																			 getFramePtr()->getPPtr(),
																			 getFramePtr()->getOPtr()));
//         ConstraintBaseShPtr odom_constraint(new ConstraintOdom2DComplexAngle(getFramePtr()->getPreviousFrame()->getPPtr(),
//                                                                              getFramePtr()->getPreviousFrame()->getOPtr(),
//                                                                              getFramePtr()->getPPtr(),
//                                                                              getFramePtr()->getOPtr()));
		getFeatureListPtr()->front()->addConstraint(odom_constraint);
	}
	else
	{
		ConstraintBaseShPtr odom_constraint(new ConstraintOdom2DTheta(getFeatureListPtr()->front().get(),
																	  getFramePtr()->getPreviousFrame()->getPPtr(),
																	  getFramePtr()->getPreviousFrame()->getOPtr(),
																	  getFramePtr()->getPPtr(),
																	  getFramePtr()->getOPtr()));
//         ConstraintBaseShPtr odom_constraint(new ConstraintOdom2DTheta(getFramePtr()->getPreviousFrame()->getPPtr(),
//                                                                       getFramePtr()->getPreviousFrame()->getOPtr(),
//                                                                       getFramePtr()->getPPtr(),
//                                                                       getFramePtr()->getOPtr()));
		getFeatureListPtr()->front()->addConstraint(odom_constraint);
	}
}

void CaptureOdom2D::integrateCapture(const CaptureRelativePtr _new_capture)
{
	//std::cout << "Trying to integrate CaptureOdom2D" << std::endl;
	assert(dynamic_cast<CaptureOdom2D*>(_new_capture) && "Trying to integrate with a CaptureOdom2D a CaptureRelativePtr which is not CaptureOdom2D");
	data_(0) += _new_capture->getData()(0);
	data_(1) += _new_capture->getData()(1);
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



