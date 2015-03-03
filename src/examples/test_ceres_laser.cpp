//std includes
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//Ceres includes
#include "ceres/ceres.h"
#include "glog/logging.h"

//Wolf includes
#include "wolf.h"
#include "sensor_base.h"
#include "sensor_odom_2D.h"
#include "sensor_gps_fix.h"
#include "feature_base.h"
#include "frame_base.h"
#include "state_point.h"
#include "state_complex_angle.h"
#include "capture_base.h"
#include "capture_relative.h"
#include "capture_odom_2D.h"
#include "capture_gps_fix.h"
#include "capture_laser_2D.h"
#include "state_base.h"
#include "constraint_sparse.h"
#include "constraint_gps_2D.h"
#include "constraint_odom_2D_theta.h"
#include "constraint_odom_2D_complex_angle.h"
#include "trajectory_base.h"
#include "map_base.h"
#include "wolf_problem.h"

// ceres wrapper include
#include "ceres_wrapper/ceres_manager.h"

//C includes for sleep, time and main args
#include "unistd.h"
#include <time.h>
#include <sys/time.h>

//GLUT
// #include <GL/glut.h>

//faramotics includes
//#include "faramotics/window.h"
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

//namespaces
using namespace std;

//function travel around
void motionCampus(unsigned int ii, Cpose3d & pose, double& displacement_, double& rotation_)
{
    if (ii<=120)
    {
    	displacement_ = 0.1;
    	rotation_ = 0;
    }
    else if ( (ii>120) && (ii<=170) )
    {
    	displacement_ = 0.2;
    	rotation_ = 1.8*M_PI/180;
    }
    else if ( (ii>170) && (ii<=220) )
    {
    	displacement_ = 0;
    	rotation_ = -1.8*M_PI/180;
    }
    else if ( (ii>220) && (ii<=310) )
    {
    	displacement_ = 0.1;
    	rotation_ = 0;
    }
    else if ( (ii>310) && (ii<=487) )
    {
    	displacement_ = 0.1;
    	rotation_ = -1.*M_PI/180;
    }
    else if ( (ii>487) && (ii<=600) )
    {
    	displacement_ = 0.2;
    	rotation_ = 0;
    }
    else if ( (ii>600) && (ii<=700) )
    {
    	displacement_ = 0.1;
    	rotation_ = -1.*M_PI/180;
    }
    else  if ( (ii>700) && (ii<=780) )
    {
    	displacement_ = 0;
    	rotation_ = -1.*M_PI/180;
    }
    else
    {
    	displacement_ = 0.3;
    	rotation_ = 0.0*M_PI/180;
    }

    pose.moveForward(displacement_);
    pose.rt.setEuler( pose.rt.head()+rotation_, pose.rt.pitch(), pose.rt.roll() );
}

class WolfManager
{
    protected:
		bool use_complex_angles_;
		WolfProblemPtr problem_;
        std::vector<Eigen::VectorXs> odom_captures_;
        std::vector<Eigen::VectorXs> gps_captures_;
        std::queue<CaptureBaseShPtr> new_captures_;
        SensorBasePtr sensor_prior_;
        unsigned int last_state_units_, window_size_;
        FrameBaseIter first_window_frame_;

    public:
        WolfManager(const SensorBasePtr& _sensor_prior, const bool _complex_angle, const unsigned int& _state_length, const unsigned int& _w_size=10) :
        	use_complex_angles_(_complex_angle),
			problem_(new WolfProblem(_state_length)),
			sensor_prior_(_sensor_prior),
			last_state_units_(0),
			window_size_(_w_size)
		{
        	Eigen::VectorXs init_frame(use_complex_angles_ ? 4 : 3);
        	if (use_complex_angles_)
        		init_frame << 2, 8, 1, 0;
        	else
        		init_frame << 2, 8, 0;
        	createFrame(init_frame, 0);
		}

        virtual ~WolfManager()
        {
        	delete problem_;
        }

        void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
        {
        	// Create frame and add it to the trajectory
        	StateBaseShPtr new_position(new StatePoint2D(problem_->getNewStatePtr()));
			problem_->addState(new_position, _frame_state.head(2));

			StateBaseShPtr new_orientation;
        	if (use_complex_angles_)
        	{
        		new_orientation = StateBaseShPtr(new StateComplexAngle(problem_->getNewStatePtr()));
        		problem_->addState(new_orientation, _frame_state.tail(2));
        	}
        	else
        	{
        		new_orientation = StateBaseShPtr(new StateTheta(problem_->getNewStatePtr()));
				problem_->addState(new_orientation, _frame_state.tail(1));
        	}

			problem_->getTrajectoryPtr()->addFrame(FrameBaseShPtr(new FrameBase(_time_stamp, new_position, new_orientation)));
        }

        void addCapture(const CaptureBaseShPtr& _capture)
        {
        	new_captures_.push(_capture);
        }

        void update(StateBasePtrList& new_state_units, ConstraintBasePtrList& new_constraints, StateBasePtrList& updated_state_units, std::list<WolfScalar*>& removed_state_units, std::list<unsigned int>& removed_constraints_idx)
        {
        	// TODO: management due to time stamps
        	while (!new_captures_.empty())
        	{
        		// EXTRACT NEW CAPTURE
        		CaptureBaseShPtr new_capture = new_captures_.front();
        		new_captures_.pop();

        		// NEW FRAME (if the specific sensor)
        		// TODO: accumulate odometries
        		if (new_capture->getSensorPtr() == sensor_prior_)
        		{
					createFrame(Eigen::VectorXs::Zero(use_complex_angles_ ? 4 : 3), new_capture->getTimeStamp());

					// ADD CAPTURE TO THE NEW FRAME
					//trajectory_->getFrameListPtr()->back()->addCapture(new_capture);
					problem_->getTrajectoryPtr()->getFrameListPtr()->back()->addCapture(new_capture);

					// COMPUTE PRIOR
        			//trajectory_->getFrameListPtr()->back()->setState(new_capture->computePrior());
					problem_->getTrajectoryPtr()->getFrameListPtr()->back()->setState(new_capture->computePrior());

					// WINDOW (remove old frames) TODO: què va aquí i què als destructors
					if (problem_->getTrajectoryPtr()->getFrameListPtr()->size() > window_size_)
					{
						//removeFrame(problem_->getTrajectoryPtr()->getFrameListPtr()->front().get(), removed_state_units, removed_constraints_idx);
						fixFrame((*first_window_frame_).get(), updated_state_units);
						first_window_frame_++;
					}
					else
						first_window_frame_ = problem_->getTrajectoryPtr()->getFrameListPtr()->begin();
        		}
        		else
        		{
        			// ADD CAPTURE TO THE LAST FRAME
					//trajectory_->getFrameListPtr()->back()->addCapture(new_capture);
        			problem_->getTrajectoryPtr()->getFrameListPtr()->back()->addCapture(new_capture);
        		}

        		// COMPUTE CAPTURE (features, constraints)
        		new_capture->processCapture();

        		// ADD NEW CORRESPONDENCES TO THE new_constraints OUTPUT PARAM
        		for (FeatureBaseIter feature_list_iter=new_capture->getFeatureListPtr()->begin(); feature_list_iter!=new_capture->getFeatureListPtr()->end(); feature_list_iter++)
					for (ConstraintBaseIter constraint_list_iter=(*feature_list_iter)->getConstraintListPtr()->begin(); constraint_list_iter!=(*feature_list_iter)->getConstraintListPtr()->end(); constraint_list_iter++)
						new_constraints.push_back((*constraint_list_iter).get());

        		// ADD NEW STATE UNITS TO THE new_state_units OUTPUT PARAM
        		auto state_unit_it=problem_->getStateListPtr()->rbegin();
        		while (last_state_units_ != problem_->getStateListPtr()->size())
        		{
        			new_state_units.push_back((*state_unit_it).get());
        			state_unit_it++;
        			last_state_units_++;
        		}
        	}
        	//std::cout << "All captures processed!" << std::endl;
        }

        void removeFrame(FrameBasePtr _removed_frame, std::list<WolfScalar*>& removed_state_units, std::list<unsigned int>& removed_constraints_idx)
        {
			//std::cout << "Removing frame " << removed_frame->nodeId() << std::endl;
			//removed_frame->print();

			// Constraints
			for (auto capture_list_iter=_removed_frame->getCaptureListPtr()->begin(); capture_list_iter!=_removed_frame->getCaptureListPtr()->end(); capture_list_iter++)
			{
				for (auto feature_list_iter=(*capture_list_iter)->getFeatureListPtr()->begin(); feature_list_iter!=(*capture_list_iter)->getFeatureListPtr()->end(); feature_list_iter++)
				{
					for (auto constraint_list_iter=(*feature_list_iter)->getConstraintListPtr()->begin(); constraint_list_iter!=(*feature_list_iter)->getConstraintListPtr()->end(); constraint_list_iter++)
					{
						//std::cout << "\tRemoving constraint " << (*constraint_list_iter)->nodeId() << std::endl;
						// Store constraint to remove in ceres
						removed_constraints_idx.push_back((*constraint_list_iter)->nodeId());

						// Check if there will be landmarks unconstrained
						if ((*constraint_list_iter)->getConstraintType() == CTR_CORNER_2D_THETA)
						{
							// TODO:  constraint_landmark_base class o hasLandmark() virtual a constraint_base (lmk_ptr_, getHits()..)
							LandmarkBasePtr ldmk_ptr = static_cast<ConstraintCorner2DTheta*>(constraint_list_iter->get())->getLandmarkPtr();
							//std::cout << "\t\tLandmark " << ldmk_ptr->nodeId() << " hits: " << ldmk_ptr->getHits() << std::endl;

							if (ldmk_ptr->getHits() == 1)
							{
								//std::cout << "\t\tRemoving landmark " << ldmk_ptr->nodeId() << std::endl;
								// Frame State Units
								if (ldmk_ptr->getPPtr())
									removed_state_units.push_back(ldmk_ptr->getPPtr()->getPtr());
								if (ldmk_ptr->getOPtr())
									removed_state_units.push_back(ldmk_ptr->getOPtr()->getPtr());
								if (ldmk_ptr->getVPtr())
									removed_state_units.push_back(ldmk_ptr->getVPtr()->getPtr());
								if (ldmk_ptr->getWPtr())
									removed_state_units.push_back(ldmk_ptr->getWPtr()->getPtr());

								problem_->getMapPtr()->removeLandmark(ldmk_ptr);
							}
							else
								ldmk_ptr->unHit();
						}
					}
				}
			}
			// Frame State Units
			//std::cout << "\tRemoving state units " << std::endl;
			if (_removed_frame->getPPtr())
				removed_state_units.push_back(_removed_frame->getPPtr()->getPtr());
			if (_removed_frame->getOPtr())
				removed_state_units.push_back(_removed_frame->getOPtr()->getPtr());
			if (_removed_frame->getVPtr())
				removed_state_units.push_back(_removed_frame->getVPtr()->getPtr());
			if (_removed_frame->getWPtr())
				removed_state_units.push_back(_removed_frame->getWPtr()->getPtr());

			// Destruct frame node
			//std::cout << "\tRemoving frame from trajectory..." << std::endl;
			problem_->getTrajectoryPtr()->getFrameListPtr()->erase(problem_->getTrajectoryPtr()->getFrameListPtr()->begin());
			//for (auto it = problem_->getTrajectoryPtr()->getFrameListPtr()->begin(); it != problem_->getTrajectoryPtr()->getFrameListPtr()->end(); it++)
			//	(*it)->printSelf();
		}

        void fixFrame(FrameBasePtr _fixed_frame, StateBasePtrList& updated_state_units)
		{
			//std::cout << "Fixing frame " << _fixed_frame->nodeId() << std::endl;
			//_fixed_frame->print();
        	_fixed_frame->setStatus(ST_FIXED);

			// Frame State Units
			if (_fixed_frame->getPPtr())
			{
				_fixed_frame->getPPtr()->setStateStatus(ST_FIXED);
				updated_state_units.push_back(_fixed_frame->getPPtr().get());
			}
			if (_fixed_frame->getOPtr())
			{
				_fixed_frame->getOPtr()->setStateStatus(ST_FIXED);
				updated_state_units.push_back(_fixed_frame->getOPtr().get());
			}
			if (_fixed_frame->getVPtr())
			{
				_fixed_frame->getVPtr()->setStateStatus(ST_FIXED);
				updated_state_units.push_back(_fixed_frame->getVPtr().get());
			}
			if (_fixed_frame->getWPtr())
			{
				_fixed_frame->getWPtr()->setStateStatus(ST_FIXED);
				updated_state_units.push_back(_fixed_frame->getWPtr().get());
			}
		}

        const Eigen::VectorXs getState() const
        {
        	return problem_->getState();
        }

        StateBaseList* getStateListPtr()
		{
        	return problem_->getStateListPtr();
		}

        std::list<ConstraintBaseShPtr> getConstraintsList()
        {
        	std::list<ConstraintBaseShPtr> corr_list;

        	for (FrameBaseIter frame_list_iter=problem_->getTrajectoryPtr()->getFrameListPtr()->begin(); frame_list_iter!=problem_->getTrajectoryPtr()->getFrameListPtr()->end(); frame_list_iter++)
			{
				for (CaptureBaseIter capture_list_iter=(*frame_list_iter)->getCaptureListPtr()->begin(); capture_list_iter!=(*frame_list_iter)->getCaptureListPtr()->end(); capture_list_iter++)
				{
					for (FeatureBaseIter feature_list_iter=(*capture_list_iter)->getFeatureListPtr()->begin(); feature_list_iter!=(*capture_list_iter)->getFeatureListPtr()->end(); feature_list_iter++)
					{
						corr_list.insert(corr_list.end(),(*feature_list_iter)->getConstraintListPtr()->begin(), (*feature_list_iter)->getConstraintListPtr()->end());
					}
				}
			}
        	return corr_list;
        }

        WolfProblemPtr getProblemPtr()
        {
        	return problem_;
        }

        void printTree()
        {
        	problem_->getTrajectoryPtr()->print();
        }
};

int main(int argc, char** argv)
{
	std::cout << "\n ========= 2D Robot with odometry and GPS ===========\n";

	// USER INPUT ============================================================================================
	if (argc!=4 || atoi(argv[1])<1 || atoi(argv[1])>1100 || atoi(argv[2]) < 0 || atoi(argv[3]) < 0 || atoi(argv[3]) > 1)
	{
		std::cout << "Please call me with: [./test_ceres_manager NI PRINT ORIENTATION_MODE], where:" << std::endl;
		std::cout << "     - NI is the number of iterations (0 < NI < 1100)" << std::endl;
		std::cout << "     - WS is the window size (0 < WS)" << std::endl;
		std::cout << "     - ORIENTATION_MODE: 0 for theta, 1 for complex angle" << std::endl;
		std::cout << "EXIT due to bad user input" << std::endl << std::endl;
		return -1;
	}

	clock_t t1, t2;
	t1=clock();

	unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution
	unsigned int window_size = (unsigned int) atoi(argv[2]);
	bool complex_angle = (bool) atoi(argv[3]);

	// INITIALIZATION ============================================================================================
	//init random generators
	WolfScalar odom_std= 0.01;
	WolfScalar gps_std= 1;
	std::default_random_engine generator(1);
	std::normal_distribution<WolfScalar> distribution_odom(0.001, odom_std); //odometry noise
	std::normal_distribution<WolfScalar> distribution_gps(0.0, gps_std); //GPS noise

	//init google log
	google::InitGoogleLogging(argv[0]);

	// Ceres initialization
	ceres::Solver::Options ceres_options;
	ceres_options.minimizer_type = ceres::LINE_SEARCH;//ceres::TRUST_REGION;
	ceres_options.max_line_search_step_contraction = 1e-3;
	//    ceres_options.minimizer_progress_to_stdout = false;
	//    ceres_options.line_search_direction_type = ceres::LBFGS;
	//    ceres_options.max_num_iterations = 100;
	ceres::Problem* ceres_problem = new ceres::Problem();
	CeresManager* ceres_manager = new CeresManager(ceres_problem);
	std::ofstream log_file, landmark_file;  //output file

	// Faramotics stuff
	CdynamicSceneRender *myRender;
	CrangeScan2D *myScanner;
	Cpose3d viewPoint;
	Cpose3d devicePose;
	vector<Cpose3d> devicePoses;
	vector<float> myScan;
	string modelFileName;
	unsigned int ii;
	double dt;
	//model and initial view point
	modelFileName = "/home/jvallve/iri-lab/faramotics/models/campusNordUPC.obj";
    //modelFileName = "/home/acoromin/dev/br/faramotics/models/campusNordUPC.obj";
    //modelFileName = "/home/andreu/dev/faramotics/models/campusNordUPC.obj";
	devicePose.setPose(2,8,0.2,0,0,0);
	//glut initialization
	//glutInit(&argc, argv);
    faramotics::initGLUT(argc, argv);
    
	//create a viewer for the 3D model and scan points
	myRender = new CdynamicSceneRender(1200,700,90*M_PI/180,90*700.0*M_PI/(1200.0*180.0),0.2,100);
	myRender->loadAssimpModel(modelFileName,true); //with wireframe
	//create scanner and load 3D model
	myScanner = new CrangeScan2D(HOKUYO_UTM30LX_180DEG);//HOKUYO_UTM30LX_180DEG or LEUZE_RS4
	myScanner->loadAssimpModel(modelFileName);
    
	//variables
	Eigen::Vector2s odom_reading, gps_fix_reading;
	Eigen::VectorXs odom_inc_true(n_execution*2);//invented motion
	Eigen::VectorXs pose_odom(3); //current odometry integred pose
	Eigen::VectorXs ground_truth(n_execution*3); //all true poses
	Eigen::VectorXs odom_trajectory(n_execution*3); //open loop trajectory
	Eigen::VectorXs odom_readings(n_execution*2); // all odometry readings
	Eigen::VectorXs gps_fix_readings(n_execution*3); //all GPS fix readings
	StateBasePtrList new_state_units; // new state units in wolf that must be added to ceres
	ConstraintBasePtrList new_constraints; // new constraints in wolf that must be added to ceres
	std::list<WolfScalar*> remove_state_units; // state units to be removed in ceres (parameter blocks)
	std::list<unsigned int> remove_constraints_idx; // constraints to be removed in ceres (residual blocks)
	StateBasePtrList update_state_units; // state units to be updated in ceres (parameter blocks)
	TimeStamp time_stamp;
	Eigen::VectorXs mean_times = Eigen::VectorXs::Zero(6);

	// Wolf manager initialization
	SensorOdom2D odom_sensor(Eigen::MatrixXs::Zero(6,1), odom_std, odom_std);
	//SensorGPSFix gps_sensor(Eigen::MatrixXs::Zero(6,1), gps_std);
	Eigen::VectorXs laser_pose(6);
	laser_pose << 0,0,0,0,0,0; //origin, no rotation
	//SensorLaser2D laser_sensor(Eigen::MatrixXs::Zero(6,1),-M_PI/2, M_PI/2, M_PI/720, 0.2, 100.0, 0.01);//OLD FASHION
	SensorLaser2D laser_sensor(Eigen::MatrixXs::Zero(6,1));
	WolfManager* wolf_manager = new WolfManager(&odom_sensor, complex_angle, 1e9, window_size);

	// Initial pose
	pose_odom << 2,8,0;
	ground_truth.head(3) = pose_odom;
	odom_trajectory.head(3) = pose_odom;
    
	// START TRAJECTORY ============================================================================================
	for (uint step=1; step < n_execution; step++)
	{
		//get init time
		t2=clock();

		// ROBOT MOVEMENT ---------------------------
		// moves the device position
		t1=clock();
		motionCampus(step, devicePose, odom_reading(0), odom_reading(1));
		devicePoses.push_back(devicePose);


		// SENSOR DATA ---------------------------
		// store groundtruth
		ground_truth.segment(step*3,3) << devicePose.pt(0), devicePose.pt(1), devicePose.rt.head();

		// compute odometry
		odom_reading(0) += distribution_odom(generator);
		odom_reading(1) += distribution_odom(generator);

		// odometry integration
		pose_odom(0) = pose_odom(0) + odom_reading(0) * cos(pose_odom(2));
		pose_odom(1) = pose_odom(1) + odom_reading(0) * sin(pose_odom(2));
		pose_odom(2) = pose_odom(2) + odom_reading(1);
		odom_trajectory.segment(step*3,3) = pose_odom;

		// compute GPS
		//gps_fix_reading << devicePose.pt(0), devicePose.pt(1), 0;
		//gps_fix_reading(0) += distribution_gps(generator);
		//gps_fix_reading(1) += distribution_gps(generator);

		//compute scan
		myScan.clear();
		myScanner->computeScan(devicePose,myScan);
		vector<double> myScanDoubles(myScan.begin(), myScan.end());
		Eigen::Map<Eigen::VectorXs> scan_reading(myScanDoubles.data(), 720);
		mean_times(0) += ((double)clock()-t1)/CLOCKS_PER_SEC;


		// ADD CAPTURES ---------------------------
		t1=clock();
		// adding new sensor captures
	    time_stamp.setToNow();
		wolf_manager->addCapture(CaptureBaseShPtr(new CaptureOdom2D(time_stamp, &odom_sensor, odom_reading, odom_std * Eigen::MatrixXs::Identity(2,2))));
		//wolf_manager->addCapture(CaptureBaseShPtr(new CaptureGPSFix(time_stamp, &gps_sensor, gps_fix_reading, gps_std * MatrixXs::Identity(3,3))));
		wolf_manager->addCapture(CaptureBaseShPtr(new CaptureLaser2D(time_stamp, &laser_sensor, scan_reading)));

        // updating problem
		wolf_manager->update(new_state_units, new_constraints, update_state_units, remove_state_units, remove_constraints_idx);
		mean_times(1) += ((double)clock()-t1)/CLOCKS_PER_SEC;


		// UPDATING CERES ---------------------------
		t1=clock();
		// adding new state units and constraints to ceres
		ceres_manager->addStateUnits(new_state_units);
		ceres_manager->addConstraints(new_constraints);
		// removing and updating state units and constraints from ceres
		ceres_manager->removeStateUnits(remove_state_units);
		ceres_manager->updateStateUnitStatus(update_state_units);
		//ceres_manager->removeConstraints(remove_constraints_idx);
		new_state_units.clear();
		new_constraints.clear();
		update_state_units.clear();
		remove_state_units.clear();
		remove_constraints_idx.clear();
		mean_times(2) += ((double)clock()-t1)/CLOCKS_PER_SEC;


		// SOLVE OPTIMIZATION ---------------------------
		t1=clock();
		ceres::Solver::Summary summary = ceres_manager->solve(ceres_options);
		mean_times(3) += ((double)clock()-t1)/CLOCKS_PER_SEC;


		// DRAWING STUFF ---------------------------
		t1=clock();
		// draw detected corners
		std::list<Eigen::Vector4s> corner_list;
		std::vector<double> corner_vector;
		CaptureLaser2D last_scan(time_stamp, &laser_sensor, scan_reading);
		last_scan.extractCorners(corner_list);
		for (std::list<Eigen::Vector4s>::iterator corner_it = corner_list.begin(); corner_it != corner_list.end(); corner_it++ )
		{
			corner_vector.push_back(corner_it->x());
			corner_vector.push_back(corner_it->y());
		}
		myRender->drawCorners(devicePose,corner_vector);

		// draw landmarks
		std::vector<double> landmark_vector;
		for (auto landmark_it = wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++ )
		{
			WolfScalar* position_ptr = (*landmark_it)->getPPtr()->getPtr();
			landmark_vector.push_back(*position_ptr); //x
			landmark_vector.push_back(*(position_ptr+1)); //y
			landmark_vector.push_back(0.2); //z
		}
		myRender->drawLandmarks(landmark_vector);

		//Set view point and render the scene
		//locate visualization view point, somewhere behind the device
		viewPoint.setPose(devicePose);
		viewPoint.rt.setEuler( viewPoint.rt.head(), viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
		viewPoint.moveForward(-5);
		myRender->setViewPoint(viewPoint);
		myRender->drawPoseAxis(devicePose);
		myRender->drawScan(devicePose,myScan,180.*M_PI/180.,90.*M_PI/180.); //draw scan
		myRender->render();
		mean_times(4) += ((double)clock()-t1)/CLOCKS_PER_SEC;

		// TIME MANAGEMENT ---------------------------
		mean_times(5) += ((double)clock()-t2)/CLOCKS_PER_SEC;

		if (mean_times(5) < 0.1)
			usleep(100000-1e-6*mean_times(5));
	}

	// DISPLAY RESULTS ============================================================================================
	mean_times /= n_execution;
	std::cout << "\nSIMULATION AVERAGE LOOP DURATION [s]" << std::endl;
	std::cout << "  data generation:    " << mean_times(0) << std::endl;
	std::cout << "  wolf managing:      " << mean_times(1) << std::endl;
	std::cout << "  ceres managing:     " << mean_times(2) << std::endl;
	std::cout << "  ceres optimization: " << mean_times(3) << std::endl;
	std::cout << "  results drawing:    " << mean_times(4) << std::endl;
	std::cout << "  loop time:          " << mean_times(5) << std::endl;

	// Draw Final result -------------------------
	std::vector<double> landmark_vector;
	for (auto landmark_it = wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++ )
	{
		WolfScalar* position_ptr = (*landmark_it)->getPPtr()->getPtr();
		landmark_vector.push_back(*position_ptr); //x
		landmark_vector.push_back(*(position_ptr+1)); //y
		landmark_vector.push_back(0.2); //z
	}
	myRender->drawLandmarks(landmark_vector);
	viewPoint.setPose(devicePoses.front());
	viewPoint.moveForward(10);
	viewPoint.rt.setEuler( viewPoint.rt.head()+M_PI/4, viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
	viewPoint.moveForward(-10);
	myRender->setViewPoint(viewPoint);
	myRender->render();

	// Print Final result in a file -------------------------
	// Vehicle poses
	int i = 0;
	Eigen::VectorXs state_poses(n_execution * 3);
	for (auto frame_it = wolf_manager->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()->begin(); frame_it != wolf_manager->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()->end(); frame_it++ )
	{
		if (complex_angle)
			state_poses.segment(i,3) << *(*frame_it)->getPPtr()->getPtr(), *((*frame_it)->getPPtr()->getPtr()+1), atan2(*(*frame_it)->getOPtr()->getPtr(), *((*frame_it)->getOPtr()->getPtr()+1));
		else
			state_poses.segment(i,3) << *(*frame_it)->getPPtr()->getPtr(), *((*frame_it)->getPPtr()->getPtr()+1), *(*frame_it)->getOPtr()->getPtr();
		i+=3;
	}

	// Landmarks
	i = 0;
	Eigen::VectorXs landmarks(wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->size()*2);
	for (auto landmark_it = wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++ )
	{
		Eigen::Map<Eigen::Vector2s> landmark((*landmark_it)->getPPtr()->getPtr());
		landmarks.segment(i,2) = landmark;
		i+=2;
	}

	// Print log files
	std::string filepath = getenv("HOME") + (complex_angle ? std::string("/Desktop/log_file_3.txt") : std::string("/Desktop/log_file_2.txt"));
	log_file.open(filepath, std::ofstream::out); //open log file

	if (log_file.is_open())
	{
		log_file << 0 << std::endl;
		for (unsigned int ii = 0; ii<n_execution; ii++)
			log_file << state_poses.segment(ii*3,3).transpose()
					 << "\t" << ground_truth.segment(ii*3,3).transpose()
					 << "\t" << (state_poses.segment(ii*3,3)-ground_truth.segment(ii*3,3)).transpose()
					 << "\t" << odom_trajectory.segment(ii*3,3).transpose()
					 << "\t" << gps_fix_readings.segment(ii*3,3).transpose() << std::endl;
		log_file.close(); //close log file
		std::cout << std::endl << "Result file " << filepath << std::endl;
	}
	else
		std::cout << std::endl << "Failed to write the log file " << filepath << std::endl;

	std::string filepath2 = getenv("HOME") + (complex_angle ? std::string("/Desktop/landmarks_file_3.txt") : std::string("/Desktop/landmarks_file_2.txt"));
	landmark_file.open(filepath2, std::ofstream::out); //open log file

	if (landmark_file.is_open())
	{
		for (unsigned int ii = 0; ii<landmarks.size(); ii+=2)
			landmark_file << landmarks.segment(ii,2).transpose() << std::endl;
		landmark_file.close(); //close log file
		std::cout << std::endl << "Landmark file " << filepath << std::endl;
	}
	else
		std::cout << std::endl << "Failed to write the landmark file " << filepath << std::endl;

	std::cout << "Press any key for ending... " << std::endl << std::endl;
	std::getchar();

	std::cout << " ========= END ===========" << std::endl << std::endl;

	delete ceres_manager;
	delete ceres_problem;
	delete wolf_manager;
    delete myRender;
    delete myScanner;

	//exit
	return 0;
}
