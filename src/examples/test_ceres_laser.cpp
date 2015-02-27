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
#include <GL/glut.h>

//faramotics includes
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
        unsigned int last_state_units_;

    public:
        WolfManager(const SensorBasePtr& _sensor_prior, const bool _complex_angle, const unsigned int& _state_length) :
        	use_complex_angles_(_complex_angle),
			//trajectory_(new TrajectoryBase()),
			problem_(new WolfProblem(TrajectoryBaseShPtr(new TrajectoryBase), MapBaseShPtr(new MapBase), _state_length)),
			sensor_prior_(_sensor_prior),
			last_state_units_(0)
		{
        	Eigen::VectorXs init_frame(use_complex_angles_ ? 4 : 3);
        	if (use_complex_angles_)
        		init_frame << 0, 0, 1, 0;
        	else
        		init_frame << 0, 0, 0;
        	createFrame(init_frame, 0);
		}

        virtual ~WolfManager()
        {
        	//delete trajectory_;
        	delete problem_;
        }

        void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
        {
        	// Create frame and add it to the trajectory
        	if (use_complex_angles_)
        	{
        		StateBaseShPtr new_position(new StatePoint2D(problem_->getNewStatePtr()));
        		problem_->addState(new_position, _frame_state.head(2));
        		StateBaseShPtr new_orientation(new StateComplexAngle(problem_->getNewStatePtr()));
        		problem_->addState(new_orientation, _frame_state.tail(2));
                FrameBaseShPtr new_frame(new FrameBase(_time_stamp, new_position, new_orientation));

                problem_->getTrajectoryPtr()->addFrame(new_frame);
        	}
        	else
        	{
        		StateBaseShPtr new_position(new StatePoint2D(problem_->getNewStatePtr()));
				problem_->addState(new_position, _frame_state.head(2));
				StateBaseShPtr new_orientation(new StateTheta(problem_->getNewStatePtr()));
				problem_->addState(new_orientation, _frame_state.tail(1));
				FrameBaseShPtr new_frame(new FrameBase(_time_stamp, new_position, new_orientation));

        		problem_->getTrajectoryPtr()->addFrame(new_frame);
        	}
        }

        void addCapture(const CaptureBaseShPtr& _capture)
        {
        	new_captures_.push(_capture);
        }

        void update(StateBasePtrList& new_state_units, ConstraintBasePtrList& new_constraints)
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
        		while (last_state_units_ == problem_->getStateListPtr()->size())
        		{
        			new_state_units.push_back((*state_unit_it).get());
        			state_unit_it++;
        			last_state_units_++;
        		}
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
	if (argc!=3 || atoi(argv[1])<1 || atoi(argv[1])>1100 || atoi(argv[2]) < 0 || atoi(argv[2]) > 1)
	{
		std::cout << "Please call me with: [./test_ceres_manager NI PRINT ORIENTATION_MODE], where:" << std::endl;
		std::cout << "     - NI is the number of iterations (0 < NI < 1100)" << std::endl;
		std::cout << "     - ORIENTATION_MODE: 0 for theta, 1 for complex angle" << std::endl;
		std::cout << "EXIT due to bad user input" << std::endl << std::endl;
		return -1;
	}

	clock_t t1, t2;
	t1=clock();

	unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution
	bool complex_angle = (bool) atoi(argv[2]);

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
	std::ofstream log_file;  //output file

	// Faramotics stuff
	CdynamicSceneRender *myRender;
	CrangeScan2D *myScanner;
	Cpose3d viewPoint;
	Cpose3d devicePose;
	vector<Cpose3d> devicePoses;
	vector<float> myScan;
	string modelFileName;
	unsigned int ii;
	timeval t_ini_step,t_fin_step;
	double dt;
	//model and initial view point
	modelFileName = "/home/jvallve/iri-lab/faramotics/models/campusNordUPC.obj";
	devicePose.setPose(2,8,0.2,0,0,0);
	//glut initialization
	glutInit(&argc, argv);
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
	TimeStamp time_stamp;

	// Wolf manager initialization
	SensorOdom2D odom_sensor(Eigen::MatrixXs::Zero(6,1), odom_std, odom_std);
	//SensorGPSFix gps_sensor(Eigen::MatrixXs::Zero(6,1), gps_std);
	Eigen::VectorXs laser_pose(6);
	laser_pose << 0,0,0,0,0,0; //origin, no rotation
	SensorLaser2D laser_sensor(Eigen::MatrixXs::Zero(6,1),-M_PI/2, M_PI/2, M_PI/720, 0.2, 100.0, 0.01);
	WolfManager* wolf_manager = new WolfManager(&odom_sensor, complex_angle, 1e6);

	// Initial pose
	pose_odom << 0,0,0;
	ground_truth.head(3) = pose_odom;
	odom_trajectory.head(3) = pose_odom;

	// START TRAJECTORY ============================================================================================
	for (uint step=1; step < n_execution; step++)
	{
		//get init time
		gettimeofday(&t_ini_step, NULL);

		// ROBOT MOVEMENT ---------------------------
		// moves the device position
		motionCampus(step, devicePose, odom_reading(0), odom_reading(1));

		// SENSOR DATA ---------------------------
		// store groundtruth
		ground_truth.segment(ii*3,3) << devicePose.pt(0), devicePose.pt(1), devicePose.rt.head();


		// compute odometry
		odom_reading(0) += distribution_odom(generator);
		odom_reading(1) += distribution_odom(generator);


		// odometry integration
		pose_odom(0) = pose_odom(0) + odom_reading(0) * cos(pose_odom(2));
		pose_odom(1) = pose_odom(1) + odom_reading(0) * sin(pose_odom(2));
		pose_odom(2) = pose_odom(2) + odom_reading(1);
		odom_trajectory.segment(ii*3,3) << pose_odom;


		// compute GPS
		//gps_fix_reading << devicePose.pt(0), devicePose.pt(1), 0;
		//gps_fix_reading(0) += distribution_gps(generator);
		//gps_fix_reading(1) += distribution_gps(generator);

		//compute scan
		myScan.clear();
		myScanner->computeScan(devicePose,myScan);
		vector<double> myScanDoubles(myScan.begin(), myScan.end());
		Eigen::Map<Eigen::VectorXs> scan_reading(myScanDoubles.data(), 720);

		//print scan
//		cout << endl;
//		for (unsigned int jj = 0; jj < myScan.size(); jj++ ) cout << myScan[jj] << ",";
//		cout << endl << endl;

		//draws the device frame, scan hits and depth image
		myRender->drawPoseAxis(devicePose);
		myRender->drawScan(devicePose,myScan,180.*M_PI/180.,90.*M_PI/180.); //draw scan
		devicePoses.push_back(devicePose);

		// ADD CAPTURES ---------------------------
		// adding new sensor captures
	    time_stamp.setToNow();
		wolf_manager->addCapture(CaptureBaseShPtr(new CaptureOdom2D(time_stamp, &odom_sensor, odom_reading, odom_std * Eigen::MatrixXs::Identity(2,2))));
		//wolf_manager->addCapture(CaptureBaseShPtr(new CaptureGPSFix(time_stamp, &gps_sensor, gps_fix_reading, gps_std * MatrixXs::Identity(3,3))));
		wolf_manager->addCapture(CaptureBaseShPtr(new CaptureLaser2D(time_stamp, &laser_sensor, scan_reading)));

		// updating problem
		wolf_manager->update(new_state_units, new_constraints);

		// draw detected corners
		std::list<Eigen::Vector4s> corner_list;
		std::vector<double> corner_vector;
		CaptureLaser2D last_scan(time_stamp, &laser_sensor, scan_reading);
		last_scan.extractCorners(corner_list);
		//std::cout << "Corners detected: " << corner_list.size() << std::endl;
		for (std::list<Eigen::Vector4s>::iterator corner_it = corner_list.begin(); corner_it != corner_list.end(); corner_it++ )
		{
			corner_vector.push_back(corner_it->x());
			corner_vector.push_back(corner_it->y());
		}
		myRender->drawCorners(devicePose,corner_vector);

		//Set view point and render the scene
		//locate visualization view point, somewhere behind the device
		viewPoint.setPose(devicePose);
		viewPoint.rt.setEuler( viewPoint.rt.head(), viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
		viewPoint.moveForward(-5);
		myRender->setViewPoint(viewPoint);
		myRender->render();

		// adding new state units and constraints to ceres
		ceres_manager->addStateUnits(new_state_units);
		ceres_manager->addConstraints(new_constraints);


		// TIME MANAGEMENT ---------------------------
		//get end time
		gettimeofday(&t_fin_step, NULL);

		//constant step delay
		dt = (t_fin_step.tv_sec+t_fin_step.tv_usec/1e6) - (t_ini_step.tv_sec+t_ini_step.tv_usec/1e9);
		dt = dt*1e6; //dt in milliseconds
		if (dt < 50000)
			usleep(50000-dt);
	}

	//std::cout << "Resulting tree:\n";
	//wolf_manager->printTree();

	// Draw previous to optimization
	std::vector<double> landmark_vector;
	for (auto landmark_it = wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++ )
	{
		WolfScalar* position_ptr = (*landmark_it)->getPPtr()->getPtr();
		landmark_vector.push_back(*position_ptr+2); //x
		landmark_vector.push_back(*(position_ptr+1)+8); //y
		landmark_vector.push_back(0.2); //z
	}
	myRender->drawLandmarks(landmark_vector);
	viewPoint.setPose(devicePoses.front());
	viewPoint.moveForward(10);
	viewPoint.rt.setEuler( viewPoint.rt.head()+M_PI/2, viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
	viewPoint.moveForward(-10);
	myRender->setViewPoint(viewPoint);
	myRender->render();

	// SOLVE OPTIMIZATION ============================================================================================
	ceres::Solver::Summary summary = ceres_manager->solve(ceres_options);
	t2=clock();
	double seconds = ((double)t2-t1)/CLOCKS_PER_SEC;

	// DISPLAY RESULTS ============================================================================================
	std::cout << summary.FullReport() << std::endl;
	std::cout << "optimization seconds: " << summary.total_time_in_seconds << std::endl;
	std::cout << "total seconds: " << seconds << std::endl;

	// Draw Final result
	landmark_vector.clear();
	for (auto landmark_it = wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != wolf_manager->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++ )
	{
		WolfScalar* position_ptr = (*landmark_it)->getPPtr()->getPtr();
		landmark_vector.push_back(*position_ptr+2); //x
		landmark_vector.push_back(*(position_ptr+1)+8); //y
		landmark_vector.push_back(0.2); //z
	}
	myRender->drawLandmarks(landmark_vector);
	viewPoint.setPose(devicePoses.front());
	viewPoint.moveForward(10);
	viewPoint.rt.setEuler( viewPoint.rt.head()+M_PI/2, viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
	viewPoint.moveForward(-10);
	myRender->setViewPoint(viewPoint);
	myRender->render();

	// change from complex angle to theta
	Eigen::VectorXs state = wolf_manager->getState();
	Eigen::VectorXs state_theta(n_execution * 3);
	if (complex_angle)
		for (uint ii = 0; ii<n_execution; ii++)
			state_theta.segment(ii*3,3) << state(ii*4), state(ii*4+1), atan2(state(ii*4+2), state(ii*4+3));
	else
		state_theta = state;

	// Print log file
	std::string filepath = getenv("HOME") + (complex_angle ? std::string("/Desktop/log_file_3.txt") : std::string("/Desktop/log_file_2.txt"));
	log_file.open(filepath, std::ofstream::out); //open log file

	if (log_file.is_open())
	{
		log_file << seconds << std::endl;
		for (unsigned int ii = 0; ii<n_execution; ii++)
			log_file << state_theta.segment(ii*3,3).transpose()
					 << "\t" << ground_truth.segment(ii*3,3).transpose()
					 << "\t" << (state_theta.segment(ii*3,3)-ground_truth.segment(ii*3,3)).transpose()
					 << "\t" << odom_trajectory.segment(ii*3,3).transpose()
					 << "\t" << gps_fix_readings.segment(ii*3,3).transpose() << std::endl;
		log_file.close(); //close log file
		std::cout << std::endl << "Result file " << filepath << std::endl;
	}
	else
		std::cout << std::endl << "Failed to write the file " << filepath << std::endl;

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
