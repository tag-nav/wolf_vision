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
#include "frame_base.h"
#include "state_point.h"
#include "state_complex_angle.h"
#include "capture_base.h"
#include "capture_relative.h"
#include "capture_odom_2D.h"
#include "capture_gps_fix.h"
#include "state_base.h"
#include "constraint_sparse.h"
#include "constraint_gps_2D.h"
#include "constraint_odom_2D_theta.h"
#include "constraint_odom_2D_complex_angle.h"

// ceres wrapper include
#include "ceres_wrapper/ceres_manager.h"

/**
 * This test implements an optimization using CERES of a vehicle trajectory using odometry and GPS simulated data.
 *
 **/

using namespace Eigen;

class WolfManager
{
    protected:
		VectorXs state_;
		unsigned int first_empty_state_;
		bool use_complex_angles_;
		TrajectoryBase* trajectory_;
        std::vector<VectorXs> odom_captures_;
        std::vector<VectorXs> gps_captures_;
        std::queue<CaptureBase*> new_captures_;
        SensorBase* sensor_prior_;

    public: 
        WolfManager(SensorBase* _sensor_prior, const bool _complex_angle, const unsigned int& _state_length=1000) :
        	state_(_state_length),
			first_empty_state_(0),
        	use_complex_angles_(_complex_angle),
			trajectory_(new TrajectoryBase()),
			sensor_prior_(_sensor_prior)
		{
        	VectorXs init_frame(use_complex_angles_ ? 4 : 3);
        	if (use_complex_angles_)
        		init_frame << 0, 0, 1, 0;
        	else
        		init_frame << 0, 0, 0;
        	createFrame(init_frame, 0);
		}

        virtual ~WolfManager()
        {
        	delete trajectory_;
        }

        void createFrame(const VectorXs& _frame_state, const TimeStamp& _time_stamp)
        {
        	// Store in state_
        	state_.segment(first_empty_state_, use_complex_angles_ ? 4 : 3) << _frame_state;

        	// Create frame and add it to the trajectory
        	if (use_complex_angles_)
        	{
                FrameBase* new_frame = new FrameBase(_time_stamp,
                                                     new StatePoint2D(state_.data()+first_empty_state_),
                                                     new StateComplexAngle(state_.data()+first_empty_state_+2));
        		trajectory_->addFrame(new_frame);
        	}
        	else
        	{
                FrameBase* new_frame = new FrameBase(_time_stamp,
                                                     new StatePoint2D(state_.data()+first_empty_state_),
                                                     new StateTheta(state_.data()+first_empty_state_+2));
        		trajectory_->addFrame(new_frame);
        	}

        	// Update first free state location index
        	first_empty_state_ += use_complex_angles_ ? 4 : 3;
        }

        void addCapture(CaptureBase* _capture)
        {
        	new_captures_.push(_capture);
        }

        void update(std::list<StateBase*>& new_state_units, std::list<ConstraintBase*>& new_constraints)
        {
        	// TODO: management due to time stamps
        	while (!new_captures_.empty())
        	{
        		// EXTRACT NEW CAPTURE
        		CaptureBase* new_capture = new_captures_.front();
        		new_captures_.pop();

        		// NEW FRAME (if the specific sensor)
        		// TODO: accumulate odometries
        		if (new_capture->getSensorPtr() == sensor_prior_)
        		{
        			// ADD CAPTURE TO THE PREVIOUS FRAME
					trajectory_->getFrameListPtr()->back()->addCapture(new_capture);

					// COMPUTE PRIOR
					trajectory_->getFrameListPtr()->back()->setState(new_capture->computePrior());

					// CREATE NEW FRAME
					createFrame(VectorXs::Zero(use_complex_angles_ ? 4 : 3), new_capture->getTimeStamp());

					// NEW STATE UNITS TO BE ADDED BY CERES
					//new_state_units.insert(new_state_units.end(), trajectory_.getFrameList.back()->getStateList().begin(), trajectory_.getFrameList.back()->getStateList().end());
					new_state_units.push_back(trajectory_->getFrameListPtr()->back()->getPPtr());
					new_state_units.push_back(trajectory_->getFrameListPtr()->back()->getOPtr());
        		}
        		else
        		{
        			// ADD CAPTURE TO THE LAST FRAME
					trajectory_->getFrameListPtr()->back()->addCapture(new_capture);
        		}

        		// COMPUTE CAPTURE (features, constraints)
        		new_capture->processCapture();

        		// ADD CORRESPONDENCES TO THE new_constraints OUTPUT PARAM
        		for (FeatureBaseIter feature_list_iter=new_capture->getFeatureListPtr()->begin(); feature_list_iter!=new_capture->getFeatureListPtr()->end(); feature_list_iter++)
				{
					for (ConstraintBaseIter constraint_list_iter=(*feature_list_iter)->getConstraintListPtr()->begin(); constraint_list_iter!=(*feature_list_iter)->getConstraintListPtr()->end(); constraint_list_iter++)
					{
						new_constraints.push_back(*constraint_list_iter);
					}
				}
        	}
        }

        VectorXs getState()
        {
        	return state_;
        }

        std::list<StateBase*> getStateList()
		{
        	std::list<StateBase*> st_list;

        	for (FrameBaseIter frame_list_iter=trajectory_->getFrameListPtr()->begin(); frame_list_iter!=trajectory_->getFrameListPtr()->end(); frame_list_iter++)
			{
        		//st_list.insert(st_list.end(), (*frame_list_iter)->getStateList().begin(), (*frame_list_iter)->getStateList().end());
        		st_list.push_back((*frame_list_iter)->getPPtr());
        		st_list.push_back((*frame_list_iter)->getOPtr());
			}

			return st_list;
		}

        std::list<ConstraintBase*> getConstraintsList()
        {
        	std::list<ConstraintBase*> corr_list;

        	for (FrameBaseIter frame_list_iter=trajectory_->getFrameListPtr()->begin(); frame_list_iter!=trajectory_->getFrameListPtr()->end(); frame_list_iter++)
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

        void printTree()
        {
        	trajectory_->print();
        }
};

int main(int argc, char** argv) 
{
	std::cout << "\n ========= 2D Robot with odometry and GPS ===========\n";

    // USER INPUT ============================================================================================
	if (argc!=3 || atoi(argv[1])<1 || atoi(argv[2]) < 0 || atoi(argv[2]) > 1)
	{
		std::cout << "Please call me with: [./test_ceres_manager NI PRINT ORIENTATION_MODE], where:" << std::endl;
		std::cout << "     - NI is the number of iterations (NI > 0)" << std::endl;
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
	ceres::Problem::Options problem_options;
	problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	CeresManager* ceres_manager = new CeresManager(problem_options);
	std::ofstream log_file;  //output file


	//variables
	Eigen::VectorXs odom_inc_true(n_execution*2);//invented motion
	Eigen::VectorXs pose_true(3); //current true pose
	Eigen::VectorXs pose_odom(3); //current true pose
	Eigen::VectorXs ground_truth(n_execution*3); //all true poses
	Eigen::VectorXs odom_trajectory(n_execution*3); //all true poses
	Eigen::VectorXs odom_readings(n_execution*2); // all odometry readings
	Eigen::VectorXs gps_fix_readings(n_execution*3); //all GPS fix readings
	std::list<StateBase*> new_state_units; // new state units in wolf that must be added to ceres
	std::list<ConstraintBase*> new_constraints; // new constraints in wolf that must be added to ceres

	// Wolf manager initialization
	SensorOdom2D odom_sensor(Eigen::MatrixXs::Zero(6,1), odom_std, odom_std);
	SensorGPSFix gps_sensor(Eigen::MatrixXs::Zero(6,1), gps_std);
	WolfManager* wolf_manager = new WolfManager(&odom_sensor, complex_angle, n_execution * (complex_angle ? 4 : 3));

	// Initial pose
	pose_true << 0,0,0;
	pose_odom << 0,0,0;
	ground_truth.head(3) = pose_true;
	odom_trajectory.head(3) = pose_true;

	// SENSOR DATA ============================================================================================
	for (unsigned int ii = 1; ii<n_execution; ii++)
	{
		// inventing odometry ground truth
		if ( ii < (unsigned int)floor(n_execution/2) )
			odom_inc_true.segment(ii*2,2) << fabs(cos(ii/10.)) , fabs(sin(ii/2000.)); //invented motion increments.
		else
			odom_inc_true.segment(ii*2,2) << fabs(cos(ii/10.)) , -fabs(sin((ii-floor(n_execution/2))/2000.)); //invented motion increments.

		// Computing ground truth trajectory
		pose_true(0) = pose_true(0) + odom_inc_true(ii*2) * cos(pose_true(2) + odom_inc_true(ii*2+1));
		pose_true(1) = pose_true(1) + odom_inc_true(ii*2) * sin(pose_true(2) + odom_inc_true(ii*2+1));
		pose_true(2) = pose_true(2) + odom_inc_true(ii*2+1);
		ground_truth.segment(ii*3,3) << pose_true;

		// corrupting sensor readings (odometry and GPS)
		odom_readings.segment(ii*2,2) << odom_inc_true(ii*2) + distribution_odom(generator),
										 odom_inc_true(ii*2+1) + distribution_odom(generator); //true range and theta with noise
		gps_fix_readings.segment(ii*3,3) << pose_true(0) + distribution_gps(generator),
											pose_true(1) + distribution_gps(generator),
											0. + distribution_gps(generator);

		// Computing ground truth trajectory
		pose_odom(0) = pose_odom(0) + odom_readings(ii*2) * cos(pose_odom(2) + odom_readings(ii*2+1));
		pose_odom(1) = pose_odom(1) + odom_readings(ii*2) * sin(pose_odom(2) + odom_readings(ii*2+1));
		pose_odom(2) = pose_odom(2) + odom_readings(ii*2+1);
		odom_trajectory.segment(ii*3,3) << pose_odom;
	}

	// START TRAJECTORY ============================================================================================
    new_state_units = wolf_manager->getStateList(); // First pose to be added in ceres
    for (unsigned int step=1; step < n_execution; step++)
	{
    	// adding new sensor captures
		wolf_manager->addCapture(new CaptureOdom2D(TimeStamp(step*0.01), &odom_sensor, odom_readings.segment(step*2,2), odom_std * MatrixXs::Identity(2,2)));
		wolf_manager->addCapture(new CaptureGPSFix(TimeStamp(step*0.01), &gps_sensor, gps_fix_readings.segment(step*3,3), gps_std * MatrixXs::Identity(3,3)));

		// updating problem
		wolf_manager->update(new_state_units, new_constraints);

		// adding new state units and constraints to ceres
		ceres_manager->addStateUnits(&new_state_units);
		ceres_manager->addConstraints(&new_constraints);
	}

	//std::cout << "Resulting tree:\n";
	//wolf_manager->printTree();

    // SOLVE OPTIMIZATION ============================================================================================
	ceres::Solver::Summary summary = ceres_manager->solve(ceres_options);
	t2=clock();
	double seconds = ((double)t2-t1)/CLOCKS_PER_SEC;

	// DISPLAY RESULTS ============================================================================================
	std::cout << summary.FullReport() << std::endl;
	std::cout << "optimization seconds: " << summary.total_time_in_seconds << std::endl;
	std::cout << "total seconds: " << seconds << std::endl;

	// change from complex angle to theta
	VectorXs state = wolf_manager->getState();
	VectorXs state_theta(n_execution * 3);
	if (complex_angle)
		for (unsigned int ii = 0; ii<n_execution; ii++)
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

    std::cout << " ========= END ===========" << std::endl << std::endl;

    delete ceres_manager;
    delete wolf_manager;

    //exit
    return 0;
}
