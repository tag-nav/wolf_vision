// std includes
#include <iostream>
#include <memory>
#include <random>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Ceres includes
#include "ceres/ceres.h"
#include "glog/logging.h"

// Wolf includes
#include "wolf.h"

/**
 * This class emulates a Wolf top tree node class, such as vehicle. 
 * This class will be removed when effective linking to Wolf, and using actual Vehicle class. 
 * It holds:
 *      - a map to a state vector
 *      - a map to an error vector
 *      - a method to compute the error from the state
 * 
 **/
using namespace Eigen;

class WolfVehicle
{
    protected:
        VectorXs state_prior_; //state storage where to compute prior
        Map<VectorXs> state_map_; //state point to be evaluated by Wolf tree constraints
        
        //Just to generate fake measurements
        std::default_random_engine generator_; //just to generate measurements
        std::normal_distribution<WolfScalar> absolute_distribution_, distance_distribution_; //just to generate measurements
        MatrixXs absolute_measurements_, distance_measurements_; //just a set of invented measurements

        unsigned int absolute_measurements_size_, distance_measurements_size_, state_size_;

    public: 
        WolfVehicle() :
            state_prior_(),
            state_map_(nullptr,0),
			absolute_distribution_(0.0,0.1),
			distance_distribution_(0.0,0.01),
			absolute_measurements_(0),
			distance_measurements_(0),
			state_size_(0)
        {
            //
        }
        
        virtual ~WolfVehicle()
        {
            //
        }
        
        WolfScalar *getPrior()
        {
            return state_prior_.data();
        }
        
        void resizeState(unsigned int _state_size)
        {
        	state_size_ = _state_size;
            state_prior_.resize(_state_size);
        }

        void inventAbsoluteMeasurements(unsigned int _sz)
        {
        	// for testing
        	absolute_measurements_size_ = _sz;
        	absolute_measurements_.resize(state_size_,absolute_measurements_size_);

            for(unsigned int ii=0; ii<absolute_measurements_size_*state_size_; ii++)
            	absolute_measurements_(ii) = 1 + absolute_distribution_(generator_); //just inventing a sort of noise measurements

            //std::cout << "invented measurements_ = " << std::endl << measurements_.transpose() << std::endl;
        }

        void inventDistanceMeasurements(unsigned int _sz)
        {
        	// for testing
        	distance_measurements_size_ = _sz;
        	distance_measurements_.resize(state_size_,distance_measurements_size_);

            for(unsigned int ii=0; ii<distance_measurements_size_*state_size_; ii++)
            	distance_measurements_(ii) = 1 + distance_distribution_(generator_); //just inventing a sort of noise measurements

            //std::cout << "invented measurements_ = " << std::endl << measurements_.transpose() << std::endl;
        }
        
        void computePrior()
        {
            state_prior_.setZero();
        }
        
        template <typename T>
        void computeAbsoluteResiduals(const Matrix<T,Dynamic,1> &state_map_const, Matrix<T,Dynamic,Dynamic> &mapped_absolute_residuals)
        {

        }

        template <typename T>
        void computeDistanceResiduals(const Matrix<T,Dynamic,1> &state_map_const, Matrix<T,Dynamic,Dynamic> &mapped_absolute_residuals)
        {

        }

        template <typename T>
        bool operator()(const T* const _x, T* _residuals) const
        {
        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const(_x, state_size_);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,Dynamic>> mapped_absolute_residuals(_residuals, state_size_, absolute_measurements_size_);
			Map<Matrix<T,Dynamic,Dynamic>> mapped_distance_residuals(_residuals + state_size_ * absolute_measurements_size_, state_size_, distance_measurements_size_);

			// Compute error or residuals
			computeAbsoluteResiduals(state_map_const, mapped_absolute_residuals);
			computeDistanceResiduals(state_map_const, mapped_distance_residuals);
			mapped_residuals = measurements_.cast<T>() - state_map_const.replicate(1,measurements_size_);

			// for (unsigned int i = 0; i<measurements_size_; i++)
			//	for (unsigned int j = 0; j<state_size_; j++)
			// 		std::cout << "mapped_residuals(" << j << "," << i <<") = " << mapped_residuals(j,i) << std::endl;

        	return true;
        }
        bool operator()(const WolfScalar* const _x, WolfScalar* _residuals) const
		{
        	// Remap the vehicle state to the const evaluation point
			Map<const VectorXs> state_map_const_(_x, state_size_);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<MatrixXs> mapped_residuals(_residuals, state_size_, measurements_size_);

			// Compute error or residuals
			mapped_residuals = measurements_ - state_map_const_.replicate(1,measurements_size_);

			return true;
		}

        void print()
        {
            //std::cout << "measurements_: " << std::endl << measurements_.transpose() << std::endl << std::endl;
            std::cout << "state_prior_ : " << std::endl << state_prior_.transpose() << std::endl << std::endl;
            //std::cout << "state_const_ : " << std::endl << state_map_const_.transpose() << std::endl << std::endl;
        }
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= Autodiff case ===========" << std::endl << std::endl;
    
    //dimension 
    const unsigned int POINT_DIM = 3;
    const unsigned int N_POINTS = 50;
    const unsigned int STATE_DIM = N_POINTS * POINT_DIM; 		// Dimension of the state
    const unsigned int N_ABS_MEAS = 1; // Amount of measurements
    const unsigned int N_DIST_MEAS = 1; // Amount of measurements
    const unsigned int MEASUREMENT_DIM = STATE_DIM*N_ABS_MEAS + N_POINTS * N_DIST_MEAS;
	const unsigned int SimulationSteps = 2;
    
    // init
    google::InitGoogleLogging(argv[0]);
    
    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;

    //wolf vehicle & ceres functor
    WolfVehicle *functorPtr = new WolfVehicle();

    // Ceres problem initialization
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.minimizer_type = ceres::TRUST_REGION;
    options.line_search_direction_type = ceres::LBFGS;
    options.max_num_iterations = 10;
    ceres::Solver::Summary summary;
    ceres::Problem problem;

    // fixed dim problem
    functorPtr->resizeState(STATE_DIM);
    functorPtr->computePrior();

    //start Wolf iteration
    for (uint i=0; i<SimulationSteps; i++)
    {
    	std::cout << " :::::::::::::::::::: step " << i << " ::::::::::::::::::::" << std::endl << std::endl;

        // set measures. This will be replaced by the WOLF-ROS front-end, getting sensor readings from sensors and performing measurements to build the whole wolf tree
    	functorPtr->inventAbsoluteMeasurements(N_ABS_MEAS);
    	functorPtr->inventDistanceMeasurements(N_DIST_MEAS);
        
        // Resizing & remapping


    	// cost function
		CostFunction* cost_function = new AutoDiffCostFunction<WolfVehicle,MEASUREMENT_DIM,STATE_DIM>(functorPtr);
    	problem.AddResidualBlock(cost_function, NULL, functorPtr->getPrior());

    	// run Ceres Solver
        ceres::Solve(options, &problem, &summary);

        //display results
        std::cout << summary.BriefReport() << "\n";
        functorPtr->print();
	}
    //clean
    std::cout << "Cleaning ... " << std::endl << std::endl;
    //problem.RemoveResidualBlock(rbId);
    //delete functorPtr;
    
    //end Wolf iteration
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

