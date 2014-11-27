//std includes
#include <iostream>
#include <memory>
#include <random>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//Ceres includes
#include "ceres/ceres.h"
#include "glog/logging.h"

//Wolf includes
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
        std::normal_distribution<double> distribution_; //just to generate measurements
        MatrixXs measurements_; //just a set of invented measurements

        unsigned int measurements_size_, state_size_;

    public: 
        WolfVehicle() :
            state_prior_(),
            state_map_(nullptr,0),
            distribution_(0.0,0.01),
			measurements_size_(0),
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

        void inventMeasurements(unsigned int _sz)
        {
        	// for testing
        	measurements_size_ = _sz;
        	measurements_.resize(state_size_,measurements_size_);

            for(unsigned int ii=0; ii<measurements_size_*state_size_; ii++)
            {
                measurements_(ii) = 1 + distribution_(generator_); //just inventing a sort of noise measurements
            }
            //std::cout << "invented measurements_ = " << std::endl << measurements_.transpose() << std::endl;
        }
        
        void computePrior()
        {
            state_prior_.setZero();
        }
        
        template <typename T>
        bool operator()(const T* const _x, T* _residuals) const
        {
        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const_(_x, state_size_);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,Dynamic>> mapped_residuals(_residuals, state_size_, measurements_size_);

			// Compute error or residuals
			mapped_residuals = measurements_.cast<T>() - state_map_const_.replicate(1,measurements_size_);

			// for (unsigned int i = 0; i<measurements_size_; i++)
			// {
			// 	for (unsigned int j = 0; j<state_size_; j++)
			// 	{
			// 		mapped_residuals(j,i) = T(measurements_(j,i)) - state_map_const_(j);
			// 		std::cout << "mapped_residuals(" << j << "," << i <<") = " << mapped_residuals(j,i) << std::endl;
			// 		std::cout << "residuals_test  (" << j << "," << i <<") = " << residuals_test(j,i) << std::endl;
			// 		std::cout << "diff = " << residuals_test(j,i) - mapped_residuals(j,i) << std::endl;
			// 	}
			// }
			//std::cout << "residuals computed" << std::endl << std::endl;
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
    std::cout << " ========= Static Numeric case ===========" << std::endl << std::endl;
    
    //dimension 
    const unsigned int STATE_DIM = 5; //just to test, all will be DIM-dimensional
    const unsigned int MEASUREMENT_DIM = 10; //just to test, all will be DIM-dimensional
	const unsigned int SimulationSteps = 1;
    
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
    	functorPtr->inventMeasurements(MEASUREMENT_DIM);
        
        // Resizing & remapping


    	// cost function
		CostFunction* cost_function = new AutoDiffCostFunction<WolfVehicle,STATE_DIM*MEASUREMENT_DIM,STATE_DIM>(functorPtr);
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
    //delete cost_function_static;
    //delete functorPtr;
    
    //end Wolf iteration
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

