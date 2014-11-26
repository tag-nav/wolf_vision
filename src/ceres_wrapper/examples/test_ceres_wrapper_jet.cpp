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
        Map<const VectorXs> state_map_const_; //state point to be evaluated by Wolf tree constraints
        
        //Just to generate fake measurements
        std::default_random_engine generator_; //just to generate measurements
        std::normal_distribution<double> distribution_; //just to generate measurements
        MatrixXs measurements_; //just a set of invented measurements

        unsigned int measurements_size_, state_size_;

    public: 
        WolfVehicle() :
            state_prior_(),
            state_map_(nullptr,0),
            state_map_const_(nullptr,0),
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

        void remapState(WolfScalar *_ptr)
        {
        	new (&state_map_) Map<VectorXs>(_ptr, state_size_);

            //std::cout << "remapState:  state_map_ = " << state_map_.transpose() << std::endl;
        }

        void remapState(WolfScalar *_ptr, unsigned int _state_size)
        {
        	state_size_ = _state_size;
            new (&state_map_) Map<VectorXs>(_ptr, _state_size);

            //std::cout << "remapState:  state_map_ = " << state_map_.transpose() << std::endl;
        }

        void remapConstState(const WolfScalar * const _ptr)
        {
        	new (&state_map_const_) Map<const VectorXs>(_ptr, state_size_);

            //std::cout << "remapConstState:  state_map_const_ = " << state_map_const_.transpose() << std::endl;
        }

        void remapConstState(const WolfScalar * const _ptr, unsigned int _state_size)
        {
        	state_size_ = _state_size;
            new (&state_map_const_) Map<const VectorXs>(_ptr, state_size_);

            //std::cout << "remapConstState:  state_map_const_ = " << state_map_const_.transpose() << std::endl;
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
        
        void computePrior(VectorXs & _v)
        {
            state_prior_.setOnes();
            _v.resize(state_prior_.size());
            _v = state_prior_;
        }
        
        // template <typename T>
        // void computeError(Map<Matrix<T,Dynamic,1>> & _residuals, unsigned int & _index)
//        void computeError(double* _residuals)
//        {
//        	Map<VectorXs> mapped_residuals(_residuals, measurements_size_);
//
//        	//std::cout << "state_map_const  = " << state_map_const_.transpose() << std::endl;
//        	//std::cout << "measurements     = " << measurements_.transpose() << std::endl;
//        	std::cout << "measurements_     = " << std::endl << state_map_const_ * MatrixXs::Ones(1,measurements_size_) << std::endl;
//        	std::cout << "state_map_const_ * Ones(1,measurements_size_) = " << std::endl << state_map_const_ * MatrixXs::Ones(1,measurements_size_) << std::endl;
//        	mapped_residuals = (measurements_ - state_map_const_ * MatrixXs::Ones(1,measurements_size_)) * MatrixXs::Ones(measurements_size_,1);
//        	//for(unsigned int ii=0; ii< mapped_residuals.size(); ii++)
//        	//	mapped_residuals(ii) = measurements_(ii) - state_map_const_(ii); //just a trivial error function
//
//            //std::cout << "mapped_residuals = " << mapped_residuals.transpose() << std::endl << std::endl;
//        }
        template <typename T>
        bool operator()(const T* const _x, T* _residuals) const
        {
        	//std::cout << "state_size_ = " << state_size_ << std::endl;
        	//std::cout << "measurements_size_ = " << measurements_size_ << std::endl;

        	// Remap the vehicle state to the const evaluation point
			//remapConstState(_x);
			Map<const Matrix<T,Dynamic,1>> state_map_const_(_x, state_size_);

			// Compute error or residuals
			//computeError(_residuals);
			Map<Matrix<T,Dynamic,Dynamic>> mapped_residuals(_residuals, state_size_, measurements_size_);

			for (unsigned int i = 0; i<measurements_size_; i++)
			{
				for (unsigned int j = 0; j<state_size_; j++)
				{
					mapped_residuals(j,i) = T(measurements_(j,i)) - state_map_const_(j);
					//std::cout << "mapped_residuals(" << j << "," << i <<") = " << mapped_residuals(j,i) << std::endl;
				}
			}
			//std::cout << "residuals computed" << std::endl << std::endl;
        	return true;
        }
//        bool operator()(const WolfScalar * const _x, double* _residuals) const
//		{
//        	//std::cout << std::endl << "operator ()" << std::endl;
//			// Remap the vehicle state to the const evaluation point
//			//remapConstState(_x);
//        	Map<const VectorXs> state_map_const_(_x, state_size_);
//
//			// Compute error or residuals
//			//computeError(_residuals);
//        	Map<MatrixXs> mapped_residuals(_residuals, state_size_, measurements_size_);
//
//        	//std::cout << "measurements_     = " << std::endl << measurements_ << std::endl;
//			//std::cout << "state_map_const_ * Ones(1,measurements_size_) = " << std::endl << state_map_const_ * MatrixXs::Ones(1,measurements_size_) << std::endl;
//			//std::cout << "measurements_ - state_map_const_ * Ones(1,measurements_size_) = " << std::endl << measurements_ - state_map_const_ * MatrixXs::Ones(1,measurements_size_) << std::endl;
//			mapped_residuals = measurements_ - state_map_const_ * MatrixXs::Ones(1,measurements_size_);
//			//std::cout << "mapped_residuals     = " << std::endl << mapped_residuals << std::endl;
//        	//mapped_residuals = measurements_ - state_map_const_;
//c
//			return true;
//		}

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
	const unsigned int SimulationSteps = 20;
    
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

    	//ceres::AutoDiffCostFunction<WolfVehicle,ceres::CENTRAL,STATE_DIM*MEASUREMENT_DIM,STATE_DIM>*
		//   	   cost_function_static = new ceres::NumericDiffCostFunction<WolfVehicle,ceres::CENTRAL,STATE_DIM*MEASUREMENT_DIM,STATE_DIM>(functorPtr);
        //problem.AddResidualBlock(cost_function_static, nullptr, functorPtr->getPrior());

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

