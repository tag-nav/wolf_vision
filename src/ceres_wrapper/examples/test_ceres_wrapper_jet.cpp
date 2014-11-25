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
        VectorXs measurements_; //just a set of invented measurements

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
        	measurements_size_ = _sz;
            measurements_.resize(_sz);
            for(unsigned int ii=0; ii<_sz; ii++)
            {
                measurements_(ii) = 1 + distribution_(generator_); //just inventing a sort of noise measurements
            }
        }
        
        void computePrior()
        {
            state_prior_.setOnes();
        }        
        
        void computePrior(VectorXs & _v)
        {
            state_prior_.setOnes();
            _v.resize(state_prior_.size());
            _v = state_prior_;
        }
        
        // template <typename T>
        // void computeError(Map<Matrix<T,Dynamic,1>> & _residuals, unsigned int & _index)
        void computeError(double* _residuals)
        {
        	Map<VectorXs> mapped_residuals(_residuals, measurements_size_);

        	//std::cout << "state_map_const  = " << state_map_const_.transpose() << std::endl;
        	//std::cout << "measurements     = " << measurements_.transpose() << std::endl;

        	mapped_residuals = measurements_ - state_map_const_;
        	//for(unsigned int ii=0; ii< mapped_residuals.size(); ii++)
        	//	mapped_residuals(ii) = measurements_(ii) - state_map_const_(ii); //just a trivial error function

            //std::cout << "mapped_residuals = " << mapped_residuals.transpose() << std::endl << std::endl;
        }

        void print()
        {
            std::cout << "measurements_: " << std::endl << measurements_.transpose() << std::endl << std::endl;
            std::cout << "state_prior_ : " << std::endl << state_prior_.transpose() << std::endl << std::endl;
            std::cout << "state_const_ : " << std::endl << state_map_const_.transpose() << std::endl << std::endl;
        }
};

/**
 * This class wrapps the Wolf Vehicle and implements the operator(), to be used by CERES optimizer
 *      - the operator() overloaded
**/
class CeresWolfFunctor
{
    protected: 
        std::shared_ptr<WolfVehicle> vehicle_ptr_; //pointer to Wolf Vehicle object
        
    public:
        CeresWolfFunctor(std::shared_ptr<WolfVehicle> & _wv) :
            vehicle_ptr_(_wv)
        {
            std::cout << "CeresWolfFunctor(): " << vehicle_ptr_.use_count() << std::endl;
        }
        
        virtual ~CeresWolfFunctor()
        {
            //
        }
        
        bool operator()(const WolfScalar * const _x, double* _residuals) const
        {
            // Remap the vehicle state to the const evaluation point
            vehicle_ptr_->remapConstState(_x);
            //std::cout << "_x         = " << *_x << " " << *(_x+1) << " " << *(_x+2) << " " << *(_x+3) << " " << *(_x+4) << std::endl;

            // Compute error or residuals
            vehicle_ptr_->computeError(_residuals);
            //std::cout << "_residuals = " << *_residuals << " " << *(_residuals+1) << " " << *(_residuals+2) << " " << *(_residuals+3) << " " << *(_residuals+4) << std::endl << std::endl;

            return true;
        }
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= Static Numeric case ===========" << std::endl << std::endl;
    
    //dimension 
    const unsigned int DIM = 5; //just to test, all will be DIM-dimensional
    const unsigned int SimulationSteps = 3;
    //aux vector
//     Eigen::VectorXs aux;
    
    // init
    google::InitGoogleLogging(argv[0]);
    
    //wolf vehicle 
    //std::shared_ptr<WolfVehicle> vehicle(new WolfVehicle);
    std::shared_ptr<WolfVehicle> vehiclePtr(std::make_shared<WolfVehicle>());
    
    //ceres functor
    CeresWolfFunctor *functorPtr = new CeresWolfFunctor(vehiclePtr);
    //std::shared_ptr<CeresWolfFunctor> functor(new CeresWolfFunctor(vehicle));
    //std::shared_ptr<CeresWolfFunctor> functorPtr(std::make_shared<CeresWolfFunctor>(vehiclePtr));
  
    // Allocate the cost function !!!! Difference is to create in the sameline both objects -> in the last (): (new ... ) SEE test_ceres_basic.cpp
    ceres::NumericDiffCostFunction<CeresWolfFunctor,ceres::CENTRAL,DIM,DIM>*
           cost_function_static = new ceres::NumericDiffCostFunction<CeresWolfFunctor,ceres::CENTRAL,DIM,DIM>(functorPtr);  

    // Ceres problem initialization
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.minimizer_type = ceres::TRUST_REGION;
    options.line_search_direction_type = ceres::LBFGS;
    options.max_num_iterations = 10;
    ceres::Solver::Summary summary;
    ceres::Problem problem;

    // fixed dim problem
    vehiclePtr->resizeState(DIM);
    vehiclePtr->resizeState(DIM);
    vehiclePtr->computePrior();
    vehiclePtr->print();

    //start Wolf iteration
    for (uint i=0; i<SimulationSteps; i++)
    {
        // set measures. This will be replaced by the WOLF-ROS front-end, getting sensor readings from sensors and performing measurements to build the whole wolf tree
        vehiclePtr->inventMeasurements(DIM);
        
        // Resizing & remapping. Dimensions may come from a call to WolfVehicle::getSizes()
        //functorPtr->setSizes(DIM,DIM);
//         aux.resize(DIM);
                      
        // Compute and gets the Prior (initial guess). This would be the prior given by Wolf
        //vehiclePtr->computePrior();
//         vehiclePtr->computePrior(aux);
               
        // build Ceres problem 
        //ceres::Problem *problem = new ceres::Problem();

//         ceres::ResidualBlockId rbId = problem.AddResidualBlock(cost_function_static, nullptr, vehiclePtr->getPrior());
        problem.AddResidualBlock(cost_function_static, nullptr, vehiclePtr->getPrior());
//         problem.AddResidualBlock(cost_function_static, nullptr, aux.data());

        // run Ceres Solver
        ceres::Solve(options, &problem, &summary);

        //display results
        std::cout << summary.BriefReport() << "\n";
        vehiclePtr->print();
        //std::cout << "aux: " << aux.transpose() << std::endl;
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

