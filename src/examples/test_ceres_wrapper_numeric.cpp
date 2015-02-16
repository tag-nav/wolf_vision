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
class WolfVehicle
{
    protected:
        Eigen::VectorXs state_; //state storage where to compute prior, and where result is placed. Is the allocation for the state vector
        Eigen::Map<const Eigen::VectorXs> state_map_; //state point to be evaluated by Wolf tree constraints 
        Eigen::Map<Eigen::VectorXs> error_map_; //error computed by the wolf tree
        unsigned int state_size_;
        unsigned int error_size_;        
        
        //Just to generate fake measurements
        std::default_random_engine generator_; //just to generate measurements
        std::normal_distribution<double> distribution_; //just to generate measurements
        Eigen::VectorXs measurements_; //just a set of invented measurements

    public: 
        WolfVehicle() :
            state_(),
            state_map_(nullptr,0),
            error_map_(nullptr,0),
            state_size_(0),
            error_size_(0),
            distribution_(0.0,0.1)
        {
            //
        }
        
        virtual ~WolfVehicle()
        {
            //
        }
        
        void setSizes(unsigned int _state_size, unsigned int _error_size)
        {
            state_size_ = _state_size;
            error_size_ = _error_size;
            state_.resize(_state_size);
        }
        
        void remapState(const WolfScalar *_ptr)
        {
            new (&state_map_) Eigen::Map<const Eigen::VectorXs>(_ptr, state_size_);            
        }        

        void remapError(WolfScalar *_ptr)
        {
            new (&error_map_) Eigen::Map<Eigen::VectorXs>(_ptr, error_size_);            
        }                
        
        void inventMeasurements(unsigned int _sz)
        {
            measurements_.resize(_sz);
            for(unsigned int ii=0; ii<_sz; ii++)
            {
                measurements_(ii) = 1 + distribution_(generator_); //just inventing a sort of noise measurements
            }
        }
        
        WolfScalar *getState()
        {
            return state_.data();
        }        
                
        void getState(Eigen::VectorXs & _v)
        {  
            _v.resize(state_.size());
            _v = state_;
        }
        
        void computePrior()
        {
            state_.setOnes();//just a fake prior
        }                
        
        void computeError()
        {
            for(unsigned int ii=0; ii<error_map_.size(); ii++)
            {
                error_map_(ii) = measurements_(ii) - state_map_(ii); //just a trivial error function
            }
        }

        void print()
        {
            std::cout << "measurements_: " << measurements_.transpose() << std::endl;
            std::cout << "state_: " << state_.transpose() << std::endl;
            //std::cout << "state_map_: " << state_map_.transpose() << std::endl;
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
            //std::cout << "CeresWolfFunctor(): " << vehicle_ptr_.use_count() << " " << state_ptr_.use_count() << " " << std::endl;
        }
        
        virtual ~CeresWolfFunctor()
        {
            //
        }
        bool operator()(const WolfScalar * const _x, double* _residuals) const
        {
            // 1. Remap the vehicle state to the provided x
            vehicle_ptr_->remapState(_x);

            // 2. Remap the error to the provided address
            vehicle_ptr_->remapError(_residuals);

            // 3. Compute error
            vehicle_ptr_->computeError();

            // 4. return 
            return true;
        }
        
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= Static Numeric case ===========" << std::endl << std::endl;
    
    //dimension 
    const unsigned int DIM = 19; //just to test, all will be DIM-dimensional
    
    // init
    google::InitGoogleLogging(argv[0]);
    
    //wolf vehicle 
    std::shared_ptr<WolfVehicle> vehiclePtr(std::make_shared<WolfVehicle>());
    
    //ceres functor
    CeresWolfFunctor *functorPtr = new CeresWolfFunctor(vehiclePtr);
  
    // Allocate the cost function !!!! Difference is to create in the same line both objects -> in the last (): (new ... ) SEE test_ceres_basic.cpp
    ceres::NumericDiffCostFunction<CeresWolfFunctor,ceres::CENTRAL,DIM,DIM>* 
           cost_function_static = new ceres::NumericDiffCostFunction<CeresWolfFunctor,ceres::CENTRAL,DIM,DIM>(functorPtr);  
           
    //********************** start Wolf iteration *************************
           
        // set measures. This will be replaced by the WOLF-ROS front-end, getting sensor reading from sensors (callbacks) and performing measurements to build the whole wolf tree
        vehiclePtr->inventMeasurements(DIM);
        
        // Resizing & remapping. Dimensions may come from a call to WolfVehicle::getSizes()
        vehiclePtr->setSizes(DIM,DIM);

        // Compute and gets the Prior (initial guess). This would be the prior given by Wolf
        vehiclePtr->computePrior();
               
        // build Ceres problem 
        ceres::Problem *problem = new ceres::Problem();
        problem->AddResidualBlock(cost_function_static, nullptr, vehiclePtr->getState());

        // run Ceres Solver
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.minimizer_type = ceres::TRUST_REGION;
        options.line_search_direction_type = ceres::LBFGS;
        options.max_num_iterations = 10;
        ceres::Solver::Summary summary;
        ceres::Solve(options, problem, &summary);

        //display results
        std::cout << std::endl<< "Ceres Report:" << std::endl;
        std::cout << summary.BriefReport() << "\n";
        std::cout << std::endl << "Wolf vectors:" << std::endl;        
        vehiclePtr->print();
        
    //********************** end Wolf iteration *************************        
        
    //clean
    //std::cout << "Cleaning ... " << std::endl << std::endl;
    //something to do ??
    delete problem;
    //delete vehiclePtr;
    //delete cost_function_static;
    //delete cost_function_static;
       
    //end Wolf iteration
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

