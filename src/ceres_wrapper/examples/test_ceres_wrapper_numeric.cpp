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
        Eigen::VectorXs state_prior_; //state storage where to compute prior
        Eigen::Map<Eigen::VectorXs> state_map_; //state point to be evaluated by Wolf tree constraints 
        
        //Just to generate fake measurements
        std::default_random_engine generator_; //just to generate measurements
        std::normal_distribution<double> distribution_; //just to generate measurements
        Eigen::VectorXs measurements_; //just a set of invented measurements

    public: 
        WolfVehicle() :
            state_prior_(),
            state_map_(nullptr,0),
            distribution_(0.0,0.1)
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
            state_prior_.resize(_state_size);
        }        

        void remapState(WolfScalar *_ptr, unsigned int _state_size)
        {
            new (&state_map_) Eigen::Map<Eigen::VectorXs>(_ptr, _state_size);            
        }        

        void inventMeasurements(unsigned int _sz)
        {
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
        
        void computePrior(Eigen::VectorXs & _v)
        {
            state_prior_.setOnes();
            _v.resize(state_prior_.size());
            _v = state_prior_;
        }
        
        void computeError(Eigen::Map<Eigen::VectorXs> & _residuals, unsigned int & _index)
        {
            for(unsigned int ii=_index; ii<_residuals.size(); ii++)
            {
                _residuals(ii) = measurements_(ii) - state_map_(ii); //just a trivial error function
            }
        }

        void print()
        {
            std::cout << "measurements_: " << measurements_.transpose() << std::endl;
            std::cout << "state_prior_: " << state_prior_.transpose() << std::endl;
            std::cout << "state_: " << state_map_.transpose() << std::endl;
        }                
};

/**
 * This class wrapps the Wolf Vehicle and implements the operator(), to be used by CERES optimizer
 *      - the operator() overloaded
**/
class CeresWolfFunctor
{
    protected: 
        //WolfVehicle *vehicle_ptr_; //pointer to Wolf Vehicle object
        std::shared_ptr<WolfVehicle> vehicle_ptr_; //pointer to Wolf Vehicle object
        std::shared_ptr<Eigen::VectorXs> state_ptr_; //pointer to state storage. 
        std::shared_ptr<Eigen::Map<Eigen::VectorXs> > error_ptr_; //Pointer to a Map to an error vector. Ceres call to () will indicate where to map it.
        unsigned int state_size_;
        unsigned int error_size_;
        
    protected:
        void setState(const WolfScalar * const _st) const //, unsigned int _state_size)
        {
            for (unsigned int ii=0; ii<state_ptr_->size(); ii++) (*state_ptr_)(ii) = _st[ii];
        }
        
    public:
        CeresWolfFunctor(std::shared_ptr<WolfVehicle> & _wv) :
            vehicle_ptr_(_wv),
            state_ptr_(std::make_shared<Eigen::VectorXs>()),
            error_ptr_(std::make_shared<Eigen::Map<Eigen::VectorXs> >(nullptr,0)),
            state_size_(0),
            error_size_(0)
        {
            //std::cout << "CeresWolfFunctor(): " << vehicle_ptr_.use_count() << " " << state_ptr_.use_count() << " " << std::endl;
        }
        
        virtual ~CeresWolfFunctor()
        {
            //
        }
                
        void setStateSize(unsigned int _state_size)
        {
            state_size_ = _state_size;
            state_ptr_->resize(_state_size);
            vehicle_ptr_->resizeState(_state_size);
        }

        void setErrorSize(unsigned int _error_size) 
        {
            error_size_ = _error_size;
        }
        
        void setSizes(unsigned int _state_size, unsigned int _error_size)
        {
            setStateSize(_state_size);
            setErrorSize(_error_size);    
        }
        
        bool operator()(const WolfScalar * const _x, double* _residuals) const
        {
            unsigned int error_index = 0;
            
            // 1. set the state
            this->setState(_x); //hard copy from x_ to state_. Assumes sizes are ok
            
            // 2. Remap the vehicle state to the local copy
            vehicle_ptr_->remapState(state_ptr_->data(), state_size_);

            // 3. Remap the error to the provided address
            new (error_ptr_.get()) Eigen::Map<Eigen::VectorXs>(_residuals, error_size_);

            // 4. Compute error
            vehicle_ptr_->computeError(*error_ptr_, error_index);

            // 5. return 
            return true;
        }
        
        void printState()
        {
            std::cout << "state_: " << state_ptr_->transpose() << std::endl;
        }        
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= Static Numeric case ===========" << std::endl << std::endl;
    
    //dimension 
    const unsigned int DIM = 19; //just to test, all will be DIM-dimensional
    
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
           
//    ceres::NumericDiffCostFunction<CeresWolfFunctor,ceres::CENTRAL,DIM,DIM>
//        *cost_function_static = new ceres::NumericDiffCostFunction<CeresWolfFunctor,ceres::CENTRAL,DIM,DIM>(new CeresWolfFunctor(vehiclePtr));  
           

    //start Wolf iteration
        // set measures. This will be replaced by the WOLF-ROS front-end, getting sensor readings from sensors and performing measurements to build the whole wolf tree
        vehiclePtr->inventMeasurements(DIM);
        
        // Resizing & remapping. Dimensions may come from a call to WolfVehicle::getSizes()
        functorPtr->setSizes(DIM,DIM);
//         aux.resize(DIM);
                      
        // Compute and gets the Prior (initial guess). This would be the prior given by Wolf
        vehiclePtr->computePrior();
//         vehiclePtr->computePrior(aux);
               
        // build Ceres problem 
        //ceres::Problem *problem = new ceres::Problem();
        ceres::Problem problem;
//         ceres::ResidualBlockId rbId = problem.AddResidualBlock(cost_function_static, nullptr, vehiclePtr->getPrior());
        problem.AddResidualBlock(cost_function_static, nullptr, vehiclePtr->getPrior());
//         problem.AddResidualBlock(cost_function_static, nullptr, aux.data());

        // run Ceres Solver
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.minimizer_type = ceres::TRUST_REGION;
        options.line_search_direction_type = ceres::LBFGS;
        options.max_num_iterations = 10;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        //display results
        std::cout << summary.BriefReport() << "\n";
        vehiclePtr->print();
        //std::cout << "aux: " << aux.transpose() << std::endl;
        
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

