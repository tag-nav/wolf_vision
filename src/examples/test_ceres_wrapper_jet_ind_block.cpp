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

class AbsoluteCorrespondence
{
    protected:
        Map<VectorXs> state_block_mapped_;

        //Just to generate fake measurements
        std::default_random_engine generator_; //just to generate measurements
        std::normal_distribution<WolfScalar> distribution_; //just to generate measurements
        VectorXs measurement_; // invented measurement
        ceres::CostFunction* cost_function;

    public:

        AbsoluteCorrespondence(WolfScalar* _statePtr, const unsigned int & _block_size) :
        	state_block_mapped_(_statePtr,_block_size),
            distribution_(0.0,0.01),
			measurement_(_block_size),
			cost_function(new ceres::AutoDiffCostFunction<AbsoluteCorrespondence,1,1>(this))
        {
        }

        virtual ~AbsoluteCorrespondence()
        {
        }

        WolfScalar *getPrior()
        {
            return state_block_mapped_.data();
        }

        void inventMeasurement(std::default_random_engine& _generator)
        {
            for(unsigned int ii=0; ii<state_block_mapped_.size(); ii++)
                measurement_(ii) = 1 + distribution_(_generator); //just inventing a sort of noise measurements
        }

        template <typename T>
        bool operator()(const T* const _x, T* _residuals) const
        {
        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const_(_x, state_block_mapped_.size());

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, state_block_mapped_.size());

			// Compute error or residuals
			mapped_residuals = measurement_.cast<T>() - state_map_const_;

			// std::cout << typeid(T).name() << std::endl;
			// for (unsigned int j = 0; j<block_size_; j++)
			// {
			// 	std::cout << "_x(" << j <<") = " << _x[j] << std::endl;
			// 	std::cout << "mapped_residuals(" << j <<") = " << mapped_residuals(j) << std::endl;
			// }

        	return true;
        }

        void addBlock(ceres::Problem & ceres_problem)
        {
        	ceres_problem.AddResidualBlock(cost_function, NULL, getPrior());
        }
};

class WolfProblem
{
    protected:
        VectorXs state_; //state storage
        std::vector<AbsoluteCorrespondence*> correspondences_;

        unsigned int state_size_;

    public: 
        WolfProblem() :
            state_(),
			correspondences_(0),
			state_size_(0)
        {
        }
        
        virtual ~WolfProblem()
        {
        }
        
        WolfScalar *getPrior()
		{
			return state_.data();
		}

        AbsoluteCorrespondence* getCorrespondence(const unsigned int _idx)
        {
            return correspondences_[_idx];
        }

        unsigned int getCorrespondencesSize()
        {
        	return correspondences_.size();
        }
        
        void resizeState(unsigned int _state_size)
        {
        	state_size_ = _state_size;
            state_.resize(_state_size);
        }

        void addCorrespondence(AbsoluteCorrespondence* _absCorrPtr)
        {
        	correspondences_.push_back(_absCorrPtr);
        }

        void inventMeasurements(std::default_random_engine& _generator)
        {
        	for (std::vector<AbsoluteCorrespondence*>::iterator it = correspondences_.begin(); it != correspondences_.end(); it++)
        		(*it)->inventMeasurement(_generator);
        }
        
        void computePrior()
        {
            state_.setZero();
        }

        void print()
        {
            std::cout << "state_ : " << std::endl << state_.transpose() << std::endl << std::endl;
        }
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= Static Numeric case ===========" << std::endl << std::endl;
    
    //dimension 
    const unsigned int STATE_DIM = 50; //just to test, all will be DIM-dimensional
    const unsigned int N_MEASUREMENTS = 1000;
    // init
    google::InitGoogleLogging(argv[0]);
    std::default_random_engine generator;

    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;

    // Ceres problem initialization
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.minimizer_type = ceres::TRUST_REGION;
    options.line_search_direction_type = ceres::LBFGS;
    options.max_num_iterations = 10;
    ceres::Solver::Summary summary;
    ceres::Problem ceres_problem;

    //wolf problem
	WolfProblem *wolf_problem = new WolfProblem();
    wolf_problem->resizeState(STATE_DIM);
    wolf_problem->computePrior();
    for(unsigned int st=0; st < N_MEASUREMENTS; st++)
		for (unsigned int st=0; st < STATE_DIM; st++)
			wolf_problem->addCorrespondence(new AbsoluteCorrespondence(wolf_problem->getPrior()+st,1));

	// set measures. This will be replaced by the WOLF-ROS front-end, getting sensor readings from sensors and performing measurements to build the whole wolf tree
    wolf_problem->inventMeasurements(generator);

	// cost function
    std::cout << "Number of blocks: " << std::endl << wolf_problem->getCorrespondencesSize() << std::endl;
	for (unsigned int block=0; block < wolf_problem->getCorrespondencesSize(); block++)
		wolf_problem->getCorrespondence(block)->addBlock(ceres_problem);

	// run Ceres Solver
	ceres::Solve(options, &ceres_problem, &summary);

	//display results
	std::cout << summary.FullReport() << "\n";
    wolf_problem->print();

    //clean
    std::cout << "Cleaning ... " << std::endl << std::endl;
    //ceres_problem.RemoveResidualBlock(rbId);
    delete wolf_problem;
    
    //end Wolf iteration
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

