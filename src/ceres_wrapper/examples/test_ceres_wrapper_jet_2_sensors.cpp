//std includes
#include <iostream>
#include <memory>
#include <random>
#include <typeinfo>

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

class correspondence_base
{
	protected:
    	VectorXs measurement_;

    public:

        correspondence_base(const unsigned int & _measurement_size) :
			measurement_(_measurement_size)
        {
        }

        virtual ~correspondence_base()
        {
        }

        virtual void inventMeasurement(const VectorXs& _measurement, std::default_random_engine& _generator, std::normal_distribution<WolfScalar>& _distribution)
		{
			measurement_ = _measurement;
			for(unsigned int ii=0; ii<measurement_.size(); ii++)
				measurement_(ii) += _distribution(_generator); //just inventing a sort of noise measurements
			//std::cout << "measurement_" << measurement_ << std::endl;
		}
};

class correspondence_ceres_base
{
	protected:
        ceres::CostFunction* cost_function_;

    public:

        correspondence_ceres_base()
        {
        }

        virtual ~correspondence_ceres_base()
        {
        }

        virtual void addBlock(ceres::Problem & ceres_problem) = 0;
};

template <unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_SIZE = 1>
class correspondence_1_sparse: public correspondence_base
{
    protected:
    	Map<Matrix<WolfScalar, BLOCK_SIZE, 1>> state_block_map_;

    public:

        correspondence_1_sparse(WolfScalar* _statePtr) :
        	correspondence_base(MEASUREMENT_SIZE),
			state_block_map_(_statePtr,BLOCK_SIZE)
        {
        }

        virtual ~correspondence_1_sparse()
        {
        }

        virtual WolfScalar* getBlockPtr()
        {
            return state_block_map_.data();
        }

        template <typename T>
        //void compute_residuals(const Matrix<T,Dynamic,1>& _x, Matrix<T,Dynamic,1> residuals)
        void compute_residuals(const T* const _x, T* _residuals)
        {
				// Remap the vehicle state to the const evaluation point
				Map<const Matrix<T,Dynamic,1>> state_map_const(_x, BLOCK_SIZE);

				// Map residuals vector to matrix (with sizes of the measurements matrix)
				Map<Matrix<T,Dynamic,1>> residuals(_residuals, MEASUREMENT_SIZE);

        	residuals = measurement_.cast<T>() - _x;
        	// std::cout << typeid(T).name() << std::endl;
			// for (unsigned int j = 0; j<BLOCK_SIZE; j++)
			// {
			// 	std::cout << "_x(" << j <<") = " << _x[j] << std::endl;
			// 	std::cout << "mapped_residuals(" << j <<") = " << mapped_residuals(j) << std::endl;
			// }
        }
};

template <unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_SIZE = 1>
class correspondence_1_sparse_ceres: public correspondence_ceres_base, public correspondence_1_sparse<MEASUREMENT_SIZE, BLOCK_SIZE>
{
    public:

        correspondence_1_sparse_ceres(WolfScalar* _statePtr, const unsigned int & _block_size) :
			correspondence_ceres_base(),
        	correspondence_1_sparse<MEASUREMENT_SIZE, BLOCK_SIZE>(_statePtr)
        {
			cost_function_  = new ceres::AutoDiffCostFunction<correspondence_1_sparse_ceres,MEASUREMENT_SIZE,BLOCK_SIZE>(this);
        }

        virtual ~correspondence_1_sparse_ceres()
        {
        }

        template <typename T>
        bool operator()(const T* const _x, T* _residuals) const
        {
        	//this->template compute_residuals(_x, _residuals);

        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const(_x, BLOCK_SIZE);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, MEASUREMENT_SIZE);

			// Compute error or residuals
			//correspondence_1_sparse<MEASUREMENT_SIZE, BLOCK_SIZE>::template compute_residuals<T>(state_map_const, mapped_residuals);
			//compute_residuals<T>(_x, _residuals);
			VectorXd meas = this->measurement_;
			mapped_residuals = meas.cast<T>() - state_map_const;

        	return true;
        }

        virtual void addBlock(ceres::Problem & ceres_problem)
        {
    		std::cout << " adding residual block...";
        	ceres_problem.AddResidualBlock(cost_function_, NULL, this->state_block_map_.data());
        }

//        virtual WolfScalar* getBlockPtr()
//        {
//            return this->state_block_map_.data();
//        }
};



template <unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_1_SIZE = 1, unsigned int BLOCK_2_SIZE = 1>
class correspondence_2_sparse: public correspondence_base
{
    protected:
		Map<Matrix<WolfScalar, BLOCK_1_SIZE, 1>> state_block_1_map_;
		Map<Matrix<WolfScalar, BLOCK_2_SIZE, 1>> state_block_2_map_;

    public:

        correspondence_2_sparse(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
        	correspondence_base(MEASUREMENT_SIZE),
			state_block_1_map_(_block1Ptr,BLOCK_1_SIZE),
			state_block_2_map_(_block2Ptr,BLOCK_2_SIZE)
        {
        }

        virtual ~correspondence_2_sparse()
        {
        }

        template <typename T>
        void compute_residuals(const Matrix<T,Dynamic,1>& _st1, const Matrix<T,Dynamic,1>& _st2, Matrix<T,Dynamic,1> residuals)
        {
        	residuals = measurement_.cast<T>() - _st1 - _st2;
        	// std::cout << typeid(T).name() << std::endl;
			// for (unsigned int j = 0; j<BLOCK_SIZE; j++)
			// {
			// 	std::cout << "_x(" << j <<") = " << _x[j] << std::endl;
			// 	std::cout << "mapped_residuals(" << j <<") = " << mapped_residuals(j) << std::endl;
			// }
        }

        WolfScalar *getBlock1Ptr()
        {
            return state_block_1_map_.data();
        }

        WolfScalar *getBlock2Ptr()
        {
            return state_block_2_map_.data();
        }
};

template <unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_1_SIZE = 1, unsigned int BLOCK_2_SIZE = 1>
class correspondence_2_sparse_ceres: public correspondence_ceres_base, public correspondence_2_sparse<MEASUREMENT_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE>
{
    public:

        correspondence_2_sparse_ceres(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
			correspondence_ceres_base(),
			correspondence_2_sparse<MEASUREMENT_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE>(_block1Ptr, _block2Ptr)
        {
			cost_function_  = new ceres::AutoDiffCostFunction<correspondence_2_sparse_ceres,MEASUREMENT_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE>(this);
        }

        virtual ~correspondence_2_sparse_ceres()
        {
        }

        template <typename T>
        bool operator()(const T* const _x1, const T* const _x2, T* _residuals) const
        {
        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> x1_map_const_(_x1, BLOCK_1_SIZE);
			Map<const Matrix<T,Dynamic,1>> x2_map_const_(_x2, BLOCK_2_SIZE);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, MEASUREMENT_SIZE);

			// Compute error or residuals
			correspondence_2_sparse<MEASUREMENT_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE>::template compute_residuals(x1_map_const_, x2_map_const_, mapped_residuals);

        	return true;
        }

        virtual void addBlock(ceres::Problem & ceres_problem)
        {
        	ceres_problem.AddResidualBlock(cost_function_, NULL, correspondence_2_sparse<MEASUREMENT_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE>::state_block_1_map_.data(), correspondence_2_sparse<MEASUREMENT_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE>::state_block_2_map_.data());
        }
};

class WolfProblem
{
    protected:
        VectorXs state_; //state storage
        std::vector<correspondence_base*> correspondences_;

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

        correspondence_base* getCorrespondence(const unsigned int _idx)
        {
        	std::cout << correspondences_.size() << " correspondences" << std::endl;
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

        void addCorrespondence(correspondence_base* _absCorrPtr)
        {
        	correspondences_.push_back(_absCorrPtr);
        	//std::cout << correspondences_.size() << " correspondence added!" << std::endl;
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
    const unsigned int STATE_DIM = 5; //just to test, all will be DIM-dimensional
    const unsigned int N_MEASUREMENTS = 10;
    // init
    google::InitGoogleLogging(argv[0]);
    std::default_random_engine generator;
    std::normal_distribution<WolfScalar> distribution(0.0,0.01);
    VectorXs actualState(STATE_DIM);

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
    for(uint st=0; st < N_MEASUREMENTS; st++)
    {
    	for (uint st=0; st < STATE_DIM; st++)
		{
			correspondence_1_sparse_ceres<1,1> corrPtr(wolf_problem->getPrior()+st,1);
			corrPtr.inventMeasurement(VectorXs::Zero(1),generator,distribution);
			wolf_problem->addCorrespondence(&corrPtr);
		}
    }

	// cost function
    std::cout << "Number of blocks: " << std::endl << wolf_problem->getCorrespondencesSize() << std::endl;
	for (uint block=0; block < wolf_problem->getCorrespondencesSize(); block++)
	{
		std::cout << "block " << block << "...";
		//correspondence_ceres_base *ptr = static_cast<correspondence_ceres_base*>(wolf_problem->getCorrespondence(block));
		correspondence_1_sparse_ceres<>* ptr1 = (correspondence_1_sparse_ceres<>*)wolf_problem->getCorrespondence(block);
		correspondence_ceres_base* ptrc= (correspondence_ceres_base*)ptr1;

		std::cout << " casted...";
		//correspondence_1_sparse_ceres<>* ptr = (correspondence_1_sparse_ceres<>*)wolf_problem->getCorrespondence(block);
		ptrc->addBlock(ceres_problem);
		std::cout << " added!" << std::endl;
	}

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

