//std includes
#include <iostream>
#include <memory>
#include <random>
#include <typeinfo>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//Ceres includes
#include "ceres/jet.h"
#include "ceres/ceres.h"
#include "ceres/loss_function.h"
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

class CorrespondenceBase
{
	protected:
    	VectorXs measurement_;

    public:

        CorrespondenceBase(const unsigned int & _measurement_size) :
			measurement_(_measurement_size)
        {
        }

        virtual ~CorrespondenceBase()
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

class CorrespondenceCeresBase
{
	protected:
        ceres::CostFunction* cost_function_;

    public:

        CorrespondenceCeresBase()
        {
        }

        virtual ~CorrespondenceCeresBase()
        {
        }

        virtual void addBlock(ceres::Problem & ceres_problem) = 0;
};

template <unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_SIZE = 1>
class Correspondence1Sparse: public CorrespondenceBase
{
    protected:
    	Map<Matrix<WolfScalar, BLOCK_SIZE, 1>> state_block_map_;

    public:

        Correspondence1Sparse(WolfScalar* _statePtr) :
        	CorrespondenceBase(MEASUREMENT_SIZE),
			state_block_map_(_statePtr,BLOCK_SIZE)
        {
        }

        virtual ~Correspondence1Sparse()
        {
        }

        virtual WolfScalar* getBlockPtr()
        {
            return state_block_map_.data();
        }

        template <typename T>
        //void compute_residuals(const Matrix<T,Dynamic,1>& _x, Matrix<T,Dynamic,1> residuals)
        void compute_residuals(Map<const Matrix<T,Dynamic,1>>& _st, Map<Matrix<T,Dynamic,1>> residuals) const
        {
        	residuals = measurement_.cast<T>() - _st;
        	// std::cout << typeid(T).name() << std::endl;
			// for (unsigned int j = 0; j<BLOCK_SIZE; j++)
			// {
			// 	std::cout << "_x(" << j <<") = " << _x[j] << std::endl;
			// 	std::cout << "mapped_residuals(" << j <<") = " << mapped_residuals(j) << std::endl;
			// }
        }
};

template <unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_SIZE = 1>
class Correspondence1SparseCeres: public CorrespondenceCeresBase, public Correspondence1Sparse<MEASUREMENT_SIZE, BLOCK_SIZE>
{
    public:

        Correspondence1SparseCeres(WolfScalar* _statePtr) :
			CorrespondenceCeresBase(),
        	Correspondence1Sparse<MEASUREMENT_SIZE, BLOCK_SIZE>(_statePtr)
        {
			cost_function_  = new ceres::AutoDiffCostFunction<Correspondence1SparseCeres,MEASUREMENT_SIZE,BLOCK_SIZE>(this);
        }

        virtual ~Correspondence1SparseCeres()
        {
        }

        template <typename T>
        bool operator()(const T* const _x, T* _residuals) const
        {
        	//std::cout << "adress of x: " << _x << std::endl;

        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const(_x, BLOCK_SIZE);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, MEASUREMENT_SIZE);

			// Compute error or residuals
			this->template compute_residuals<T>(state_map_const, mapped_residuals);

        	return true;
        }

        virtual void addBlock(ceres::Problem & ceres_problem)
        {
        	//std::cout << " adding correspondence_1_sparse_ceres...";
        	ceres_problem.AddResidualBlock(cost_function_, NULL, this->state_block_map_.data());
        	//std::cout << " added!" << std::endl;
        }
};



template <unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_1_SIZE = 1, unsigned int BLOCK_2_SIZE = 1>
class Correspondence2Sparse: public CorrespondenceBase
{
    protected:
		Map<Matrix<WolfScalar, BLOCK_1_SIZE, 1>> state_block_1_map_;
		Map<Matrix<WolfScalar, BLOCK_2_SIZE, 1>> state_block_2_map_;

    public:

        Correspondence2Sparse(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
        	CorrespondenceBase(MEASUREMENT_SIZE),
			state_block_1_map_(_block1Ptr,BLOCK_1_SIZE),
			state_block_2_map_(_block2Ptr,BLOCK_2_SIZE)
        {
        }

        virtual ~Correspondence2Sparse()
        {
        }

        template <typename T>
        void compute_residuals(Map<const Matrix<T,Dynamic,1>>& _st1, Map<const Matrix<T,Dynamic,1>>& _st2, Map<Matrix<T,Dynamic,1>> residuals) const
        {
        	Matrix<T,Dynamic,1> expected_measurement = ((_st1 - _st2).transpose() * (_st1 - _st2)).cwiseSqrt();
			VectorXd meas = this->measurement_;
			residuals = (meas).cast<T>() - expected_measurement;
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
class Correspondence2SparseCeres: public CorrespondenceCeresBase, public Correspondence2Sparse<MEASUREMENT_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE>
{
    public:

        Correspondence2SparseCeres(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
			CorrespondenceCeresBase(),
			Correspondence2Sparse<MEASUREMENT_SIZE, BLOCK_1_SIZE, BLOCK_2_SIZE>(_block1Ptr, _block2Ptr)
        {
			cost_function_  = new ceres::AutoDiffCostFunction<Correspondence2SparseCeres,MEASUREMENT_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE>(this);
        }

        virtual ~Correspondence2SparseCeres()
        {
        }

        template <typename T>
        bool operator()(const T* const _x1, const T* const _x2, T* _residuals) const
        {
        	// print inputs
        	// std::cout << "_x1 = ";
        	// for (int i=0; i < BLOCK_1_SIZE; i++)
        	// 	std::cout << _x1[i] << " ";
        	// std::cout << std::endl;
        	// std::cout << "_x2 = ";
        	// for (int i=0; i < BLOCK_2_SIZE; i++)
        	// 	std::cout << _x2[i] << " ";
        	// std::cout << std::endl;
        	// std::cout << "measurement = ";
        	// for (int i=0; i < MEASUREMENT_SIZE; i++)
        	// 	std::cout << this->measurement_(i) << " ";
        	// std::cout << std::endl;

        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> x1_map_const(_x1, BLOCK_1_SIZE);
			Map<const Matrix<T,Dynamic,1>> x2_map_const(_x2, BLOCK_2_SIZE);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, MEASUREMENT_SIZE);

			// Compute error or residuals
			this->template compute_residuals(x1_map_const, x2_map_const, mapped_residuals);

			// print outputs
			// std::cout << "expected    = ";
			// for (int i=0; i < MEASUREMENT_SIZE; i++)
			// 	std::cout << expected_measurement(i) << " ";
			// std::cout << std::endl;
			// std::cout << "_residuals  = ";
			// for (int i=0; i < MEASUREMENT_SIZE; i++)
			// 	std::cout << _residuals[i] << " ";
			// std::cout << std::endl << std::endl;

			return true;
        }

        virtual void addBlock(ceres::Problem & ceres_problem)
        {
        	//std::cout << " adding correspondence_2_sparse_ceres...";
        	ceres_problem.AddResidualBlock(cost_function_, NULL, this->state_block_1_map_.data(), this->state_block_2_map_.data());
        	//std::cout << " added!" << std::endl;
        }
};

class WolfProblem
{
    protected:
        VectorXs state_; //state storage
        std::vector<CorrespondenceCeresBase*> correspondences_;

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

        CorrespondenceCeresBase* getCorrespondence(const unsigned int _idx)
        {
        	//std::cout << correspondences_.size() << " correspondences" << std::endl;
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

        void addCorrespondence(CorrespondenceCeresBase* _absCorrPtr)
        {
        	correspondences_.push_back(_absCorrPtr);
        	//std::cout << correspondences_.size() << " correspondence added!" << std::endl;
        }

        void computePrior()
        {
            state_.setRandom();
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
    const unsigned int DIM = 3;
    const unsigned int N_STATES = 10;
    const unsigned int STATE_DIM = DIM * N_STATES;
    const unsigned int MEAS_A_DIM = 3;
    const unsigned int MEAS_B_DIM = 1;
    const unsigned int N_MEAS_A = 100;
    const unsigned int N_MEAS_B = 50;
    //const double w_A = 1;
    //const double w_B = 10;

    // init
    google::InitGoogleLogging(argv[0]);
    std::default_random_engine generator;
    std::normal_distribution<WolfScalar> distribution_A(0.0,0.01);
    std::normal_distribution<WolfScalar> distribution_B(0.0,0.1);
    VectorXs actualState(STATE_DIM);
    for (unsigned int i=0;i<STATE_DIM; i++)
    	actualState(i) = i;

    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;

    // Ceres problem initialization
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.minimizer_type = ceres::TRUST_REGION;
    options.line_search_direction_type = ceres::LBFGS;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Problem ceres_problem;

    //wolf problem
	WolfProblem *wolf_problem = new WolfProblem();
    wolf_problem->resizeState(STATE_DIM);
    wolf_problem->computePrior();
    std::cout << "Real state: " << actualState.transpose() << std::endl;
    wolf_problem->print();

    // Correspondences
    // SENSOR A: Absolute measurements of the whole state
    for(unsigned int mA=0; mA < N_MEAS_A; mA++)
    {
    	for (unsigned int st=0; st < N_STATES; st++)
		{
			Correspondence1SparseCeres<MEAS_A_DIM, DIM>* corrAPtr = new Correspondence1SparseCeres<MEAS_A_DIM, DIM>(wolf_problem->getPrior()+st*DIM);
			VectorXs actualMeasurement = actualState.segment(st*DIM,DIM);
			corrAPtr->inventMeasurement(actualMeasurement,generator,distribution_A);
			wolf_problem->addCorrespondence(corrAPtr);
		}
    }
	// SENSOR B: Relative distances between points
    for(unsigned int mB=0; mB < N_MEAS_B; mB++)
	{
    	for (unsigned int st_from=0; st_from < N_STATES-1; st_from++)
    	{
    		for (unsigned int st_to=st_from+1; st_to < N_STATES; st_to++)
			{
    			Correspondence2SparseCeres<MEAS_B_DIM, DIM, DIM>* corrBPtr = new Correspondence2SparseCeres<MEAS_B_DIM, DIM, DIM>(wolf_problem->getPrior()+st_from*DIM,wolf_problem->getPrior()+st_to*DIM);
				VectorXs actualMeasurement = ((actualState.segment(st_from*DIM,DIM) - actualState.segment(st_to*DIM,DIM)).transpose() * (actualState.segment(st_from*DIM,DIM) - actualState.segment(st_to*DIM,DIM))).cwiseSqrt();
				corrBPtr->inventMeasurement(actualMeasurement,generator,distribution_B);
				wolf_problem->addCorrespondence(corrBPtr);
			}
    	}
//    	correspondence_2_sparse_ceres<MEAS_B_DIM, DIM, DIM>* corrBPtr = new correspondence_2_sparse_ceres<MEAS_B_DIM, DIM, DIM>(wolf_problem->getPrior(),wolf_problem->getPrior()+DIM);
//		VectorXs actualMeasurement = ((actualState.head(DIM) - actualState.tail(DIM)).transpose() * (actualState.head(DIM) - actualState.tail(DIM))).cwiseSqrt();
//		corrBPtr->inventMeasurement(actualMeasurement,generator,distribution_B);
//		wolf_problem->addCorrespondence(corrBPtr);
	}

	// cost function
    std::cout << "Number of blocks: " << std::endl << wolf_problem->getCorrespondencesSize() << std::endl;
	for (unsigned int block=0; block < wolf_problem->getCorrespondencesSize(); block++)
	{
		//std::cout << "block " << block << "...";
		wolf_problem->getCorrespondence(block)->addBlock(ceres_problem);
		//std::cout << " added!" << std::endl;
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

