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
//#include "ceres/loss_function.h"
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

enum correspondenceType {CORR_1_BLOCK=1, CORR_2_BLOCK=2, CORR_GPS3=3, CORR_RANGE_ONLY=4};
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

        virtual correspondenceType getType() const = 0;
        virtual WolfScalar** getBlockPtrArray() = 0;
        virtual const std::vector<WolfScalar *> getBlockPtrVector() = 0;

        // TODO: provide const expressions of block and measure sizes
        // virtual const unsigned int getMeasSize() const = 0;
        // virtual const unsigned int* getBlockSizeArray() const = 0;
};

template <const unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_SIZE = 1>
class Correspondence1Sparse: public CorrespondenceBase
{
    protected:
    	Map<Matrix<WolfScalar, BLOCK_SIZE, 1>> state_block_map_;

    public:

    	constexpr Correspondence1Sparse(WolfScalar* _statePtr) :
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

        virtual correspondenceType getType() const
        {
        	return CORR_1_BLOCK;
        }

        virtual WolfScalar** getBlockPtrArray()
        {
        	WolfScalar* block_ptrs[1] = {state_block_map_.data()};
        	return block_ptrs;
        }

		virtual const std::vector<WolfScalar *> getBlockPtrVector()
		{
			const std::vector<WolfScalar *> res{{state_block_map_.data()}};
			return res;
		}
};



template <const unsigned int MEASUREMENT_SIZE = 1, unsigned int BLOCK_1_SIZE = 1, unsigned int BLOCK_2_SIZE = 1>
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

        WolfScalar *getBlock1Ptr()
        {
            return state_block_1_map_.data();
        }

        WolfScalar *getBlock2Ptr()
        {
            return state_block_2_map_.data();
        }

        virtual correspondenceType getType() const
        {
        	return CORR_2_BLOCK;
        }

        virtual WolfScalar** getBlockPtrArray()
        {
        	WolfScalar* block_ptrs[2] = {state_block_1_map_.data(), state_block_2_map_.data()};
        	return block_ptrs;
        }

		virtual const std::vector<WolfScalar *> getBlockPtrVector()
		{
			const std::vector<WolfScalar *> res{{state_block_1_map_.data()}, {state_block_2_map_.data()}};
			return res;
		}
};

class CorrespondenceGPS3 : public Correspondence1Sparse<3,3>
{
	protected:
		 static const unsigned int N_BLOCKS = 1;
		 static const unsigned int MEASUREMENT_SIZE = 3;
		 static const unsigned int BLOCK_1_SIZE = 3;

	public:
		CorrespondenceGPS3(WolfScalar* _statePtr) :
			Correspondence1Sparse(_statePtr)
		{
		}

		~CorrespondenceGPS3()
		{
		}

        template <typename T>
        bool operator()(const T* const _x, T* _residuals) const
        {
        	//std::cout << "adress of x: " << _x << std::endl;

        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> state_map_const(_x, BLOCK_1_SIZE);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, MEASUREMENT_SIZE);

			// Compute error or residuals
			mapped_residuals = measurement_.cast<T>() - state_map_const;

        	return true;
        }

        virtual correspondenceType getType() const
        {
        	return CORR_GPS3;
        }
};

class CorrespondenceRangeOnly : public Correspondence2Sparse<1,3,3>
{
	protected:
		 static const unsigned int N_BLOCKS = 2;
		 static const unsigned int MEASUREMENT_SIZE = 1;
		 static const unsigned int BLOCK_1_SIZE = 3;
		 static const unsigned int BLOCK_2_SIZE = 3;

	public:
		CorrespondenceRangeOnly(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
			Correspondence2Sparse(_block1Ptr, _block2Ptr)
		{
		}

		~CorrespondenceRangeOnly()
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
			Matrix<T,Dynamic,1> expected_measurement = ((x1_map_const - x2_map_const).transpose() * (x1_map_const - x2_map_const)).cwiseSqrt();
			VectorXd meas = this->measurement_;
			mapped_residuals = (meas).cast<T>() - expected_measurement;

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

        virtual correspondenceType getType() const
        {
        	return CORR_RANGE_ONLY;
        }
};

class WolfProblem
{
    protected:
        VectorXs state_; //state storage
        std::list<CorrespondenceBase*> correspondences_;
        bool listChanged_;


    public: 
        WolfProblem() :
            state_(),
			correspondences_(0),
			listChanged_(false)
        {
        }
        
        virtual ~WolfProblem()
        {
        }
        
        WolfScalar *getPrior()
		{
			return state_.data();
		}

        std::list<CorrespondenceBase*> getCorrespondenceList()
        {
        	//std::cout << correspondences_.size() << " correspondences" << std::endl;
        	listChanged_ = false;
        	return correspondences_;
        }

        bool correspondenceListChanged()
        {
        	return listChanged_;
        }

        unsigned int getCorrespondencesSize()
        {
        	return correspondences_.size();
        }
        
        void resizeState(unsigned int _state_size)
        {
            state_.resize(_state_size);
        }

        void addCorrespondence(CorrespondenceBase* _absCorrPtr)
        {
        	correspondences_.push_back(_absCorrPtr);
        	listChanged_ = true;
        	//std::cout << correspondences_.size() << " correspondence added!" << std::endl;
        }

        void removeCorrespondence(CorrespondenceBase* _absCorrPtr)
        {
        	correspondences_.remove(_absCorrPtr);
        	listChanged_ = true;
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

class CeresWrapper
{
	protected:
		// WOLF
		WolfProblem* wolf_problem_ptr_;
		struct correspondence_wrapper
		{
			CorrespondenceBase* corr_ptr_;
			ceres::CostFunction* cost_function_ptr_;
			const std::vector<WolfScalar*> block_ptrs_;
		};
		std::list<correspondence_wrapper> correspondence_list_;

		// CERES
	    ceres::Problem ceres_problem_;
		ceres::Solver::Summary ceres_summary_;
		ceres::Solver::Options ceres_options_;

	public:
		CeresWrapper(WolfProblem* _wolf_problem_ptr, const ceres::Solver::Options& _ceres_options) :
			wolf_problem_ptr_(_wolf_problem_ptr),
			ceres_options_(_ceres_options)
		{
		}

		~CeresWrapper()
		{
		}

		void solve(const bool display_summary)
		{
			if (wolf_problem_ptr_->correspondenceListChanged())
				updateList();

			// add Residual Blocks
			addResidualBlocks();

			// run Ceres Solver
			ceres::Solve(ceres_options_, &ceres_problem_, &ceres_summary_);

			//display results
			if (display_summary)
				summary();
		}

		void summary()
		{
			std::cout << ceres_summary_.FullReport() << "\n";
			wolf_problem_ptr_->print();
		}

		void addResidualBlocks()
		{
			std::cout << "Adding residual blocks..." << std::endl;
			//int i = 0;
			for (std::list<correspondence_wrapper>::iterator it=correspondence_list_.begin(); it!=correspondence_list_.end(); ++it)
			{
				//std::cout << i++ << " block" << std::endl;
				ceres_problem_.AddResidualBlock(it->cost_function_ptr_, NULL, it->block_ptrs_);
			}
		}

		void updateList()
		{
			std::cout << "Updating Correspondence List..." << std::endl;
			correspondence_list_.clear();
			std::list<CorrespondenceBase*> new_corr_list = wolf_problem_ptr_->getCorrespondenceList();
			for (std::list<CorrespondenceBase*>::iterator it=new_corr_list.begin(); it!=new_corr_list.end(); ++it)
			{

				switch ((*it)->getType())
				{
					case CORR_GPS3:
					{
						// TODO: recuperar block sizes (MEASUREMENT I BLOCKS) const unsigned int MEAS_SIZE = (*it)->getMeasSize();

						CorrespondenceGPS3* corr_ptr = static_cast<CorrespondenceGPS3*>(*it);
						ceres::CostFunction* new_cost_function_ptr_ = new ceres::AutoDiffCostFunction<CorrespondenceGPS3,3,3>(corr_ptr);

						//constraint new_constraint = {(*it),new_cost_function_ptr_, new_block_ptrs_};
						correspondence_list_.push_back(correspondence_wrapper{(*it),new_cost_function_ptr_, (*it)->getBlockPtrVector()});
						break;
					}
					case CORR_RANGE_ONLY:
					{
						CorrespondenceRangeOnly* corr_ptr = static_cast<CorrespondenceRangeOnly*>(*it);
						ceres::CostFunction* new_cost_function_ptr_ = new ceres::AutoDiffCostFunction<CorrespondenceRangeOnly,1,3,3>(corr_ptr);

						correspondence_list_.push_back(correspondence_wrapper{(*it),new_cost_function_ptr_, (*it)->getBlockPtrVector()});
						break;
					}
					case CORR_1_BLOCK:
					{
						std::cout << "CORR_1_BLOCK" << std::endl;
						break;
					}
					case CORR_2_BLOCK:
					{
						std::cout << "CORR_2_BLOCK" << std::endl;
						break;
					}
					default:
						std::cout << "Unknown correspondence type!" << std::endl;
				}

			}
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
    const unsigned int N_MEAS_A = 1;
    const unsigned int N_MEAS_B = 1;
    // TODO: incorporar weights a les funcions residu (via LossFunction o directament a operador())
    //const double w_A = 1;
    //const double w_B = 10;

    // init
    google::InitGoogleLogging(argv[0]);
    std::default_random_engine generator;
    std::normal_distribution<WolfScalar> distribution_A(0.0,0.01);
    std::normal_distribution<WolfScalar> distribution_B(0.0,0.1);
    VectorXs actualState(STATE_DIM);
    for (uint i=0;i<STATE_DIM; i++)
    	actualState(i) = i;

    // Ceres problem initialization
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_progress_to_stdout = false;
    ceres_options.minimizer_type = ceres::TRUST_REGION;
    ceres_options.line_search_direction_type = ceres::LBFGS;
    ceres_options.max_num_iterations = 100;

    //wolf problem
	WolfProblem* wolf_problem = new WolfProblem();
    wolf_problem->resizeState(STATE_DIM);
    wolf_problem->computePrior();
    std::cout << "Real state: " << actualState.transpose() << std::endl;
    wolf_problem->print();

    // Ceres wrapper
    CeresWrapper ceres_wrapper(wolf_problem, ceres_options);

    // Correspondences
    // SENSOR A: Absolute measurements of the whole state
    for(uint mA=0; mA < N_MEAS_A; mA++)
    {
    	for (uint st=0; st < N_STATES; st++)
		{
    		CorrespondenceGPS3* corrAPtr = new CorrespondenceGPS3(wolf_problem->getPrior()+st*DIM);
			VectorXs actualMeasurement = actualState.segment(st*DIM, DIM);
			corrAPtr->inventMeasurement(actualMeasurement,generator,distribution_A);
			wolf_problem->addCorrespondence(corrAPtr);
		}
    }
	// SENSOR B: Relative distances between points
    for(uint mB=0; mB < N_MEAS_B; mB++)
	{
    	for (uint st_from=0; st_from < N_STATES-1; st_from++)
    	{
    		for (uint st_to=st_from+1; st_to < N_STATES; st_to++)
			{
    			std::cout << "Range only from " << st_from << " (" << st_from*DIM << "-" << st_from*DIM+DIM-1 << ")";
    			std::cout << " to " << st_to << " (" << st_to*DIM << "-" << st_to*DIM+DIM-1 << ")" << std::endl;
    			CorrespondenceRangeOnly* corrBPtr = new CorrespondenceRangeOnly(wolf_problem->getPrior()+st_from*DIM,wolf_problem->getPrior()+st_to*DIM);
				VectorXs actualMeasurement = ((actualState.segment(st_from*DIM,DIM) - actualState.segment(st_to*DIM,DIM)).transpose() * (actualState.segment(st_from*DIM,DIM) - actualState.segment(st_to*DIM,DIM))).cwiseSqrt();
				corrBPtr->inventMeasurement(actualMeasurement,generator,distribution_B);
				wolf_problem->addCorrespondence(corrBPtr);
			}
    	}
	}

	// run Ceres Solver
	ceres_wrapper.solve(true);

	// test, solve again, only should add the residual blocks but not update the correspondence list
	std::cout << "Solve again, it should only add residual blocks but not update the correspondence list" << std::endl;
	ceres_wrapper.solve(true);

    //clean
    delete wolf_problem;
    
    //end Wolf iteration
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

