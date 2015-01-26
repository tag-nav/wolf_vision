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

using namespace Eigen;
enum correspondenceType {
	CORR_1_BLOCK    = 1,
	CORR_2_BLOCK    = 2,
	CORR_N_BLOCKS   = 3,
	CORR_GPS3       = 4,
	CORR_RANGE_ONLY = 5};
enum parametrizationType {
	NONE          = 0,
	COMPLEX_ANGLE = 1,
	QUATERNION    = 2};

class StateBase
{
	protected:
		Map<VectorXs> state_map_;

	public:
		StateBase(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
			state_map_(_st_remote.data(), _idx, _size)
		{
		}

		StateBase(WolfScalar* _st_ptr, const unsigned int _size) :
			state_map_(_st_ptr, _size)
		{
		}

		virtual ~StateBase()
		{
		}

		virtual Map<VectorXs> getMap()
		{
			return state_map_;
		}

		virtual WolfScalar* getPointer()
		{
			return state_map_.data();
		}

		virtual WolfScalar* getPPointer()
		{
			return NULL;
		}

		virtual WolfScalar* getOPointer()
		{
			return NULL;
		}

		virtual parametrizationType isParametrized() const = 0;

		virtual WolfScalar* getParametrizedPointer() = 0;
};

class StatePoint3D: public StateBase
{
	public:
		StatePoint3D(VectorXs& _st_remote, const unsigned int _idx) :
			StateBase(_st_remote, _idx, 3)
		{
		}

		StatePoint3D(WolfScalar* _st_ptr) :
			StateBase(_st_ptr, 3)
		{
		}

		virtual ~StatePoint3D()
		{
		}

		virtual WolfScalar* getPPointer()
		{
			return this->getPointer();
		}

		virtual parametrizationType isParametrized() const
		{
			return NONE;
		}

		virtual WolfScalar* getParametrizedPointer()
		{
			return NULL;
		}
};

class StatePO: public StateBase
{
	public:
		StatePO(VectorXs& _st_remote, const unsigned int _idx) :
			StateBase(_st_remote, _idx, 4)
		{
		}

		StatePO(WolfScalar* _st_ptr) :
			StateBase(_st_ptr, 4)
		{
		}

		virtual WolfScalar* getPPointer()
		{
			return this->getPointer();
		}

		virtual WolfScalar* getOPointer()
		{
			return this->getPointer()+2;
		}

		virtual ~StatePO()
		{
		}

		virtual parametrizationType isParametrized() const
		{
			return COMPLEX_ANGLE;
		}

		virtual WolfScalar* getParametrizedPointer()
		{
			return this->state_map_.data() + 2;
		}
};

class ComplexAngleParameterization : public ceres::LocalParameterization
{
	public:
		virtual ~ComplexAngleParameterization()
		{
		}

		virtual bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const
		{
			x_plus_delta_raw[0] = x_raw[0] * cos(delta_raw[0]) - x_raw[1] * sin(delta_raw[0]);
			x_plus_delta_raw[1] = x_raw[1] * cos(delta_raw[0]) + x_raw[0] * sin(delta_raw[0]);

			//normalize
			double norm = sqrt(x_plus_delta_raw[0] * x_plus_delta_raw[0] + x_plus_delta_raw[1] * x_plus_delta_raw[1]);
			std::cout << "(before normalization) norm = " << norm << std::endl;
			x_plus_delta_raw[0] /= norm;
			x_plus_delta_raw[1] /= norm;

			return true;
		}

		virtual bool ComputeJacobian(const double* x, double* jacobian) const
		{
			jacobian[0] = -x[1];
			jacobian[1] =  x[0];
			return true;
		}

		virtual int GlobalSize() const
		{
			return 2;
		}

		virtual int LocalSize() const
		{
			return 1;
		}
};

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
        virtual const std::vector<WolfScalar *> getBlockPtrVector() = 0;
};

template <const unsigned int MEASUREMENT_SIZE = 1,
				unsigned int BLOCK_0_SIZE = 1,
				unsigned int BLOCK_1_SIZE = 1,
				unsigned int BLOCK_2_SIZE = 0,
				unsigned int BLOCK_3_SIZE = 0,
				unsigned int BLOCK_4_SIZE = 0,
				unsigned int BLOCK_5_SIZE = 0,
				unsigned int BLOCK_6_SIZE = 0,
				unsigned int BLOCK_7_SIZE = 0,
				unsigned int BLOCK_8_SIZE = 0,
				unsigned int BLOCK_9_SIZE = 0>
class CorrespondenceSparse: public CorrespondenceBase
{
    protected:
		std::vector<Map<VectorXs>> state_block_map_vector_;
		std::vector<WolfScalar*> state_block_ptr_vector_;
		std::vector<unsigned int> block_sizes_vector_;

    public:
		static const unsigned int measurementSize = MEASUREMENT_SIZE;
		static const unsigned int block0Size = BLOCK_0_SIZE;
		static const unsigned int block1Size = BLOCK_1_SIZE;
		static const unsigned int block2Size = BLOCK_2_SIZE;
		static const unsigned int block3Size = BLOCK_3_SIZE;
		static const unsigned int block4Size = BLOCK_4_SIZE;
		static const unsigned int block5Size = BLOCK_5_SIZE;
		static const unsigned int block6Size = BLOCK_6_SIZE;
		static const unsigned int block7Size = BLOCK_7_SIZE;
		static const unsigned int block8Size = BLOCK_8_SIZE;
		static const unsigned int block9Size = BLOCK_9_SIZE;

		CorrespondenceSparse(WolfScalar** _blockPtrArray) :
        	CorrespondenceBase(MEASUREMENT_SIZE),
			block_sizes_vector_({BLOCK_0_SIZE,
								 BLOCK_1_SIZE,
								 BLOCK_2_SIZE,
								 BLOCK_3_SIZE,
								 BLOCK_4_SIZE,
								 BLOCK_5_SIZE,
								 BLOCK_6_SIZE,
								 BLOCK_7_SIZE,
								 BLOCK_8_SIZE,
								 BLOCK_9_SIZE})
        {
			for (uint i = 0; i<block_sizes_vector_.size(); i++)
			{
				if (block_sizes_vector_.at(i) == 0)
				{
					block_sizes_vector_.resize(i);
					break;
				}
				else
				{
					state_block_map_vector_.push_back(Map<VectorXs>(_blockPtrArray[i],block_sizes_vector_.at(i)));
					state_block_ptr_vector_.push_back(_blockPtrArray[i]);
				}
			}
        }

		CorrespondenceSparse(WolfScalar* _state0Ptr,
							 WolfScalar* _state1Ptr = NULL,
							 WolfScalar* _state2Ptr = NULL,
							 WolfScalar* _state3Ptr = NULL,
							 WolfScalar* _state4Ptr = NULL,
							 WolfScalar* _state5Ptr = NULL,
							 WolfScalar* _state6Ptr = NULL,
							 WolfScalar* _state7Ptr = NULL,
							 WolfScalar* _state8Ptr = NULL,
							 WolfScalar* _state9Ptr = NULL ) :
			CorrespondenceBase(MEASUREMENT_SIZE),
			state_block_ptr_vector_({_state0Ptr,
									 _state1Ptr,
									 _state2Ptr,
									 _state3Ptr,
									 _state4Ptr,
									 _state5Ptr,
									 _state6Ptr,
									 _state7Ptr,
									 _state8Ptr,
									 _state9Ptr}),
			block_sizes_vector_({BLOCK_0_SIZE,
								 BLOCK_1_SIZE,
								 BLOCK_2_SIZE,
								 BLOCK_3_SIZE,
								 BLOCK_4_SIZE,
								 BLOCK_5_SIZE,
								 BLOCK_6_SIZE,
								 BLOCK_7_SIZE,
								 BLOCK_8_SIZE,
								 BLOCK_9_SIZE})
		{
			for (uint i = 0; i<block_sizes_vector_.size(); i++)
			{
				if (block_sizes_vector_.at(i) == 0)
				{
					block_sizes_vector_.resize(i);
					state_block_ptr_vector_.resize(i);
					break;
				}
				else
				{
					state_block_map_vector_.push_back(Map<VectorXs>(state_block_ptr_vector_.at(i),block_sizes_vector_.at(i)));
				}
			}
		}

		CorrespondenceSparse(StateBase* _state0Ptr,
							 StateBase* _state1Ptr = NULL,
							 StateBase* _state2Ptr = NULL,
							 StateBase* _state3Ptr = NULL,
							 StateBase* _state4Ptr = NULL,
							 StateBase* _state5Ptr = NULL,
							 StateBase* _state6Ptr = NULL,
							 StateBase* _state7Ptr = NULL,
							 StateBase* _state8Ptr = NULL,
							 StateBase* _state9Ptr = NULL ) :
			CorrespondenceBase(MEASUREMENT_SIZE),
			state_block_ptr_vector_({_state0Ptr->getPointer(),
			 	 	 	 	 	 	 _state1Ptr==NULL ? NULL : _state1Ptr->getPointer(),
									 _state2Ptr==NULL ? NULL : _state2Ptr->getPointer(),
									 _state3Ptr==NULL ? NULL : _state3Ptr->getPointer(),
									 _state4Ptr==NULL ? NULL : _state4Ptr->getPointer(),
									 _state5Ptr==NULL ? NULL : _state5Ptr->getPointer(),
									 _state6Ptr==NULL ? NULL : _state6Ptr->getPointer(),
									 _state7Ptr==NULL ? NULL : _state7Ptr->getPointer(),
									 _state8Ptr==NULL ? NULL : _state8Ptr->getPointer(),
									 _state9Ptr==NULL ? NULL : _state9Ptr->getPointer()}),
			block_sizes_vector_({BLOCK_0_SIZE,
								 BLOCK_1_SIZE,
								 BLOCK_2_SIZE,
								 BLOCK_3_SIZE,
								 BLOCK_4_SIZE,
								 BLOCK_5_SIZE,
								 BLOCK_6_SIZE,
								 BLOCK_7_SIZE,
								 BLOCK_8_SIZE,
								 BLOCK_9_SIZE})
		{
			for (uint i = 0; i<block_sizes_vector_.size(); i++)
			{
				if (block_sizes_vector_.at(i) == 0)
				{
					block_sizes_vector_.resize(i);
					state_block_ptr_vector_.resize(i);
					break;
				}
				else
				{
					state_block_map_vector_.push_back(Map<VectorXs>(state_block_ptr_vector_.at(i),block_sizes_vector_.at(i)));
				}
			}
		}

        virtual ~CorrespondenceSparse()
        {
        }

        virtual correspondenceType getType() const
        {
        	return CORR_N_BLOCKS;
        }

		virtual const std::vector<WolfScalar *> getBlockPtrVector()
		{
			return state_block_ptr_vector_;
		}
};

class CorrespondenceGPS3 : public CorrespondenceSparse<3,3>
{
	public:
		static const unsigned int N_BLOCKS = 1;

		CorrespondenceGPS3(WolfScalar* _statePtrs) :
			CorrespondenceSparse(_statePtrs)
		{
		}

		CorrespondenceGPS3(StateBase* _statePtr) :
			CorrespondenceSparse(_statePtr)
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
			Map<const Matrix<T,Dynamic,1>> state_map_const(_x, this->block1Size);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, this->measurementSize);

			// Compute error or residuals
			mapped_residuals = measurement_.cast<T>() - state_map_const;

			return true;
		}

		virtual correspondenceType getType() const
		{
			return CORR_GPS3;
		}
};

class CorrespondenceRangeOnly : public CorrespondenceSparse<1,3,3>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		CorrespondenceRangeOnly(WolfScalar** _blockPtrs) :
			CorrespondenceSparse(_blockPtrs)
		{
		}

		CorrespondenceRangeOnly(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
			CorrespondenceSparse(_block1Ptr, _block2Ptr)
		{
		}

		CorrespondenceRangeOnly(StatePoint3D* _state1Ptr, StatePoint3D* _state2Ptr) :
			CorrespondenceSparse(_state1Ptr, _state2Ptr)
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
        	// for (int i=0; i < this->block1Size; i++)
        	// 	std::cout << _x1[i] << " ";
        	// std::cout << std::endl;
        	// std::cout << "_x2 = ";
        	// for (int i=0; i < this->block2Size; i++)
        	// 	std::cout << _x2[i] << " ";
        	// std::cout << std::endl;
        	// std::cout << "measurement = ";
        	// for (int i=0; i < this->measurementSize; i++)
        	// 	std::cout << this->measurement_(i) << " ";
        	// std::cout << std::endl;

        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> x1_map_const(_x1, this->block1Size);
			Map<const Matrix<T,Dynamic,1>> x2_map_const(_x2, this->block2Size);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, this->measurementSize);

			// Compute error or residuals
			Matrix<T,Dynamic,1> expected_measurement = ((x1_map_const - x2_map_const).transpose() * (x1_map_const - x2_map_const)).cwiseSqrt();
			VectorXd meas = this->measurement_;
			mapped_residuals = (meas).cast<T>() - expected_measurement;

			// print outputs
			// std::cout << "expected    = ";
			// for (int i=0; i < this->measurementSize; i++)
			// 	std::cout << expected_measurement(i) << " ";
			// std::cout << std::endl;
			// std::cout << "_residuals  = ";
			// for (int i=0; i < this->measurementSize; i++)
			// 	std::cout << _residuals[i] << " ";
			// std::cout << std::endl << std::endl;

			return true;
        }

        virtual correspondenceType getType() const
        {
        	return CORR_RANGE_ONLY;
        }
};

class CorrespondenceOrientedLandmark : public CorrespondenceSparse<3,4,4>
{
	public:
		static const unsigned int N_BLOCKS = 2;

		CorrespondenceOrientedLandmark(WolfScalar** _blockPtrs) :
			CorrespondenceSparse(_blockPtrs)
		{
		}

		CorrespondenceOrientedLandmark(WolfScalar* _block1Ptr, WolfScalar* _block2Ptr) :
			CorrespondenceSparse(_block1Ptr, _block2Ptr)
		{
		}

		CorrespondenceOrientedLandmark(StatePoint3D* _state1Ptr, StatePoint3D* _state2Ptr) :
			CorrespondenceSparse(_state1Ptr, _state2Ptr)
		{
		}

		~CorrespondenceOrientedLandmark()
		{
		}

        template <typename T>
        bool operator()(const T* const _x1, const T* const _x2, T* _residuals) const
        {
        	// print inputs
        	// std::cout << "_x1 = ";
        	// for (int i=0; i < this->block1Size; i++)
        	// 	std::cout << _x1[i] << " ";
        	// std::cout << std::endl;
        	// std::cout << "_x2 = ";
        	// for (int i=0; i < this->block2Size; i++)
        	// 	std::cout << _x2[i] << " ";
        	// std::cout << std::endl;
        	// std::cout << "measurement = ";
        	// for (int i=0; i < this->measurementSize; i++)
        	// 	std::cout << this->measurement_(i) << " ";
        	// std::cout << std::endl;

        	// Remap the vehicle state to the const evaluation point
			Map<const Matrix<T,Dynamic,1>> x1_map_const(_x1, this->block1Size);
			Map<const Matrix<T,Dynamic,1>> x2_map_const(_x2, this->block2Size);

			// Map residuals vector to matrix (with sizes of the measurements matrix)
			Map<Matrix<T,Dynamic,1>> mapped_residuals(_residuals, this->measurementSize);

			// Compute error or residuals
			Matrix<T,Dynamic,1> expected_measurement = ((x1_map_const - x2_map_const).transpose() * (x1_map_const - x2_map_const)).cwiseSqrt();
			VectorXd meas = this->measurement_;
			mapped_residuals = (meas).cast<T>() - expected_measurement;

			// print outputs
			// std::cout << "expected    = ";
			// for (int i=0; i < this->measurementSize; i++)
			// 	std::cout << expected_measurement(i) << " ";
			// std::cout << std::endl;
			// std::cout << "_residuals  = ";
			// for (int i=0; i < this->measurementSize; i++)
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

        void addCorrespondence(CorrespondenceBase* _corrPtr)
        {
        	correspondences_.push_back(_corrPtr);
        	listChanged_ = true;
        }

        void removeCorrespondence(CorrespondenceBase* _corrPtr)
        {
        	correspondences_.remove(_corrPtr);
        	listChanged_ = true;
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

class CeresManager
{
	protected:
		// WOLF
		WolfProblem* wolf_problem_ptr_;
		struct CorrespondenceWrapper
		{
			CorrespondenceBase* corr_ptr_;
			ceres::CostFunction* cost_function_ptr_;
			const std::vector<WolfScalar*> block_ptrs_;
		};
		std::list<CorrespondenceWrapper> correspondence_list_;

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

		~CeresManager()
		{
		}

		void solve(const bool display_summary)
		{
			if (wolf_problem_ptr_->correspondenceListChanged())
				updateCorrespondenceList();

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
			for (std::list<CorrespondenceWrapper>::iterator it=correspondence_list_.begin(); it!=correspondence_list_.end(); ++it)
			{
				//std::cout << i++ << " block" << std::endl;
				ceres_problem_.AddResidualBlock(it->cost_function_ptr_, NULL, it->block_ptrs_);
			}
		}

		void updateCorrespondenceList()
		{
			std::cout << "Updating Correspondence List..." << std::endl;
			correspondence_list_.clear();
			std::list<CorrespondenceBase*> new_corr_list = wolf_problem_ptr_->getCorrespondenceList();
			for (std::list<CorrespondenceBase*>::iterator it=new_corr_list.begin(); it!=new_corr_list.end(); ++it)
			{
				ceres::CostFunction* cost_function_ptr;
				switch ((*it)->getType())
				{
					case CORR_GPS3:
					{
						cost_function_ptr = createCostFunction<CorrespondenceGPS3>(*it);
						break;
					}
					case CORR_RANGE_ONLY:
					{
						cost_function_ptr = createCostFunction<CorrespondenceRangeOnly>(*it);
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
				correspondence_list_.push_back(CorrespondenceWrapper{(*it), cost_function_ptr, (*it)->getBlockPtrVector()});
			}
		}

		template <typename CorrespondenceDerived>
		ceres::CostFunction* createCostFunction(CorrespondenceBase* _corrBasePtr)
		{
			CorrespondenceDerived* corrCastedPtr = static_cast<CorrespondenceDerived*>(_corrBasePtr);
			return new ceres::AutoDiffCostFunction<CorrespondenceDerived,
												   corrCastedPtr->measurementSize,
												   corrCastedPtr->block0Size,
												   corrCastedPtr->block1Size,
												   corrCastedPtr->block2Size,
												   corrCastedPtr->block3Size,
												   corrCastedPtr->block4Size,
												   corrCastedPtr->block5Size,
												   corrCastedPtr->block6Size,
												   corrCastedPtr->block7Size,
												   corrCastedPtr->block8Size,
												   corrCastedPtr->block9Size>(corrCastedPtr);
		}
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= Static Numeric case ===========" << std::endl << std::endl;
    
    //dimension 
    const unsigned int DIM = 3;
    const unsigned int N_STATES = 20;
    const unsigned int STATE_DIM = DIM * N_STATES;
    const unsigned int N_MEAS_A = 10;
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
    CeresManager ceres_wrapper(wolf_problem, ceres_options);

	// States pointers
	std::vector<StatePoint3D*> state_ptr_vector_(N_STATES);
	for (uint st=0; st < N_STATES; st++)
		state_ptr_vector_.at(st) = new StatePoint3D(wolf_problem->getPrior()+DIM*st);

	std::cout << "States created!" << std::endl;

    // Correspondences
    // SENSOR A: Absolute measurements of the whole state
    for(uint mA=0; mA < N_MEAS_A; mA++)
    {
    	std::cout << "Correspondences A set: " << mA << std::endl;
    	std::cout << "state_ptr_vector_.size() = " << state_ptr_vector_.size() << std::endl;
    	for (uint st=0; st < N_STATES; st++)
		{
        	CorrespondenceGPS3* corrAPtr = new CorrespondenceGPS3(state_ptr_vector_.at(st));
			VectorXs actualMeasurement = actualState.segment(st*DIM,DIM);
        	std::cout << "State = " << actualMeasurement.transpose() << std::endl;
			corrAPtr->inventMeasurement(actualMeasurement,generator,distribution_A);
			wolf_problem->addCorrespondence(corrAPtr);
		}
    }
    std::cout << "Correspondences A created!" << std::endl;

	// SENSOR B: Relative distances between points
    for(uint mB=0; mB < N_MEAS_B; mB++)
	{
    	for (uint st_from=0; st_from < N_STATES-1; st_from++)
    	{
    		for (uint st_to=st_from+1; st_to < N_STATES; st_to++)
			{
    			//std::cout << "Range only from " << st_from << " (" << st_from*DIM << "-" << st_from*DIM+DIM-1 << ")";
    			//std::cout << " to " << st_to << " (" << st_to*DIM << "-" << st_to*DIM+DIM-1 << ")" << std::endl;
    			CorrespondenceRangeOnly* corrBPtr = new CorrespondenceRangeOnly(state_ptr_vector_[st_from],state_ptr_vector_[st_to]);
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

