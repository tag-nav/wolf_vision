#ifndef PROBLEM_H_
#define PROBLEM_H_

// Fwd refs
namespace wolf{
class HardwareBase;
class TrajectoryBase;
class MapBase;
class ProcessorMotion;
class TimeStamp;
struct IntrinsicsBase;
struct ProcessorParamsBase;
}

//wolf includes
#include "node_base.h"
#include "sensor_base.h"
//#include "sensor_factory.h"
#include "wolf.h"

// std includes
#include <utility> // pair


namespace wolf {


/** \brief Wolf problem node element in the Wolf Tree
 * 
 * A node has five main data members:
 * - An unique ID to identify it over the whole Wolf Tree (inherited from Node)
 * - A label indicating the node nature (inherited from Node)
 * - An enum indicating tree location (see NodeLocation enum at wolf.h)
 * - down_node_list_: A list of shared pointers to derived node objects, specified by the template parameter LowerType.
 * - up_node_: A regular pointer to a derived node object, specified by the template parameter UpperType.
 *
 */
class Problem : public NodeBase
{
    public:
        typedef NodeBase* LowerNodePtr; // Necessatry for destruct() of node_linked

    protected:
        std::map<std::pair<StateBlock*, StateBlock*>, Eigen::MatrixXs> covariances_;
        NodeLocation location_; // TODO: should it be in node_base?
        TrajectoryBase* trajectory_ptr_;
        MapBase* map_ptr_;
        HardwareBase* hardware_ptr_;
        ProcessorMotion* processor_motion_ptr_;
        StateBlockList state_block_ptr_list_;
        std::list<StateBlock*> state_block_add_list_;
        std::list<StateBlock*> state_block_update_list_;
        std::list<Scalar*> state_block_remove_list_;
        std::list<ConstraintBase*> constraint_add_list_;
        std::list<unsigned int> constraint_remove_list_;

    public:

        /** \brief Constructor from frame structure
         *
         */
        Problem(FrameStructure _frame_structure);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         */
        virtual ~Problem();

        /** \brief Wolf destructor
         *
         * Wolf destructor (please use it instead of delete for guaranteeing the wolf tree integrity)
         *
         */
        virtual void destruct() final;


        /** \brief add sensor to hardware
         */
        void addSensor(SensorBase* _sen_ptr);

        /** \brief Factory method to add sensor only from its properties
         */
        SensorBase* addSensor(std::string _sen_type, std::string _unique_sensor_name, Eigen::VectorXs& _extrinsics, IntrinsicsBase* _intrinsics);

        ProcessorBase* addProcessor(std::string _sen_type, std::string _unique_processor_name, std::string _corresponding_sensor_name, ProcessorParamsBase* _prc_params);

        /** \brief Set the processor motion
         *
         * Set the processor motion. It will provide the state.
         *
         */
        void setProcessorMotion(ProcessorMotion* _processor_motion_ptr);

        /** \brief Create Frame of the correct size
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem
         */
        FrameBase* createFrame(FrameType _frameType, const TimeStamp& _time_stamp);

        /** \brief Create Frame from vector
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem
         */
        FrameBase* createFrame(FrameType _frame_type, const Eigen::VectorXs& _frame_state,
                               const TimeStamp& _time_stamp);

        /** \brief Get the state at last timestamp
         *
         * Get the state at last timestamp
         */
        Eigen::VectorXs getCurrentState();
        void getCurrentState(Eigen::VectorXs& state);

        /** \brief Get the state at a given timestamp
         *
         * Get the state at a given timestamp
         */
        Eigen::VectorXs getStateAtTimeStamp(const TimeStamp& _ts);
        void getStateAtTimeStamp(const TimeStamp& _ts, Eigen::VectorXs& state);

        /** \brief Give the permission to a processor to create a new keyFrame
         *
         * Give the permission to a processor to create a new keyFrame
         */
        bool permitKeyFrame(ProcessorBase* _processor_ptr);

        /** \brief New key frame callback
         *
         * New key frame callback: It should be called by any processor that creates a new keyframe. It calls the keyFrameCallback of the rest of processors.
         */
        void keyFrameCallback(FrameBase* _keyframe_ptr, ProcessorBase* _processor_ptr);

        LandmarkBase* addLandmark(LandmarkBase* _lmk_ptr);

        void addLandmarkList(LandmarkBaseList _lmk_list);

        /** \brief Adds a new state block to be added to solver manager
         */
        StateBlock* addStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a new state block to be updated to solver manager
         */
        void updateStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a state block to be removed to solver manager
         */
        void removeStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a new constraint to be added to solver manager
         */
        ConstraintBase* addConstraintPtr(ConstraintBase* _constraint_ptr);

        /** \brief Adds a constraint to be removed to solver manager
         */
        void removeConstraintPtr(ConstraintBase* _constraint_ptr);

        /** \brief Clear covariance
         */
        void clearCovariance();

        /** \brief Adds a new covariance block
         */
        void addCovarianceBlock(StateBlock* _state1, StateBlock* _state2, const Eigen::MatrixXs& _cov);

        /** \brief Gets a covariance block
         */
        bool getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov, const int _row = 0,
                                const int _col=0);

        /** \brief Gets the covariance of a frame
         */
        bool getFrameCovariance(FrameBase* _frame_ptr, Eigen::MatrixXs& _covariance);
        Eigen::MatrixXs getFrameCovariance(FrameBase* _frame_ptr);

        /** \brief Gets the covariance of a frame
         */
        bool getLandmarkCovariance(LandmarkBase* _landmark_ptr, Eigen::MatrixXs& _covariance);
        Eigen::MatrixXs getFrameCovariance(LandmarkBase* _landmark_ptr);

        /** \brief Adds a map
         */
        MapBase* addMap(MapBase* _map_ptr);

        /** \brief Adds a trajectory
         */
        TrajectoryBase* addTrajectory(TrajectoryBase* _trajectory_ptr);

        /** \brief Gets a pointer to map
         */
        MapBase* getMapPtr();

        /** \brief Gets a pointer to trajectory
         */
        TrajectoryBase* getTrajectoryPtr();

        /** \brief Gets a pointer to Hardware
         */
        HardwareBase* getHardwarePtr();

        /** \brief Returns a pointer to last Frame
         **/
        FrameBase* getLastFramePtr();

        /** \brief Gets a pointer to the state units list
         */
        StateBlockList* getStateListPtr();

        /** \brief Gets a queue of state blocks to be added in the solver
         */
        std::list<StateBlock*>* getStateBlockAddList();

        /** \brief Gets a queue of state blocks to be updated in the solver
         */
        std::list<StateBlock*>* getStateBlockUpdateList();

        /** \brief Gets a queue of state blocks to be removed from the solver
         */
        std::list<Scalar*>* getStateBlockRemoveList();

        /** \brief Gets a queue of constraint ids to be added in the solver
         */
        std::list<ConstraintBase*>* getConstraintAddList();

        /** \brief Gets a queue of constraint ids to be removed from the solver
         */
        std::list<unsigned int>* getConstraintRemoveList();

        /** \brief get top node
         */
        virtual Problem* getProblem();

        /** \brief Returns a true (is top)
         */
        virtual bool isTop();

        /** \brief Remove Down Node
         *
         * This empty function is needed by the destruct() node_linked function.
         */
        void removeDownNode(const LowerNodePtr _ptr){};

};

} // namespace wolf

// IMPLEMENTATION

namespace wolf
{

inline std::list<Scalar*>* Problem::getStateBlockRemoveList()
{
    return &state_block_remove_list_;
}

inline std::list<ConstraintBase*>* Problem::getConstraintAddList()
{
    return &constraint_add_list_;
}

inline std::list<unsigned int>* Problem::getConstraintRemoveList()
{
    return &constraint_remove_list_;
}

inline Problem* Problem::getProblem()
{
    return this;
}

inline bool Problem::isTop()
{
    return true;
}

} // namespace wolf


#endif // PROBLEM_H
