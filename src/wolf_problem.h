#ifndef WOLF_PROBLEM_H_
#define WOLF_PROBLEM_H_

// Fwd refs
class HardwareBase;
class TrajectoryBase;
class MapBase;
class TimeStamp;

//wolf includes
#include "node_base.h"
#include "wolf.h"

// std includes
#include <utility> // pair

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
class WolfProblem : public NodeBase
{
    public:
        typedef NodeBase* LowerNodePtr; // Necessatry for destruct() of node_linked

    protected:
        std::map<std::pair<StateBlock*, StateBlock*>, Eigen::MatrixXs> covariances_;
        NodeLocation location_; // TODO: should it be in node_base?
        TrajectoryBase* trajectory_ptr_;
        MapBase* map_ptr_;
        HardwareBase* hardware_ptr_;
        StateBlockList state_block_ptr_list_;
        std::list<StateBlock*> state_block_add_list_;
        std::list<StateBlock*> state_block_update_list_;
        std::list<WolfScalar*> state_block_remove_list_;
        std::list<ConstraintBase*> constraint_add_list_;
        std::list<unsigned int> constraint_remove_list_;

    public:

        /** \brief Constructor from frame structure
         *
         */
        WolfProblem(FrameStructure _frame_structure);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         */
        virtual ~WolfProblem();

        /** \brief Wolf destructor
         *
         * Wolf destructor (please use it instead of delete for guaranteeing the wolf tree integrity)
         *
         */
        virtual void destruct() final;

        void addSensor(SensorBase* _sen_ptr);

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

        bool permitKeyFrame(ProcessorBase* _processor_ptr);

        void addLandmark(LandmarkBase* _lmk_ptr);

        void addLandmarkList(LandmarkBaseList _lmk_list);

        /** \brief Adds a new state block to be added to solver manager
         */
        void addStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a new state block to be updated to solver manager
         */
        void updateStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a state block to be removed to solver manager
         */
        void removeStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a new constraint to be added to solver manager
         */
        void addConstraintPtr(ConstraintBase* _constraint_ptr);

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
        void getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov_block);
        /** \brief Gets a covariance block
         */
        void getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov, const int _row,
                                const int _col);

        /** \brief Gets state size
         */
        const unsigned int getStateSize() const;

        /** \brief Adds a map
         */
        void addMap(MapBase* _map_ptr);

        /** \brief Adds a trajectory
         */
        void addTrajectory(TrajectoryBase* _trajectory_ptr);

        /** \brief Adds a hardware
         */
        void addHarware(HardwareBase* _hardware_ptr);

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
        std::list<WolfScalar*>* getStateBlockRemoveList();

        /** \brief Gets a queue of constraint ids to be added in the solver
         */
        std::list<ConstraintBase*>* getConstraintAddList();

        /** \brief Gets a queue of constraint ids to be removed from the solver
         */
        std::list<unsigned int>* getConstraintRemoveList();

        /** \brief get top node
         */
        virtual WolfProblem* getWolfProblem();

        /** \brief Returns a true (is top)
         */
        virtual bool isTop();

        /** \brief Remove Down Node
         *
         * This empty function is needed by the destruct() node_linked function.
         */
        void removeDownNode(const LowerNodePtr _ptr){};

};

inline std::list<WolfScalar*>* WolfProblem::getStateBlockRemoveList()
{
    return &state_block_remove_list_;
}

inline std::list<ConstraintBase*>* WolfProblem::getConstraintAddList()
{
    return &constraint_add_list_;
}

inline std::list<unsigned int>* WolfProblem::getConstraintRemoveList()
{
    return &constraint_remove_list_;
}

inline WolfProblem* WolfProblem::getWolfProblem()
{
    return this;
}

inline bool WolfProblem::isTop()
{
    return true;
}

#endif
