#ifndef WOLF_PROBLEM_H_
#define WOLF_PROBLEM_H_

// std
#include <utility> // pair

//wolf includes
#include "node_base.h"
#include "map_base.h"
#include "trajectory_base.h"
#include "hardware_base.h"
#include "wolf.h"

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
class WolfProblem: public NodeBase
{
    protected:
        std::map<std::pair<StateBlock*, StateBlock*>, Eigen::MatrixXs> covariances_;
        Eigen::SparseMatrix<WolfScalar> covariance_;
        NodeLocation location_;// TODO: should it be in node_base?
        TrajectoryBase* trajectory_ptr_;
        MapBase* map_ptr_;
        HardwareBase* hardware_ptr_;
        StateBlockList state_block_ptr_list_;
        std::list<StateBlock*> state_block_add_list_;
        std::list<StateBlock*> state_block_update_list_;
        std::list<WolfScalar*> state_block_remove_list_;
        std::list<ConstraintBase*> constraint_add_list_;
        std::list<unsigned int> constraint_remove_list_;
        std::map<StateBlock*,unsigned int> state_idx_map_;

    public:

        /** \brief Constructor from state size
         *
         * Constructor from state size
		 * 
         */
        WolfProblem(unsigned int _size=1e3);

        /** \brief Constructor from map and trajectory shared pointers
		 *
		 * Constructor from map and trajectory shared pointers
		 *
		 */
        WolfProblem(TrajectoryBase* _trajectory_ptr, MapBase* _map_ptr=nullptr, HardwareBase* _hardware_ptr=nullptr, unsigned int _size=1e3);

        /** \brief Default destructor
         *
         * Default destructor
		 * 
         */		
        virtual ~WolfProblem();

        /** \brief Adds a new state block to be added to solver manager
		 *
		 * Adds a new state block to be added to solver manager
		 *
		 */
        void addStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a new state block to be updated to solver manager
         *
         * Adds a new state block to be updated to solver manager
         *
         */
        void updateStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a state block to be removed to solver manager
         *
         * Adds a state block to be removed to solver manager
         *
         */
        void removeStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a new constraint to be added to solver manager
         *
         * Adds a new constraint to be added to solver manager
         *
         */
        void addConstraintPtr(ConstraintBase* _constraint_ptr);

        /** \brief Adds a constraint to be removed to solver manager
         *
         * Adds a constraint to be removed to solver manager
         *
         */
        void removeConstraintPtr(ConstraintBase* _constraint_ptr);

        /** \brief Clear covariance
         *
         * Clear covariance
         *
         */
        void clearCovariance();

        /** \brief Adds a new covariance block
         *
         * Adds a new covariance block
         *
         */
        void addCovarianceBlock(StateBlock* _state1, StateBlock* _state2, const Eigen::MatrixXs& _cov);

        /** \brief Gets a covariance block
         *
         * Gets a covariance block
         *
         */
        void getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov_block);
        void getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov, const int _row, const int _col);

        /** \brief Gets state size
		 *
		 * Gets state size
		 *
		 */
        const unsigned int getStateSize() const;

        /** \brief Adds a map
         *
         * Adds a map
         *
         */		
        void addMap(MapBase* _map_ptr);

        /** \brief Adds a trajectory
		 *
		 * Adds a trajectory
		 *
		 */
		void addTrajectory(TrajectoryBase* _trajectory_ptr);

        /** \brief Adds a hardware
         *
         * Adds a hardware
         *
         */
        void addHarware(HardwareBase* _hardware_ptr);
        
        /** \brief Gets a pointer to map
         *
         * Gets a pointer to map
         *
         */
        MapBase* getMapPtr();

        /** \brief Gets a pointer to trajectory
         *
         * Gets a pointer to trajectory
         *
         */
        TrajectoryBase* getTrajectoryPtr();

        /** \brief Gets a pointer to Hardware
         *
         * Gets a pointer to Hardware
         *
         */
        HardwareBase* getHardwarePtr();

        /** \brief Returns a pointer to last Frame
         *
         * Returns a pointer to last Frame
         *
         **/
        FrameBase* getLastFramePtr();

        /** \brief Gets a pointer to the state units list
         *
         * Gets a pointer to the state units list
         *
         */
        StateBlockList* getStateListPtr();

        /** \brief Gets a queue of state blocks to be added in the solver
		 *
		 * Gets a queue of state blocks to be added in the solver
		 *
		 */
        std::list<StateBlock*>* getStateBlockAddList();

        /** \brief Gets a queue of state blocks to be updated in the solver
         *
         * Gets a queue of state blocks to be updated in the solver
         *
         */
        std::list<StateBlock*>* getStateBlockUpdateList();

        /** \brief Gets a queue of state blocks to be removed from the solver
         *
         * Gets a queue of state blocks to be removed from the solver
         *
         */
        std::list<WolfScalar*>* getStateBlockRemoveList();

        /** \brief Gets a queue of constraint ids to be added in the solver
         *
         * Gets a queue of constraint ids to be added in the solver
         *
         */
        std::list<ConstraintBase*>* getConstraintAddList();

        /** \brief Gets a queue of constraint ids to be removed from the solver
         *
         * Gets a queue of constraint ids to be removed from the solver
         *
         */
        std::list<unsigned int>* getConstraintRemoveList();

        /** \brief get top node
		 *
		 * Returns a pointer to this
		 *
		 */
        virtual WolfProblem* getTop();

        /** \brief Prints node information
         * 
		 * Prints node information.
         * \param _ntabs number of tabulations to print at the left of the printed information
         * \param _ost output stream
		 * 
         */
        virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;


    protected:

        /** \brief Prints tabulated information about this node.
         *
         * Prints information about this node. It adds a number of tabs given by _ntabs.
         *\param _ntabs the number of tabs.
         *\param _ost the stream it prints to
		 * 
         */
        void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

        /** \brief Prints recursively lower nodes info
         *
         * Prints recursively lower nodes info
         * 
         **/        
        void printLower(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
};

#endif
