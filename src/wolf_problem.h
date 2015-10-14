#ifndef WOLF_PROBLEM_H_
#define WOLF_PROBLEM_H_

//wof includes
#include "node_base.h"
#include "map_base.h"
#include "trajectory_base.h"
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
        Eigen::VectorXs state_;
        Eigen::SparseMatrix<WolfScalar> covariance_;
		unsigned int state_idx_last_;
        NodeLocation location_;// TODO: should it be in node_base?
        MapBase* map_ptr_;
        TrajectoryBase* trajectory_ptr_;
        //TODO: SensorBaseList sensor_list_;
        StateBaseList state_list_;
        std::list<WolfScalar*> removed_state_ptr_list_;
        bool reallocated_;

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
        WolfProblem(TrajectoryBase* _trajectory_ptr, MapBase* _map_ptr=nullptr, unsigned int _size=1e3);

        /** \brief Default destructor
         *
         * Default destructor
		 * 
         */		
        virtual ~WolfProblem();

        /** \brief Adds a new state unit to the state
		 *
		 * Adds a new state unit to the state. Returns true if a remapping has been done.
		 *
		 */
        bool addState(StateBase* _new_state, const Eigen::VectorXs& _new_state_values);

        /** \brief Adds a new covariance block
         *
         * Adds a new covariance block
         *
         */
        void addCovarianceBlock(StateBase* _state1, StateBase* _state2, const Eigen::MatrixXs& _cov);

        /** \brief Gets a covariance block
         *
         * Gets a covariance block
         *
         */
        void getCovarianceBlock(StateBase* _state1, StateBase* _state2, Eigen::MatrixXs& _cov_block) const;
        void getCovarianceBlock(StateBase* _state1, StateBase* _state2, Eigen::MatrixXs& _cov, const int _row, const int _col) const;

        /** \brief Removes a new state unit of the state
		 *
		 * Removes a new state unit of the state
		 *
		 */
        void removeState(StateBase* _state);

        /** \brief Gets a pointer to the state first position
         *
         * Gets a pointer to the state first position
         *
         */
        WolfScalar* getStatePtr();

        /** \brief Gets a pointer where a new state unit should be located
         *
         * Gets a pointer where a new state unit should be located
         *
         */
        WolfScalar* getNewStatePtr();

        /** \brief Gets state size
		 *
		 * Gets state size
		 *
		 */
        const unsigned int getStateSize() const;

        /** \brief Sets the index of the last occupied position of the state
		 *
		 * Sets the index of the last occupied position of the state
		 *
		 */
		//void setStateIdx(unsigned int _idx);

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
		void addTrajectory(TrajectoryBase* _Trajectory_ptr);
		
        /** \brief Gets a reference to map
         *
         * Gets a reference to map
         *
         */
        //MapBase& getMap() const;

        /** \brief Gets a reference to Trajectory
         *
         * Gets a reference to Trajectory
         *
         */
        //TrajectoryBase& getTrajectory() const;
        
        /** \brief Gets a pointer to map
         *
         * Gets a pointer to map
         *
         */
        MapBase* getMapPtr();

        /** \brief Gets a pointer to map
         *
         * Gets a pointer to map
         *
         */
        TrajectoryBase* getTrajectoryPtr();

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
        StateBaseList* getStateListPtr();

        /** \brief Gets a list of wolfscalar pointers contained by removed state units (in order to delete them in ceres)
		 *
		 * Gets a list of wolfscalar pointers contained by removed state units (in order to delete them in ceres)
		 *
		 */
        std::list<WolfScalar*>* getRemovedStateListPtr();

        /** \brief Gets the state vector
		 *
		 * Gets the state vector
		 *
		 */
        const Eigen::VectorXs getState() const;

        /** \brief Gets if the state has been reallocated
		 *
		 * Gets if the state has been reallocated
		 *
		 */
        bool isReallocated() const;

        /** \brief Turn off the reallocation flag
		 *
		 * Turn off the reallocation flag
		 *
		 */
		void reallocationDone();

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
