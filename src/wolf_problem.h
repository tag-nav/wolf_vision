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
        NodeLocation location_;// TODO: should it be in node_base?
        MapBaseShPtr map_ptr_;
        TrajectoryBaseShPtr trajectory_ptr_;

    public:

        /** \brief Constructor
         *
         * Constructor
		 * 
         */
        WolfProblem();

        /** \brief Constructor from map and trajectory shared pointers
		 *
		 * Constructor from map and trajectory shared pointers
		 *
		 */
        WolfProblem(const TrajectoryBaseShPtr& _trajectory_ptr, const MapBaseShPtr& _map_ptr={});

        /** \brief Default destructor
         *
         * Default destructor
		 * 
         */		
        virtual ~WolfProblem();

        /** \brief Adds a map
         *
         * Adds a map
         *
         */		
        void addMap(MapBaseShPtr& _map_ptr);

        /** \brief Adds a trajectory
		 *
		 * Adds a trajectory
		 *
		 */
		void addTrajectory(TrajectoryBaseShPtr& _Trajectory_ptr);
		
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
        MapBasePtr getMapPtr();

        /** \brief Gets a pointer to map
         *
         * Gets a pointer to map
         *
         */
        TrajectoryBasePtr getTrajectoryPtr();

        /** \brief Removes a down node from list, given an iterator
         *
         * Removes a down node from the list
         * @param _iter an iterator to the particular down node in the list that will be removed
         *
         */
        //void removeDownNode(const LowerNodeIter& _iter);

        /** \brief Removes a down node from the list, given a node id
         *
         * Removes a down node from the multimap
         * @param _id node id of the node that will nbe removed
         *
         */
        //void removeDownNode(const unsigned int _id);
        
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
         * Overload this function in derived classes to adapt the printed output to each object's relevant info.
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
