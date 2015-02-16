
#ifndef LANDMARK_BASE_H_
#define LANDMARK_BASE_H_

class MapBase;
class NodeTerminus;

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "map_base.h"

//class LandmarkBase
class LandmarkBase : public NodeLinked<MapBase,NodeTerminus>
{
    protected:
		LandmarkType type_; //type of frame. Either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
		StateBaseList st_list_; //List of pointers to the state corresponding to the landmark estimation
        
    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, time stamp and state pointer
         * \param _traj_ptr pointer to the trajectory.
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _p_ptr StateBase pointer to the position (default: nullptr)
         *
         **/
		LandmarkBase(const MapBasePtr& _map_ptr, const LandmarkType & _tp, const StateBaseShPtr& _p_ptr);


        /** \brief Constructor with type, time stamp and state list
         * 
         * Constructor with type, time stamp and state pointer
         * \param _traj_ptr pointer to the trajectory.
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _st_list StateBase list of the landmark estimation
         * 
         **/        
		LandmarkBase(const MapBasePtr& _map_ptr, const LandmarkType & _tp, const StateBaseList& _st_list);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~LandmarkBase();
        
        void setType(LandmarkType _ft);

        const StateBaseShPtr getPPtr();

        StateBaseList* getStateListPtr();

        //StateBaseShPtr getOPtr() const;

        //StateBaseShPtr getVPtr() const;

        //StateBaseShPtr getWPtr() const;

        //virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
        
};
#endif
