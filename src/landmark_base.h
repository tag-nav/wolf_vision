
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
		LandmarkType type_; //type of landmark. (types defined at wolf.h)
		LandmarkStatus status_; //status of the landmark. (types defined at wolf.h)
		unsigned int hit_count_; //counts how many features has been associated to this landmark
		TimeStamp stamp_; // stamp of the creation of the landmark (and stamp of destruction when status is LANDMARK_OLD)
		//StateBaseList st_list_; //List of pointers to the state corresponding to the landmark estimation
		StateBase* p_ptr_; // Position state unit pointer
		StateBase* o_ptr_; // Orientation state unit pointer
		StateBase* v_ptr_; // Velocity state unit pointer
		StateBase* w_ptr_; // Angular velocity state unit pointer
		//TODO: accelerations?
		Eigen::VectorXs descriptor_;//TODO: agree?
        
    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _tp indicates landmark type.(types defined at wolf.h)
         * \param _p_ptr StateBase pointer to the position
         * \param _o_ptr StateBase pointer to the orientation (default: nullptr)
         * \param _v_ptr StateBase pointer to the velocity (default: nullptr)
         * \param _w_ptr StateBase pointer to the angular velocity (default: nullptr)
         *
         **/
        LandmarkBase(const LandmarkType & _tp , StateBase* _p_ptr, StateBase* _o_ptr = nullptr, StateBase* _v_ptr = nullptr, StateBase* _w_ptr = nullptr);

        /** \brief Constructor with type, time stamp and state list
         * 
         * Constructor with type and state pointer list
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _stp_list StateBase list of the landmark estimation
         * 
         **/        
        //LandmarkBase(const LandmarkType & _tp, const StateBaseList& _stp_list);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~LandmarkBase();
        
        void setStatus(LandmarkStatus _st);
        
        void hit();
        
        void unhit();

        void fix();

        void unfix();

        unsigned int getHits() const;

        StateBase* getPPtr() const;

		StateBase* getOPtr() const;

		StateBase* getVPtr() const;

		StateBase* getWPtr() const;

		void setDescriptor(const Eigen::VectorXs& _descriptor);

		const Eigen::VectorXs getDescriptor() const;

        //const StateBase* getStatePtr() const;

        //const StateBaseList* getStateListPtr() const;
};
#endif
