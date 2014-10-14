/*
 * sensor_capture_base.h
 *
 *  Created on: July 11, 2014
 *      \author: acorominas
 */

#ifndef SENSOR_CAPTURE_BASE_H_
#define SENSOR_CAPTURE_BASE_H_

//std
#include <memory>

//wolf
#include "node_constrainer.h"
#include "raw_base.h"

//namespaces
using namespace std;
using namespace Eigen;


/** \brief Class for sensor captures
 * 
 * Class for sensor captures
 * 
 */
class SensorCaptureBase : public NodeConstrainer
{
    protected:
        
    public:
        shared_ptr<RawBase> raw_ptr_; ///< Pointer to raw data

        /** \brief Constructor for local storage
         * 
         * Constructor for local storage
         * \param _loc Placement of the node within the Wolf tree. See enum NodeLocation defined at file wolf.h.
         * \param _dim dimension of the error vector.
         * 
         */
//        SensorCapture(shared_ptr<RawBase> _raw_ptr) :
//            NodeConstrainer(MID),
//            raw_ptr_(_raw_ptr)
//        {
//            //
//        };
        
        SensorCaptureBase(const NodeShrPtr & _un_ptr) :
            NodeConstrainer(MID, _un_ptr)
        {
            //
        }

        /** \brief Destructor
         * 
         * Destructor
         * 
         */
        ~SensorCaptureBase()
        {
            //
        };
        
        /** \brief Returns a reference to time stamp
         * 
         * Returns a reference to time stamp
         * 
         */
//        TimeStamp & timeStamp()
//        {
//            return raw_ptr_->timeStamp();
//        };

        /** \brief Prints node label
         *
         * Prints node label
         *
         **/
        virtual void printLabel(ostream & _ost = cout) const
        {
            _ost <<"CAPTURE";
        }

};
#endif /* SENSOR_CAPTURE_BASE_H_ */
