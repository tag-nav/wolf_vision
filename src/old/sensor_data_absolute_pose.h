/*
 * sensor_data_absolute_pose.h
 *
 *  Created on: July 14, 2014
 *      \author: acorominas
 */

#ifndef SENSOR_DATA_ABSOLUTE_POSE_H_
#define SENSOR_DATA_ABSOLUTE_POSE_H_

//wolf includes
#include "sensor_data_base.h"
#include "raw_absolute_pose.h"

//namespaces
using namespace std;
using namespace Eigen;

//classes
/** \brief Class for absolute pose measurement data
 * 
 * This class holds absolute pose measurement data. This class is placed at the lowest layer 
 * of the Wolf tree, so computeExpectation() method is implemented.
 * 
 */
class SensorDataAbsolutePose : public SensorDataBase<RawAbsolutePose>
{
    protected:
                
    public:
        /** \brief Constructor from time stamp and data vector
         * 
         * Constructor from time stamp
         * \param _ts time stamp
         * \param _pose 7-vector with a pose: (x,y,z,q0,qi,qj,qk)
         * 
         */
        SensorDataAbsolutePose(shared_ptr<RawAbsolutePose> & _raw_ptr);
        //SensorDataAbsolutePose(const double & _ts, VectorXs & _pose);

        /** \brief Destructor
         * 
         * Destructor
         * 
         */
        virtual ~SensorDataAbsolutePose();
        
        /** \brief computes expectation
         * 
         * computes expectation
         * 
         **/
        void computeExpectation();
};
#endif /* RAW_ABSOLUTE_POSE_H_ */
