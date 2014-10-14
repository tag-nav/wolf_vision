/*
 * RawAbsolutePose.h
 *
 *  Created on: July 14, 2014
 *      \author: acorominas
 */

#ifndef RAW_ABSOLUTE_POSE_H_
#define RAW_ABSOLUTE_POSE_H_

//wolf includes
#include "raw_base.h"
#include "state_pose.h"

//namespaces
using namespace std;
using namespace Eigen;

//classes
/** \brief Class for absolute pose measurement data
 * 
 * This class holds absolute pose measurement data
 * 
 */
class RawAbsolutePose : public RawBase
{
    protected:
        StatePose pose_; ///< pose mapped to RawBase::raw_data_ vector
        
    public:
        /** \brief Constructor from time stamp
         * 
         * Constructor from time stamp
         * \param _ts time stamp
         * 
         */
        RawAbsolutePose(const double & _ts, const VectorXs & _pose);

        /** \brief Destructor
         * 
         * Destructor
         * 
         */
        virtual ~RawAbsolutePose();
        
//         /** \brief Gets the raw data
//          * 
//          * Returns a reference to the raw data
//          * 
//          **/
//          Map<VectorXs> & getRawData();
};
#endif /* RAW_ABSOLUTE_POSE_H_ */
