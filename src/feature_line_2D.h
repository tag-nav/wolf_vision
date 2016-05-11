#ifndef FEATURE_LINE_2D_H_
#define FEATURE_LINE_2D_H_

//Wolf includes
#include "feature_base.h"

namespace wolf {

/** \brief class FeatureLine2D
 * 
 * Line segment feature. 
 * 
 * Measurement is a 3-vector of its homogeneous coordinates, 
 * normalized according http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html
 * 
 * Descriptor are the homogeneous coordinates of its first and last point, which may or may not be extreme points of the line
 * 
 **/
class FeatureLine2D : public FeatureBase
{
    protected: 
        Eigen::Vector3s first_point_;
        Eigen::Vector3s last_point_;
        
    public:
        /** \brief Constructor 
         * 
         * Constructor, with measurement and descriptor
         * 
         */
        FeatureLine2D(const Eigen::Vector3s & _line_homogeneous_params, const Eigen::Matrix3s & _params_covariance, 
                      Eigen::Vector3s & _point1, Eigen::Vector3s & _point2); 

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         */        
        virtual ~FeatureLine2D();
        
        /** \brief Returns a const reference to first_point_
         */
        const Eigen::Vector3s & firstPoint() const; 
        
        /** \brief Returns a const reference to last_point_
         */
        const Eigen::Vector3s & lastPoint() const; 
        
                
};

} // namespace wolf

#endif
