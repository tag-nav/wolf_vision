#ifndef FEATURE_LINE_2D_H_
#define FEATURE_LINE_2D_H_

//Wolf includes
#include "feature_base.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(FeatureLine2D);
    
    
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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // to guarantee alignment (see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)

        /** \brief Constructor 
         * 
         * Constructor, with measurement and descriptor
         * 
         */
        FeatureLine2D(const Eigen::Vector3s & _line_homogeneous_params, const Eigen::Matrix3s & _params_covariance, 
                      Eigen::Vector3s & _point1, Eigen::Vector3s & _point2); 

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
