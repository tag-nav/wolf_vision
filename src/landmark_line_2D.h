
#ifndef LANDMARK_LINE_2D_H_
#define LANDMARK_LINE_2D_H_

//Wolf includes
#include "landmark_base.h"
#include "wolf.h"

//std includes


namespace wolf {

//class LandmarkLine2D
class LandmarkLine2D : public LandmarkBase
{
    protected:
        //extreme points of the line segment
        Eigen::Vector3s point1_;
        Eigen::Vector3s point2_;
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // to guarantee alignment (see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)

        /** \brief Constructor with homogeneous parameters of the line
         *
         * \param _p_ptr homogeneous parameters of the line: (a,b,c) from ax+by+c=0, normalized according line/sqrt(a*a+b*b) 
         *               See http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html
         * \param _point1 Extreme point 1 in homogeneous coordinates (3-vector)
         * \param _point2 Extreme point 2 in homogeneous coordinates (3-vector)
         *
         **/
        LandmarkLine2D(StateBlockPtr _p_ptr, Eigen::Vector3s & _point1, Eigen::Vector3s & _point2);

        virtual ~LandmarkLine2D();
        
        /** \brief Line segment update
         * Update extreme points according new inputs. 
         * Assumes normalized points (x/z, y/z, 1)
         **/
        void updateExtremePoints(Eigen::Vector3s & _q1, Eigen::Vector3s & _q2);
        
        /** \brief Gets extreme point1
         **/
        const Eigen::Vector3s & point1() const; 

        /** \brief Gets extreme point2
         **/        
        const Eigen::Vector3s & point2() const; 
        
};

} // namespace wolf

#endif
