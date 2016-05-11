
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
        Eigen::Map<Vector2s> point1_;
        Eigen::Map<Vector2s> point2_;
        
    public:
        /** \brief Constructor with homogeneous parameters of the line
         *
         * \param _p_ptr homogeneous parameters of the line: (a,b,c) from ax+by+c=0, normalized according line/sqrt(a*a+b*b) 
         *               See http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html
         * \param _point1 Extreme point 1 in Euclidean coordinates (2-vector)
         * \param _point2 Extreme point 2 in Euclidean coordinates (2-vector)
         *
         **/
        LandmarkLine2D(StateBlock* _p_ptr, Eigen::Vector2s & _point1, Eigen::Vector2s & _point2);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~LandmarkLine2D();
        
        /** \brief Line segment update
         * Update extreme points according new inputs
         **/
        void updateExtremePoints(Eigen::VectorXs & _point1, Eigen::VectorXs & _point2);
        
        /** \brief Gets extreme point1
         **/
        const Eigen::Vector2s & point1() const; 

        /** \brief Gets extreme point2
         **/        
        const Eigen::Vector2s & point2() const; 
        
};

} // namespace wolf

#endif
