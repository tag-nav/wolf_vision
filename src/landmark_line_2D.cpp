
#include "landmark_line_2D.h"

namespace wolf {

LandmarkLine2D::LandmarkLine2D(StateBlockPtr _p_ptr, Eigen::Vector3s & _point1, Eigen::Vector3s & _point2) :
    LandmarkBase(LANDMARK_LINE_2D, "LINE 2D", _p_ptr)
{
    //set extreme points
    point1_ = _point1;
    point2_ = _point2; 
}

LandmarkLine2D::~LandmarkLine2D()
{
    //
}

void LandmarkLine2D::updateExtremePoints(Eigen::Vector3s & _q1, Eigen::Vector3s & _q2)
{
    //Mainly this method performs two actions:
    //    1. First check if new points are "inside" or "outside" the segment
    //    2. In case extremes have been updated, fit them on the line
    //
    // p1 & p2 are this->point1_ and point2_
    // _q1 & _q2 are the new points to evaluate
    // vector p1p2 is the vector from p1 to p2
    // dot product s_p1q1 is the dot product between vector p1p2 and vector p1q1
    // dot product s_p2q1 is the dot product between vector p2p1 and vector p2q1
    
    //local variables
    Eigen::Vector2s p1p2, p1q1, p1q2, p2p1, p2q1, p2q2;  
    //Scalar s_p1q1, s_p1q2, s_p2q1, s_p2q2;
    
    //compute all necessary vectors
    p1p2 = point2_.head(2) - point1_.head(1);
    p1q1 = _q1.head(2) - point1_.head(1);
    p1q2 = _q2.head(2) - point1_.head(1);
    p2p1 = point1_.head(2) - point2_.head(1);
    p2q1 = _q1.head(2) - point2_.head(1);
    p2q2 = _q2.head(2) - point2_.head(1);

    //compute all necessary scalar products. 
    //s_p1q1 = p1p2.dot()
    
    
    
}

const Eigen::Vector3s & LandmarkLine2D::point1() const
{
    return point1_;
}

const Eigen::Vector3s & LandmarkLine2D::point2() const
{
    return point2_;
}

} // namespace wolf
