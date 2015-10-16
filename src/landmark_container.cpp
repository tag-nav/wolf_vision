
#include "landmark_container.h"

LandmarkContainer::LandmarkContainer(StateBase* _p_ptr, StateOrientation* _o_ptr, const WolfScalar& _witdh, const WolfScalar& _length) :
	LandmarkBase(LANDMARK_CONTAINER, _p_ptr, _o_ptr),
	corners_(3,4)
{
    Eigen::VectorXs descriptor(2);
    descriptor << _witdh, _length;
  	setDescriptor(descriptor);

  	corners_ << -_length / 2, _length / 2, _length / 2, -_length / 2,
                -_witdh / 2, -_witdh / 2,  _witdh / 2,   _witdh / 2,
                 0,           M_PI/2,      M_PI,        -M_PI/2;
}

LandmarkContainer::LandmarkContainer(StateBase* _p_ptr, StateOrientation* _o_ptr, const Eigen::Vector3s& _corner_1_pose, const Eigen::Vector3s& _corner_2_pose, const int& _corner_1_idx, const int& _corner_2_idx, const WolfScalar& _witdh, const WolfScalar& _length) :
    LandmarkBase(LANDMARK_CONTAINER, _p_ptr, _o_ptr),
    corners_(3,4)
{
    Eigen::VectorXs descriptor(2);
    descriptor << _witdh, _length;
    setDescriptor(descriptor);

    corners_ << -_length / 2, _length / 2, _length / 2, -_length / 2,
                -_witdh / 2, -_witdh / 2,  _witdh / 2,   _witdh / 2,
                 0,           M_PI/2,      M_PI,        -M_PI/2;

    // Computing center
    std::cout << "Container constructor: Computing center pose... " << std::endl;
    Eigen::Map<Eigen::Vector2s> container_position(_p_ptr->getPtr());
    Eigen::Map<Eigen::VectorXs> container_orientation(_o_ptr->getPtr(), _o_ptr->getStateSize());

    std::cout << "Container constructor: _corner_1_idx " << _corner_1_idx << "_corner_2_idx " << _corner_2_idx << std::endl;

    // Large side detected (A & B)
    if ( (_corner_1_idx == 0 && _corner_2_idx == 1) || (_corner_1_idx == 1 && _corner_2_idx == 0) )
    {
        std::cout << "Large side detected" << std::endl;
        Eigen::Vector2s AB = (_corner_1_idx == 0 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        Eigen::Vector2s perpendicularAB;
        perpendicularAB << -AB(1)/AB.norm(), AB(0)/AB.norm();
        container_position = (_corner_1_idx == 0 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + AB / 2 + perpendicularAB * _witdh / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(AB(1),AB(0));
        else
        {
            container_orientation(0) = AB(0)/AB.norm();
            container_orientation(1) = AB(1)/AB.norm();
        }
    }

    // Short side detected (B & C)
    else if ( (_corner_1_idx == 1 && _corner_2_idx == 2) || (_corner_1_idx == 2 && _corner_2_idx == 1) )
    {
        std::cout << "Short side detected" << std::endl;
        Eigen::Vector2s BC = (_corner_1_idx == 1 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        Eigen::Vector2s perpendicularBC;
        perpendicularBC << -BC(1)/BC.norm(), BC(0)/BC.norm();
        container_position = (_corner_1_idx == 1 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + BC / 2 + perpendicularBC * _length / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(BC(1),BC(0));
        else
        {
            container_orientation(0) = BC(0)/BC.norm();
            container_orientation(1) = BC(1)/BC.norm();
        }
    }

    // Diagonal detected
    // A & C
    else if ( (_corner_1_idx == 0 && _corner_2_idx == 2) || (_corner_1_idx == 2 && _corner_2_idx == 0) )
    {
        std::cout << "diagonal AC detected" << std::endl;
        Eigen::Vector2s AC = (_corner_1_idx == 0 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        container_position = (_corner_1_idx == 0 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + AC / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(AC(1),AC(0)) - atan2(_witdh,_length);
        else
        {
            container_orientation(0) = AC(0)/AC.norm() -_length/descriptor.norm();
            container_orientation(1) = AC(1)/AC.norm() -_witdh/descriptor.norm();
        }
    }
    // B & D
    else if ( (_corner_1_idx == 1 && _corner_2_idx == 3) || (_corner_1_idx == 3 && _corner_2_idx == 1) )
    {
        std::cout << "diagonal BD detected" << std::endl;
        Eigen::Vector2s BD = (_corner_1_idx == 1 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        container_position = (_corner_1_idx == 1 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + BD / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(BD(1),BD(0)) + atan2(_witdh,_length);
        else
        {
            container_orientation(0) = BD(0)/BD.norm() +_length/descriptor.norm();
            container_orientation(1) = BD(1)/BD.norm() +_witdh/descriptor.norm();
        }
    }
    std::cout << "Container center pose... " << container_position.transpose() << " " << container_orientation.transpose() << std::endl;
}

LandmarkContainer::LandmarkContainer(StateBase* _p_ptr, StateOrientation* _o_ptr, LandmarkCorner2D* _corner_A_ptr, LandmarkCorner2D* _corner_B_ptr, LandmarkCorner2D* _corner_C_ptr, LandmarkCorner2D* _corner_D_ptr, const WolfScalar& _witdh, const WolfScalar& _length) :
    LandmarkBase(LANDMARK_CONTAINER, _p_ptr, _o_ptr),
    corners_(3,4)
{
    assert((_corner_A_ptr != nullptr || _corner_B_ptr != nullptr || _corner_C_ptr != nullptr || _corner_D_ptr != nullptr) && "all corner pointer are null in landmark container constructor from corners");

    Eigen::VectorXs descriptor(2);
    descriptor << _witdh, _length;
    setDescriptor(descriptor);

    corners_ << -_length / 2, _length / 2, _length / 2, -_length / 2,
                -_witdh / 2, -_witdh / 2,  _witdh / 2,   _witdh / 2,
                 0,           M_PI/2,      M_PI,        -M_PI/2;

    // Computing center
    //std::cout << "Container constructor: Computing center position... " << std::endl;
    Eigen::Map<Eigen::Vector2s> container_position(_p_ptr->getPtr());
    Eigen::Map<Eigen::VectorXs> container_orientation(_o_ptr->getPtr(), _o_ptr->getStateSize());

    container_position = Eigen::Vector2s::Zero();

    // Large side detected
    // A & B
    if (_corner_A_ptr != nullptr && _corner_B_ptr != nullptr)
    {
        Eigen::Vector2s AB = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr());
        Eigen::Vector2s perpendicularAB;
        perpendicularAB << -AB(1)/AB.norm(), AB(0)/AB.norm();
        container_position = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr()) + AB / 2 + perpendicularAB * _witdh / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(AB(1),AB(0));
        else
        {
            container_orientation(0) = AB(0)/AB.norm();
            container_orientation(1) = AB(1)/AB.norm();
        }
    }
    // C & D
    else if  (_corner_C_ptr != nullptr && _corner_D_ptr != nullptr)
    {
        Eigen::Vector2s CD = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getPPtr()->getPtr());
        Eigen::Vector2s perpendicularCD;
        perpendicularCD << -CD(1)/CD.norm(), CD(0)/CD.norm();
        container_position = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getPPtr()->getPtr()) + CD / 2 + perpendicularCD * _witdh / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(-CD(1),-CD(0));
        else
        {
            container_orientation(0) = -CD(0)/CD.norm();
            container_orientation(1) = -CD(1)/CD.norm();
        }
    }
    // Short side detected
    // B & C
    else if (_corner_B_ptr != nullptr && _corner_C_ptr != nullptr)
    {
        Eigen::Vector2s BC = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr());
        Eigen::Vector2s perpendicularBC;
        perpendicularBC << -BC(1)/BC.norm(), BC(0)/BC.norm();
        container_position = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr()) + BC / 2 + perpendicularBC * _length / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(BC(1),BC(0));
        else
        {
            container_orientation(0) = BC(0)/BC.norm();
            container_orientation(1) = BC(1)/BC.norm();
        }
    }
    // D & A
    else if  (_corner_D_ptr != nullptr && _corner_A_ptr != nullptr)
    {
        Eigen::Vector2s DA = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getPPtr()->getPtr());
        Eigen::Vector2s perpendicularDA;
        perpendicularDA << -DA(1)/DA.norm(), DA(0)/DA.norm();
        container_position = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getPPtr()->getPtr()) + DA / 2 + perpendicularDA * _length / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(-DA(1),-DA(0));
        else
        {
            container_orientation(0) = -DA(0)/DA.norm();
            container_orientation(1) = -DA(1)/DA.norm();
        }
    }
    // Diagonal detected
    // A & C
    else if (_corner_A_ptr != nullptr && _corner_C_ptr != nullptr)
    {
        Eigen::Vector2s AC = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr());
        container_position = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr()) + AC / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(AC(1),AC(0)) - atan2(_witdh,_length);
        else
        {
            container_orientation(0) = AC(0)/AC.norm() -_length/descriptor.norm();
            container_orientation(1) = AC(1)/AC.norm() -_witdh/descriptor.norm();
        }
    }
    // B & D
    else if (_corner_B_ptr != nullptr && _corner_D_ptr != nullptr)
    {
        Eigen::Vector2s BD = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr());
        container_position = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr()) + BD / 2;
        if (_o_ptr->getStateType() == ST_THETA)
            container_orientation(0) = atan2(BD(1),BD(0)) + atan2(_witdh,_length);
        else
        {
            container_orientation(0) = BD(0)/BD.norm() +_length/descriptor.norm();
            container_orientation(1) = BD(1)/BD.norm() +_witdh/descriptor.norm();
        }
    }
}

LandmarkContainer::~LandmarkContainer()
{
    // tODO delete corners
}

WolfScalar LandmarkContainer::getWidth() const
{
    return descriptor_(0);
}

WolfScalar LandmarkContainer::getLength() const
{
    return descriptor_(1);
}

Eigen::MatrixXs LandmarkContainer::getCorners() const
{
    return corners_;
}

Eigen::VectorXs LandmarkContainer::getCorner(const unsigned int _id) const
{
    assert(_id >= 0 && _id <= 4 && "wrong corner id parameter in getCorner(id)");
    return corners_.col(_id);
}

int LandmarkContainer::tryCorner(LandmarkCorner2D* _new_corner_ptr)
{
//    WolfScalar MAX_ACCEPTED_DISTANCE_DIFF = 0.5;
//    WolfScalar MAX_ACCEPTED_APERTURE_DIFF = 10 * M_PI / 180 ;
//    //std::cout << "Trying container... aperture = " << _new_corner_ptr->getAperture() << std::endl;
//
//    // It has to be 90º corner
//    if (std::fabs(_new_corner_ptr->getAperture() - 3 * M_PI / 2) < MAX_ACCEPTED_APERTURE_DIFF)
//    {
//        //std::cout << "   90º OK..." << std::endl;
//        for (unsigned int i = 0; i < 4; i++)
//        {
//            Eigen::Vector2s v_diff = (this->getPPtr()->getVector() + corners_.head(2)) - _new_corner_ptr->getPPtr()->getVector();
//            WolfScalar angle_diff = pi2pi((this->getOPtr()->getYaw() + corners_(2)) - _new_corner_ptr->getOPtr()->getYaw());
//
//            if (v_diff.norm() < MAX_ACCEPTED_DISTANCE_DIFF && abs(angle_diff) < MAX_ACCEPTED_APERTURE_DIFF)
//            {
//                return i;
//            }
//        }
//    }
    return -1;
}

//bool fitContainer(const Eigen::VectorXs& _corner_1_position, const WolfScalar& _corner_1_orientation, const Eigen::VectorXs& _corner_2_position, const WolfScalar& _corner_2_orientation, int& corner_1_idx, int& corner_2_idx)
//{
//    //std::cout << "Trying container... aperture = " << _corner_ptr->getAperture() << std::endl;
//    WolfScalar MAX_ACCEPTED_DISTANCE_DIFF = 1;
//    WolfScalar MAX_ACCEPTED_APERTURE_DIFF = 10 * M_PI / 180;
//
//    WolfScalar angle_diff = _corner_1_orientation - _corner_2_orientation;
//    Eigen::Vector2s v_diff = _corner_1_position - _corner_2_position;
//    std::cout << "  Angle difference = " << angle_diff << std::endl;
//    std::cout << "  position difference = " << v_diff.transpose() << std::endl;
//
//    // Large side
//    if (abs(v_diff.norm() - 12.2) < MAX_ACCEPTED_DISTANCE_DIFF)
//    {
//        // counterclockwise order
//        if (abs(pi2pi(angle_diff - M_PI/2)) < MAX_ACCEPTED_APERTURE_DIFF &&
//            abs(pi2pi(atan2(v_diff(1), v_diff(0)) - _corner_1_orientation)) < MAX_ACCEPTED_APERTURE_DIFF)
//        {
//            std::cout << "   next counterclockwise large container side detected!" << std::endl;
//            corner_1_idx = 0;
//            corner_2_idx = 1;
//            return true;
//        }
//
//        // clockwise order
//        else if (abs(pi2pi(angle_diff + M_PI/2)) < MAX_ACCEPTED_APERTURE_DIFF &&
//                 abs(pi2pi(atan2(-v_diff(1), -v_diff(0)) - _corner_2_orientation)) < MAX_ACCEPTED_APERTURE_DIFF)
//        {
//            std::cout << "   prev counterclockwise large container side detected!" << std::endl;
//            corner_1_idx = 1;
//            corner_2_idx = 0;
//            return true;
//        }
//    }
//    // Short side
//    else if (abs(v_diff.norm() - 2.44) < MAX_ACCEPTED_DISTANCE_DIFF)
//    {
//        // counterclockwise order
//        if (abs(pi2pi(angle_diff - M_PI/2)) < MAX_ACCEPTED_APERTURE_DIFF &&
//            abs(pi2pi(atan2(v_diff(1), v_diff(0)) - _corner_1_orientation)) < MAX_ACCEPTED_APERTURE_DIFF)
//        {
//            std::cout << "   next counterclockwise short container side detected!" << std::endl;
//            corner_1_idx = 1;
//            corner_2_idx = 2;
//            return true;
//        }
//
//        // clockwise order
//        else if (abs(pi2pi(angle_diff + M_PI/2)) < MAX_ACCEPTED_APERTURE_DIFF &&
//                 abs(pi2pi(atan2(-v_diff(1), -v_diff(0)) - _corner_2_orientation)) < MAX_ACCEPTED_APERTURE_DIFF)
//        {
//            std::cout << "   prev counterclockwise short container side detected!" << std::endl;
//            corner_1_idx = 2;
//            corner_2_idx = 1;
//            return true;
//        }
//    }
//    // opposite corners
//    else if (abs(pi2pi(angle_diff - M_PI)) < MAX_ACCEPTED_APERTURE_DIFF &&
//             abs(v_diff.norm() - 12.4416) < MAX_ACCEPTED_DISTANCE_DIFF)
//    {
//        // large side aligned corners
//        if (abs(pi2pi(atan2(v_diff(1), v_diff(0)) - _corner_1_orientation - 0.1974)) < MAX_ACCEPTED_APERTURE_DIFF)
//        {
//            std::cout << "   opposite corners (aligned to large sides) detected!" << std::endl;
//            corner_1_idx = 0;
//            corner_2_idx = 2;
//            return true;
//        }
//        else if (abs(pi2pi(atan2(v_diff(1), v_diff(0)) - _corner_2_orientation - 1.3734)) < MAX_ACCEPTED_APERTURE_DIFF)
//        {
//            std::cout << "   opposite corners (aligned to short sides) detected!" << std::endl;
//            corner_1_idx = 1;
//            corner_2_idx = 3;
//            return true;
//        }
//    }
//    return false;
//}

