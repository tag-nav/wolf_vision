
#include "landmark_container.h"
#include "state_block.h"

namespace wolf {

LandmarkContainer::LandmarkContainer(StateBlock* _p_ptr, StateBlock* _o_ptr, const Scalar& _witdh, const Scalar& _length) :
	LandmarkBase(LANDMARK_CONTAINER, _p_ptr, _o_ptr),
	corners_(3,4)
{
    setType("CONTAINER");
    Eigen::VectorXs descriptor(2);
    descriptor << _witdh, _length;
  	setDescriptor(descriptor);

  	corners_ << -_length / 2, _length / 2, _length / 2, -_length / 2,
                -_witdh / 2, -_witdh / 2,  _witdh / 2,   _witdh / 2,
                M_PI / 4,     3 * M_PI / 4,-3 * M_PI / 4,-M_PI / 4;
}

LandmarkContainer::LandmarkContainer(StateBlock* _p_ptr, StateBlock* _o_ptr, const Eigen::Vector3s& _corner_1_pose, const Eigen::Vector3s& _corner_2_pose, const int& _corner_1_idx, const int& _corner_2_idx, const Scalar& _witdh, const Scalar& _length) :
    LandmarkBase(LANDMARK_CONTAINER, _p_ptr, _o_ptr),
    corners_(3,4)
{
    setType("CONTAINER");
    Eigen::VectorXs descriptor(2);
    descriptor << _witdh, _length;
    setDescriptor(descriptor);

    corners_ << -_length / 2, _length / 2, _length / 2,  -_length / 2,
                -_witdh / 2, -_witdh / 2,  _witdh / 2,    _witdh / 2,
                M_PI / 4,     3 * M_PI / 4,-3 * M_PI / 4,-M_PI / 4;

    // Computing center
    std::cout << "Container constructor: Computing center pose... " << std::endl;
    Eigen::Map<Eigen::Vector2s> container_position(getPPtr()->getPtr());
    Eigen::Map<Eigen::Vector1s> container_orientation(getOPtr()->getPtr());

    std::cout << "Container constructor: _corner_1_idx " << _corner_1_idx << "_corner_2_idx " << _corner_2_idx << std::endl;

    // Large side detected (A & B)
    if ( (_corner_1_idx == 0 && _corner_2_idx == 1) || (_corner_1_idx == 1 && _corner_2_idx == 0) )
    {
        std::cout << "Large side detected" << std::endl;
        Eigen::Vector2s AB = (_corner_1_idx == 0 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        Eigen::Vector2s perpendicularAB;
        perpendicularAB << -AB(1)/AB.norm(), AB(0)/AB.norm();
        container_position = (_corner_1_idx == 0 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + AB / 2 + perpendicularAB * _witdh / 2;
        container_orientation(0) = atan2(AB(1),AB(0));
    }

    // Short side detected (B & C)
    else if ( (_corner_1_idx == 1 && _corner_2_idx == 2) || (_corner_1_idx == 2 && _corner_2_idx == 1) )
    {
        std::cout << "Short side detected" << std::endl;
        Eigen::Vector2s BC = (_corner_1_idx == 1 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        Eigen::Vector2s perpendicularBC;
        perpendicularBC << -BC(1)/BC.norm(), BC(0)/BC.norm();
        container_position = (_corner_1_idx == 1 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + BC / 2 + perpendicularBC * _length / 2;
        container_orientation(0) = atan2(BC(1),BC(0)) - M_PI / 2;
    }

    // Diagonal detected
    // A & C
    else if ( (_corner_1_idx == 0 && _corner_2_idx == 2) || (_corner_1_idx == 2 && _corner_2_idx == 0) )
    {
        std::cout << "diagonal AC detected" << std::endl;
        Eigen::Vector2s AC = (_corner_1_idx == 0 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        container_position = (_corner_1_idx == 0 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + AC / 2;
        container_orientation(0) = atan2(AC(1),AC(0)) - atan2(_witdh,_length);
    }
    // B & D
    else if ( (_corner_1_idx == 1 && _corner_2_idx == 3) || (_corner_1_idx == 3 && _corner_2_idx == 1) )
    {
        std::cout << "diagonal BD detected" << std::endl;
        Eigen::Vector2s BD = (_corner_1_idx == 1 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        container_position = (_corner_1_idx == 1 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + BD / 2;
        container_orientation(0) = atan2(BD(1),BD(0)) + atan2(_witdh,_length);
    }
    else
        assert(0 && "index corners combination not implemented!");

    std::cout << "_corner_1_pose... " << _corner_1_pose.transpose() << std::endl;
    std::cout << "_corner_2_pose... " << _corner_2_pose.transpose() << std::endl;
    std::cout << "Container center pose... " << container_position.transpose() << " " << container_orientation.transpose() << std::endl;
}

//LandmarkContainer::LandmarkContainer(StateBlock* _p_ptr, StateBlock* _o_ptr, LandmarkCorner2D* _corner_A_ptr, LandmarkCorner2D* _corner_B_ptr, LandmarkCorner2D* _corner_C_ptr, LandmarkCorner2D* _corner_D_ptr, const Scalar& _witdh, const Scalar& _length) :
//    LandmarkBase(LANDMARK_CONTAINER, _p_ptr, _o_ptr),
//    corners_(3,4)
//{
//    assert((_corner_A_ptr != nullptr || _corner_B_ptr != nullptr || _corner_C_ptr != nullptr || _corner_D_ptr != nullptr) && "all corner pointer are null in landmark container constructor from corners");
//
//    Eigen::VectorXs descriptor(2);
//    descriptor << _witdh, _length;
//    setDescriptor(descriptor);
//
//    corners_ << -_length / 2, _length / 2, _length / 2, -_length / 2,
//                -_witdh / 2, -_witdh / 2,  _witdh / 2,   _witdh / 2,
//                 0,           M_PI/2,      M_PI,        -M_PI/2;
//
//    // Computing center
//    //std::cout << "Container constructor: Computing center position... " << std::endl;
//    Eigen::Map<Eigen::Vector2s> container_position(_p_ptr->getPtr());
//    Eigen::Map<Eigen::VectorXs> container_orientation(_o_ptr->getPtr(), _o_ptr->getSize());
//
//    container_position = Eigen::Vector2s::Zero();
//
//    // Large side detected
//    // A & B
//    if (_corner_A_ptr != nullptr && _corner_B_ptr != nullptr)
//    {
//        Eigen::Vector2s AB = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr());
//        Eigen::Vector2s perpendicularAB;
//        perpendicularAB << -AB(1)/AB.norm(), AB(0)/AB.norm();
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr()) + AB / 2 + perpendicularAB * _witdh / 2;
//        container_orientation(0) = atan2(AB(1),AB(0));
//    }
//    // C & D
//    else if  (_corner_C_ptr != nullptr && _corner_D_ptr != nullptr)
//    {
//        Eigen::Vector2s CD = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getPPtr()->getPtr());
//        Eigen::Vector2s perpendicularCD;
//        perpendicularCD << -CD(1)/CD.norm(), CD(0)/CD.norm();
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getPPtr()->getPtr()) + CD / 2 + perpendicularCD * _witdh / 2;
//        container_orientation(0) = atan2(-CD(1),-CD(0));
//    }
//    // Short side detected
//    // B & C
//    else if (_corner_B_ptr != nullptr && _corner_C_ptr != nullptr)
//    {
//        Eigen::Vector2s BC = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr());
//        Eigen::Vector2s perpendicularBC;
//        perpendicularBC << -BC(1)/BC.norm(), BC(0)/BC.norm();
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr()) + BC / 2 + perpendicularBC * _length / 2;
//        container_orientation(0) = atan2(BC(1),BC(0));
//    }
//    // D & A
//    else if  (_corner_D_ptr != nullptr && _corner_A_ptr != nullptr)
//    {
//        Eigen::Vector2s DA = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getPPtr()->getPtr());
//        Eigen::Vector2s perpendicularDA;
//        perpendicularDA << -DA(1)/DA.norm(), DA(0)/DA.norm();
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getPPtr()->getPtr()) + DA / 2 + perpendicularDA * _length / 2;
//        container_orientation(0) = atan2(-DA(1),-DA(0));
//    }
//    // Diagonal detected
//    // A & C
//    else if (_corner_A_ptr != nullptr && _corner_C_ptr != nullptr)
//    {
//        Eigen::Vector2s AC = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr());
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getPPtr()->getPtr()) + AC / 2;
//        container_orientation(0) = atan2(AC(1),AC(0)) - atan2(_witdh,_length);
//    }
//    // B & D
//    else if (_corner_B_ptr != nullptr && _corner_D_ptr != nullptr)
//    {
//        Eigen::Vector2s BD = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getPPtr()->getPtr()) - Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr());
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getPPtr()->getPtr()) + BD / 2;
//        container_orientation(0) = atan2(BD(1),BD(0)) + atan2(_witdh,_length);
//    }
//}

LandmarkContainer::~LandmarkContainer()
{
    // tODO delete corners
}

Scalar LandmarkContainer::getWidth() const
{
    return descriptor_(0);
}

Scalar LandmarkContainer::getLength() const
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

} // namespace wolf
