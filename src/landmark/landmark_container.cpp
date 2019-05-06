
#include "base/landmark/landmark_container.h"
#include "base/state_block/state_block.h"

namespace wolf {

LandmarkContainer::LandmarkContainer(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const Scalar& _witdh, const Scalar& _length) :
	LandmarkBase("CONTAINER", _p_ptr, _o_ptr),
	corners_(3,4)
{
    Eigen::VectorXs descriptor(2);
    descriptor << _witdh, _length;
  	setDescriptor(descriptor);

  	corners_ << -_length / 2, _length / 2, _length / 2, -_length / 2,
                -_witdh / 2, -_witdh / 2,  _witdh / 2,   _witdh / 2,
                M_PI / 4,     3 * M_PI / 4,-3 * M_PI / 4,-M_PI / 4;
}

LandmarkContainer::LandmarkContainer(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const Eigen::Vector3s& _corner_1_pose, const Eigen::Vector3s& _corner_2_pose, const int& _corner_1_idx, const int& _corner_2_idx, const Scalar& _witdh, const Scalar& _length) :
    LandmarkBase("CONTAINER", _p_ptr, _o_ptr),
    corners_(3,4)
{
    Eigen::VectorXs descriptor(2);
    descriptor << _witdh, _length;
    setDescriptor(descriptor);

    corners_ << -_length / 2, _length / 2, _length / 2,  -_length / 2,
                -_witdh / 2, -_witdh / 2,  _witdh / 2,    _witdh / 2,
                M_PI / 4,     3 * M_PI / 4,-3 * M_PI / 4,-M_PI / 4;

    // Computing center
    WOLF_DEBUG( "Container constructor: Computing center pose... " );
    Eigen::Vector2s container_position(getP()->getState());
    Eigen::Vector1s container_orientation(getO()->getState());

    WOLF_DEBUG( "Container constructor: _corner_1_idx ", _corner_1_idx,
                "_corner_2_idx ", _corner_2_idx );

    // Large side detected (A & B)
    if ( (_corner_1_idx == 0 && _corner_2_idx == 1) || (_corner_1_idx == 1 && _corner_2_idx == 0) )
    {
        WOLF_DEBUG( "Large side detected" );
        Eigen::Vector2s AB = (_corner_1_idx == 0 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        Eigen::Vector2s perpendicularAB;
        perpendicularAB << -AB(1)/AB.norm(), AB(0)/AB.norm();
        container_position = (_corner_1_idx == 0 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + AB / 2 + perpendicularAB * _witdh / 2;
        container_orientation(0) = atan2(AB(1),AB(0));
    }

    // Short side detected (B & C)
    else if ( (_corner_1_idx == 1 && _corner_2_idx == 2) || (_corner_1_idx == 2 && _corner_2_idx == 1) )
    {
        WOLF_DEBUG( "Short side detected" );
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
        WOLF_DEBUG( "diagonal AC detected" );
        Eigen::Vector2s AC = (_corner_1_idx == 0 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        container_position = (_corner_1_idx == 0 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + AC / 2;
        container_orientation(0) = atan2(AC(1),AC(0)) - atan2(_witdh,_length);
    }
    // B & D
    else if ( (_corner_1_idx == 1 && _corner_2_idx == 3) || (_corner_1_idx == 3 && _corner_2_idx == 1) )
    {
        WOLF_DEBUG( "diagonal BD detected" );
        Eigen::Vector2s BD = (_corner_1_idx == 1 ? _corner_2_pose.head(2) - _corner_1_pose.head(2) : _corner_1_pose.head(2) - _corner_2_pose.head(2));
        container_position = (_corner_1_idx == 1 ? _corner_1_pose.head(2) : _corner_2_pose.head(2)) + BD / 2;
        container_orientation(0) = atan2(BD(1),BD(0)) + atan2(_witdh,_length);
    }
    else
        assert(0 && "index corners combination not implemented!");

    WOLF_DEBUG( "_corner_1_pose... ", _corner_1_pose.transpose() );
    WOLF_DEBUG( "_corner_2_pose... ", _corner_2_pose.transpose() );
    WOLF_DEBUG( "Container center pose... ", container_position.transpose(), " ", container_orientation.transpose() );

    getP()->setState(container_position);
    getO()->setState(container_orientation);
}

//LandmarkContainer::LandmarkContainer(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, LandmarkCorner2D* _corner_A_ptr, LandmarkCorner2D* _corner_B_ptr, LandmarkCorner2D* _corner_C_ptr, LandmarkCorner2D* _corner_D_ptr, const Scalar& _witdh, const Scalar& _length) :
//    LandmarkBase(_p_ptr, _o_ptr),
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
//    Eigen::Map<Eigen::Vector2s> container_position(_p_ptr->get());
//    Eigen::Map<Eigen::VectorXs> container_orientation(_o_ptr->get(), _o_ptr->getSize());
//
//    container_position = Eigen::Vector2s::Zero();
//
//    // Large side detected
//    // A & B
//    if (_corner_A_ptr != nullptr && _corner_B_ptr != nullptr)
//    {
//        Eigen::Vector2s AB = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getP()->get()) - Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getP()->get());
//        Eigen::Vector2s perpendicularAB;
//        perpendicularAB << -AB(1)/AB.norm(), AB(0)/AB.norm();
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getP()->get()) + AB / 2 + perpendicularAB * _witdh / 2;
//        container_orientation(0) = atan2(AB(1),AB(0));
//    }
//    // C & D
//    else if  (_corner_C_ptr != nullptr && _corner_D_ptr != nullptr)
//    {
//        Eigen::Vector2s CD = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getP()->get()) - Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getP()->get());
//        Eigen::Vector2s perpendicularCD;
//        perpendicularCD << -CD(1)/CD.norm(), CD(0)/CD.norm();
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getP()->get()) + CD / 2 + perpendicularCD * _witdh / 2;
//        container_orientation(0) = atan2(-CD(1),-CD(0));
//    }
//    // Short side detected
//    // B & C
//    else if (_corner_B_ptr != nullptr && _corner_C_ptr != nullptr)
//    {
//        Eigen::Vector2s BC = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getP()->get()) - Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getP()->get());
//        Eigen::Vector2s perpendicularBC;
//        perpendicularBC << -BC(1)/BC.norm(), BC(0)/BC.norm();
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getP()->get()) + BC / 2 + perpendicularBC * _length / 2;
//        container_orientation(0) = atan2(BC(1),BC(0));
//    }
//    // D & A
//    else if  (_corner_D_ptr != nullptr && _corner_A_ptr != nullptr)
//    {
//        Eigen::Vector2s DA = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getP()->get()) - Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getP()->get());
//        Eigen::Vector2s perpendicularDA;
//        perpendicularDA << -DA(1)/DA.norm(), DA(0)/DA.norm();
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getP()->get()) + DA / 2 + perpendicularDA * _length / 2;
//        container_orientation(0) = atan2(-DA(1),-DA(0));
//    }
//    // Diagonal detected
//    // A & C
//    else if (_corner_A_ptr != nullptr && _corner_C_ptr != nullptr)
//    {
//        Eigen::Vector2s AC = Eigen::Map<Eigen::Vector2s>(_corner_C_ptr->getP()->get()) - Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getP()->get());
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_A_ptr->getP()->get()) + AC / 2;
//        container_orientation(0) = atan2(AC(1),AC(0)) - atan2(_witdh,_length);
//    }
//    // B & D
//    else if (_corner_B_ptr != nullptr && _corner_D_ptr != nullptr)
//    {
//        Eigen::Vector2s BD = Eigen::Map<Eigen::Vector2s>(_corner_D_ptr->getP()->get()) - Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getP()->get());
//        container_position = Eigen::Map<Eigen::Vector2s>(_corner_B_ptr->getP()->get()) + BD / 2;
//        container_orientation(0) = atan2(BD(1),BD(0)) + atan2(_witdh,_length);
//    }
//}

LandmarkContainer::~LandmarkContainer()
{
    //
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
    assert(/*_id >= 0 &&*/ _id <= 4 && "wrong corner id parameter in getCorner(id)");
    return corners_.col(_id);
}

} // namespace wolf
