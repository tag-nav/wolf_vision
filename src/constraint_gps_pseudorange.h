//
// Created by ptirindelli on 3/12/15.
//

#ifndef CONSTRAINT_GPS_PSEUDORANGE_H_
#define CONSTRAINT_GPS_PSEUDORANGE_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

//TODO indenta tutto di 1

class ConstraintGPSPseudorange//: public ConstraintSparse<1, 3, 1> TODO da rimettere quando lavorero' con ceres e wolf
{

public:

//    // TODO 1: tutti i parametri li ho copiati da gps2D, non so cosa mi servira' realmente
//    // TODO 2: scopri come funziona wolf e i vari parametri.
//    ConstraintGPSPseudorange(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, ConstraintStatus _status = CTR_ACTIVE) :
//            ConstraintSparse<1, 3, 1>(_ftr_ptr,CTR_GPS_PR_3D, _status, _frame_ptr->getPPtr())
//    {
//        createConstraint((FeatureGPSPseudorange*) _ftr_ptr);
//    }

    ConstraintGPSPseudorange(FeatureBase* _ftr_ptr) :
            SPEED_(299792458)// TODO vedi se usarla come costante o come variabile, e dove metterla
    {
        createConstraint((FeatureGPSPseudorange*) _ftr_ptr);
    }

    virtual ~ConstraintGPSPseudorange()
    {

    }

    void createConstraint(FeatureGPSPseudorange* _ptr)
    {

        //todo assegna un valore serio a sat_pos e pseudorange
        sat_position_ = Eigen::Vector3s(11000, 12000, 13000); // TODO
        pseudorange_ = 10001; // TODO

        std::cout << "Creating constraint from " << _ptr->getSatId_()
                  << " with: pr=" << pseudorange_
                  << " and satPos=" << sat_position_
                  << std::endl;
        //TODO 0: crea il constraint
    }

    template <typename T>
    bool operator()(const T* const _x, const T* const _bias, T* _residual) const
    {
        T square_sum = T(0);
        for (int i = 0; i < 3; ++i) {
            square_sum += pow(_x[i] - T(sat_position_[i]) , 2);
        }
        T distance = (square_sum != T(0)) ? sqrt(square_sum) : T(0) ;

        //     error = (expected measurement)       - (actual measurement)
        _residual[0] = (distance + _bias[0]*SPEED_) - (pseudorange_);

        return true;
    }


protected:
    //TODO vedi se tenere qui sta roba o se c'e' gia nella classe base
    Eigen::Vector3s sat_position_;
    WolfScalar pseudorange_;
    const WolfScalar SPEED_; // TODO vedi se usarla come costante o come variabile, e dove metterla

};

#endif //CONSTRAINT_GPS_PSEUDORANGE_H_
