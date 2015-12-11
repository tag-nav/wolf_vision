#ifndef CONSTRAINT_GPS_PSEUDORANGE_H_
#define CONSTRAINT_GPS_PSEUDORANGE_H_

#define LIGHT_SPEED 299792458

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"
#include "feature_gps_pseudorange.h"

//TODO indenta tutto di 1

class ConstraintGPSPseudorange: public ConstraintSparse<1, 3, 4, 3, 4, 1>// TODO da rimettere quando lavorero' con ceres e wolf
{

public:

//    // TODO 1: tutti i parametri li ho copiati da gps2D, non so cosa mi servira' realmente
//    // TODO 2: scopri come funziona wolf e i vari parametri.
    ConstraintGPSPseudorange(FeatureBase* _ftr_ptr, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintSparse<1, 3, 4, 3, 4, 1>(_ftr_ptr,CTR_GPS_PR_3D, _status,
                                               _ftr_ptr->getCapturePtr()->getFramePtr()->getPPtr(), _ftr_ptr->getCapturePtr()->getFramePtr()->getOPtr(),
                                               _ftr_ptr->getCapturePtr()->getSensorPPtr(), _ftr_ptr->getCapturePtr()->getSensorOPtr(),
                                               _ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr())
    {
        createConstraint((FeatureGPSPseudorange*) _ftr_ptr);
    }

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
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

    template <typename T>///TODO rename sensor and vehicle p and q
    bool operator()(const T* const _v_p, const T* const _v_q,const T* const _s_p,const T* const _s_q, const T* const _bias, T* _residual) const
    {
        T square_sum = T(0);
        for (int i = 0; i < 3; ++i) {
            square_sum += pow(_v_p[i] - T(sat_position_[i]) , 2);
        }
        T distance = (square_sum != T(0)) ? sqrt(square_sum) : T(0) ;

        //     error = (expected measurement)       - (actual measurement)
        _residual[0] = (distance + _bias[0]*T(LIGHT_SPEED)) - (pseudorange_);

        return true;
    }


protected:
    //TODO vedi se tenere qui sta roba o se c'e' gia nella classe base
    Eigen::Vector3s sat_position_;
    WolfScalar pseudorange_;

};

#endif //CONSTRAINT_GPS_PSEUDORANGE_H_
