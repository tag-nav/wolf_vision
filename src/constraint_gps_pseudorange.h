#ifndef CONSTRAINT_GPS_PSEUDORANGE_H_
#define CONSTRAINT_GPS_PSEUDORANGE_H_

#define LIGHT_SPEED 299792458

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"
#include "feature_gps_pseudorange.h"
#include "raw_data_satellite.h"

//TODO indenta tutto di 1

class ConstraintGPSPseudorange: public ConstraintSparse<1, 3, 4, 3, 4, 1>// TODO da rimettere quando lavorero' con ceres e wolf
{

public:

    ConstraintGPSPseudorange(FeatureBase* _ftr_ptr, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintSparse<1, 3, 4, 3, 4, 1>(_ftr_ptr,CTR_GPS_PR_3D, _status,
                                               _ftr_ptr->getCapturePtr()->getFramePtr()->getPPtr(), _ftr_ptr->getCapturePtr()->getFramePtr()->getOPtr(), // position and orientation of the car's frame
                                               _ftr_ptr->getCapturePtr()->getSensorPPtr(), _ftr_ptr->getCapturePtr()->getSensorOPtr(), // position and orientation of the sensor (gps antenna)
                                               _ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr()) //intrinsic parameter  = receiver bias (for now)
    {
        // createConstraint


        //todo assegna un valore serio a sat_pos e pseudorange
        sat_position_ = Eigen::Vector3s(11000, 12000, 13000); // TODO
        pseudorange_ = 1001; // TODO

//        std::cout << "Creating constraint from " << ((FeatureGPSPseudorange*) _ftr_ptr)->getObs().toString();
        //TODO 0: crea il constraint

        //TODO qui
        //TODO qui
        //TODO qui
        //TODO qui      segnati sat_pos e pr dall'oggetto obs!
        //TODO qui
        //TODO qui
        //TODO qui


    }

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~ConstraintGPSPseudorange()
    {
        destruct();
    }


    template <typename T>
    bool operator()(const T* const _vehicle_p, const T* const _vehicle_q, const T* const _sensor_p,const T* const _sensor_q, const T* const _bias, T* _residual) const
    {
        T square_sum = T(0);
        for (int i = 0; i < 3; ++i) {
            square_sum += pow(_vehicle_p[i] - T(sat_position_[i]) , 2);
        }
        T distance = (square_sum != T(0)) ? sqrt(square_sum) : T(0) ;

        //     error = (expected measurement)       - (actual measurement)
        _residual[0] = (distance + _bias[0]*T(LIGHT_SPEED)) - (pseudorange_);


        /* TODO importante
         * credo che il residuo sia la differenza delle misure, NORMALIZZATA PER LA COVARIANZA
         */

        return true;
    }


protected:
    Eigen::Vector3s sat_position_;
    WolfScalar pseudorange_;

};

#endif //CONSTRAINT_GPS_PSEUDORANGE_H_
