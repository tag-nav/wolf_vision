#ifndef CONSTRAINT_GPS_PSEUDORANGE_H_
#define CONSTRAINT_GPS_PSEUDORANGE_H_

#define LIGHT_SPEED 299792458

//Wolf includes
#include "sensor_gps.h"
#include "feature_gps_pseudorange.h"
#include "constraint_sparse.h"
/*
 * NB:
 * FROM THIS CLASS AND ALL THE CLASS INCLUDED, THE LIBRARY RAW_GPS_UTILS
 * MUST NOT BE REACHABLE!
 * OTHERWISE WOLF WON'T COMPILE WITHOUT INSTALLING THIS LIBRARY.
 *
 */



class ConstraintGPSPseudorange: public ConstraintSparse<1, 3, 4, 3, 1, 3, 4>
{

public:

    ConstraintGPSPseudorange(FeatureBase* _ftr_ptr, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintSparse<1, 3, 4, 3, 1, 3, 4>(_ftr_ptr, CTR_GPS_PR_3D, _status,
                            _ftr_ptr->getCapturePtr()->getFramePtr()->getPPtr(), // position of the vehicle's frame with respect to the initial pos frame
                            _ftr_ptr->getCapturePtr()->getFramePtr()->getOPtr(), // orientation of the vehicle's frame
                            _ftr_ptr->getCapturePtr()->getSensorPPtr(), // position of the sensor (gps antenna) with respect to the vehicle frame
                                                                        // orientation of antenna is not needed, because omnidirectional
                            _ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr(), //intrinsic parameter  = receiver time bias
                            ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getInitVehiclePPtr(), // initial vehicle position (ecef)
                            ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getInitVehicleOPtr()  // initial vehicle orientation (ecef)
            ) //TODO attention at the last 2 params and the casts
    {
        sat_position_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getSatPosition();
        pseudorange_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getPseudorange();

        std::cout << "#ConstraintGPSPseudorange() constructor:   pr=" << pseudorange_ << "\tsat_pos=(" << sat_position_[0] << ", " << sat_position_[1] << ", " << sat_position_[2] << ")" << std::endl;
    }


    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~ConstraintGPSPseudorange()
    {
        //std::cout << "deleting ConstraintGPSPseudorange " << nodeId() << std::endl;
    }


    template <typename T>
    bool operator()(const T* const _vehicle_p, const T* const _vehicle_q, const T* const _sensor_p, const T* const _bias, const T* const _init_vehicle_p, const T* const _init_vehicle_q, T* _residual) const
    {
        /*
         * TODO DO THE MATH
         *
         * fai i conti seriamente, usando tutte ste robe
         */



        /*
         * TODO improve naming for more coherence.
         *
         * origin = init_vehicle
         * base = vehicle
         */
        Eigen::Vector4s sensor_p_ecef; //sensor position with respect to ecef coordinate system
        Eigen::Vector4s sensor_p_base; //sensor position with respect to the base (the vehicle)
                                    //((double)_sensor_p[0], (double)_sensor_p[1], (double)_sensor_p[2]);
                                    //TODo è da paddare con 0 o 1?

        Eigen::Matrix4s conv_origin_to_ecef;
        Eigen::Matrix4s conv_base_to_origin;

        //TODO riempi le matrici correttamente



        // transformation from
        sensor_p_ecef = conv_origin_to_ecef * conv_base_to_origin * sensor_p_base;

        //sensor_p_ecef deve essere trasformato come template
        //T* sensor_p_ecef_template; //TODO assegna le prime 3 componenti di sensor_p_ecef


        //il codice qui sotto è quello vecchio, adattato in modo da usare la posizione del sensore rispetto a ecef, calcolata qui sopra
        T square_sum = T(0);
        for (int i = 0; i < 3; ++i)
        { //TODO qui va usata la posizione del sensore rispetto a ecef e non vehicle_p
            //TODO  ovvero sensor_p_ecef_template; ora c'è vehicle per  evitare sed fault, visto che sensor__p_ecef è vuoto
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
