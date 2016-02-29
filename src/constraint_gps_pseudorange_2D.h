#ifndef CONSTRAINT_GPS_PSEUDORANGE_2D_H_
#define CONSTRAINT_GPS_PSEUDORANGE_2D_H_

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



class ConstraintGPSPseudorange2D : public ConstraintSparse<1, 2, 1, 3, 1, 3, 1>
{
    public:
        bool verbose = false; // TODO only for debug purpose

        ConstraintGPSPseudorange2D(FeatureBase* _ftr_ptr, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<1, 2, 1, 3, 1, 3, 1>(_ftr_ptr,
                                                      CTR_GPS_PR_2D,
                                                      _status,
                                                      _ftr_ptr->getCapturePtr()->getFramePtr()->getPPtr(), // position of the vehicle's frame with respect to the initial pos frame
                        _ftr_ptr->getCapturePtr()->getFramePtr()->getOPtr(), // orientation of the vehicle's frame
                        _ftr_ptr->getCapturePtr()->getSensorPPtr(), // position of the sensor (gps antenna) with respect to the vehicle frame
                                                                    // orientation of antenna is not needed, because omnidirectional
                        _ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr(), //intrinsic parameter = receiver time bias
                        ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getInitVehiclePPtr(), // initial vehicle position (ecef)
                        ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getInitVehicleOPtr() // initial vehicle orientation (ecef)
                        )
        {
            sat_position_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getSatPosition();
            pseudorange_ = ((FeatureGPSPseudorange*)_ftr_ptr)->getPseudorange();

            //std::cout << "ConstraintGPSPseudorange2D()  pr=" << pseudorange_ << "\tsat_pos=(" << sat_position_[0] << ", " << sat_position_[1] << ", " << sat_position_[2] << ")" << std::endl;
        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintGPSPseudorange2D()
        {
            //std::cout << "deleting ConstraintGPSPseudorange2D " << nodeId() << std::endl;
        }

        template<typename T>
        bool operator ()(const T* const _vehicle_p, const T* const _vehicle_o, const T* const _sensor_p,
                         const T* const _bias, const T* const _init_vehicle_p, const T* const _init_vehicle_o,
                         T* _residual) const;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return AUTO;
        }

    protected:
        Eigen::Vector3s sat_position_;
        WolfScalar pseudorange_;

};

template<typename T>
inline bool ConstraintGPSPseudorange2D::operator ()(const T* const _vehicle_p, const T* const _vehicle_o,
                                                    const T* const _sensor_p, const T* const _bias,
                                                    const T* const _init_vehicle_p, const T* const _init_vehicle_o,
                                                    T* _residual) const
{
    if (verbose)
    {
        std::cout << std::setprecision(12);
        std::cout << "\n++++++OPERATOR()++++++\n";
        std::cout << "_init_vehicle_p: " << _init_vehicle_p[0] << ", " << _init_vehicle_p[1] << ", "
                << _init_vehicle_p[2] << std::endl;
        std::cout << "_init_vehicle_o: " << _init_vehicle_o[0] << std::endl;
        std::cout << "_vehicle_p(_map): " << _vehicle_p[0] << ", " << _vehicle_p[1] << std::endl;
        std::cout << "_vehicle_o(_map): " << _vehicle_o[0] << std::endl;
        std::cout << "_sensor_p(_base): " << _sensor_p[0] << ", " << _sensor_p[1] << ", " << _sensor_p[2] << std::endl;
    }
    Eigen::Matrix<T, 3, 1> sensor_p_base(_sensor_p[0], _sensor_p[1], _sensor_p[2]); //sensor position with respect to the base (the vehicle)
    Eigen::Matrix<T, 3, 1> vehicle_p_map(_vehicle_p[0], _vehicle_p[1], T(0));
    Eigen::Matrix<T, 3, 1> init_vehicle_p(_init_vehicle_p[0], _init_vehicle_p[1], _init_vehicle_p[2]);
    /*
     * Base-to-map transform matrix
     */
    Eigen::Matrix<T, 3, 3> T_base2map = Eigen::Matrix<T, 3, 3>::Identity();
    T_base2map(0, 0) = T(cos(_vehicle_o[0]));
    T_base2map(0, 1) = T(sin(_vehicle_o[0]));
    T_base2map(1, 0) = T(-sin(_vehicle_o[0]));
    T_base2map(1, 1) = T(cos(_vehicle_o[0]));
    Eigen::Matrix<T, 3, 1> sensor_p_map; // sensor position with respect to map frame (initial frame of the experiment)
    sensor_p_map = T_base2map * sensor_p_base + vehicle_p_map;
    if (verbose)
    {
        std::cout << "1st trasform:  ";
        std::cout << "sensor_p_map: " << sensor_p_map[0] << ", " << sensor_p_map[1] << ", " << sensor_p_map[2]
                << std::endl;
    }
    /*
     * _init_vehicle_p from ecef to lla
     */
    // WGS84 ellipsoid constants
    T a = T(6378137); // earth's radius
    T e = T(8.1819190842622e-2); // eccentricity
    T asq = a * a;
    T esq = e * e;
    T b = T(sqrt(asq * (T(1) - esq)));
    T bsq = T(b * b);
    T ep = T(sqrt((asq - bsq) / bsq));
    T p = T(sqrt(_init_vehicle_p[0] * _init_vehicle_p[0] + _init_vehicle_p[1] * _init_vehicle_p[1]));
    T th = T(atan2(a * _init_vehicle_p[2], b * p));
    T lon = T(atan2(_init_vehicle_p[1], _init_vehicle_p[0]));
    T lat = T(atan2((_init_vehicle_p[2] + ep * ep * b * pow(sin(th), 3)), (p - esq * a * pow(cos(th), 3))));
    //        T N = T(a/( sqrt(T(1)-esq*pow(sin(lat),2)) ));
    //        T alt = T(p / cos(lat) - N);
    // mod lat to 0-2pi
    while (lon < T(0))
        lon += T(2 * M_PI);
    while (lon > 2 * M_PI)
        lon -= T(2 * M_PI);
    // correction for altitude near poles left out.
    if (verbose)
    {
        std::cout << "_init_vehicle_p: " << _init_vehicle_p[0] << ", " << _init_vehicle_p[1] << ", "
                << _init_vehicle_p[2] << std::endl;
        //        std::cout << "_init_vehicle_p LLA: " << lat << ", " << lon /*<< ", " << alt*/ << std::endl;
        //        std::cout << "_init_vehicle_p LLA degrees: " << lat * T(180 / M_PI) << ", " << lon * T(180 / M_PI) /*<< ", " << alt*/ << std::endl;
    }
    /*
     * map-to-ECEF transform matrix
     */
    Eigen::Matrix<T, 3, 3> R1 = Eigen::Matrix<T, 3, 3>::Identity();
    R1(0, 0) = T(cos(lon));
    R1(0, 1) = T(sin(lon));
    R1(1, 0) = T(-sin(lon));
    R1(1, 1) = T(cos(lon));
    Eigen::Matrix<T, 3, 3> R2 = Eigen::Matrix<T, 3, 3>::Identity();
    R2(0, 0) = T(cos(lat));
    R2(0, 2) = T(sin(lat));
    R2(2, 0) = T(-sin(lat));
    R2(2, 2) = T(cos(lat));
    Eigen::Matrix<T, 3, 3> R3 = Eigen::Matrix<T, 3, 3>::Zero();
    R3(0, 1) = R3(1, 2) = R3(2, 0) = T(1);
    Eigen::Matrix<T, 3, 3> R4 = Eigen::Matrix<T, 3, 3>::Identity();
    R4(0, 0) = T(cos(_init_vehicle_o[0]));
    R4(0, 1) = T(sin(_init_vehicle_o[0]));
    R4(1, 0) = T(-sin(_init_vehicle_o[0]));
    R4(1, 1) = T(cos(_init_vehicle_o[0]));
    Eigen::Matrix<T, 3, 3> T_map2ecef = (R4 * R3 * R2 * R1).inverse();
    /*
     * result I want to find: sensor position with respect to ecef
     */
    Eigen::Matrix<T, 3, 1> sensor_p_ecef; //sensor position with respect to ecef coordinate system
    sensor_p_ecef = T_map2ecef * sensor_p_map + init_vehicle_p;
    if (verbose)
        std::cout << "!!! sensor_p_ecef: " << sensor_p_ecef[0] << ", " << sensor_p_ecef[1] << ", " << sensor_p_ecef[2]
                << std::endl;

    //il codice qui sotto è quello vecchio, adattato in modo da usare la posizione del sensore rispetto a ecef, calcolata qui sopra
    T square_sum = T(0);
    for (int i = 0; i < 3; ++i)
    {
        square_sum += pow(sensor_p_ecef[i] - T(sat_position_[i]), 2);
    }
    T distance = (square_sum != T(0)) ? sqrt(square_sum) : T(0);
    //     error = (expected measurement)       - (actual measurement)
    _residual[0] = (distance + _bias[0] * T(LIGHT_SPEED)) - (pseudorange_);
    if (verbose)
    {
        //        std::cout << "Expected: " << (distance + _bias[0]*T(LIGHT_SPEED)) << "\nreceived = " << pseudorange_ << "\n";
        std::cout << "Residual " << _residual[0] << "\n";
    }
    /* TODO importante
     * credo che il residuo sia la differenza delle misure, NORMALIZZATA PER LA COVARIANZA
     */
    if (verbose)
        std::cout << "------ END OPERATOR()------\n";

    return true;
}

#endif //CONSTRAINT_GPS_PSEUDORANGE_2D_H_
