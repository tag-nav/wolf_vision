#ifndef CONSTRAINT_GPS_PSEUDORANGE_2D_H_
#define CONSTRAINT_GPS_PSEUDORANGE_2D_H_

#define LIGHT_SPEED_ 299792458

//Wolf includes
#include "sensor_gps.h"
#include "feature_gps_pseudorange.h"
#include "constraint_sparse.h"

//std
#include <string>
#include <sstream>

namespace wolf {

/*
 * NB:
 * FROM THIS CLASS AND ALL THE CLASS INCLUDED, THE LIBRARY RAW_GPS_UTILS
 * MUST NOT BE REACHABLE!
 * OTHERWISE WOLF WON'T COMPILE WITHOUT INSTALLING THIS LIBRARY.
 *
 * TODO maybe is no more true
 */



class ConstraintGPSPseudorange2D : public ConstraintSparse<1, 2, 1, 3, 1, 3, 1>
{
public:
    ConstraintGPSPseudorange2D(FeatureBase* _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintSparse<1, 2, 1, 3, 1, 3, 1>(_ftr_ptr,
                                                  CTR_GPS_PR_2D,
                                                  _apply_loss_function,
                                                  _status,
                                                  _ftr_ptr->getCapturePtr()->getFramePtr()->getPPtr(), // position of the vehicle's frame with respect to the initial pos frame
                                                  _ftr_ptr->getCapturePtr()->getFramePtr()->getOPtr(), // orientation of the vehicle's frame
                                                  _ftr_ptr->getCapturePtr()->getSensorPPtr(), // position of the sensor (gps antenna) with respect to the vehicle frame
                                                                                              // orientation of antenna is not needed, because omnidirectional
                                                  _ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr(), //intrinsic parameter = receiver time bias
                                                  ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getMapPPtr(), // map position with respect to ecef
                                                  ((SensorGPS*)_ftr_ptr->getCapturePtr()->getSensorPtr())->getMapOPtr()) // map orientation with respect to ecef
    {
        setType("GPS PR 2D");
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
                     const T* const _bias, const T* const _map_p, const T* const _map_o,
                     T* _residual) const;

    /** \brief Returns the jacobians computation method
     *
     * Returns the jacobians computation method
     *
     **/
    virtual JacobianMethod getJacobianMethod() const
    {
        return JAC_AUTO;
    }

protected:
    Eigen::Vector3s sat_position_;
    Scalar pseudorange_;

public:
    static wolf::ConstraintBase* create(FeatureBase* _feature_ptr, //
                                        NodeBase* _correspondant_ptr = nullptr)
    {
        return new ConstraintGPSPseudorange2D(_feature_ptr);
    }

};

/*
 * naming convention for transformation matrix:
 * T_a_b is "transformation from b to a"
 * To transform a point from b to a:    p_a = T_a_b * P_b
 * T_a_b also means "the pose of b expressed in frame a"
 */

template<typename T>
inline bool ConstraintGPSPseudorange2D::operator ()(const T* const _vehicle_p, const T* const _vehicle_o,
                                                    const T* const _sensor_p, const T* const _bias,
                                                    const T* const _map_p, const T* const _map_o,
                                                    T* _residual) const
{
    int verbose_level_ = 0; // 0=nothing printed. only for debug purpose

    std::stringstream aux;
    aux << std::setprecision(12);
    std::cout << std::setprecision(12);

    if (verbose_level_ >= 2)
    {
        std::cout << "+OPERATOR()+" << nodeId() << std::endl;
        std::cout << "_sensor_p(_base): " << _sensor_p[0] << ", " << _sensor_p[1] << ", " << _sensor_p[2] << std::endl;
        std::cout << "_vehicle_p(_map): " << _vehicle_p[0] << ", " << _vehicle_p[1] << std::endl;
        std::cout << "_vehicle_o(_map): " << _vehicle_o[0] << std::endl;
        std::cout << "_map_p: " << _map_p[0] << ", " << _map_p[1] << ", " << _map_p[2] << std::endl;
        std::cout << "_map_o: " << _map_o[0] << std::endl;
    }
    //Filling Eigen vectors
    Eigen::Matrix<T, 4, 1> sensor_p_base(_sensor_p[0], _sensor_p[1], _sensor_p[2], T(1)); //sensor position with respect base frame


    /*
     * Base-to-map transform matrix
     */
    Eigen::Matrix<T, 4, 4> T_map_base = Eigen::Matrix<T, 4, 4>::Identity();
    T_map_base(0, 0) = T(cos(_vehicle_o[0]));
    T_map_base(0, 1) = T(-sin(_vehicle_o[0]));
    T_map_base(1, 0) = T(sin(_vehicle_o[0]));
    T_map_base(1, 1) = T(cos(_vehicle_o[0]));
    T_map_base(0, 3) = T(_vehicle_p[0]);
    T_map_base(1, 3) = T(_vehicle_p[1]);

    // sensor position with respect to map frame
    Eigen::Matrix<T, 4, 1> sensor_p_map = T_map_base * sensor_p_base;

    if (verbose_level_ >= 2)
    {
        aux.str(std::string());
        aux << sensor_p_map(0);
        if (aux.str().substr(0, 1) != "[")
            std::cout << "!!! sensor_p_map: " << sensor_p_map[0] << ", " << sensor_p_map[1] << ", " << sensor_p_map[2] << std::endl;
        else {
            std::cout << "!!! sensor_p_map: " << aux.str().substr(1, aux.str().find(" ") - 1) << ", ";
            aux.str(std::string());
            aux << sensor_p_map(1);
            std::cout << aux.str().substr(1, aux.str().find(" ") - 1) << ", ";
            aux.str(std::string());
            aux << sensor_p_map(2);
            std::cout << aux.str().substr(1, aux.str().find(" ") - 1) << std::endl;
            aux.str(std::string());
        }
    }

    /*
     * _map_p from ECEF to LLA (math from https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf )
     */
    // WGS84 ellipsoid constants
    T a = T(6378137); // earth's radius
    T e = T(8.1819190842622e-2); // eccentricity
    T asq = a * a;
    T esq = e * e;
    T b = T(sqrt(asq * (T(1) - esq)));
    T bsq = T(b * b);
    T ep = T(sqrt((asq - bsq) / bsq));
    T p = T(sqrt(_map_p[0] * _map_p[0] + _map_p[1] * _map_p[1]));
    T th = T(atan2(a * _map_p[2], b * p));
    T lon = T(atan2(_map_p[1], _map_p[0]));
    T lat = T(atan2((_map_p[2] + ep * ep * b * pow(sin(th), 3)), (p - esq * a * pow(cos(th), 3))));

    if (verbose_level_ >= 3)
    {
        std::cout << "_map_p: " << _map_p[0] << ", " << _map_p[1] << ", " << _map_p[2] << std::endl;
        std::cout << "_map_p LLA: " << lat << ", " << lon << std::endl;
        std::cout << "_map_p LLA degrees: " << lat * T(180 / M_PI) << ", " << lon * T(180 / M_PI) << std::endl;
    }

    /*
     * map-to-ECEF transform matrix
     * made by the product of the next 4 matrixes
     */
    Eigen::Matrix<T, 4, 4> T_ecef_aux = Eigen::Matrix<T, 4, 4>::Identity();
    T_ecef_aux(0, 3) = T(_map_p[0]);
    T_ecef_aux(1, 3) = T(_map_p[1]);
    T_ecef_aux(2, 3) = T(_map_p[2]);

    Eigen::Matrix<T, 4, 4> T_aux_lon = Eigen::Matrix<T, 4, 4>::Identity();
    T_aux_lon(0, 0) = T(cos(lon));
    T_aux_lon(0, 1) = T(-sin(lon));
    T_aux_lon(1, 0) = T(sin(lon));
    T_aux_lon(1, 1) = T(cos(lon));

    Eigen::Matrix<T, 4, 4> T_lon_lat = Eigen::Matrix<T, 4, 4>::Identity();
    T_lon_lat(0, 0) = T(cos(lat));
    T_lon_lat(0, 2) = T(-sin(lat));
    T_lon_lat(2, 0) = T(sin(lat));
    T_lon_lat(2, 2) = T(cos(lat));


    Eigen::Matrix<T, 4, 4> T_lat_enu = Eigen::Matrix<T, 4, 4>::Zero();
    T_lat_enu(0, 2) = T_lat_enu(1, 0) = T_lat_enu(2, 1) = T_lat_enu(3, 3) = T(1);

    Eigen::Matrix<T, 4, 4> T_enu_map = Eigen::Matrix<T, 4, 4>::Identity();
    T_enu_map(0, 0) = T(cos(_map_o[0]));
    T_enu_map(0, 1) = T(-sin(_map_o[0]));
    T_enu_map(1, 0) = T(sin(_map_o[0]));
    T_enu_map(1, 1) = T(cos(_map_o[0]));

    Eigen::Matrix<T, 4, 4> T_ecef_map = T_ecef_aux * T_aux_lon * T_lon_lat * T_lat_enu * T_enu_map;

    //sensor position with respect to ecef coordinate system
    Eigen::Matrix<T, 4, 1> sensor_p_ecef = T_ecef_map * sensor_p_map;


    /*
     * calculate the residual
     */
    T square_sum = T(0);
    for (int i = 0; i < 3; ++i)
        square_sum += (sensor_p_ecef[i] - T(sat_position_[i]))*(sensor_p_ecef[i] - T(sat_position_[i]));

    T distance = (square_sum != T(0)) ? sqrt(square_sum) : T(0);

    //     error = (expected measurement)       - (actual measurement)
    _residual[0] = (distance + _bias[0] * T(LIGHT_SPEED_)) - T(pseudorange_);

    if (verbose_level_ >= 2)
        std::cout << "!!! Residual: " << _residual[0] << "\n";

    // normalizing by the covariance
    _residual[0] = _residual[0] / T(getMeasurementCovariance()(0, 0));//T(sqrt(getMeasurementCovariance()(0, 0)));


    if (verbose_level_ >= 1)
    {
        aux.str(std::string());
        aux << sensor_p_ecef(0);
        if (aux.str().substr(0, 1) != "[")
            std::cout << "!!! sensor_p_ecef: " << sensor_p_ecef[0] << ", " << sensor_p_ecef[1] << ", " << sensor_p_ecef[2];
        else {
            std::cout << "!!! sensor_p_ecef: " << aux.str().substr(1, aux.str().find(" ") - 1) << ", ";
            aux.str(std::string());
            aux << sensor_p_ecef(1);
            std::cout << aux.str().substr(1, aux.str().find(" ") - 1) << ", ";
            aux.str(std::string());
            aux << sensor_p_ecef(2);
            std::cout << aux.str().substr(1, aux.str().find(" ") - 1);
        }

        //std::cout << "Expected: " << (distance + _bias[0]*T(LIGHT_SPEED)) << "\nreceived = " << pseudorange_ << "\n";
        aux.str(std::string());
        aux << _residual[0];
        if (aux.str().substr(0,1) != "[" )
            std::cout << "\t Residual norm: " << _residual[0] << "\n";
        else
        {
            std::cout << "\t Residual norm: " << aux.str().substr(1, aux.str().find(" ") - 1)  << "\n";
        }
    }

    return true;
}

} // namespace wolf

#endif //CONSTRAINT_GPS_PSEUDORANGE_2D_H_
