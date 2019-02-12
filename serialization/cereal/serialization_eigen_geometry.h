#ifndef _WOLF_IO_CEREAL_EIGEN_GEOMETRY_H_
#define _WOLF_IO_CEREAL_EIGEN_GEOMETRY_H_

// Wolf includes
#include <Eigen/Geometry>

#include "serialization_eigen_core.h"

namespace cereal {

template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
inline void save(Archive& ar,
                 const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& t)
{
  save(ar, t.matrix());
}

template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
inline void load(Archive& ar,
                 Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& t)
{
  load(ar, t.matrix());
}

template<class Archive, typename _Scalar>
void serialize(Archive & ar,
               Eigen::Quaternion<_Scalar>& q,
               const std::uint32_t /*version*/)
{
  ar(cereal::make_nvp("w", q.w()));
  ar(cereal::make_nvp("x", q.x()));
  ar(cereal::make_nvp("y", q.y()));
  ar(cereal::make_nvp("z", q.z()));
}

} // namespace cereal

#endif /* _WOLF_IO_CEREAL_EIGEN_GEOMETRY_H_ */
