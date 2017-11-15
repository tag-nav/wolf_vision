#ifndef _WOLF_IO_CEREAL_EIGEN_GEOMETRY_H_
#define _WOLF_IO_CEREAL_EIGEN_GEOMETRY_H_

// Wolf includes
#include <Eigen/Sparse>

#include "serialization_eigen_core.h"
#include <cereal/types/vector.hpp>

namespace cereal {

template<class Archive, typename Scalar, typename Index>
inline void save(Archive& ar,
                 const Eigen::Triplet<Scalar, Index>& t)
{
  ar(cereal::make_nvp("row", t.row()));
  ar(cereal::make_nvp("col", t.col()));
  ar(cereal::make_nvp("value", t.value()));
}

template<class Archive, typename Scalar, typename Index>
inline void load(Archive& ar,
                 Eigen::Triplet<Scalar, Index>& t)
{
  Index row, col;
  Scalar value;

  ar(cereal::make_nvp("row", row));
  ar(cereal::make_nvp("col", col));
  ar(cereal::make_nvp("value", value));

  t = Eigen::Triplet<Scalar, Index>(row, col, value);
}

template <class Archive, typename _Scalar, int _Options, typename _Index>
void save(Archive& ar,
          const Eigen::SparseMatrix<_Scalar, _Options, _Index>& m)
{
  _Index inner_size = m.innerSize();
  _Index outer_size = m.outerSize();

  using Triplet = Eigen::Triplet<_Scalar>;
  std::vector<Triplet> triplets;

  for (_Index i=0; i < outer_size; ++i)
    for (typename Eigen::SparseMatrix<_Scalar, _Options, _Index>::InnerIterator it(m,i); it; ++it)
      triplets.emplace_back( it.row(), it.col(), it.value() );

  ar(cereal::make_nvp("inner_size", inner_size));
  ar(cereal::make_nvp("outer_size", outer_size));
  ar(cereal::make_nvp("triplets",   triplets));
}

template <class Archive, typename _Scalar, int _Options, typename _Index>
void load(Archive& ar,
          Eigen::SparseMatrix<_Scalar, _Options, _Index>& m)
{
  _Index inner_size;
  _Index outer_size;

  ar(cereal::make_nvp("inner_size", inner_size));
  ar(cereal::make_nvp("outer_size", outer_size));

  _Index rows = (m.IsRowMajor)? outer_size : inner_size;
  _Index cols = (m.IsRowMajor)? inner_size : outer_size;

  m.resize(rows, cols);

  using Triplet = Eigen::Triplet<_Scalar>;
  std::vector<Triplet> triplets;

  ar(cereal::make_nvp("triplets", triplets));

  m.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace cereal

#endif /* _WOLF_IO_CEREAL_EIGEN_GEOMETRY_H_ */
