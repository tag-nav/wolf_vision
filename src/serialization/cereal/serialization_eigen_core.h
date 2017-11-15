#ifndef _WOLF_IO_CEREAL_EIGEN_H_
#define _WOLF_IO_CEREAL_EIGEN_H_

// Wolf includes
#include <Eigen/Dense>
#include <cereal/cereal.hpp>

namespace cereal {

/**
 * @brief Save Eigen::Matrix<...> to text based archives
 */
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline typename std::enable_if<!traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value,
void>::type save(Archive& ar,
                 const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& mat)
{
  decltype(mat.rows()) rows = mat.rows();
  decltype(mat.cols()) cols = mat.cols();

  ar(cereal::make_nvp("rows", rows));
  ar(cereal::make_nvp("cols", cols));

  /// @todo find out something
  std::cerr << "Saving Eigen type to text-based archive is NOT supported !\n";
}

/**
 * @brief Save Eigen::Matrix<...> to binary based archives
 */
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value,
void>::type save(Archive& ar,
                 const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& mat)
{
  decltype(mat.rows()) rows = mat.rows();
  decltype(mat.cols()) cols = mat.cols();

  ar(rows);
  ar(cols);

  ar(binary_data(mat.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}

/**
 * @brief Load compile-time sized Eigen::Matrix from text based archives
 */
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline typename std::enable_if<!traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value and
                                _Rows != Eigen::Dynamic and _Cols != Eigen::Dynamic,
void>::type load(Archive& ar,
                 Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& mat)
{
  decltype(mat.rows()) rows;
  decltype(mat.cols()) cols;

  ar(cereal::make_nvp("rows", rows));
  ar(cereal::make_nvp("cols", cols));

  /// @todo find out something
  std::cerr << "Saving Eigen type to text-based archive is NOT supported !\n";
}

/**
 * @brief Load dynamic sized Eigen::Matrix from text based archives
 */
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline typename std::enable_if<!traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value and
                                (_Rows == Eigen::Dynamic or _Cols == Eigen::Dynamic),
void>::type load(Archive& ar,
                 Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& mat)
{
  decltype(mat.rows()) rows;
  decltype(mat.cols()) cols;

  ar(cereal::make_nvp("rows", rows));
  ar(cereal::make_nvp("cols", cols));

  /// @todo find out something
  std::cerr << "Saving Eigen type to text-based archive is NOT supported !\n";

  //mat.resize(rows, cols);
}

/**
 * @brief Load compile-time sized Eigen::Matrix from binary based archives
 */
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value and
                               _Rows != Eigen::Dynamic and _Cols != Eigen::Dynamic,
void>::type load(Archive& ar,
                 Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& mat)
{
  decltype(mat.rows()) rows;
  decltype(mat.cols()) cols;

  ar(rows);
  ar(cols);

  ar(binary_data(mat.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}

/**
 * @brief Load dynamic sized Eigen::Matrix from binary based archives
 */
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value and
                               (_Rows == Eigen::Dynamic or _Cols == Eigen::Dynamic),
void>::type load(Archive& ar,
                 Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& mat)
{
  decltype(mat.rows()) rows;
  decltype(mat.cols()) cols;

  ar(rows);
  ar(cols);

  mat.resize(rows, cols);

  ar(binary_data(mat.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}

} // namespace cereal

#endif /* _WOLF_IO_CEREAL_EIGEN_H_ */
