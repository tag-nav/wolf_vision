#ifndef _WOLF_EIGEN_ASSERT_H_
#define _WOLF_EIGEN_ASSERT_H_

#include <Eigen/Dense>

namespace Eigen {

//////////////////////////////////////////////////////////
/** Check Eigen Matrix sizes, both statically and dynamically
 *
 * Help:
 *
 * The WOLF project implements many template functions using Eigen Matrix and Quaternions, in different versions
 * (Static size, Dynamic size, Map, Matrix expression).
 *
 * Eigen provides some macros for STATIC assert of matrix sizes, the most common of them are (see Eigen's StaticAssert.h):
 *
 *      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE
 *      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE
 *      EIGEN_STATIC_ASSERT_VECTOR_ONLY
 *
 * but they do not work if the evaluated types are of dynamic size.
 *
 * In order to achieve full templatization over both dynamic and static sizes, we use extensively a prototype of this kind:
 *
 * template<typename Derived>
 * inline Eigen::Matrix<typename Derived::Scalar, 3, 3> function(const Eigen::MatrixBase<Derived>& _v){
 *
 *     MatrixSizeCheck<3,1>::check(_v); /// We check here the size of the input parameter
 *
 *     typedef typename Derived::Scalar T;
 *
 *     ... code ...
 *
 *     return M;
 *     }
 *
 * The function :  MatrixSizeCheck <Rows, Cols>::check(M)   checks that the Matrix M is of size ( Rows x Cols ).
 * This check is performed statically or dynamically, depending on the type of argument provided.
 */
template<int Size, int DesiredSize>
struct StaticDimCheck
{
  template<typename T>
  StaticDimCheck(const T&)
  {
    static_assert(Size == DesiredSize, "Size of static Vector or Matrix does not match");
  }
};

template<int DesiredSize>
struct StaticDimCheck<Eigen::Dynamic, DesiredSize>
{
  template<typename T>
  StaticDimCheck(const T& t)
  {
    if (t != DesiredSize) std::cerr << "t : " << t << " != DesiredSize : " << DesiredSize << std::endl;
    assert(t == DesiredSize && "Size of dynamic Vector or Matrix does not match");
  }
};

template<int DesiredR, int DesiredC>
struct MatrixSizeCheck
{
  /** \brief Assert matrix size dynamically or statically
   *
   * @param t the variable for which we wish to assert the size.
   *
   * Usage: to assert that t is size (Rows x Cols)
   *
   *  MatrixSizeCheck<Rows, Cols>::check(t);
   */
  template<typename T>
  static void check(const Eigen::MatrixBase<T>& t)
  {
    StaticDimCheck<Eigen::MatrixBase<T>::RowsAtCompileTime, DesiredR>(t.rows());
    StaticDimCheck<Eigen::MatrixBase<T>::ColsAtCompileTime, DesiredC>(t.cols());
  }
};

template <int Dim>
using VectorSizeCheck = MatrixSizeCheck<Dim, 1>;

template <int Dim>
using RowVectorSizeCheck = MatrixSizeCheck<1, Dim>;

} /* namespace Eigen */

#endif /* _WOLF_EIGEN_ASSERT_H_ */
