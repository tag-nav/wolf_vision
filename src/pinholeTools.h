#ifndef PINHOLETOOLS_H
#define PINHOLETOOLS_H

/**
 * \file pinholeTools.h
 *
 * \date 06/04/2010
 * \author jsola
 *
 *
 *  ## Add a description here ##
 *
 */

#include "wolf.h"

#include <iostream>

namespace wolf {
/**
 * Namespace for operations related to the pin-hole model of a camera.
 *
 * The model incorporates radial distortion.
 *
 */
namespace pinhole {

using Eigen::MatrixBase;
using Eigen::Matrix;

/**
 * Pin-hole canonical projection.
 * \param _v a 3D point to project
 * \param _up the projected point in the normalized 2D plane
 */
template<typename Derived1, typename Derived2>
inline void
projectPointToNormalizedPlane(const MatrixBase<Derived1>& _v,
                              MatrixBase<Derived2>&       _up)
{
    MatrixSizeCheck<3, 1>::check(_v);
    MatrixSizeCheck<2, 1>::check(_up);

    _up(0) = _v(0) / _v(2);
    _up(1) = _v(1) / _v(2);
}

/**
 * Pin-hole canonical projection.
 * \return the projected point in the normalized 2D plane
 * \param _v a 3D point
 */
template<typename Derived>
inline Matrix<typename Derived::Scalar, 2, 1>
projectPointToNormalizedPlane(const MatrixBase<Derived>& _v)
{
    MatrixSizeCheck<3, 1>::check(_v);

    typedef typename Derived::Scalar T;
    Matrix<T, 2, 1> up;

    projectPointToNormalizedPlane(_v, up);

    return up;
}

/**
 * Pin-hole canonical projection, return also distance (not depth!).
 * \param _v a 3D point
 * \param _up the projected point in the normalized 2D plane
 * \param _dist the distance from the camera to the point
 */
template<typename Derived1, typename Derived2>
inline void
projectPointToNormalizedPlane(const MatrixBase<Derived1>& _v,
                              MatrixBase<Derived2>&       _up,
                              typename Derived1::Scalar&  _dist)
{
    MatrixSizeCheck<3, 1>::check(_v);
    MatrixSizeCheck<2, 1>::check(_up);

    projectPointToNormalizedPlane(_v, _up);
    _dist = _v.norm();
}

/**
 * Pin-hole canonical projection, with jacobian
 * \param _v the 3D point to project
 * \param _up the projected 2D point
 * \param _UP_v the Jacibian of \a u wrt \a v
 */
template<typename Derived1, typename Derived2, typename Derived3>
inline void
projectPointToNormalizedPlane(const MatrixBase<Derived1>& _v,
                              MatrixBase<Derived2>&       _up,
                              MatrixBase<Derived3>&       _UP_v)
{
    MatrixSizeCheck<3, 1>::check(_v);
    MatrixSizeCheck<2, 1>::check(_up);
    MatrixSizeCheck<2, 3>::check(_UP_v);

    projectPointToNormalizedPlane(_v, _up);

    _UP_v(0, 0) = 1.0 / _v(2);
    _UP_v(0, 1) = 0.0;
    _UP_v(0, 2) = -_v(0) / (_v(2) * _v(2));
    _UP_v(1, 0) = 0.0;
    _UP_v(1, 1) = 1.0 / _v(2);
    _UP_v(1, 2) = -_v(1) / (_v(2) * _v(2));
}

/**
 * Pin-hole canonical projection, with distance (not depth!) and jacobian
 * \param _v the 3D point to project
 * \param _up the projected 2D point
 * \param _UP_v the Jacibian of \a u wrt \a v
 */
template<typename Derived1, typename Derived2, typename Derived3>
inline void
projectPointToNormalizedPlane(const MatrixBase<Derived1>& _v,
                              MatrixBase<Derived2>&       _up,
                              typename Derived1::Scalar&  _dist,
                              MatrixBase<Derived3>&       _UP_v)
{
    MatrixSizeCheck<3, 1>::check(_v);
    MatrixSizeCheck<2, 1>::check(_up);
    MatrixSizeCheck<2, 3>::check(_UP_v);

    projectPointToNormalizedPlane(_v, _up, _UP_v);

    _dist = _v.norm();
}


/**
 * Canonical back-projection.
 * \param _u the 2D point in the image plane
 * \param _depth point's depth orthogonal to image plane. Defaults to 1.0
 * \return the back-projected 3D point at the given depth
 */
template<typename Derived>
Matrix<typename Derived::Scalar, 3, 1>
backprojectPointFromNormalizedPlane(const MatrixBase<Derived> &    _u,
                                    const typename Derived::Scalar _depth = 1)
{
    MatrixSizeCheck<2,1>::check(_u);

    Matrix<typename Derived::Scalar, 3, 1> p;

    p << _depth*_u ,
         _depth;

    return p;
}

/**
 * Canonical back-projection.
 * \param u the 2D point in the image plane.
 * \param depth point's depth orthogonal to image plane.
 * \param p the 3D point.
 * \param P_u Jacobian of p wrt u.
 * \param P_depth Jacobian of p wrt depth.
 */
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
void backprojectPointFromNormalizedPlane(const MatrixBase<Derived1> &    u,
                                         const typename Derived1::Scalar depth,
                                         MatrixBase<Derived2>&           p,
                                         MatrixBase<Derived3>&           P_u,
                                         MatrixBase<Derived4>&           P_depth )
{
    MatrixSizeCheck<2,1>::check(u);
    MatrixSizeCheck<3,1>::check(p);
    MatrixSizeCheck<3,2>::check(P_u);
    MatrixSizeCheck<3,1>::check(P_depth);

    p = backprojectPointFromNormalizedPlane(u, depth);

    P_u(0, 0) = depth;
    P_u(0, 1) = 0.0;
    P_u(1, 0) = 0.0;
    P_u(1, 1) = depth;
    P_u(2, 0) = 0.0;
    P_u(2, 1) = 0.0;

    P_depth(0, 0) = u(0);
    P_depth(1, 0) = u(1);
    P_depth(2, 0) = 1.0;
}

/**
 * Distortion factor for the model s = 1 + d_0 * r^2 + d_1 * r^4 + d_2 * r^6 + ...
 * \param d the distortion parameters vector
 * \param r2 the square of the radius to evaluate, r2 = r^2.
 * \return the distortion factor so that rd = s*r
 */
template<typename Derived, typename T>
T distortionFactor(const MatrixBase<Derived> &  d,
                   T                            r2)
{
    StaticSizeCheck<Derived::ColsAtCompileTime, 1>(d.cols());

    if (d.size() == 0)
        return (T)1.0;
    T s = (T)1.0;
    T r2i = (T)1.0;
    for (Size i = 0; i < d.size(); i++)
    {                           //   here we are doing:
        r2i = r2i * r2;         //      r2i = r^(2*(i+1))
        s  += d(i) * r2i;       //      s   = 1 + d_0 * r^2 + d_1 * r^4 + d_2 * r^6 + ...
    }
    /*
     The model is not valid out of the image, and it can bring back landmarks very quickly after they got out.
     We should compute the point where the monotony changes by solving d/du u.f(u,v) = 0, but it would require a lot
     more computations to check the validity of the transform than to compute the distortion... and it should be done
     manually for every size of d (fortunately only 2 or 3 usually).
     So we just make a rough test on s, no camera should have a distortion greater than this, and anyway it will just
     prevent from observing some points on the borders of the image. In that case the landmark will seem to suddenly
     jump outside of the image.
     */
    if (s < 0.6)
        s = (T)1.0;
    return s;
}

/**
 * Correction factor for the model s = 1 + c_0 * r^2 + c_1 * r^4 + c_2 * r^6 + ...
 * \param c the correction parameters vector
 * \param r2 the square of the radius to evaluate, r2 = r^2.
 * \return the correction factor so that rc = s*r
 */
template<typename Derived, typename T>
T correctionFactor(const MatrixBase<Derived> &  c,
                   T                            r2)
{
    StaticSizeCheck<Derived::ColsAtCompileTime, 1>(c.cols());

    /*
     * Since we use the same polynomial kernel as for the distortion factor, we just call distortionFactor()
     */
    return distortionFactor(c, r2);
}

/**
 * Radial distortion: ud = (1 + d_0 * r^2 + d_1 * r^4 + d_2 * r^6 + etc) * u
 * \param d the distortion parameters vector
 * \param up the point to distort
 * \return the distorted point
 */
template<typename Derived1, typename Derived2>
Matrix<typename Derived2::Scalar, 2, 1> distortPoint(const MatrixBase<Derived1> & d,
                                                     const MatrixBase<Derived2> & up)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(d.cols());
    MatrixSizeCheck<2,1>::check(up);

    Size n = d.size();
    if (n == 0)
        return up;
    else {
        typename Derived2::Scalar r2 = up(0) * up(0) + up(1) * up(1); // this is the norm squared: r2 = ||u||^2
        return distortionFactor(d, r2) * up;
    }
}


/**
 * Radial distortion: ud = (1 + d_0 * r^2 + d_1 * r^4 + d_2 * r^6 + etc) * u, with jacobians
 * \param d the radial distortion parameters vector
 * \param up the point to distort
 * \param ud the distorted point
 * \param UD_up the Jacobian of \a ud wrt \a up
 */
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
void distortPoint(const MatrixBase<Derived1> &  d,
                  const MatrixBase<Derived2> &  up,
                  MatrixBase<Derived3> &        ud,
                  MatrixBase<Derived4> &        UD_up)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(d.cols());
    MatrixSizeCheck<2,1>::check(up);
    MatrixSizeCheck<2,1>::check(ud);
    MatrixSizeCheck<2,2>::check(UD_up);

    typedef typename Derived2::Scalar T;

    Matrix<T, 2, 1> R2_up;
    Matrix<T, 2, 1> S_up;

    Size n = d.size();
    if (n == 0) {
        ud = up;
        UD_up.setIdentity();
    }

    else {
        T r2    = up(0) * up(0) + up(1) * up(1); // this is the norm squared: r2 = ||u||^2
        T s     = (T) 1.0;
        T r2i   = (T) 1.0;
        T r2im1 = (T) 1.0; //r2*(i-1)
        T S_r2  = (T) 0.0;

        for (Size i = 0; i < n; i++) { //.. here we are doing:
            r2i = r2i * r2; //................. r2i = r^(2*(i+1))
            s += d(i) * r2i; //................ s = 1 + d_0 * r^2 + d_1 * r^4 + d_2 * r^6 + ...

            S_r2 = S_r2 + (i + 1) * d(i) * r2im1; //jacobian of s wrt r2 : S_r2 = d_0 + 2 * d1 * r^2 + 3 * d_2 * r^4 +  ...
            r2im1 = r2im1 * r2;
        }

        if (s < (T)0.6) s = (T)1.0; // because the model is not valid too much out of the image, avoid to wrongly bring them back in the field of view
                                    // see extensive note in distortionFactor()
        ud = s * up; // finally ud = (1 + d_0 * r^2 + d_1 * r^4 + d_2 * r^6 + ...) * u;

        R2_up(0) = 2 * up(0);
        R2_up(1) = 2 * up(1);

        S_up(0) = R2_up(0) * S_r2;
        S_up(1) = R2_up(1) * S_r2;

        UD_up(0, 0) = S_up(0) * up(0) + s;
        UD_up(0, 1) = S_up(1) * up(0);
        UD_up(1, 0) = S_up(0) * up(1);
        UD_up(1, 1) = S_up(1) * up(1) + s;
    }

}

template<typename Derived1, typename Derived2>
Matrix<typename Derived2::Scalar, 2, 1> undistortPoint(const MatrixBase<Derived1>& c,
                                                       const MatrixBase<Derived2>& ud)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(c.cols());
    MatrixSizeCheck<2,1>::check(ud);

    Size n = c.size();
    if (n == 0)
        return ud;
    else {
        typename Derived2::Scalar r2 = ud(0) * ud(0) + ud(1) * ud(1); // this is the norm squared: r2 = ||u||^2
        return correctionFactor(c, r2) * ud;
    }
}

template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
void undistortPoint(const MatrixBase<Derived1>& c,
                    const MatrixBase<Derived2>& ud,
                    MatrixBase<Derived3>&       up,
                    MatrixBase<Derived4>&       UP_ud)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(c.cols());
    MatrixSizeCheck<2,1>::check(ud);
    MatrixSizeCheck<2,1>::check(up);
    MatrixSizeCheck<2,2>::check(UP_ud);

    typedef typename Derived1::Scalar T;

    Size n = c.size();
    Matrix<typename Derived4::Scalar, 2, 1> R2_ud;
    Matrix<typename Derived4::Scalar, 2, 1> S_ud;

    if (n == 0)
    {
        up = ud;
        UP_ud.setIdentity();
    }

    else
    {
        T r2   = ud(0) * ud(0) + ud(1) * ud(1); // this is the norm squared: r2 = ||u||^2
        T s    = (T)1.0;
        T r2i  = (T)1.0;
        T r2im1 = (T)1.0; //r2*(i-1)
        T S_r2 = (T)0.0;

        for (Size i = 0; i < n; i++)
        { //.. here we are doing:
            r2i = r2i * r2; //................. r2i = r^(2*(i+1))
            s += c(i) * r2i; //................ s = 1 + c_0 * r^2 + c_1 * r^4 + c_2 * r^6 + ...

            S_r2 = S_r2 + (i + 1) * c(i) * r2im1; //jacobian of s wrt r2 : S_r2 = c_0 + 2 * d1 * r^2 + 3 * c_2 * r^4 +  ...
            r2im1 = r2im1 * r2;
        }
        if (s < (T)0.6) s = (T)1.0; // because the model is not valid too much out of the image, avoid to wrongly bring them back in the field of view
                                    // see extensive note in distortionFactor()

        up = s * ud; // finally up = (1 + c_0 * r^2 + c_1 * r^4 + c_2 * r^6 + ...) * u;

        R2_ud(0) = 2 * ud(0);
        R2_ud(1) = 2 * ud(1);

        S_ud(0) = R2_ud(0) * S_r2;
        S_ud(1) = R2_ud(1) * S_r2;

        UP_ud(0, 0) = S_ud(0) * ud(0) + s;
        UP_ud(0, 1) = S_ud(1) * ud(0);
        UP_ud(1, 0) = S_ud(0) * ud(1);
        UP_ud(1, 1) = S_ud(1) * ud(1) + s;
    }
}


/**
 * Pixellization from k = [u_0, v_0, a_u, a_v]
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param ud the point to pixellize, adimensional
 * \return the point in pixels coordinates
 */
template<typename Derived1, typename Derived2>
Matrix<typename Derived2::Scalar, 2, 1> pixellizePoint(const MatrixBase<Derived1>& k,
                                                       const MatrixBase<Derived2>& ud)
{
    MatrixSizeCheck<4,1>::check(k);
    MatrixSizeCheck<2,1>::check(ud);

    typedef typename Derived2::Scalar T;

    T u_0 = k(0);
    T v_0 = k(1);
    T a_u = k(2);
    T a_v = k(3);
    Matrix<T, 2, 1> u;

    u(0) = u_0 + a_u * ud(0);
    u(1) = v_0 + a_v * ud(1);

    return u;
}



/**
 * Pixellization from k = [u_0, v_0, a_u, a_v] with jacobians
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param ud the point to pixellize, adimensional
 * \param u the pixellized point
 * \param U_ud the Jacobian of \a u wrt \a ud
 */
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
void pixellizePoint(const MatrixBase<Derived1>& k,
                    const MatrixBase<Derived2>& ud,
                    MatrixBase<Derived3>&       u,
                    MatrixBase<Derived4>&       U_ud)
{
    MatrixSizeCheck<4,1>::check(k);
    MatrixSizeCheck<2,1>::check(ud);
    MatrixSizeCheck<2,1>::check(u);
    MatrixSizeCheck<2,2>::check(U_ud);

    typedef typename Derived1::Scalar T;

    u = pixellizePoint(k, ud);

    T a_u = k(2);
    T a_v = k(3);

    U_ud(0, 0) = a_u;
    U_ud(0, 1) = 0;
    U_ud(1, 0) = 0;
    U_ud(1, 1) = a_v;
}


/**
 * Depixellization from k = [u_0, v_0, a_u, a_v]
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param u the point to depixellize, in pixels
 * \return the depixellized point, adimensional
 */
template<typename Derived1, typename Derived2>
Matrix<typename Derived2::Scalar, 2, 1> depixellizePoint(const MatrixBase<Derived1>& k,
                                                         const MatrixBase<Derived2>& u)
{
    MatrixSizeCheck<4,1>::check(k);
    MatrixSizeCheck<2,1>::check(u);

    typedef typename Derived1::Scalar T;

    T u_0 = k(0);
    T v_0 = k(1);
    T a_u = k(2);
    T a_v = k(3);
    Matrix<typename Derived2::Scalar, 2, 1> ud;

    ud(0) = (u(0) - u_0) / a_u;
    ud(1) = (u(1) - v_0) / a_v;

    return ud;
}


/**
 * Depixellization from k = [u_0, v_0, a_u, a_v], with Jacobians
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param u the point to depixellize, in pixels
 * \param ud the depixellized point
 * \param UD_u the Jacobian of \a ud wrt \a u
 */
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
void depixellizePoint(const MatrixBase<Derived1>&   k,
                      const MatrixBase<Derived2>&   u,
                      MatrixBase<Derived3>&         ud,
                      MatrixBase<Derived4>&         UD_u)
{
    MatrixSizeCheck<4,1>::check(k);
    MatrixSizeCheck<2,1>::check(u);
    MatrixSizeCheck<2,1>::check(ud);
    MatrixSizeCheck<2,2>::check(UD_u);

    typedef typename Derived1::Scalar T;
    ud = depixellizePoint(k, u);

    T a_u = k(2);
    T a_v = k(3);

    UD_u(0, 0) = 1.0 / a_u;
    UD_u(0, 1) = 0.0;
    UD_u(1, 0) = 0.0;
    UD_u(1, 1) = 1.0 / a_v;
}


/**
 * Project a point into a pin-hole camera with radial distortion
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param d the radial distortion parameters vector
 * \param v the 3D point to project, or the 3D director vector
 * \return the projected and distorted point
 */
template<typename Derived1, typename Derived2, typename Derived3>
Matrix<typename Derived3::Scalar, 2, 1> projectPoint(const MatrixBase<Derived1>& k,
                                                     const MatrixBase<Derived2>& d,
                                                     const MatrixBase<Derived3>& v)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(k.cols());
    StaticSizeCheck<Derived2::ColsAtCompileTime, 1>(d.cols());
    MatrixSizeCheck<3,1>::check(v);

    return pixellizePoint( k, distortPoint( d, projectPointToNormalizedPlane( v )));
}

/**
 * Project a point into a pin-hole camera with radial distortion.
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param d the radial distortion parameters vector
 * \param v the 3D point to project, or the 3D director vector
 * \param u the projected and distorted point
 * \param dist distance from the optical center to the 3D point
 */
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename T>
void projectPoint(const MatrixBase<Derived1>& k,
                  const MatrixBase<Derived2>& d,
                  const MatrixBase<Derived3>& v,
                  MatrixBase<Derived4>&       u,
                  T&                          dist)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(k.cols());
    StaticSizeCheck<Derived2::ColsAtCompileTime, 1>(d.cols());
    MatrixSizeCheck<3,1>::check(v);
    MatrixSizeCheck<2,1>::check(u);

    Matrix<typename Derived4::Scalar, 2, 1> up;
    projectPointToNormalizedPlane(v, up, dist);

    u = pixellizePoint( k, distortPoint( d, up ));
}



/**
 * Project a point into a pin-hole camera with radial distortion
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param d the radial distortion parameters vector
 * \param v the 3D point to project, or the 3D director vector
 * \param u the projected and distorted point
 * \param U_v the Jacobian of \a u wrt \a v
 */
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
void projectPoint(const MatrixBase<Derived1>& k,
                  const MatrixBase<Derived2>& d,
                  const MatrixBase<Derived3>& v,
                  MatrixBase<Derived4>&       u,
                  MatrixBase<Derived5>&       U_v)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(k.cols());
    StaticSizeCheck<Derived2::ColsAtCompileTime, 1>(d.cols());
    MatrixSizeCheck<3,1>::check(v);
    MatrixSizeCheck<2,1>::check(u);
    MatrixSizeCheck<2,3>::check(U_v);

    Matrix<typename Derived4::Scalar, 2, 1> up, ud;
    Matrix<typename Derived5::Scalar, 2, 3> UP_v; /// Check this one -> mat23
    Matrix<typename Derived5::Scalar, 2, 2> UD_up, U_ud;

    projectPointToNormalizedPlane   (v, up,     UP_v);
    distortPoint                    (d, up, ud, UD_up);
    pixellizePoint                  (k, ud, u,  U_ud);

    U_v.noalias() = U_ud * UD_up * UP_v;
}

/**
 * Project a point into a pin-hole camera with radial distortion
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param d the radial distortion parameters vector
 * \param v the 3D point to project, or the 3D director vector
 * \param u the projected and distorted point
 * \param dist the distance from the camera to the point
 * \param U_v the Jacobian of \a u wrt \a v
 */
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename T, typename Derived5>
void projectPoint(const MatrixBase<Derived1>& k,
                  const MatrixBase<Derived2>& d,
                  const MatrixBase<Derived3>& v,
                  MatrixBase<Derived4>&       u,
                  T&                          dist,
                  MatrixBase<Derived5>&       U_v)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(k.cols());
    StaticSizeCheck<Derived2::ColsAtCompileTime, 1>(d.cols());
    MatrixSizeCheck<3,1>::check(v);
    MatrixSizeCheck<2,1>::check(u);
    MatrixSizeCheck<2,3>::check(U_v);

    Matrix<typename Derived4::Scalar, 2, 1> up, ud;
    Matrix<typename Derived5::Scalar, 2, 3> UP_v;
    Matrix<typename Derived5::Scalar, 2, 2> UD_up, U_ud;

    projectPointToNormalizedPlane   (v, up, dist, UP_v);
    distortPoint                    (d, up, ud,   UD_up);
    pixellizePoint                  (k, ud, u,    U_ud);

    U_v.noalias() = U_ud * UD_up * UP_v;
}


/**
 * Back-Project a point from a pin-hole camera with radial distortion
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param c the radial undistortion parameters vector
 * \param u the 2D pixel
 * \param depth the depth prior
 * \return the back-projected 3D point
 */
template<typename Derived1, typename Derived2, typename Derived3, typename T>
Matrix<typename Derived3::Scalar, 3, 1> backprojectPoint(const MatrixBase<Derived1>& k,
                                                     const MatrixBase<Derived2>&     c,
                                                     const MatrixBase<Derived3>&     u,
                                                     const T&                        depth = 1.0)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(k.cols());
    StaticSizeCheck<Derived2::ColsAtCompileTime, 1>(c.cols());
    MatrixSizeCheck<2,1>::check(u);

    return backprojectPointFromNormalizedPlane(undistortPoint(c, depixellizePoint(k, u)), depth);
}

/**
 * Back-Project a point from a pin-hole camera with radial distortion; give Jacobians
 * \param k the vector of intrinsic parameters, k = [u0, v0, au, av]
 * \param c the radial undistortion parameters vector
 * \param u the 2D pixel
 * \param depth the depth prior
 * \param p the back-projected 3D point
 * \param P_u Jacobian of p wrt u
 * \param P_depth Jacobian of p wrt depth
 */
template<typename Derived1, typename Derived2, typename Derived3, typename T, typename Derived4, typename Derived5, typename Derived6>
void backprojectPoint(const MatrixBase<Derived1>& k,
                      const MatrixBase<Derived2>& c,
                      const MatrixBase<Derived3>& u,
                      const T&                    depth,
                      MatrixBase<Derived4>&       p,
                      MatrixBase<Derived5>&       P_u,
                      MatrixBase<Derived6>&       P_depth)
{
    StaticSizeCheck<Derived1::ColsAtCompileTime, 1>(k.cols());
    StaticSizeCheck<Derived2::ColsAtCompileTime, 1>(c.cols());
    MatrixSizeCheck<2,1>::check(u);
    MatrixSizeCheck<3,2>::check(P_u);

    Matrix<typename Derived3::Scalar, 2, 1> up, ud;
    Matrix<typename Derived5::Scalar, 3, 2> P_up;
    Matrix<typename Derived5::Scalar, 2, 2> UP_ud, UD_u;
    depixellizePoint(k, u, ud, UD_u);
    undistortPoint(c, ud, up, UP_ud);
    backprojectPointFromNormalizedPlane(up, depth, p, P_up, P_depth);

    P_u.noalias() = P_up * UP_ud * UD_u;
}


/**
 * Determine if a pixel is inside the region of interest
 * \param pix the pixel to test
 * \param x the region of interest, top-left x
 * \param y the region of interest, top-left y
 * \param width the region of interest width
 * \param height the region of interest height
 */
template<typename VPix>
bool isInRoi(const MatrixBase<VPix> & pix, const int x, const int y, const int width, const int height) {
    return ((pix(0) >= x) && (pix(0) <= x + width - 1) && (pix(1) >= y) && (pix(1) <= y + height - 1));
}

/**
 * Determine if a pixel is inside the image
 * \param pix the pixel to test
 * \param width the image width, in pixels
 * \param height the image height, in pixels
 */
template<typename VPix>
bool isInImage(const MatrixBase<VPix> & pix, const int & width, const int & height) {
    return isInRoi(pix, 0, 0, width, height);
}


/**
 * Compute distortion correction parameters.
 *
 * This method follows the one in Joan Sola's thesis [1], pag 46--49.
 * \param k the intrinsic parameters vector
 * \param d the distortion parameters vector
 * \param c the correction parameters vector. Provide it with the desired size.
 */
template<class Vk, class Vd, class Vc>
void computeCorrectionModel(const Vk & k, const Vd & d, Vc & c)

{
    Size size = c.size();

    if (size != 0)
    {

        Scalar r_max = sqrt(k(0) * k(0) / (k(2) * k(2)) + k(1) * k(1) / (k(3) * k(3)));
        Scalar rd_max = 1.2 * r_max;

        Size N_samples = 200; // number of samples
        Scalar iN_samples = 1 / (Scalar)N_samples;
        Scalar rd_n, rc_2, rd_2;
        Eigen::VectorXs rd(N_samples + 1), rc(N_samples + 1);
        Eigen::MatrixXs Rd(N_samples + 1, size);

        for (Size sample = 0; sample <= N_samples; sample++)
        {

            rc(sample) = sample * rd_max * iN_samples; // sample * rd_max / N_samples
            rc_2 = rc(sample) * rc(sample);
            rd(sample) = distortionFactor(d, rc_2) * rc(sample);
            rd_2 = rd(sample) * rd(sample);

            rd_n = rd(sample); // start with rd

            for (Size order = 0; order < size; order++)
            {
                rd_n *= rd_2; // increment:
                Rd(sample, order) = rd_n; // we have : rd^3, rd^5, rd^7, ...
            }
        }

        // solve Rd*c = (rc-rd) for c, with least-squares SVD method:
        // the result is c = pseudo_inv(Rd)*(rc-rd)
        //  with pseudo_inv(Rd) = (Rd'*Rd)^-1 * Rd'

        // this does not work:
        // jmath::LinearSolvers::solve_Cholesky(Rd, (rc - rd), c);


        // therefore we solve manually the pseudo-inverse:
        Eigen::MatrixXs RdtRd(size, size);
        RdtRd = Rd.transpose() * Rd;
        Eigen::MatrixXs iRdtRd(size, size);
        //jmath::ublasExtra::inv(RdtRd, iRdtRd);
        // I understood that iRdtRd is the inverse of RdtRd)
        iRdtRd = RdtRd.inverse();
        Eigen::MatrixXs iRd = iRdtRd * Rd.transpose();

        c = iRd * (rc - rd);
    }
}

} // namespace pinhole



} // namespace wolf


#endif // PINHOLETOOLS_H
