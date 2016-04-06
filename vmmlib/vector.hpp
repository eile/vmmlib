/*
 * Copyright (c) 2006-2016, Visualization and Multimedia Lab,
 *                          University of Zurich <http://vmml.ifi.uzh.ch>,
 *                          Eyescale Software GmbH,
 *                          Blue Brain Project, EPFL
 *
 * This file is part of VMMLib <https://github.com/VMML/vmmlib/>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.  Redistributions in binary
 * form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided
 * with the distribution.  Neither the name of the Visualization and Multimedia
 * Lab, University of Zurich nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __VMML__VECTOR__HPP__
#define __VMML__VECTOR__HPP__

#include <vmmlib/enable_if.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <stdint.h>
#include <string>
#include <vector>

namespace vmml
{
template< size_t M, typename T > class Vector
{
public:
    // constructors
    Vector() : array() {} // http://stackoverflow.com/questions/5602030
    explicit Vector( const T& a ); // sets all components to a;
    template< typename U > Vector( U x, U y,
                                   typename enable_if< M == 2, U >::type* = 0 );
    template< typename U > Vector( U x, U y, U z,
                                   typename enable_if< M == 3, U >::type* = 0 );
    template< typename U > Vector( U x, U y, U z, U w,
                                   typename enable_if< M == 4, U >::type* = 0 );
    Vector( const T* values );

#ifdef __OSG_MATH
    template< typename OSGVEC3 >
    Vector( const OSGVEC3& from,
            typename enable_if< M == 3, OSGVEC3 >::type* = 0 );
#endif

    // vec< M > with homogeneous coordinates <-> vec< M-1 > conversion ctor
    // to-homogenous-coordinates ctor
    template< size_t N >
    Vector( const Vector< N, T >& source_,
            typename enable_if< N == M - 1 >::type* = 0 );

    // from-homogenous-coordinates vector
    template< size_t N >
    Vector( const Vector< N, T >& source_,
            typename enable_if< N == M + 1 >::type* = 0  );

    template< typename U > Vector( const Vector< M, U >& source_ );

    inline T& operator[]( size_t index );
    inline const T& operator[]( size_t index ) const;

    // accessors
    inline T& operator()( size_t index );
    inline const T& operator()( size_t index ) const;

    inline T& at( size_t index );
    inline const T& at( size_t index ) const;

    // element accessors for M <= 4;
    inline T& x();
    inline T& y();
    inline T& z();
    inline T& w();
    inline const T& x() const;
    inline const T& y() const;
    inline const T& z() const;
    inline const T& w() const;

    // pixel color element accessors for M<= 4
    inline T& r();
    inline T& g();
    inline T& b();
    inline T& a();
    inline const T& r() const;
    inline const T& g() const;
    inline const T& b() const;
    inline const T& a() const;

    bool operator==( const Vector& other ) const;
    bool operator!=( const Vector& other ) const;
    bool equals( const Vector& other,
                 T tolerance = std::numeric_limits< T >::epsilon( )) const;
    bool operator<( const Vector& other ) const;

    Vector& operator=( const Vector& other );

    // to-homogenous-coordinates assignment operator
    // non-chainable because of sfinae
    template< size_t N >
    typename enable_if< N == M - 1 >::type*
        operator=( const Vector< N, T >& source_ );

    // from-homogenous-coordinates assignment operator
    // non-chainable because of sfinae
    template< size_t N >
    typename enable_if< N == M + 1 >::type*
        operator=( const Vector< N, T >& source_ );

    Vector operator*( const Vector& other ) const;
    Vector operator/( const Vector& other ) const;
    Vector operator+( const Vector& other ) const;
    Vector operator-( const Vector& other ) const;

    void operator*=( const Vector& other );
    void operator/=( const Vector& other );
    void operator+=( const Vector& other );
    void operator-=( const Vector& other );

    Vector operator*( const T other ) const;
    Vector operator/( const T other ) const;
    Vector operator+( const T other ) const;
    Vector operator-( const T other ) const;

    void operator*=( const T other );
    void operator/=( const T other );
    void operator+=( const T other );
    void operator-=( const T other );

    Vector operator-() const;

    // compute the cross product of two Vectors
    // note: there's also a free function:
    // Vector<> cross( const Vector<>, const Vector<> )
    template< typename TT >
    Vector< M, T >& cross( const Vector< M, TT >& b,
                           typename enable_if< M == 3, TT >::type* = 0 );

    // compute the dot product of two Vectors
    // note: there's also a free function:
    // T dot( const Vector<>, const Vector<> );
    inline T dot( const Vector& other ) const;

    // normalize the Vector
    // note: there's also a free function:
    // Vector<> normalize( const Vector<> );
    inline T normalize();

    inline T length() const;
    inline T lengthSquared() const;
    inline T distance( const Vector& other ) const;
    inline T distanceSquared( const Vector& other ) const;

    /** @return the product of all elements of this Vector */
    T product() const;

    template< typename TT >
    Vector< 3, T >& rotate( T theta, Vector< M, TT > axis,
                            typename enable_if< M == 3, TT >::type* = 0 );

    /** @return the sub Vector of the given length at the given offset. */
    template< size_t N, size_t O = 0 >
    Vector< N, T > getSubVector( typename enable_if< M >= N+O >::type* = 0 )
        const;

    /** Set the sub Vector of the given length at the given offset. */
    template< size_t N, size_t O = 0 >
    void setSubVector( const Vector< N, T >& sub,
                         typename enable_if< M >= N+O >::type* = 0 );

    // sphere functions - sphere layout: center xyz, radius w
    template< typename TT >
    inline Vector< 3, T > project_point_onto_sphere(
        const Vector< 3, TT >& point,
        typename enable_if< M == 4, TT >::type* = 0 ) const;

    // returns a negative distance if the point lies in the sphere
    template< typename TT >
    inline T distance_to_sphere( const Vector< 3, TT >& point,
        typename enable_if< M == 4, TT >::type* = 0 ) const;

    // plane functions - plane layout; normal xyz, distance w
    template< typename TT >
    inline T distance_to_plane( const Vector< 3, TT >& point,
        typename enable_if< M == 4, TT >::type* = 0 ) const;

    template< typename TT >
    inline Vector< 3, T > project_point_onto_plane(
        const Vector< 3, TT >& point,
        typename enable_if< M == 4, TT >::type* = 0 ) const;

    // test each component of the Vector for isnan and isinf
    //  inline bool is_valid() const; -> moved to class validator

    friend std::ostream& operator<< ( std::ostream& os, const Vector& Vector_ )
    {
        const std::ios::fmtflags flags = os.flags();
        const int                prec  = os.precision();

        os.setf( std::ios::right, std::ios::adjustfield );
        os.precision( 5 );
        os << "[ ";
        for( size_t index = 0; index < M; ++index )
            os << std::setw(10) << Vector_.at( index ) << " ";
        os << "]";
        os.precision( prec );
        os.setf( flags );
        return os;
    }

    T array[ M ];    //!< storage
}; // class Vector

/** @name Vector3 unit vectors */
//@{
namespace Vector3
{
template< class T > Vector< 3, T > forward()
    { return Vector< 3, T >( 0, 0, -1 ); }

template< class T > Vector< 3, T > backward()
   { return Vector< 3, T >( 0, 0, 1 ); }

template< class T > Vector< 3, T > up()
    { return Vector< 3, T >( 0, 1, 0 ); }

template< class T > Vector< 3, T > down()
    { return Vector< 3, T >( 0, -1, 0 ); }

template< class T > Vector< 3, T > left()
    { return Vector< 3, T >( -1, 0, 0 ); }

template< class T > Vector< 3, T > right()
    { return Vector< 3, T >( 1, 0, 0 ); }

template< class T > Vector< 3, T > x() { return Vector< 3, T >( 1, 0, 0 ); }
template< class T > Vector< 3, T > y() { return Vector< 3, T >( 0, 1, 0 ); }
template< class T > Vector< 3, T > z() { return Vector< 3, T >( 0, 0, 1 ); }
}
//@}

// allows float * vector, not only vector * float
template< size_t M, typename T >
static Vector< M, T > operator* ( T factor, const Vector< M, T >& Vector_ )
{
    return Vector_ * factor;
}

template< size_t M, typename T >
inline T dot( const Vector< M, T >& first, const Vector< M, T >& second )
{
    return first.dot( second );
}

template< size_t M, typename T >
inline Vector< M, T > cross( Vector< M, T > a, const Vector< M, T >& b )
{
    return a.cross( b );
}

template< size_t M, typename T >
Vector< M, T > compute_normal( const Vector< M, T >& a, const Vector< M, T >& b,
                               const Vector< M, T >& c )
{
    // right hand system, CCW triangle
    const Vector< M, T > u = b - a;
    const Vector< M, T > v = c - a;
    Vector< M, T > w = cross( u, v );
    w.normalize();
    return w;
}

template< typename T >
Vector< 3, T > rotate( Vector< 3, T > vec, const T theta,
                       const Vector< 3, T >& axis )
{
    return vec.rotate( theta, axis );
}


template< size_t M, typename T >
inline Vector< M, T > normalize( Vector< M, T > Vector_ )
{
    Vector_.normalize();
    return Vector_;
}

template< typename T >
inline Vector< 4, T > compute_plane( const Vector< 3, T >& a,
                                     const Vector< 3, T >& b,
                                     const Vector< 3, T >& c )
{
    const Vector< 3, T > cb = b - c;
    const Vector< 3, T > ba = a - b;

    Vector< 4, T > plane = Vector< 4, T >( cross( cb, ba ));
    plane.normalize();
    plane.w() = -plane.x() * a.x() - plane.y() * a.y() - plane.z() * a.z();
    return plane;
}
//@}

template< size_t M, typename T > Vector< M, T >::Vector( const T& value )
{
    for( size_t i = 0; i < M; ++i )
        array[ i ] = value;
}

template< size_t M, typename T > template< typename U >
Vector< M, T >::Vector( const U _x, const U _y,
                        typename enable_if< M == 2, U >::type* )
{
    array[ 0 ] = T( _x );
    array[ 1 ] = T( _y );
}

template< size_t M, typename T > template< typename U >
Vector< M, T >::Vector( const U _x, const U _y, const U _z,
                        typename enable_if< M == 3, U >::type* )
{
    array[ 0 ] = T( _x );
    array[ 1 ] = T( _y );
    array[ 2 ] = T( _z );
}

template< size_t M, typename T > template< typename U >
Vector< M, T >::Vector( const U _x, const U _y, const U _z, const U _w,
                        typename enable_if< M == 4, U >::type* )
{
    array[ 0 ] = T( _x );
    array[ 1 ] = T( _y );
    array[ 2 ] = T( _z );
    array[ 3 ] = T( _w );
}

template< size_t M, typename T >
Vector< M, T >::Vector( const T* values )
{
    memcpy( array, values, M * sizeof( T ));
}

#ifdef __OSG_MATH
template< size_t M, typename T >
template< typename OSGVEC3 >
Vector< M, T >::Vector( const OSGVEC3& from,
                        typename enable_if< M == 3, OSGVEC3 >::type* )
{
    array[ 0 ] = from.x();
    array[ 1 ] = from.y();
    array[ 2 ] = from.z();
}
#endif

// to-homogenous-coordinates ctor
template< size_t M, typename T > template< size_t N >
Vector< M, T >::Vector( const Vector< N, T >& source_,
                        typename enable_if< N == M - 1 >::type* )
{
    (*this) = source_;
}

// from-homogenous-coordinates ctor
template< size_t M, typename T > template< size_t N >
Vector< M, T >::Vector( const Vector< N, T >& source_,
                        typename enable_if< N == M + 1 >::type* )
{
    (*this) = source_;
}

template< size_t M, typename T > template< typename U >
Vector< M, T >::Vector( const Vector< M, U >& source_ )
{
    (*this) = source_;
}

template< size_t M, typename T > inline
T& Vector< M, T >::operator()( size_t index )
{
    return at( index );
}

template< size_t M, typename T > inline
const T& Vector< M, T >::operator()( size_t index ) const
{
    return at( index );
}

template< size_t M, typename T > inline
T& Vector< M, T >::at( size_t index )
{
    if( index >= M )
        throw std::runtime_error( "at() - index out of bounds" );
    return array[ index ];
}

template< size_t M, typename T > inline
const T& Vector< M, T >::at( size_t index ) const
{
    if ( index >= M )
        throw std::runtime_error( "at() - index out of bounds" );
    return array[ index ];
}

template< size_t M, typename T >
T& Vector< M, T >::operator[]( size_t index )
{
    return at( index );
}

template< size_t M, typename T >
const T& Vector< M, T >::operator[]( size_t index ) const
{
    return at( index );
}

template< size_t M, typename T >
Vector< M, T > Vector< M, T >::operator*( const Vector< M, T >& other ) const
{
    Vector< M, T > result;
    for( size_t index = 0; index < M; ++index )
        result.at( index ) = at( index ) * other.at( index );
    return result;
}

template< size_t M, typename T >
Vector< M, T > Vector< M, T >::operator/( const Vector< M, T >& other ) const
{
    Vector< M, T > result;
    for( size_t index = 0; index < M; ++index )
        result.at( index ) = at( index ) / other.at( index );
    return result;
}

template< size_t M, typename T >
Vector< M, T > Vector< M, T >::operator+( const Vector< M, T >& other ) const
{
    Vector< M, T > result;
    for( size_t index = 0; index < M; ++index )
        result.at( index ) = at( index ) + other.at( index );
    return result;
}

template< size_t M, typename T >
Vector< M, T > Vector< M, T >::operator-( const Vector< M, T >& other ) const
{
    Vector< M, T > result;
    for( size_t index = 0; index < M; ++index )
        result.at( index ) = at( index ) - other.at( index );
    return result;
}

template< size_t M, typename T >
void Vector< M, T >::operator*=( const Vector< M, T >& other )
{
    for( size_t index = 0; index < M; ++index )
        at( index ) *= other.at( index );
}

template< size_t M, typename T >
void Vector< M, T >::operator/=( const Vector< M, T >& other )
{
    for( size_t index = 0; index < M; ++index )
        at( index ) /= other.at( index );
}

template< size_t M, typename T >
void Vector< M, T >::operator+=( const Vector< M, T >& other )
{
    for( size_t index = 0; index < M; ++index )
        at( index ) += other.at( index );
}

template< size_t M, typename T >
void
Vector< M, T >::operator-=( const Vector< M, T >& other )
{
    for( size_t index = 0; index < M; ++index )
        at( index ) -= other.at( index );
}

template< size_t M, typename T >
Vector< M, T >
Vector< M, T >::operator*( const T other ) const
{
    Vector< M, T > result;
    for( size_t index = 0; index < M; ++index )
        result.at( index ) = at( index ) * other;
    return result;
}

template< size_t M, typename T >
Vector< M, T >
Vector< M, T >::operator/( const T other ) const
{
    Vector< M, T > result;
    for( size_t index = 0; index < M; ++index )
        result.at( index ) = at( index ) / other;
    return result;
}

template< size_t M, typename T >
Vector< M, T >
Vector< M, T >::operator+( const T other ) const
{
    Vector< M, T > result;
    for( size_t index = 0; index < M; ++index )
        result.at( index ) = at( index ) + other;
    return result;
}

template< size_t M, typename T >
Vector< M, T >
Vector< M, T >::operator-( const T other ) const
{
    Vector< M, T > result;
    for( size_t index = 0; index < M; ++index )
        result.at( index ) = at( index ) - other;
    return result;
}

template< size_t M, typename T >
void
Vector< M, T >::operator*=( const T other )
{
    for( size_t index = 0; index < M; ++index )
        at( index ) *= other;
}

template< size_t M, typename T >
void
Vector< M, T >::operator/=( const T other )
{
    for( size_t index = 0; index < M; ++index )
        at( index ) /= other;
}

template< size_t M, typename T >
void
Vector< M, T >::operator+=( const T other )
{
    for( size_t index = 0; index < M; ++index )
        at( index ) += other;
}

template< size_t M, typename T >
void
Vector< M, T >::operator-=( const T other )
{
    for( size_t index = 0; index < M; ++index )
        at( index ) -= other;
}

template< size_t M, typename T >
Vector< M, T > Vector< M, T >::operator-() const
{
    Vector< M, T > v( *this );
    for( size_t i = 0; i < M; ++i )
        v.array[ i ] = -array[ i ];
    return v;
}

template< size_t M, typename T > T& Vector< M, T >::x()
{
    return array[ 0 ];
}

template< size_t M, typename T > T& Vector< M, T >::y()
{
    if( M < 2 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 1 ];
}

template< size_t M, typename T > T& Vector< M, T >::z()
{
    if( M < 3 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 2 ];
}

template< size_t M, typename T > T& Vector< M, T >::w()
{
    if( M < 4 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 3 ];
}

template< size_t M, typename T > const T& Vector< M, T >::x() const
{
    return array[ 0 ];
}

template< size_t M, typename T > const T& Vector< M, T >::y() const
{
    if( M < 2 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 1 ];
}

template< size_t M, typename T > const T& Vector< M, T >::z() const
{
    if( M < 3 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 2 ];
}

template< size_t M, typename T > const T& Vector< M, T >::w() const
{
    if( M < 4 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 3 ];
}

template< size_t M, typename T > T& Vector< M, T >::r()
{
    return array[ 0 ];
}

template< size_t M, typename T > T& Vector< M, T >::g()
{
    if( M < 2 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 1 ];
}

template< size_t M, typename T > T& Vector< M, T >::b()
{
    if( M < 3 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 2 ];
}

template< size_t M, typename T > T& Vector< M, T >::a()
{
    if( M < 4 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 3 ];
}

template< size_t M, typename T > const T& Vector< M, T >::r() const
{
    return array[ 0 ];
}

template< size_t M, typename T > const T& Vector< M, T >::g() const
{
    if( M < 2 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 1 ];
}

template< size_t M, typename T > const T& Vector< M, T >::b() const
{
    if( M < 3 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 2 ];
}

template< size_t M, typename T > const T& Vector< M, T >::a() const
{
    if( M < 4 )
        throw std::runtime_error( "out of bounds read" );
    return array[ 3 ];
}

template< size_t M, typename T > template< typename TT >
Vector< M, T >& Vector< M, T >::cross( const Vector< M, TT >& rhs,
                                       typename enable_if< M == 3, TT >::type* )
{
    const T x_ = y() * rhs.z() - z() * rhs.y();
    const T y_ = z() * rhs.x() - x() * rhs.z();
    const T z_ = x() * rhs.y() - y() * rhs.x();
    x() = x_;
    y() = y_;
    z() = z_;
    return *this;
}

template< size_t M, typename T >
inline T Vector< M, T >::dot( const Vector< M, T >& other ) const
{
    T tmp = 0.0;
    for( size_t index = 0; index < M; ++index )
        tmp += at( index ) * other.at( index );

    return tmp;
}

template< size_t M, typename T > inline T Vector< M, T >::normalize()
{
    const T len = length();
    if ( len <= std::numeric_limits< T >::epsilon( ))
        return 0;

    const T tmp = 1.0 / len;
    (*this) *= tmp;
    return len;
}

template< size_t M, typename T >
inline T Vector< M, T >::length() const
{
    return std::sqrt( lengthSquared( ));
}

template< size_t M, typename T >
inline T Vector< M, T >::lengthSquared() const
{
    T length2 = T( 0 );
    for( size_t i = 0; i < M; ++i )
        length2 += array[i] * array[i];
    return length2;
}

template< size_t M, typename T >
inline T
Vector< M, T >::distance( const Vector< M, T >& other ) const
{
    return std::sqrt( distanceSquared( other ) );
}

template< size_t M, typename T >
inline T Vector< M, T >::distanceSquared( const Vector< M, T >& other ) const
{
    Vector< M, T > tmp( *this );
    tmp -= other;
    return tmp.lengthSquared();
}

template< size_t M, typename T > inline T Vector< M, T >::product() const
{
    T result = at( 0 );
    for( size_t i = 1; i < M; ++i )
        result *= at( i );
    return result;
}

template< size_t M, typename T > template< typename TT >
Vector< 3, T >& Vector< M, T >::rotate( const T theta, Vector< M, TT > axis,
                                        typename enable_if< M==3, TT >::type* )
{
    const T costheta = std::cos( theta );
    const T sintheta = std::sin( theta );

    axis.normalize();
    return *this = Vector< 3, T >(
        (costheta + ( 1 - costheta ) * axis.x() * axis.x() ) * x()    +
        (( 1 - costheta ) * axis.x() * axis.y() - axis.z() * sintheta ) * y() +
        (( 1 - costheta ) * axis.x() * axis.z() + axis.y() * sintheta ) * z(),

        (( 1 - costheta ) * axis.x() * axis.y() + axis.z() * sintheta ) * x() +
        ( costheta + ( 1 - costheta ) * axis.y() * axis.y() ) * y() +
        (( 1 - costheta ) * axis.y() * axis.z() - axis.x() * sintheta ) * z(),

        (( 1 - costheta ) * axis.x() * axis.z() - axis.y() * sintheta ) * x() +
        (( 1 - costheta ) * axis.y() * axis.z() + axis.x() * sintheta ) * y() +
        ( costheta + ( 1 - costheta ) * axis.z() * axis.z() ) * z( ));
}

// sphere layout: center xyz, radius w
template< size_t M, typename T > template< typename TT > inline Vector< 3, T >
Vector< M, T >::project_point_onto_sphere( const Vector< 3, TT >& point,
    typename enable_if< M == 4, TT >::type* ) const
{
    const Vector< 3, T >& center_ = getSubVector< 3 >();

    Vector< 3, T > projected_point( point );
    projected_point -= center_;
    projected_point.normalize();
    projected_point *= w();
    return center_ + projected_point;
}

// sphere layout: center xyz, radius w
template< size_t M, typename T > template< typename TT > inline T
Vector< M, T >::distance_to_sphere( const Vector< 3, TT >& point,
                                    typename enable_if< M == 4, TT >::type* )
    const
{
    const Vector< 3, T >& center_ = getSubVector< 3 >();
    return ( point - center_ ).length() - w();
}

template< size_t M, typename T > template< size_t N, size_t O >
Vector< N, T > Vector< M, T >::getSubVector(
    typename enable_if< M >= N+O >::type* ) const
{
    return Vector< N, T >( array + O );
}

template< size_t M, typename T > template< size_t N, size_t O >
void Vector< M, T >::setSubVector( const Vector< N, T >& sub,
                                     typename enable_if< M >= N+O >::type* )
{
    ::memcpy( array + O, sub.array, N * sizeof( T ));
}

// plane: normal xyz, distance w
template< size_t M, typename T > template< typename TT >
inline T Vector< M, T >::distance_to_plane( const Vector< 3, TT >& point,
    typename enable_if< M == 4, TT >::type* ) const
{
    const Vector< 3, T >& normal = getSubVector< 3 >();
    return normal.dot( point ) + w();
}

// plane: normal xyz, distance w
template< size_t M, typename T > template< typename TT > Vector< 3, T >
Vector< M, T >::project_point_onto_plane( const Vector< 3, TT >& point,
    typename enable_if< M == 4, TT >::type* ) const
{
    const Vector< 3, T >& normal = getSubVector< 3 >();
    return point - ( normal * distance_to_plane( point ) );
}

template< size_t M, typename T >
bool Vector< M, T >::operator==( const Vector< M, T >& other ) const
{
    return memcmp( array, other.array, sizeof( array )) == 0;
}

template< size_t M, typename T >
bool Vector< M, T >::operator!=( const Vector< M, T >& other ) const
{
    return ! this->operator==( other );
}

template< size_t M, typename T >
bool Vector< M, T >::equals( const Vector< M, T >& other, T tolerance ) const
{
    for( size_t index = 0; index < M; ++index )
        if( fabs( at( index ) - other( index ) ) >= tolerance )
            return false;
    return true;

}

template< size_t M, typename T >
bool Vector< M, T >::operator<( const Vector< M, T >& other ) const
{
    for(size_t index = 0; index < M; ++index )
    {
        if (at( index ) < other.at( index )) return true;
        if (other.at( index ) < at( index )) return false;
    }
    return false;
}

// to-homogenous-coordinates assignment operator
// non-chainable because of sfinae
template< size_t M, typename T > template< size_t N >
typename enable_if< N == M - 1 >::type*
Vector< M, T >::operator=( const Vector< N, T >& source )
{
    ::memcpy( array, source.array, N * sizeof( T ));
    at( M - 1 ) = T( 1 );
    return 0;
}

// from-homogenous-coordinates assignment operator
// non-chainable because of sfinae
template< size_t M, typename T > template< size_t N >
typename enable_if< N == M + 1 >::type*
Vector< M, T >::operator=( const Vector< N, T >& source )
{
    const T wInv = T( 1 ) / source( M );
    for( size_t i = 0; i < M; ++i )
        array[i] = source( i ) * wInv;
    return 0;
}

template< size_t M, typename T >
Vector< M, T >& Vector< M, T >::operator=( const Vector< M, T >& other )
{
    if( this != &other )
        ::memcpy( array, other.array, M * sizeof( T ) );
    return *this;
}

} // namespace vmml

#endif
