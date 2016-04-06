
/* Copyright (c) 2014-2016, Stefan.Eilemann@epfl.ch
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Eyescale Software GmbH nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <vmmlib/vector.hpp>
#include <vmmlib/types.hpp>

#define BOOST_TEST_MODULE vector
#include <boost/test/unit_test.hpp>

using namespace vmml;

BOOST_AUTO_TEST_CASE(base)
{
    Vector< 4, double > v( 1, 2, 3, 4 );

    // tests copyFrom1DimCArray function
    size_t tmp = 1;
    for( size_t index = 0; index < 4; ++index, ++tmp )
    {
        BOOST_CHECK(v.at( index ) == tmp);
    }

    tmp = 4;
    v = Vector< 4, double >( 4, 3, 2, 1 );
    for( size_t index = 0; index < 4; ++index, --tmp )
    {
        BOOST_CHECK(v.at( index ) == tmp);
    }
}

BOOST_AUTO_TEST_CASE(plus)
{
    Vector< 4, double > v;
    double data[] = { 1, 2, 3, 4 };

    // tests operator+ function
    Vector< 4, double > v_other;
    Vector< 4, double > v_result;

    v = data;

    double datad[] = { 4, 3, 2, 1 };
    v_other = datad;

    v_result = v + v_other;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == 5);
    }

    v_result = v;
    v_result += v_other;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == 5);
    }

    v = data;
    v_result = v + 2.;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == index + 3);
    }

    v_result = v;
    v_result += 2;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == index + 3);
    }
}

BOOST_AUTO_TEST_CASE(minus)
{
    Vector< 4, double > v;
    double data[] = { 1, 2, 3, 4 };

    // tests operator- function
    Vector< 4, double > v_other;
    Vector< 4, double > v_result;
    v = data;

    double datad[] = { 1, 2, 3, 4 };
    v_other = datad;

    v_result = v - v_other;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == 0);
    }

    v_result = v;
    v_result -= v_other;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == 0);
    }


    v_result = v - 1.0;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == index);
    }

    v_result = v;
    v_result -= 1.0;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == index);
    }
}

BOOST_AUTO_TEST_CASE(times)
{
    Vector< 4, double > v;
    double data[] = { 1, 2, 3, 4 };

    // tests operator* function
    Vector< 4, double > v_other;
    Vector< 4, double > v_result;

    v = data;

    double datad[] = { 24, 12, 8, 6 };
    v_other = datad;

    v_result = v * v_other;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == 24);
    }

    v_result = v;
    v_result *= v_other;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == 24);
    }

    v_result = v * 2.0;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == v.at( index ) * 2.0);
    }

    v_result = v;
    v_result *= 2.0;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(v_result.at( index ) == v.at( index ) * 2.0);
    }
}

BOOST_AUTO_TEST_CASE(divide)
{
    Vector< 4, double > v;
    double data[] = { 1, 2, 3, 4 };

    // tests operator/ function
    Vector< 4, double > v_other;
    Vector< 4, double > v_result;

    v = data;

    double datad[] = { 2, 4, 6, 8 };
    v_other = datad;

    v_result = v / v_other;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(( v_result.at( index ) - 0.5 ) < 1e-12);
    }

    v_result = v;
    v_result /= v_other;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(( v_result.at( index ) - 0.5 ) < 1e-12);
    }


    v_result = v / 1.5;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(( v_result.at( index ) - ( v.at( index ) / 1.5 ) ) < 1e-12);
    }

    v_result = v;
    v_result /= 1.5;
    for( size_t index = 0; index < 4; ++index )
    {
        BOOST_CHECK(( v_result.at( index ) - ( v.at( index ) / 1.5 ) ) < 1e-12);
    }
}

BOOST_AUTO_TEST_CASE(vec_norm)
{
    Vector< 4, double > v;
    double data[] = { 1, 2, 3, 4 };

    // tests norm / normSquared (length/lengthSquared) computation
    Vector< 4, double > vec;
    vec = data;

    double normSquared = vec.lengthSquared();
    BOOST_CHECK(normSquared == 1 * 1 + 2 * 2 + 3 * 3 + 4 * 4);

    double norm = vec.length();
    BOOST_CHECK( std::sqrt( normSquared ) == norm );

    // tests normalize
    vec = data;
    vec.normalize();
    BOOST_CHECK_CLOSE( vec.length(), 1.0, 0.0000001 );


    // constructor tests
    double vData[] = { 1, 2, 3, 4 };
    Vector< 4, double > v4( 1, 2, 3, 4 );

    Vector< 2, double > v2C;
    v2C = vData;
    Vector< 2, double > v2( 1, 2 );

    BOOST_CHECK(v2 == v2C );

    Vector< 3, double > v3C;
    v3C = vData;
    Vector< 3, double > v3( 1, 2, 3 );

    BOOST_CHECK(v3 == v3C );

    Vector< 4, double > v4C;
    v4C = vData;

    BOOST_CHECK(v4 == v4C);

    double vData2[] = { 23, 23, 23, 23 };
    v4C = vData2;

    Vector< 4, double > v4_( 23 );
    BOOST_CHECK(v4_ == v4C);

    const Vector< 4, double > homogenous( 1., 2., 3., 0.25 );
    const Vector< 3, double > nonh( 4.0, 8.0, 12.0 );
    const Vector< 4, double > htest( nonh );

    // to-homogenous-coordinates ctor
    BOOST_CHECK((htest == Vector< 4, double >( 4., 8., 12., 1. ) ));

    Vector< 3, double > nhtest( homogenous );

    // from homogenous-coordiates ctor
    BOOST_CHECK(nhtest == nonh );

    // component accessors
    Vector< 4, double > vd( 1, 2, 3, 4 );
    BOOST_CHECK( vd.x() == 1 && vd.y() == 2 && vd.z() == 3 && vd.w() == 4 );
}

BOOST_AUTO_TEST_CASE(dotprod)
{
    // dot product
    Vector< 3, float > v0( 1, 2, 3 );
    Vector< 3, float > v1( -6, 5, -4 );
    BOOST_CHECK( v0.dot( v1 ) == -8 );
}

BOOST_AUTO_TEST_CASE(crossprod)
{
    Vector< 3, float > v0( 1, 2, 3 );
    const Vector< 3, float > v1( -6, 5, -4 );
    const Vector< 3, float > vcorrect( -23, -14, 17 );

    BOOST_CHECK_EQUAL( cross( v0, v1 ), vcorrect );
    BOOST_CHECK_EQUAL( v0.cross( v1 ), vcorrect );
}

BOOST_AUTO_TEST_CASE(normal)
{
    const Vector< 3, float > v1( 0.f, 0.f, 0.f );
    const Vector< 3, float > v2( 0.f, 1.f, 0.f );
    const Vector< 3, float > v3( 1.f, 0.f, 0.f );
    const Vector< 3, float > n( 0.f, 0.f, -1.f );
    BOOST_CHECK_EQUAL( vmml::compute_normal( v1, v2, v3 ), n );
}

BOOST_AUTO_TEST_CASE(product)
{
    const Vector< 3, float > v0( 1, 2, 3 );
    BOOST_CHECK_EQUAL( v0.product(), 6 );

    const Vector< 3, int > v1( -6, 5, -4 );
    BOOST_CHECK_EQUAL( v1.product(), 120 );
}

BOOST_AUTO_TEST_CASE(tbd1)
{
    Vector< 4, float > v1( -1.0f, 3.0f, -99.0f, -0.9f );
    float f = 4.0f;
    Vector< 4, float > v_scaled = f * v1;

    BOOST_CHECK(v_scaled == (Vector< 4, float >( -4.0f, 12.0f, -396.0f, -3.6f ) ));
}

BOOST_AUTO_TEST_CASE(subVector)
{
    Vector< 4, float > v4( 3.0, 2.0, 1.0, 1.0 );
    Vector< 3, float > v3 = v4.getSubVector< 3 >();
    BOOST_CHECK(v3.x() == v4.x() && v3.y() == v4.y());
    v3.normalize();

    BOOST_CHECK_NE( v3.x(), v4.x( ));
    BOOST_CHECK_NE( v3.y(), v4.y( ));

    v4.setSubVector< 3, 1 >( v3 );
    BOOST_CHECK_EQUAL( v3.x(), v4.y( ));
    BOOST_CHECK_EQUAL( v3.y(), v4.z( ));
}

BOOST_AUTO_TEST_CASE(rotateVec)
{
    vmml::Vector3f vector = vmml::Vector3::forward< float >();
    vector.rotate( float( M_PI ), vmml::Vector3::up< float >( ));
    BOOST_CHECK_MESSAGE( vector.equals( vmml::Vector3::backward< float >( )),
                         vector );
    BOOST_CHECK_MESSAGE( vmml::rotate( vector, float( M_PI ),
                                       vmml::Vector3::left< float >( )).equals(
                                           vmml::Vector3::forward< float >( )),
                         vmml::rotate( vector, float( M_PI ),
                                       vmml::Vector3::left< float >( )));
}

// Verify code by instantiating some templates:
template class vmml::Vector< 1, float >;
template class vmml::Vector< 2, double >;
template class vmml::Vector< 3, short >;
template class vmml::Vector< 3, int >;
template class vmml::Vector< 4, long >;
