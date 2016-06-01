//
//  flVector.cpp
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 30..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#include "nbVector.h"
#include <math.h>
#include "nbMatrix.h"

namespace nbd {

vec2::vec2(float x, float y) : x(x), y(y) {}

vec2 vec2::add(const vec2& v) const
{
    return vec2(x + v.x, y+v.y);
}

vec2 vec2::diff(const vec2& v) const
{
    return vec2(x - v.x, y - v.y);
}

void vec2::operator=(const vec3& v3)
{
    x = v3.x;
    y = v3.y;
}

vec3::vec3(float x, float y, float z) : x(x), y(y), z(z) {}

vec3 vec3::add(const vec3& v) const
{
    return vec3(x + v.x, y + v.y, z + v.z);
}

vec3 vec3::diff(const vec3& v) const
{
    return vec3(x - v.x, y - v.y, z - v.z);
}

vec3 vec3::mul(const vec3&v) const
{
    return vec3(x * v.x, y * v.y, z * v.z);
}

void vec3::clear()
{
    x = y = z = 0;
}

/*!
	Calculate the dot product of a vector 3D.
	
	\param[in] v The vector 3D to calculate the dot product for.
	
	\return Return dot product of the vector 3D.
 */

float vec3::dot() const
{
    return (x * x) + (y * y) + (z * z);
}



/*!
	Calculate the dot product between two vector 3D.
	
	\param[in] v0 The first vector 3D to use in the dot product operation.
	\param[in] v1 The second vector 3D to use in the dot product operation.
	
	\return Return the result of the dot product operation between the two vector 3D.
 */
float vec3::dot(const vec3&v) const
{
    return (x * v.x) + (y * v.y) + (z * v.z);
}
/*!
	Calculate the lenght of a vector 3D.
	
	\param[in] v The vector 3D to calculate the lenght for.
 
	\return Return the lenght of the vector 3D.
 */
//float vec3_length( vec3 *v )
//{ return sqrtf( vec3_dot( v ) ); }

float vec3::length() const
{
    return sqrtf(dot());
}

/*!
	Helper function to normalize a vector 3D.
	
	\param[in,out] dst Variable to store the result of the normalization operation.
	\param[in] v The vector 3D to normalize.
	
	\return Return the lenght of the vector 3D.
 */
//float vec3_normalize( vec3 *dst, vec3 *v )
//{
//    float l = vec3_length( v );
//    
//    if( l )
//    {
//        float m = 1.0f / l;
//        
//        dst->x = v->x * m;
//        dst->y = v->y * m;
//        dst->z = v->z * m;
//    }
//    
//    return l;
//}

float vec3::normalize()
{
    float l = length();
    if (l) {
        float m = 1.0f / l;
        x = x * m;
        y = y * m;
        z = z * m;
    }
    
    return l;
}

vec3 vec3::normalized() const
{
    vec3 v = *this;
    v.normalize();
    return v;
}


/*!
	Calculate the cross product between two vector 3D.
	
	\param[in,out] dst This variable is used to store the result of the cross operation.
	\param[in] v0 The first vector 3D to use in the dot product operation.
	\param[in] v1 The second vector 3D to use in the dot product operation.
 
 */
//void vec3_cross( vec3 *dst, vec3 *v0, vec3 *v1 )
//{
//    dst->x = ( v0->y * v1->z ) - ( v1->y * v0->z );
//    dst->y = ( v0->z * v1->x ) - ( v1->z * v0->x );
//    dst->z = ( v0->x * v1->y ) - ( v1->x * v0->y );
//}

vec3 vec3::cross(const vec3&v) const
{
    float xx = (y * v.z) - (v.y * z);
    float yy = (z * v.x) - (v.z * x);
    float zz = (x * v.y) - (v.x * y);
    return vec3(xx, yy, zz);
}


/*!
	Calculate the distance between two vertex.
	
	\param[in] v0 The first vertex 3D.
	\param[in] v1 The second vertex 3D.
	
	\return Return the distance between the two vertex 3D.
 */
//float vec3_dist( vec3 *v0, vec3 *v1 )
//{
//    vec3 v;
//    
//    vec3_diff( &v, v0, v1 );
//    
//    return vec3_length( &v );
//}
float vec3::dist(const vec3&v) const
{
    vec3 vv = diff(v);
    return vv.length();
}

/*!
	Calculate the middle pointe between two vertex 3D.
	
	\param[in,out] dst Variable used to store the middle pointer between the two vertex.
	\param[in] v0 The first vertex 3D.
	\param[in] v1 The second vertex 3D.
 */
//void vec3_mid( vec3 *dst, vec3 *v0, vec3 *v1 )
//{
//    dst->x = ( v0->x + v1->x ) * 0.5f;
//    dst->y = ( v0->y + v1->y ) * 0.5f;
//    dst->z = ( v0->z + v1->z ) * 0.5f;
//}

vec3 vec3::mid(const vec3&v) const
{
    float xx = (x + v.x) * 0.5f;
    float yy = (y + v.y) * 0.5f;
    float zz = (z + v.z) * 0.5f;
    
    return vec3(xx, yy, zz);
}


/*!
	Inverse a vector 3D.
 
	\param[in,out] dst Variable used to store inverse vector 3D.
	\param[in] v The vector 3D to inverse.
 */
//void vec3_invert( vec3 *dst, vec3 *v )
//{
//    dst->x = -v->x;
//    dst->y = -v->y;
//    dst->z = -v->z;
//}

vec3 vec3::invert() const
{
    return vec3(-x, -y, -z);
}

/*!
	Linearly interpolate between two vertex 3D.
	
	\param[in,out] dst Variable used to store final interpolated vertex 3D.
	\param[in] v0 The first vertex 3D.
	\param[in] v1 The second vertex 3D.
	\param[in] t Value ranging from the range of 0 to 1 to interpolate the XYZ value of the two vertex 3D.
 */
//void vec3_lerp( vec3 *dst, vec3 *v0, vec3 *v1, float t )
//{
//    if( t == 1.0f )
//    {
//        dst->x = v1->x;
//        dst->y = v1->y;
//        dst->z = v1->z;
//        
//        return;
//    }
//    else if( t == 0.0f )
//    {
//        dst->x = v0->x;
//        dst->y = v0->y;
//        dst->z = v0->z;
//        
//        return;
//    }
//    
//    dst->x = v0->x + t * ( v1->x - v0->x );
//    dst->y = v0->y + t * ( v1->y - v0->y );
//    dst->z = v0->z + t * ( v1->z - v0->z );
//}


vec3 vec3::lerp(const vec3&v, float t) const
{
    if( t == 1.0f )
    {
        return vec3(v.x, v.y, v.z);
    }
    else if( t == 0.0f )
    {
        return vec3(x, y, z);
    }

    float xx = x + t * (v.x - x);
    float yy = y + t * (v.y - y);
    float zz = z + t * (v.z - z);
    return vec3(xx, yy, zz);
}

/*!
	Alternate method to linearly interpolate between two vertex 3D.
	
	\param[in,out] dst Variable used to store final interpolated vertex 3D.
	\param[in] v0 The first vertex 3D.
	\param[in] v1 The second vertex 3D.
	\param[in] t Value ranging from the range of 0 to 1 to interpolate the XYZ value of the two vertex 3D.
 */
//void vec3_lerp2( vec3 *dst, vec3 *v0, vec3 *v1, float t )
//{
//    dst->x = v0->x * t + v1->x * ( 1.0f - t );
//    dst->y = v0->y * t + v1->y * ( 1.0f - t );
//    dst->z = v0->z * t + v1->z * ( 1.0f - t );
//}

vec3 vec3::lerp2(const vec3&v, float t) const
{
    float xx = x * t + v.x * (1.0f - t);
    float yy = y * t + v.y * (1.0f - t);
    float zz = z * t + v.z * (1.0f - t);
    return vec3(xx, yy, zz);
}


/*!
	Rotate a vertex 3D by a quaternion (vector 4D).
	
	\param[in,out] dst Variable used to store the result of the rotation.
	\param[in] v0 The vertex 3D.
	\param[in] v1 The quaternion to use to rotate the vertex.
 */
//TODO:
//void vec3_rotate_vec4( vec3 *dst, vec3 *v0, vec4 *v1 )
//{
//    vec4 i, t, f;
//    
//    vec4_conjugate( &i, v1 );
//    
//    vec4_normalize( &i, &i );
//    
//    vec4_multiply_vec3( &t, v1, v0 );
//    
//    vec4_multiply_vec4( &f, &t, &i );
//    
//    memcpy( dst, &f, sizeof( vec3 ) );
//}

vec3 vec3::rotate(const vec4&v) const
{
    vec4 i = v.conjugate();
    i.normalize();
    vec4 t = v.multiply(*this);
    vec4 f = t.multiply(i);
    return vec3(f.x, f.y, f.w);
}


/*!
	Helper that convert a vertex 3D to the Recast default coordinate system (up axis Y to up axis Z).
 
	\param[in,out] v The vertex 3D to convert.
 */
//void vec3_to_recast( vec3 *v )
//{
//    vec3 tmp = {  v->x,
//        v->z,
//			     -v->y };
//    
//    memcpy( v, &tmp, sizeof( vec3 ) );
//}

vec3 vec3::toRecast() const
{
    return vec3(x, z, -y);
}

/*!
	Helper function that convert a Recast based coordinate system vector 3D to an up axis Z coordinate system.
 
	\param[in,out] v The vertex 3D to convert.	
 */
//void recast_to_vec3( vec3 *v )
//{
//    vec3 tmp = {  v->x,
//			     -v->z,
//        v->y };
//    
//    memcpy( v, &tmp, sizeof( vec3 ) );
//}

vec3 vec3::recastTo() const
{
    return vec3(x, -z, y);
}


/*!
	Multiply a vec3 by a 3x3 matrix.
	
	\param[in,out] dst A valid vec3 pointer that will be used as the destination variable where the result of the operation will be stored.
	\param[in] v A valid vec3 pointer.
	\param[in] m A valid 3x3 matrix pointer.
 */
//void vec3_multiply_mat3( vec3 *dst, vec3 *v, mat3 *m )
//{
//    dst->x = ( v->x * m->m[ 0 ].x ) +
//			 ( v->y * m->m[ 1 ].x ) +
//			 ( v->z * m->m[ 2 ].x );
//    
//    dst->y = ( v->x * m->m[ 0 ].y ) +
//			 ( v->y * m->m[ 1 ].y ) +
//			 ( v->z * m->m[ 2 ].y );
//    
//    dst->z = ( v->x * m->m[ 0 ].z ) +
//			 ( v->y * m->m[ 1 ].z ) +
//			 ( v->z * m->m[ 2 ].z );
//}

vec3 vec3::multiply(const mat3&m) const
{
    float xx = ( x * m.m[ 0 ].x ) +
			 ( y * m.m[ 1 ].x ) +
			 ( z * m.m[ 2 ].x );
    
    float yy = ( x * m.m[ 0 ].y ) +
			 ( y * m.m[ 1 ].y ) +
			 ( z * m.m[ 2 ].y );
    
    float zz = ( x * m.m[ 0 ].z ) +
			 ( y * m.m[ 1 ].z ) +
			 ( z * m.m[ 2 ].z );
    
    return vec3(xx, yy, zz);
}


/*!
	Multiply a vec3 by a 4x4 matrix.
	
	\param[in,out] dst A valid vec3 pointer that will be used as the destination variable where the result of the operation will be stored.
	\param[in] v A valid vec3 pointer.
	\param[in] m A valid 4x4 matrix pointer.
 */
//void vec3_multiply_mat4( vec3 *dst, vec3 *v, mat4 *m )
//{
//    dst->x = ( v->x * m->m[ 0 ].x ) +
//			 ( v->y * m->m[ 1 ].x ) +
//			 ( v->z * m->m[ 2 ].x );
//    
//    dst->y = ( v->x * m->m[ 0 ].y ) +
//			 ( v->y * m->m[ 1 ].y ) +
//			 ( v->z * m->m[ 2 ].y );
//    
//    dst->z = ( v->x * m->m[ 0 ].z ) +
//			 ( v->y * m->m[ 1 ].z ) +
//			 ( v->z * m->m[ 2 ].z );
//}

vec3 vec3::multiply(const mat4&m) const
{
    float xx = ( x * m.m[ 0 ].x ) +
			 ( y * m.m[ 1 ].x ) +
			 ( z * m.m[ 2 ].x );
    
    float yy = ( x * m.m[ 0 ].y ) +
			 ( y * m.m[ 1 ].y ) +
			 ( z * m.m[ 2 ].y );
    
    float zz = ( x * m.m[ 0 ].z ) +
			 ( y * m.m[ 1 ].z ) +
			 ( z * m.m[ 2 ].z );
    
    return vec3(xx, yy, zz);
}

vec4::vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

void vec4::operator=(const vec3& v3)
{
    x = v3.x;
    y = v3.y;
    z = v3.z;
}


vec4 vec4::add(const vec4& v) const
{
    return vec4(x + v.x, y + v.y, z + v.z, w + v.w);
}

vec4 vec4::diff(const vec4& v) const
{
    return vec4(x - v.x, y - v.y, z - v.z, w - v.w);
}


/*!
	Helper function to re-calculate the W component of a quaternion.
	
	\param[in,out] v The quaternion to use in order to re-calculate the W component.
 */
//void vec4_build_w( vec4 * v )
//{
//    float l = 1.0f - ( v->x * v->x ) -
//    ( v->y * v->y ) -
//    ( v->z * v->z );
//    
//    v->w = ( l < 0.0f ) ? 0.0f : -sqrtf( l );
//}

void vec4::buildW()
{
    float l = 1.0f - (x * x) - (y * y) - (z * z);
    w = (l < 0.0f) ? 0.0f : -sqrtf(l);
}

/*!
	Calculate the dot product of a vector 4D.
	
	\param[in] v The vector 4D to calculate the dot product for.
	
	\return Return the dot product of the vector 4D.
 */
//float vec4_dot( vec4 *v )
//{
//    return ( v->x * v->x ) +
//    ( v->y * v->y ) +
//    ( v->z * v->z ) +
//    ( v->w * v->w );
//}
float vec4::dot() const
{
    return (x * x) + (y * y) + (z * z) + (w * w);
}
/*!
	Calculate the dot product between two vector 4D.
	
	\param[in] v0 The first vector 4D to use in the dot product operation.
	\param[in] v1 The second vector 4D to use in the dot product operation.
	
	\return Return the result of the dot product operation between the two vector 4D.
 */
//float vec4_dot_vec4( vec4 *v0, vec4 *v1 )
//{
//    return ( ( v0->x * v1->x ) +
//            ( v0->y * v1->y ) +
//            ( v0->z * v1->z ) +
//            ( v0->w * v1->w ) );
//}
float vec4::dot(const vec4&v) const
{
    return (x * v.x) + (y * v.y) + (z * v.z) + (w * v.w);
}

/*!
	Calculate the lenght of a vector 4D.
	
	\param[in] v The vector 4D to calculate the lenght for.
 
	\return Return the lenght of the vector 4D.
 */
//float vec4_length( vec4 *v )
//{
//    return sqrtf( vec4_dot( v ) );
//}

float vec4::length() const
{
    return sqrtf(dot());
}

/*!
	Helper function to normalize a vector 4D.
	
	\param[in,out] dst Variable to store the result of the normalization operation.
	\param[in] v The vector 4D to normalize.
	
	\return Return the lenght of the vector 4D.
 */
//float vec4_normalize( vec4 *dst, vec4 *v )
//{
//    float l = vec4_length( v ),
//		  m = l ? 1.0f / l: 0.0f;
//    
//    dst->x = v->x * m;
//    dst->y = v->y * m;
//    dst->z = v->z * m;
//    dst->w = v->w * m;
//    
//    return l;
//}

float vec4::normalize()
{
    float l = length();
    float m = l ? 1.0f / l : 0.0;
    
    x = x * m;
    y = y * m;
    z = z * m;
    w = w * m;
    
    return l;
}

vec4 vec4::normalized() const
{
    vec4 v = *this;
    v.normalize();
    return v;
}


/*!
	Multiply a quaternion by a vector 3D.
	
	\param[in,out] dst The result of the multiplication operation.
	\param[in] v0 The quaternion to use for the multiplication.
	\param[in] v1 The vector 3D to use for the multiplication.
 */
//void vec4_multiply_vec3( vec4 *dst, vec4 *v0, vec3 *v1 )
//{
//    vec4 v;
//    
//    v.x =  ( v0->w * v1->x ) + ( v0->y * v1->z ) - ( v0->z * v1->y );
//    v.y =  ( v0->w * v1->y ) + ( v0->z * v1->x ) - ( v0->x * v1->z );
//    v.z =  ( v0->w * v1->z ) + ( v0->x * v1->y ) - ( v0->y * v1->x );
//    v.w = -( v0->x * v1->x ) - ( v0->y * v1->y ) - ( v0->z * v1->z );
//    
//    memcpy( dst, &v, sizeof( vec4 ) );
//}

vec4 vec4::multiply(const vec3&v) const
{
    float xx = (w * v.x) + (y * v.z) - (z * v.y);
    float yy = (w * v.y) + (z * v.x) - (x * v.z);
    float zz = (w * v.z) + (x * v.y) - (y * v.x);
    float ww = -(x * v.x) - (y * v.y) - (z * v.z);
    return vec4(xx, yy, zz, ww);
}


/*!
	Multiply two quaternions.
	
	\param[in,out] dst The result of the multiplication operation.
	\param[in] v0 The first quaternion to use for the multiplication.
	\param[in] v1 The second quaternion to use for the multiplication.
 */
//void vec4_multiply_vec4( vec4 *dst, vec4 *v0, vec4 *v1 )
//{
//    vec4 v;
//    
//    v.x = ( v0->x * v1->w ) + ( v0->w * v1->x ) + ( v0->y * v1->z ) - ( v0->z * v1->y );
//    v.y = ( v0->y * v1->w ) + ( v0->w * v1->y ) + ( v0->z * v1->x ) - ( v0->x * v1->z );
//    v.z = ( v0->z * v1->w ) + ( v0->w * v1->z ) + ( v0->x * v1->y ) - ( v0->y * v1->x );
//    v.w = ( v0->w * v1->w ) - ( v0->x * v1->x ) - ( v0->y * v1->y ) - ( v0->z * v1->z );
//    
//    memcpy( dst, &v, sizeof( vec4 ) );
//}

vec4 vec4::multiply(const vec4&v) const
{
    float xx = (x * v.w) + (w * v.x) + (y * v.z) - (z * v.y);
    float yy = (y * v.w) + (w * v.y) + (z * v.x) - (x * v.z);
    float zz = (z * v.w) + (w * v.z) + (x * v.y) - (y * v.x);
    float ww = (w * v.w) - (x * v.x) - (y * v.y) - (z * v.z);
    return vec4(xx, yy, zz, ww);
}


/*!
	Conjugate a quaterion.
	
	\param[in,out] dst The result conjugate operation.
	\param[in] v The quaternion to conjugate.
 */
//void vec4_conjugate( vec4 *dst, vec4 *v )
//{
//    dst->x = -v->x;
//    dst->y = -v->y;
//    dst->z = -v->z;
//    dst->w =  v->w;
//}

vec4 vec4::conjugate() const
{
    return vec4(-x, -y, -z, w);
}
/*!
	Inverse a quaternion.
 
	\param[in,out] dst Variable used to store the inverse quaternion.
	\param[in] v The quaternion to inverse.
 */
//void vec4_invert( vec4 *dst, vec4 *v )
//{
//    dst->x = -v->x;
//    dst->y = -v->y;
//    dst->z = -v->z;
//    dst->w = -v->w;
//}

vec4 vec4::invert() const
{
    return vec4(-x, -y, -z, -w);
}


/*!
	Linearly interpolate between two quaternions.
	
	\param[in,out] dst Variable used to store final interpolated quaternion.
	\param[in] v0 The first quaternion.
	\param[in] v1 The second quaternion.
	\param[in] t Value ranging from the range of 0 to 1 to interpolate the XYZ value of the two quaternion.
 */
//void vec4_lerp( vec4 *dst, vec4 *v0, vec4 *v1, float t )
//{
//    float dot = vec4_dot_vec4( v0, v1 ),
//		  k0,
//		  k1;
//    
//    vec4 tmp = { v1->x,
//        v1->y,
//        v1->z,
//        v1->w };
//    
//    if( t == 1.0f )
//    {
//        dst->x = v1->x;
//        dst->y = v1->y;
//        dst->z = v1->z;
//        dst->w = v1->w;
//        
//        return;
//    }
//    else if( t == 0.0f )
//    {
//        dst->x = v0->x;
//        dst->y = v0->y;
//        dst->z = v0->z;
//        dst->w = v0->w;
//        
//        return;
//    }
//    
//    
//    if( dot < 0.0f )
//    {
//        tmp.x = -tmp.x;
//        tmp.y = -tmp.y;
//        tmp.z = -tmp.z;
//        tmp.w = -tmp.w;
//        
//        dot = -dot;
//    }
//    
//    
//    if( dot > 0.999999f )
//    {
//        k0 = 1.0f - t;
//        k1 = t;
//    }
//    else
//    {
//        float s  = sqrtf( 1.0f - ( dot * dot ) ),
//        o  = atan2f( s, dot ),
//        o1 = 1.0f / s;
//        
//        k0 = sinf( ( 1.0f - t ) * o ) * o1;
//        k1 = sinf( t * o ) * o1;
//    }
//    
//    dst->x = ( k0 * v0->x ) + ( k1 * tmp.x );
//    dst->y = ( k0 * v0->y ) + ( k1 * tmp.y );
//    dst->z = ( k0 * v0->z ) + ( k1 * tmp.z );
//    dst->w = ( k0 * v0->w ) + ( k1 * tmp.w );
//}

vec4 vec4::lerp(const vec4&v, float t) const
{
    float d = dot(v);
    float k0, k1;
    
    if( t == 1.0f )
    {
        return vec4(v.x, v.y, v.z, v.w);
    }
    else if( t == 0.0f )
    {
        return vec4(x, y, z, w);
    }
    
    vec4 tmp = vec4(v.x, v.y, v.z, v.w);
    if (d < 0.0f) {
        tmp = tmp.invert();
        d = -d;
    }
    
    if (d > 0.999999f) {
        k0 = 1.0f - t;
        k1 = t;
    }
    else {
        float s  = sqrtf( 1.0f - ( d * d ) ),
        o  = atan2f( s, d ),
        o1 = 1.0f / s;
        
        k0 = sinf( ( 1.0f - t ) * o ) * o1;
        k1 = sinf( t * o ) * o1;
    }

    float xx = (k0 * x) + (k1 * tmp.x);
    float yy = (k0 * y) + (k1 * tmp.y);
    float zz = (k0 * z) + (k1 * tmp.z);
    float ww = (k0 * w) + (k1 * tmp.w);
    return vec4(xx, yy, zz, ww);
}


/*!
	Spherically interpolate between two quaternions.
	
	\param[in,out] dst Variable used to store final interpolated quaternion.
	\param[in] v0 The first quaternion.
	\param[in] v1 The second quaternion.
	\param[in] t Value ranging from the range of 0 to 1 to interpolate the XYZ value of the two quaternion.
 */
//void vec4_slerp( vec4 *dst, vec4 *v0, vec4 *v1, float t )
//{
//    float c = vec4_dot_vec4( v0, v1 ),
//		  k0,
//		  k1;
//    
//    vec4 tmp = { v1->x,
//        v1->y,
//        v1->z,
//        v1->w };
//    
//    if( t == 1.0f )
//    {
//        dst->x = v1->x;
//        dst->y = v1->y;
//        dst->z = v1->z;
//        dst->w = v1->w;
//        
//        return;
//    }
//    else if( t == 0.0f )
//    {
//        dst->x = v0->x;
//        dst->y = v0->y;
//        dst->z = v0->z;
//        dst->w = v0->w;
//        
//        return;
//    }
//    
//    
//    if( c < 0.0f )
//    {
//        tmp.x = -tmp.x;
//        tmp.y = -tmp.y;
//        tmp.z = -tmp.z;
//        tmp.w = -tmp.w;
//        
//        c = -c;
//    }
//    
//    
//    if( c > 0.999999f )
//    {
//        k0 = 1.0f - t;
//        k1 = t;
//    }
//    else
//    {
//        float s  = sqrtf( 1.0f - ( c * c ) ),
//        o  = atan2f( s, c ),
//        o1 = 1.0f / s;
//        
//        k0 = sinf( ( 1.0f - t ) * o ) * o1;
//        k1 = sinf( t * o ) * o1;
//    }
//    
//    dst->x = ( k0 * v0->x ) + ( k1 * tmp.x );
//    dst->y = ( k0 * v0->y ) + ( k1 * tmp.y );
//    dst->z = ( k0 * v0->z ) + ( k1 * tmp.z );		
//    dst->w = ( k0 * v0->w ) + ( k1 * tmp.w );
//}

vec4 vec4::slerp(const vec4&v, float t) const
{
    float d = dot(v);
    float k0, k1;
    
    
    if( t == 1.0f )
    {
        return vec4(v.x, v.y, v.z, v.w);
    }
    else if( t == 0.0f )
    {
        return vec4(x, y, z, w);
    }
    
    vec4 tmp = vec4(v.x, v.y, v.z, v.w);
    if (d < 0.0f) {
        tmp = tmp.invert();
        d = -d;
    }
    
    if (d > 0.999999f) {
        k0 = 1.0f - t;
        k1 = t;
    }
    else {
        float s  = sqrtf( 1.0f - ( d * d ) ),
        o  = atan2f( s, d ),
        o1 = 1.0f / s;
        
        k0 = sinf( ( 1.0f - t ) * o ) * o1;
        k1 = sinf( t * o ) * o1;
    }
    
    float xx = (k0 * x) + (k1 * tmp.x);
    float yy = (k0 * y) + (k1 * tmp.y);
    float zz = (k0 * z) + (k1 * tmp.z);
    float ww = (k0 * w) + (k1 * tmp.w);
    return vec4(xx, yy, zz, ww);
}





/*!
	Multiply a vec4 by a 4x4 matrix.
	
	\param[in,out] dst A valid vec4 pointer that will be used as the destination variable where the result of the operation will be stored.
	\param[in] v A valid vec4 pointer.
	\param[in] m A valid 4x4 matrix pointer.
 */
//void vec4_multiply_mat4( vec4 *dst, vec4 *v, mat4 *m )
//{
//    dst->x = ( v->x * m->m[ 0 ].x ) +
//			 ( v->y * m->m[ 1 ].x ) +
//			 ( v->z * m->m[ 2 ].x ) +
//			 ( v->w * m->m[ 3 ].x );
//
//    dst->y = ( v->x * m->m[ 0 ].y ) +
//			 ( v->y * m->m[ 1 ].y ) +
//			 ( v->z * m->m[ 2 ].y ) +
//			 ( v->w * m->m[ 3 ].y );
//
//    dst->z = ( v->x * m->m[ 0 ].z ) +
//			 ( v->y * m->m[ 1 ].z ) +
//			 ( v->z * m->m[ 2 ].z ) +
//			 ( v->w * m->m[ 3 ].z );
//
//    dst->w = ( v->x * m->m[ 0 ].w ) +
//			 ( v->y * m->m[ 1 ].w ) +
//			 ( v->z * m->m[ 2 ].w ) +
//			 ( v->w * m->m[ 3 ].w );
//}
//
//
//

vec4 vec4::multiply(const mat4&m) const
{
    float xx = ( x * m.m[ 0 ].x ) +
			 ( y * m.m[ 1 ].x ) +
			 ( z * m.m[ 2 ].x ) +
			 ( w * m.m[ 3 ].x );
    
    float yy = ( x * m.m[ 0 ].y ) +
			 ( y * m.m[ 1 ].y ) +
			 ( z * m.m[ 2 ].y ) +
			 ( w * m.m[ 3 ].y );
    
    float zz = ( x * m.m[ 0 ].z ) +
			 ( y * m.m[ 1 ].z ) +
			 ( z * m.m[ 2 ].z ) +
			 ( w * m.m[ 3 ].z );
    
    float ww = ( x * m.m[ 0 ].w ) +
			 ( y * m.m[ 1 ].w ) +
			 ( z * m.m[ 2 ].w ) +
			 ( w * m.m[ 3 ].w );
    
    return vec4(xx, yy, zz, ww);
}
    
}


