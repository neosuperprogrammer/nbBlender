//
//  MatrixMgr.cpp
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 2. 13..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#include "nbMatrixMgr.h"

#include <math.h>


namespace nbd {
    
    void MatrixMgr::initMatrix()
    {
        modelViewMatrixList.push(*(new mat4()));
        projectionMatrixList.push(*(new mat4()));
        textureMatrixList.push(*(new mat4()));
        
        mode = kModelViewMatrix;
    }
    
    void MatrixMgr::setMatrixMode(eMatrixMode mode)
    {
        this->mode = mode;
    }
    
    mat4& MatrixMgr::currentMatrix()
    {
        return currentMatrixList().top();
    }
    
    void MatrixMgr::pushMatrix(void)
    {
        mat4 m;
        m.copy(currentMatrix());
        currentMatrixList().push(m);
    }
    
    void MatrixMgr::pushMatrix(mat4 &m)
    {
        currentMatrixList().push(m);
    }
    
    void MatrixMgr::popMatrix(void)
    {
        currentMatrixList().pop();
    }
    
    stack<mat4>& MatrixMgr::currentMatrixList(void)
    {
        switch (mode) {
            case kModelViewMatrix: {
                return modelViewMatrixList;
                break;
            }
            case kProjectionMatrix: {
                return projectionMatrixList;
                break;
            }
            case kTextureMatrix: {
                return textureMatrixList;
                break;
            }
        }
    }
    
    void MatrixMgr::loadIdentity()
    {
        currentMatrix().identity();
    }
    
    void MatrixMgr::setPerspective(float fovy, float aspect_ratio, float clip_start, float clip_end, float screen_orientation)
    {
        
        float top = clip_start * tanf(fovy * 3.14 / 360.0f);
        float bottom = -top;
        float left = bottom * aspect_ratio;
        float right = top * aspect_ratio;
        
        set_perspective(left, right, bottom, top, clip_start, clip_end, screen_orientation);
        
    }
    
    void MatrixMgr::set_perspective(float left, float right, float bottom, float top, float near, float far, float screen_orientation)
    {
        mat4 mat;
        
        float a = 2 * near / (right - left);
        float b = 2 * near / (top - bottom);
        float c = (right + left) / (right - left);
        float d = (top + bottom) / (top - bottom);
        float e = - (far + near) / (far - near);
        float f = -2 * far * near / (far - near);
        
        //	mat4_identity( &mat );
        mat.identity();
        
        mat.m[ 0 ].x = a;
        mat.m[ 1 ].y = b;
        mat.m[ 2 ].x = c;
        mat.m[ 2 ].y = d;
        mat.m[ 2 ].z = e;
        mat.m[ 2 ].w = -1.0f;
        mat.m[ 3 ].z = f;
        mat.m[ 3 ].w = 1.0f;
        
        multiply(mat);
        
        if (screen_orientation) {
            rotate(screen_orientation, 0.0f, 0.0f, 1.0f);
        }
    }
    
    void MatrixMgr::set_orthographic( float screen_ratio, float scale, float aspect_ratio, float clip_start, float clip_end, float screen_orientation )
    {
        scale = ( scale * 0.5f ) * aspect_ratio;
        
        currentMatrix().ortho( -1.0f,
                              1.0f,
                              -screen_ratio,
                              screen_ratio,
                              clip_start,
                              clip_end );
        
        this->scale(1.0f / scale, 1.0f / scale, 1.0f);
        
        if (screen_orientation) {
            rotate(screen_orientation, 0.0f, 0.0f, 1.0f);
        }
    }

    void MatrixMgr::multiply(const mat4& m)
    {
        currentMatrix() = currentMatrix().multiply(m);
    }
    
    void MatrixMgr::rotate(float angle, float x, float y, float z)
    {
        if (!angle) {
            return;
        }
        
        vec4 v = vec4(x, y, z, angle);
        currentMatrix().rotate(v);
    }
    
    void MatrixMgr::lookAt(vec3& eye, vec3& center, vec3& up)
    {
        vec3 f, s, u;
        
        mat4 mat;
        
        //	mat4_identity( &mat );
        mat.identity();
        
        //	vec3_diff( &f, center, eye );
        f = center.diff(eye);
        
        //	vec3_normalize( &f, &f );
        
        f.normalize();
        
        //	vec3_cross( &s, &f, up );
        s = f.cross(up);
        
        //	vec3_normalize( &s, &s );
        s.normalize();
        
        //	vec3_cross( &u, &s, &f );
        u = s.cross(f);
        
        mat.m[ 0 ].x = s.x;
        mat.m[ 1 ].x = s.y;
        mat.m[ 2 ].x = s.z;
        
        mat.m[ 0 ].y = u.x;
        mat.m[ 1 ].y = u.y;
        mat.m[ 2 ].y = u.z;
        
        mat.m[ 0 ].z = -f.x;
        mat.m[ 1 ].z = -f.y;
        mat.m[ 2 ].z = -f.z;
        
        multiply(mat);
        
        translate(-eye.x, -eye.y, -eye.z);
    }
    
    
    void MatrixMgr::translate(float x, float y, float z)
    {
        vec3 v = vec3(x, y, z);
        
        currentMatrix().translate(v);
    }
    
    void MatrixMgr::loadMatrix(mat4& m)
    {
        currentMatrix() = m;
    }
    
    
    void MatrixMgr::scale(float x, float y, float z)
    {
        static vec3 scale = vec3(1.0f, 1.0f, 1.0f);
        
        vec3 v = vec3(x, y, z);
        
        if( !memcmp( &v, &scale, sizeof( vec3 ) ) ) return;
        
        currentMatrix() = currentMatrix().scale(v);
    }
    
    
    /*!
     Return the modelview matrix pointer on the top of the modelview matrix stack.
     
     \return Return a 4x4 matrix pointer of the top most modelview matrix.
     */
    mat4& MatrixMgr::getModelViewMatrix(void)
    {
        return modelViewMatrixList.top();
    }
    
    
    /*!
     Return the projection matrix pointer on the top of the projection matrix stack.
     
     \return  Return a 4x4 matrix pointer of the top most projection matrix index.
     */
    mat4& MatrixMgr::getProjectionMatrix(void)
    {
        return projectionMatrixList.top();
    }
    
    
    /*!
     Return the texture matrix pointer on the top of the texture matrix stack.
     
     \return  Return a 4x4 matrix pointer of the top most texture matrix index.
     */
    mat4& MatrixMgr::getTextureMatrix(void)
    {
        return textureMatrixList.top();
    }
    
    
    /*!
     Return the result of the of the top most modelview matrix multiplied by the top
     most projection matrix.
     
     \return Return the 4x4 matrix pointer of the projection matrix index.
     */
    mat4& MatrixMgr::getModelViewProjectionMatrix(void)
    {
        modelViewProjectionMatrix = getProjectionMatrix().multiply(getModelViewMatrix());
        return modelViewProjectionMatrix;
    }
    
    
    /*!
     Return the result of the inverse and transposed operation of the top most modelview matrix applied
     on the rotation part of the matrix.
     
     \return Return the 3x3 matrix pointer that represent the invert and transpose
     result of the top most model view matrix.
     */
    mat3& MatrixMgr::getNormalMatrix(void)
    {
        mat4 mat = getModelViewMatrix();
        mat.invertFull();
        mat.transpose();
        normalMatrix.copy(mat);
        
        return normalMatrix;
    }
    
    /*!
     Map window coordinates to object coordinates.
     
     \param[in] winx Specify the window X coordinate.
     \param[in] winy Specify the window Y coordinate.
     \param[in] winz Specify the window Z coordinate.
     \param[in] modelview_matrix Specifies the current modelview matrix.
     \param[in] projection_matrix Specifies the current projection matrix.
     \param[in] viewport_matrix Specifies the current viewport matrix.
     \param[in] objx Return the computed X object coordinate.
     \param[in] objy Return the computed Y object coordinate.
     \param[in] objz Return the computed Z object coordinate.
     
     \sa http://www.opengl.org/sdk/docs/man/xhtml/gluUnProject.xml
     */
    int MatrixMgr::unproject( float winx, float winy, float winz, mat4 *modelview_matrix, mat4 *projection_matrix, int *viewport_matrix, float *objx, float *objy, float *objz )
    {
        mat4 final;
        
        vec4 vin,
        vout;
        
        final = projection_matrix->multiply(*modelview_matrix);

        final.invertFull();
        
        vin.x = winx;
        vin.y = winy;
        vin.z = winz;
        vin.w = 1.0f;
        
        vin.x = ( vin.x - viewport_matrix[ 0 ] ) / viewport_matrix[ 2 ];
        vin.y = ( vin.y - viewport_matrix[ 1 ] ) / viewport_matrix[ 3 ];
        
        vin.x = vin.x * 2.0f - 1.0f;
        vin.y = vin.y * 2.0f - 1.0f;
        vin.z = vin.z * 2.0f - 1.0f;
        
        vout = vin.multiply(final);
        
        
        if( !vout.w ) return 0;
        
        vout.x /= vout.w;
        vout.y /= vout.w;
        vout.z /= vout.w;
        
        *objx = vout.x;
        *objy = vout.y;
        *objz = vout.z;
        
        return 1;
    }
    
    /*!
     Map object coordinates to window coordinates.
     
     \param[in] objx Specify the object X coordinate.
     \param[in] objy Specify the object Y coordinate.
     \param[in] objz Specify the object Z coordinate.
     \param[in] modelview_matrix Specifies the current modelview matrix.
     \param[in] projection_matrix Specifies the current projection matrix.
     \param[in] viewport_matrix Specifies the current viewport matrix.
     \param[in] winx Return the computed X window coordinate.
     \param[in] winy Return the computed Y window coordinate.
     \param[in] winz Return the computed Z window coordinate.
     
     \sa http://www.opengl.org/sdk/docs/man/xhtml/gluProject.xml
     */
    int MatrixMgr::project( float objx, float objy, float objz, mat4 *modelview_matrix, mat4 *projection_matrix, int *viewport_matrix, float *winx, float *winy, float *winz )
    {
        vec4 vin,
        vout;
        
        vin.x = objx;
        vin.y = objy;
        vin.z = objz;
        vin.w = 1.0f;
        
        vout = vin.multiply(*modelview_matrix);
        vin = vout.multiply(*projection_matrix);
        
//        vec4_multiply_mat4( &vout, &vin, modelview_matrix );
//        
//        vec4_multiply_mat4( &vin, &vout, projection_matrix );
        
        if( !vin.w ) return 0;
        
        vin.x /= vin.w;
        vin.y /= vin.w;
        vin.z /= vin.w;
        
        vin.x = vin.x * 0.5f + 0.5f;
        vin.y = vin.y * 0.5f + 0.5f;
        vin.z = vin.z * 0.5f + 0.5f;
        
        vin.x = vin.x * viewport_matrix[ 2 ] + viewport_matrix[ 0 ];
        vin.y = vin.y * viewport_matrix[ 3 ] + viewport_matrix[ 1 ];
        
        *winx = vin.x;
        *winy = vin.y;
        *winz = vin.z;
        
        return 1;
    }
    
}
