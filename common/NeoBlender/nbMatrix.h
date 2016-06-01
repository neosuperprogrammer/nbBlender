//
//  flMatrix.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 30..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__flMatrix__
#define __NeoBlender__flMatrix__

#include "nbVector.h"

namespace nbd {
    class mat3 {
        
    public:
        vec3 m[3];
    public:
        
        mat3();
        //    virtual ~mat3() {}
        
        void identity();
        void copy(const mat4&m4);
    };
    
    class mat4 {
        
    public:
        vec4 m[4];
    public:
        
        mat4();
        //    virtual ~mat4() {}
        
        void identity();
        void copy(const mat4& m4);
        void copy(const mat3& m3);
        void translate(mat4& dst, const vec3& v) const;
        void translate(const vec3& v);
        void rotateFast(const vec4& v);
        void rotate(const vec4& v);
        mat4 multiply(const mat4& m1) const;
        mat3 multiply(const mat3& m1) const;
        mat4 scale(const vec3& v) const;
        unsigned char invert();
        unsigned char invertFull();
        void ortho(float left, float right, float bottom, float top, float clip_start, float clip_end );
        
        void transpose();
        
    };
}

#endif /* defined(__NeoBlender__flMatrix__) */
