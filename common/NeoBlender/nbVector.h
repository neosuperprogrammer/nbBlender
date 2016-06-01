//
//  flVector.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 30..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__flVector__
#define __NeoBlender__flVector__

namespace nbd {

class vec3;

class vec2 {
    
public:
    float x;
    float y;
public:
    vec2() : x(0), y(0) {}
    vec2(float x, float y);
//    virtual ~vec2() {}
//    const float *Pointer() const { return &x; }
//    int Size() const { return sizeof(x) * 2; }
    vec2 add(const vec2& v) const;
    vec2 diff(const vec2& v) const;
    void operator=(const vec3& v3);


};

class vec4;
class mat3;
class mat4;

class vec3 {
    
public:
    float x;
    float y;
    float z;
public:
    vec3() : x(0), y(0), z(0) {}
    vec3(float x, float y, float z);
//    virtual ~vec3() {}
    void clear();
//    const float *Pointer() const { return &x; }
//    int Size() const { return sizeof(x) * 3; }
    vec3 add(const vec3& v) const;
    vec3 diff(const vec3& v) const;
    vec3 mul(const vec3&v) const;
    float dot() const;
    float dot(const vec3&v) const;
    float length() const;
    float normalize();
    vec3 normalized() const;
    vec3 cross(const vec3&v) const;
    float dist(const vec3&v) const;
    vec3 mid(const vec3&v) const;
    vec3 invert() const;
    vec3 lerp(const vec3&v, float t) const;
    vec3 lerp2(const vec3&v, float t) const;
    vec3 rotate(const vec4&v) const;
    vec3 toRecast() const;
    vec3 recastTo() const;
    vec3 multiply(const mat3&m) const;
    vec3 multiply(const mat4&m) const;

};

class vec4 {
    
public:
    float x;
    float y;
    float z;
    float w;
public:
    vec4() : x(0), y(0), z(0), w(0) {}
    vec4(float x, float y, float z, float w);
//    virtual ~vec4() {}
//    const float *Pointer() const { return &x; }
//    int Size() const { return sizeof(x) * 4; }
    void operator=(const vec3& v3);
    vec4 add(const vec4& v) const;
    vec4 diff(const vec4& v) const;
    void buildW();
    float dot() const;
    float dot(const vec4&v) const;
    float length() const;
    float normalize();
    vec4 normalized() const;
    vec4 multiply(const vec3&v) const;
    vec4 multiply(const vec4&v) const;
    vec4 conjugate() const;
    vec4 invert() const;
    vec4 lerp(const vec4&v, float t) const;
    vec4 slerp(const vec4&v, float t) const;
    vec4 multiply(const mat4&m) const;


};
    
}


#endif /* defined(__NeoBlender__flVector__) */
