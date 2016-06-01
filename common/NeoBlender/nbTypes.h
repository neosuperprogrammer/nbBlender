//
//  nbTypes.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 2. 13..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef NeoBlender_nbTypes_h
#define NeoBlender_nbTypes_h
namespace nbd {
    
#define MAX_CHAR	64
    
#define MAX_PATH	256
    
#define DEG_TO_RAD	M_PI / 180.0f
    
#define RAD_TO_DEG	90.0f / M_PI
    
#define BUFFER_OFFSET( x ) ( ( char * )NULL + x )
    
#define CLAMP( x, low, high ) ( ( x > high ) ? high : ( ( x < low ) ? low : x ) )
    
    
    //! RGBA color component as unsigned char.
    typedef struct
    {
        unsigned char r;
        unsigned char g;
        unsigned char b;
        unsigned char a;
        
    } ucol4;
    
}

#endif
