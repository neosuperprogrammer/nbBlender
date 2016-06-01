//
//  flShader.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 29..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__flShader__
#define __NeoBlender__flShader__

#include "nbTypes.h"

namespace nbd {
    
    class Shader {
        
    public:
        //! Internal name for the shader.
        char			name[ MAX_CHAR ];
        
        //! The type of shader. (Either GL_VERTEX_SHADER or GL_FRAGMENT_SHADER)
        unsigned int	type;
        
        //! The internal shader id maintain by GLES.
        unsigned int	sid;
        
    public:
        
        Shader(char *name, unsigned int type);
        
        virtual ~Shader();
        
        Shader *init( char *name, unsigned int type );
        
        Shader *free();
        
        unsigned char compile(const char *code, unsigned char debug );
        
        void deleteId();
        
    };
    
}

#endif /* defined(__NeoBlender__flShader__) */
