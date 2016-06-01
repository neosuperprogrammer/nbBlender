//
//  flShader.cpp
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 29..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#include "nbShader.h"
#include "nbHeader.h"

namespace nbd {
    
    Shader::Shader(char *name, unsigned int type) : sid(0)
    {
        strcpy( this->name, name );
        
        this->type = type;
    }
    
    
    Shader::~Shader()
    {
        deleteId();
    }
    
    void Shader::deleteId()
    {
        if (sid) {
            glLog("glDeleteShader");
            glDeleteShader(sid);
            sid = 0;
        }
    }
    
    unsigned char Shader::compile(const char *code, unsigned char debug )
    {
        char type[ MAX_CHAR ] = {""};
        
        int loglen,
        status;
        
        if (sid) {
            return 0;
        }
        
        glLog("glCreateShader : %d", this->type);
        sid = glCreateShader(this->type);
        
        glLog("glShaderSource");
        glShaderSource(sid, 1, &code, NULL);
        
        glLog("glCompileShader");
        glCompileShader(sid);
        
        if(debug) {
            if (this->type == GL_VERTEX_SHADER) {
                strcpy(type, "GL_VERTEX_SHADER");
            }
            else {
                strcpy(type, "GL_FRAGMENT_SHADER");
            }
            
            glLogVerbose("glGetShaderiv : GL_INFO_LOG_LENGTH");
            glGetShaderiv(sid, GL_INFO_LOG_LENGTH, &loglen);
            
            if (loglen) {
                char *log = (char *)malloc(loglen);
                
                glLogVerbose("glGetShaderInfoLog");
                glGetShaderInfoLog(sid, loglen, &loglen, log);
                
                printf("[ %s:%s ]\n%s", this->name, type, log );
                
                ::free(log);
            }
        }
        
        glLogVerbose("glGetShaderiv : GL_COMPILE_STATUS");
        glGetShaderiv(sid, GL_COMPILE_STATUS, &status);
        
        if (status == GL_FALSE) {
            deleteId();
            return 0;
        }
        
        return 1;
    }
    
}
