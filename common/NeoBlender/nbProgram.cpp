//
//  flProgram.cpp
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 29..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#include "nbProgram.h"
#include "nbHeader.h"

namespace nbd {
    
    Program::Program(char *filename)
    {
        strcpy(this->name, name);
        vertexShader = 0;
        fragmentShader = 0;
        pid = 0;
        //    uniform_count = 0;
        //    uniform_array = NULL;
        //    vertex_attrib_count = 0;
        //    vertex_attrib_array = NULL;
        programdrawcallback = NULL;
        programbindattribcallback = NULL;
    }
    
    Program::~Program()
    {
        if (vertexShader) {
            delete vertexShader;
            vertexShader = NULL;
        }
        
        if (fragmentShader) {
            delete fragmentShader;
            fragmentShader = NULL;
        }
        
        //    if (this->uniform_array) {
        //        free(this->uniform_array);
        //    }
        
        uniformList.clear();
        
        //    if (this->vertex_attrib_array) {
        //        free(this->vertex_attrib_array);
        //    }
        
        attributeList.clear();
        
        if (this->pid) {
            delete_id();
        }
    }
    
    Program *Program::create( char						  *name,
                             char						  *vertex_shader_filename,
                             char						  *fragment_shader_filename,
                             unsigned char				  relative_path,
                             unsigned char				  debug_shader,
                             PROGRAMBINDATTRIBCALLBACK	  *programbindattribcallback,
                             PROGRAMDRAWCALLBACK		  *programdrawcallback )
    {
        Program *program = new Program(name);
        
        Memory *m = Memory::open(vertex_shader_filename, relative_path);
        if (m) {
            program->vertexShader = new Shader(vertex_shader_filename, GL_VERTEX_SHADER);
            program->vertexShader->compile(( char * )m->buffer, debug_shader);
            delete m;
        }
        
        m = Memory::open(fragment_shader_filename, relative_path);
        if (m) {
            program->fragmentShader = new Shader(vertex_shader_filename, GL_FRAGMENT_SHADER);
            program->fragmentShader->compile(( char * )m->buffer, debug_shader);
            delete m;
        }
        
        program->programbindattribcallback = programbindattribcallback;
        
        program->programdrawcallback = programdrawcallback;
        
        program->link(debug_shader);
        
        return program;
    }
    
    unsigned char Program::link(unsigned char debug)
    {
        unsigned int i = 0,
        type;
        
        char *log,
        name[ MAX_CHAR ];
        
        int status,
        len,
        total,
        size;
        
        if (this->pid) {
            return 0;
        }
        
        glLog("glCreateProgram");
        this->pid = glCreateProgram();
        
        glLog("glAttachShader : vertexShader");
        glAttachShader(this->pid, this->vertexShader->sid);
        
        glLog("glAttachShader : fragmentShader");
        glAttachShader(this->pid, this->fragmentShader->sid);
        
        if(this->programbindattribcallback) {
            this->programbindattribcallback(this);
        }
        
        glLog("glLinkProgram");
        glLinkProgram(this->pid);
        
        if (debug) {
            glLogVerbose("glGetProgramiv : GL_INFO_LOG_LENGTH");
            glGetProgramiv(this->pid, GL_INFO_LOG_LENGTH, &len);
            
            if (len) {
                log = (char *) malloc(len);
                glLogVerbose("glGetProgramInfoLog");
                glGetProgramInfoLog(this->pid, len, &len, log);
                printf("[ %s ]\n%s", this->name, log);
                free(log);
            }
        }
        
        glLogVerbose("glGetProgramiv : GL_LINK_STATUS");
        glGetProgramiv(this->pid, GL_LINK_STATUS, &status);
        
        if (status == GL_FALSE) {
            goto delete_program;
        }
        
        if (debug) {
            glLogVerbose("glValidateProgram");
            glValidateProgram(this->pid);
            
            glLogVerbose("glGetProgramiv : GL_INFO_LOG_LENGTH");
            glGetProgramiv(this->pid, GL_INFO_LOG_LENGTH, &len);
            
            if (len) {
                log = (char *) malloc(len);
                glLogVerbose("glGetProgramInfoLog");
                glGetProgramInfoLog(this->pid, len, &len, log);
                printf("[ %s ]\n%s", this->name, log);
                free(log);
            }
            
            glLogVerbose("glGetProgramiv : GL_VALIDATE_STATUS");
            glGetProgramiv(this->pid, GL_VALIDATE_STATUS, &status);
            
            if (status == GL_FALSE) {
                goto delete_program;
            }
        }
        
        glLog("glGetProgramiv : GL_ACTIVE_ATTRIBUTES");
        glGetProgramiv(this->pid, GL_ACTIVE_ATTRIBUTES, &total);
        
        i = 0;
        while (i != total) {
            glGetActiveAttrib(this->pid,
                              i,
                              MAX_CHAR,
                              &len,
                              &size,
                              &type,
                              name);
            glLog("glGetActiveAttrib : %s", name);
            
            //        add_vertex_attrib(name, type);
            
            addVertexAttribute(name, type);
            
            ++i;
        }
        
        glLog("glGetProgramiv : GL_ACTIVE_UNIFORMS");
        glGetProgramiv(this->pid, GL_ACTIVE_UNIFORMS, &total);
        
        i = 0;
        while(i != total) {
            glGetActiveUniform(this->pid,
                               i,
                               MAX_CHAR,
                               &len,
                               &size,
                               &type,
                               name);
            glLog("glGetActiveUniform : %s", name);
            
            //        add_uniform(name, type);
            addUniform(name, type);
            
            ++i;
        }
        
        return 1;
        
        
    delete_program:
        
        delete_id();
        
        return 0;
    }
    
    void Program::set_draw_callback(PROGRAMDRAWCALLBACK *programdrawcallback)
    {
        this->programdrawcallback = programdrawcallback;
    }
    
    void Program::set_bind_attrib_location_callback(PROGRAMBINDATTRIBCALLBACK *programbindattribcallback)
    {
        this->programbindattribcallback = programbindattribcallback;
    }
    
    //char Program::get_vertex_attrib_location(char *name)
    //{
    //    unsigned char i = 0;
    //
    //    while (i != this->vertex_attrib_count) {
    //        if (!strcmp(this->vertex_attrib_array[i].name, name)) {
    //            return this->vertex_attrib_array[i].location;
    //        }
    //        ++i;
    //    }
    //
    //    return -1;
    //}
    
    char Program::getVertexAttribLocation(char *name)
    {
        for (int i = 0; i < attributeList.size(); i++) {
            if (!strcmp(attributeList[i].name, name)) {
                return attributeList[i].location;
            }
        }
        return -1;
    }
    
    //char Program::get_uniform_location(char *name)
    //{
    //    unsigned char i = 0;
    //
    //    while (i != this->uniform_count) {
    //        if(!strcmp(this->uniform_array[i].name, name)) {
    //            return this->uniform_array[i].location;
    //        }
    //        ++i;
    //    }
    //    return -1;
    //}
    
    char Program::getUniformLocation(char *name)
    {
        for (int i = 0; i < uniformList.size(); i++) {
            if (!strcmp(uniformList[i].name, name)) {
                return uniformList[i].location;
            }
        }
        return -1;
    }
    
    void Program::delete_id()
    {
        if (this->pid) {
            glLog("glDeleteProgram");
            glDeleteProgram(this->pid);
            this->pid = 0;
        }
    }
    
    void Program::draw()
    {
        glLogVerbose("Program::draw > glUseProgram");
        glUseProgram(this->pid);
        
        if(this->programdrawcallback) {
            this->programdrawcallback(this);
        }
    }
    
    //unsigned char Program::add_vertex_attrib(char *name, unsigned int type)
    //{
    //    unsigned char vertex_attrib_index = this->vertex_attrib_count;
    //
    //    ++this->vertex_attrib_count;
    //
    //    this->vertex_attrib_array = (VERTEX_ATTRIB *)realloc(this->vertex_attrib_array,
    //                                                               this->vertex_attrib_count * sizeof( VERTEX_ATTRIB));
    //
    //    memset(&this->vertex_attrib_array[vertex_attrib_index], 0, sizeof(VERTEX_ATTRIB));
    //
    //    strcpy(this->vertex_attrib_array[vertex_attrib_index].name, name);
    //
    //    this->vertex_attrib_array[vertex_attrib_index].type = type;
    //
    //    glLog("glGetAttribLocation : %s", name);
    //    this->vertex_attrib_array[vertex_attrib_index].location = glGetAttribLocation(this->pid, name);
    //
    //    return vertex_attrib_index;
    //}
    
    unsigned char Program::addVertexAttribute(char *name, unsigned int type)
    {
        VERTEX_ATTRIB attribute;
        memset(&attribute, 0, sizeof(attribute));
        strcpy(attribute.name, name);
        attribute.type = type;
        glLog("glGetAttribLocation : %s", name);
        attribute.location = glGetAttribLocation(this->pid, name);
        attributeList.push_back(attribute);
        
        return attributeList.size() - 1;
    }
    
    
    //unsigned char Program::add_uniform(char *name, unsigned int type)
    //{
    //    unsigned char uniform_index = this->uniform_count;
    //
    //    ++this->uniform_count;
    //
    //    this->uniform_array = (UNIFORM *)realloc(this->uniform_array,
    //                                                   this->uniform_count * sizeof(UNIFORM));
    //
    //    memset(&this->uniform_array[ uniform_index ], 0, sizeof(UNIFORM));
    //
    //    strcpy(this->uniform_array[ uniform_index ].name, name);
    //
    //    this->uniform_array[ uniform_index ].type = type;
    //
    //    glLog("glGetUniformLocation : %s", name);
    //    this->uniform_array[ uniform_index ].location = glGetUniformLocation(this->pid, name);
    //
    //    return uniform_index;
    //}
    
    unsigned char Program::addUniform(char *name, unsigned int type)
    {
        UNIFORM uniform;
        memset(&uniform, 0, sizeof(uniform));
        strcpy(uniform.name, name);
        uniform.type = type;
        glLog("glGetUniformLocation : %s", name);
        uniform.location = glGetUniformLocation(this->pid, name);
        uniformList.push_back(uniform);
        
        return uniformList.size() - 1;
    }
    
    unsigned char Program::load_gfx(PROGRAMBINDATTRIBCALLBACK	*programbindattribcallback,
                                    PROGRAMDRAWCALLBACK			*programdrawcallback,
                                    char						*filename,
                                    unsigned char				 debug_shader,
                                    unsigned char				 relative_path)
    {
        Memory *m = Memory::open(filename, relative_path);
        
        if (m) {
            char vertex_token [MAX_CHAR] = { "GL_VERTEX_SHADER" },
            fragment_token[MAX_CHAR] = { "GL_FRAGMENT_SHADER" },
            *vertex_shader				= strstr((char *)m->buffer, vertex_token),
            *fragment_shader			= strstr((char *)m->buffer, fragment_token);
            
            get_file_name(filename, this->name);
            
            if ((vertex_shader && fragment_shader) && (fragment_shader > vertex_shader)) {
                this->vertexShader = new Shader(this->name, GL_VERTEX_SHADER);
                vertex_shader += strlen(vertex_token);
                *fragment_shader = 0;
                this->vertexShader->compile(vertex_shader, debug_shader);
                this->fragmentShader = new Shader(this->name, GL_FRAGMENT_SHADER);
                fragment_shader += strlen( fragment_token );
                this->fragmentShader->compile(fragment_shader, debug_shader);
                this->programbindattribcallback = programbindattribcallback;
                this->programdrawcallback = programdrawcallback;
                link(debug_shader);
            }
            delete m;
            
            return 1;
        }
        
        return 0;
    }
    
}

