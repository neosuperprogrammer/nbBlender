//
//  flProgram.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 29..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__flProgram__
#define __NeoBlender__flProgram__

#include "nbTypes.h"
#include "nbShader.h"
#include <vector>

using namespace std;

namespace nbd {
    
    //! Structure to deal with GLSL uniform variables.
    typedef struct
    {
        //! The name of the uniform.
        char			name[ MAX_CHAR ];
        
        //! The variable type for this uniform.
        unsigned int	type;
        
        //! The location id maintained by GLSL for this uniform.
        int				location;
        
        //! Determine if the uniform is constant or shoud be updated every frame.
        unsigned char	constant;
        
    } UNIFORM;
    
    
    //! Structure to deal with vertex attribute variables.
    typedef struct
    {
        //! The name of the vertex attribute.
        char			name[ MAX_CHAR ];
        
        //! The variable type for this vertex attribute.
        unsigned int	type;
        
        //! The location of the id maintained GLSL for this vertex attribute.
        int				location;
        
    } VERTEX_ATTRIB;
    
    
    //! Draw callback function prototype.
    typedef void( PROGRAMDRAWCALLBACK( void * ) );
    
    //! Callback function prototypes for binding vertex attribute before compilation.
    typedef void( PROGRAMBINDATTRIBCALLBACK( void * ) );
    
    
    class Program {
        
    public:
        //! The internal name to use for the shader program.
        char						 name[ MAX_CHAR ];
        
        //! The vertex shader.
        //	SHADER						 *vertex_shader;
        Shader                     *vertexShader;
        
        //! The fragment shader.
        //	SHADER						 *fragment_shader;
        Shader                     *fragmentShader;
        
        //! The internal GLES program id for the shader program.
        unsigned int				 pid;
        
        //    //! The number of uniforms.
        //    unsigned char				 uniform_count;
        //
        //    //! Array of UNIFORM variables.
        //    UNIFORM						 *uniform_array;
        
        vector<UNIFORM> uniformList;
        
        //    //! The number of vertex attributes.
        //    unsigned char				 vertex_attrib_count;
        //
        //    //! Array of vertex attributes.
        //    VERTEX_ATTRIB				 *vertex_attrib_array;
        
        vector<VERTEX_ATTRIB> attributeList;
        
        //! The program draw callback.
        PROGRAMDRAWCALLBACK			 *programdrawcallback;
        
        //! The bind attribute callback.
        PROGRAMBINDATTRIBCALLBACK	 *programbindattribcallback;
        
        
    public:
        
        Program(char *name);
        
        virtual ~Program();
        
        static Program *create(char *name, char *vertex_shader_filename, char *fragment_shader_filename, unsigned char relative_path, unsigned char debug_shader, PROGRAMBINDATTRIBCALLBACK *programbindattribcallback, PROGRAMDRAWCALLBACK *programdrawcallback);
        
        unsigned char link(unsigned char debug);
        
        void set_draw_callback(PROGRAMDRAWCALLBACK *programdrawcallback);
        
        void set_bind_attrib_location_callback(PROGRAMBINDATTRIBCALLBACK *programbindattribcallback);
        
        //    char get_vertex_attrib_location(char *name);
        char getVertexAttribLocation(char *name);
        
        //    char get_uniform_location(char *name);
        char getUniformLocation(char *name);
        
        void delete_id();
        
        void draw();
        
        //    unsigned char add_vertex_attrib(char *name, unsigned int type);
        
        unsigned char addVertexAttribute(char *name, unsigned int type);
        
        //    unsigned char add_uniform(char *name, unsigned int type);
        unsigned char addUniform(char *name, unsigned int type);
        
        unsigned char load_gfx(PROGRAMBINDATTRIBCALLBACK	*programbindattribcallback, PROGRAMDRAWCALLBACK	*programdrawcallback, char *filename, unsigned char	debug_shader, unsigned char relative_path );
        
    };
    
}

#endif /* defined(__NeoBlender__flProgram__) */
