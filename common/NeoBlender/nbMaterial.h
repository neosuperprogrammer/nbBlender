//
//  Material.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 2. 13..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__Material__
#define __NeoBlender__Material__

#include "nbVector.h"
#include "nbTexture.h"
#include "nbProgram.h"
#include <vector>

using namespace std;

namespace nbd {
    
    //! OBJMATERIAL draw callback function prototype.
    typedef void( MATERIALDRAWCALLBACK( void * ) );
    
    class Material {
        
    public:
        //! The material name.
        char					name[ MAX_CHAR ];				// newmtl
        
        //! The ambient color of the material.
        vec4					ambient;						// Ka
        
        //! The diffuse color of the material.
        vec4					diffuse;						// Kd
        
        //! The specular color of the material.
        vec4					specular;						// Ks
        
        //! The transmission filter of the material.
        vec3					transmission_filter;			// Tf
        
        //! The illumination model of the material.
        int						illumination_model;				// illum
        
        //! The material dissolve factor (aka Alpha).
        float					dissolve;						// d
        
        //! The specular exponent of the material (aka Hardness or Shiness).
        float					specular_exponent;				// Ns
        
        //! The optical density of the material.
        float					optical_density;				// Ni
        
        //! The ambient texture channel filename.
        char					map_ambient[ MAX_CHAR ];		// map_Ka
        
        //! The diffuse texture channel filename.
        char					map_diffuse[ MAX_CHAR ];		// map_Kd
        
        //! The specular texture channel filename.
        char					map_specular[ MAX_CHAR ];		// map_Ks
        
        //! The translucency texture channel filename.
        char					map_translucency[ MAX_CHAR ];	// map_Tr
        
        //! The displacement map texture channel filename.
        char					map_disp[ MAX_CHAR ];			// disp or map_disp
        
        //! The bump map (aka Normal Map) texture channel filename.
        char					map_bump[ MAX_CHAR ];			// bump or map_bump
        
        //! The ambient TEXTURE pointer.
        Texture					*texture_ambient;
        
        //! The diffuse TEXTURE pointer.
        Texture					*texture_diffuse;
        
        //! The specular TEXTURE pointer.
        Texture					*texture_specular;
        
        //! The translucency TEXTURE pointer.
        Texture					*texture_translucency;
        
        //! The displacement TEXTURE pointer.
        Texture					*texture_disp;
        
        //! The bumpmap TEXTURE pointer.
        Texture					*texture_bump;
        
        //! The shader PROGRAM to use when drawing the material.
        Program					*program;
        
        //! The material draw callback function pointer to use.
        MATERIALDRAWCALLBACK	*materialdrawcallback;
        
        
    public:
        Material() { init(); }
        virtual ~Material() { freeObj(); }
        
        void init();
        void freeObj() {}
        
        void bindTexture(vector<Texture *> &texureList);
        Texture *getTexture(vector<Texture *> &textureList, char *filename);
        bool hasTexture(void);

        void draw();
    };
}

#endif /* defined(__NeoBlender__Material__) */
