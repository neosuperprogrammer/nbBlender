//
//  Material.cpp
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 2. 13..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#include "nbMaterial.h"
#include "nbHeader.h"

namespace nbd {
    void Material::init()
    {
        memset(name, 0, sizeof(name));
        
        illumination_model = 0;
        dissolve = 0;
        specular_exponent = 0;
        optical_density = 0;
        
        memset(map_ambient, 0, sizeof(map_ambient));
        texture_ambient = NULL;
        
        memset(map_diffuse, 0, sizeof(map_diffuse));
        texture_diffuse = NULL;
        
        memset(map_specular, 0, sizeof(map_specular));
        texture_specular = NULL;
        
        memset(map_translucency, 0, sizeof(map_translucency));
        texture_translucency = NULL;
        
        memset(map_disp, 0, sizeof(map_disp));
        texture_disp = NULL;
        
        memset(map_bump, 0, sizeof(map_bump));
        texture_bump = NULL;
        
        program = NULL;
        
        materialdrawcallback = NULL;
    }
    
    void Material::bindTexture(vector<Texture *> &textureList)
    {
        texture_ambient = getTexture(textureList, map_ambient);
        texture_diffuse = getTexture(textureList, map_diffuse);
        texture_specular = getTexture(textureList, map_specular);
        texture_translucency = getTexture(textureList, map_translucency);
        texture_disp = getTexture(textureList, map_disp);
        texture_bump = getTexture(textureList, map_bump);
    }
    
    bool Material::hasTexture(void)
    {
        return texture_ambient != NULL ||
        texture_diffuse != NULL ||
        texture_specular != NULL ||
        texture_translucency != NULL ||
        texture_disp != NULL ||
        texture_bump != NULL;
    }
    
    Texture *Material::getTexture(vector<Texture *> &textureList, char *filename)
    {
        if (strlen(filename) < 1) {
            return NULL;
        }
        
        for (int i = 0; i < textureList.size(); i++) {
            if (!strcmp(filename, textureList[i]->name)) {
                return textureList[i];
            }
        }
        return NULL;
    }
    
    void Material::draw()
    {
        if (program) {
            program->draw();
        }
        
        if (texture_ambient) {
            glLogVerbose("glActiveTexture : GL_TEXTURE0");
            glActiveTexture( GL_TEXTURE0 );
            texture_ambient->draw();
        }
        
        if (texture_diffuse) {
            glLogVerbose("glActiveTexture : GL_TEXTURE1");
            glActiveTexture( GL_TEXTURE1 );
            texture_diffuse->draw();
        }
        
        if (texture_specular) {
            glLogVerbose("glActiveTexture : GL_TEXTURE2");
            glActiveTexture( GL_TEXTURE2 );
            texture_specular->draw();
        }
        
        if (texture_disp) {
            glLogVerbose("glActiveTexture : GL_TEXTURE3");
            glActiveTexture( GL_TEXTURE3 );
            texture_disp->draw();
        }
        
        if (texture_bump){
            glLogVerbose("glActiveTexture : GL_TEXTURE4");
            glActiveTexture( GL_TEXTURE4 );
            texture_bump->draw();
        }
        
        if (texture_translucency) {
            glLogVerbose("glActiveTexture : GL_TEXTURE5");
            glActiveTexture( GL_TEXTURE5 );
            texture_translucency->draw();
        }
        
        if (materialdrawcallback) {
            materialdrawcallback(this);
        }
    }
}