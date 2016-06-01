//
//  Object.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 2. 13..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__Object__
#define __NeoBlender__Object__

#include "nbMesh.h"
#include <vector>

using namespace std;

namespace nbd {
    class Object {
        
    private:
        //! The texture path (By default relative to the location of the .mtl file).
        char			texturePath[ MAX_PATH ];
        
    public:
        vector<Mesh *> meshList;
        vector<Texture *> textureList;
        vector<Program *> programList;
        vector<Material *> materialList;
        VertexMgr vertexMgr;
        
    public:
        Object() { init(); }
        virtual ~Object() { freeObj(); }
        
        void init();
        void freeObj();
        void freeVertextData();
        
        void loadObjFile(char *filename, unsigned char relative_path);
        bool justLoadObjFile(char *filename, unsigned char relative_path);
        void buildMaterial(char *vertexShaderFile, char *fragmentShaderFile,
                           PROGRAMBINDATTRIBCALLBACK *programbindattribcallback,
                           MATERIALDRAWCALLBACK *materialdrawcallback);
        void buildMaterial();

        void buildNormalAndTangent();
        bool _loadObjFile(char *filename, unsigned char relative_path);
        Material *getMaterial(const char *name, bool exactName);
        
        unsigned char load_mtl(char *filename, unsigned char relative_path);
        
        void addProgram(char *filename);
        int getProgramIndex(char *filename);
        Program *getProgram(const char *name, unsigned char exactName);
        
        Mesh *getMesh(const char *name, bool exactName );
        int get_mesh_index(const char *name, unsigned char exact_name );
        Texture *get_texture(const char *name, unsigned char exact_name );
        
        void addTexture(char *filename);
        int getTextureIndex(char *filename);
    };
}


#endif /* defined(__NeoBlender__Object__) */
