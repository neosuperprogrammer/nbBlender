//
//  nbManager.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 2. 8..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__nbManager__
#define __NeoBlender__nbManager__

#import <UIKit/UIKit.h>
#include "nbHeader.h"

using namespace nbd;

class nbManager {
    
public:
    static MatrixMgr *matrixMgr;
    
private:
    nbd::Object nbObj;
    
    int width, height;
    nbd::vec2 touche;
    nbd::vec3 rotAngle;;
    
public:
    nbManager() { init(); }
    virtual ~nbManager() { freeObj(); }
    
    void freeObj();
    
    void start(int width, int height);
    void initOpenGL();
    void draw(void);
    void drawMesh(Mesh *mesh);
    
    void toucheBegan( float x, float y, unsigned int tap_count );
    void toucheMoved( float x, float y, unsigned int tap_count );
    
private:
    void init();
    void loadObject(void);
    void error(void);
    
    static void programBindAttrLocation(void *ptr);
    static void materialDrawCallback(void *ptr);
    static void programDrawCallback(void *ptr);

};

#endif /* defined(__NeoBlender__nbManager__) */
