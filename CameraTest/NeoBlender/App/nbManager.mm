//
//  nbManager.cpp
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 2. 8..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#include "nbManager.h"

#include "btBulletDynamicsCommon.h"
#include "btSoftRigidDynamicsWorld.h"
#include "btSoftBodyRigidBodyCollisionConfiguration.h"
#include "btShapeHull.h"
#include "btSoftBodyHelpers.h"
#include "btSoftBody.h"
#include "btGImpactShape.h"
#include "btGImpactCollisionAlgorithm.h"
#include "btBulletWorldImporter.h"
#include "nbUtils.h"

#define OBJ_FILE                ( char * )"Scene.obj"
#define VERTEX_SHADER_FILE      ( char * )"vertex.glsl"
#define FRAGMENT_SHADER_FILE    ( char * )"fragment.glsl"

MatrixMgr *nbManager::matrixMgr;

nbd::vec3 location = { 0.0f, 0.0f, 1.84f };

float rotz = 0.0f, rotx = 90.0f;

nbd::vec4 frustum[ 6 ];

float screen_size = 0.0f;

nbd::vec2 view_location,
view_delta = { 0.0f, 0.0f };

nbd::vec3 move_location = { 0.0f, 0.0f, 0.0f },
move_delta;


void nbManager::init()
{
    matrixMgr = new MatrixMgr();
    program = NULL;
    obj = new Object;
}

void nbManager::freeObj()
{
    delete obj;
    delete matrixMgr;
    delete program;
}

void nbManager::start(int width, int height)
{
    this->width = width;
    this->height = height;
    
    screen_size = ( width > height ) ? width : height;
    
    initOpenGL();
    
    matrixMgr->setMatrixMode(kProjectionMatrix);
    matrixMgr->loadIdentity();
    matrixMgr->setPerspective(80.0f,
                              (float)width / (float)height,
                              0.1f,
                              100.0f,
                              -90.0f );
    loadObject();
}


void nbManager::loadObject(void)
{
    obj->justLoadObjFile(OBJ_FILE, 1);
    
    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
        mesh->build(&obj->vertexMgr);
        mesh->freeVertexData();
    }
    
    obj->freeVertextData();
    
    for (int i = 0; i < obj->textureList.size(); i++) {
        Texture *texture = obj->textureList[i];
        texture->build(TEXTURE_MIPMAP, TEXTURE_FILTER_2X, 0.0f);
    }
    
    obj->buildMaterial();
    
    program = Program::create((char *)"default", VERTEX_SHADER_FILE, FRAGMENT_SHADER_FILE, 1, 1, nbManager::programBindAttrLocation, NULL);
    
    program->draw();
    
    glUniform1i( program->getUniformLocation(( char * )"DIFFUSE" ), 1 );
}

void nbManager::draw(void)
{
    glLogVerbose("BlenderApp::Draw");
    
    glLogVerbose("glClearColor");
    glClearColor( 0.5f, 0.5f, 0.5f, 1.0f );
    glLogVerbose("glClear");
    glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
    
    glLogVerbose("glViewport");
    glViewport( 0.0f, 0.0f, width, height);
    
    matrixMgr->setMatrixMode(kModelViewMatrix);
    matrixMgr->loadIdentity();
    
    
    if( view_delta.x || view_delta.y ) {
        
        if( view_delta.y ) {
            rotz -= view_delta.y;
        }
        
        if( view_delta.x ) {
            rotx += view_delta.x;
            rotx = CLAMP( rotx, 0.0f, 180.0f );
        }
        
        view_delta.x =
        view_delta.y = 0.0f;
    }
    
    
    if( move_delta.z )
    {
        nbd::vec3 forward;
        
        float r = rotz * DEG_TO_RAD,
        c = cosf( r ),
        s = sinf( r );
        
        forward.x = c * move_delta.y - s * move_delta.x;
        forward.y = s * move_delta.y + c * move_delta.x;
        
        location.x += forward.x * move_delta.z * 0.1f;
        location.y += forward.y * move_delta.z * 0.1f;
        
        forward.z = sinf( ( rotx - 90.0f ) * DEG_TO_RAD );
        
        if( move_delta.x < -0.99f )
            location.z -= forward.z * move_delta.z * 0.1f;
        
        else if( move_delta.x > 0.99f )
            location.z += forward.z * move_delta.z * 0.1f;
    }

    matrixMgr->translate(location.x,
                         location.y,
                         location.z);
    
    matrixMgr->rotate(rotz, 0.0f, 0.0f, 1.0f );
    
    matrixMgr->rotate(rotx, 1.0f, 0.0f, 0.0f );
    
    matrixMgr->getModelViewMatrix().invert();
    
    nbd::build_frustum(frustum, &matrixMgr->getModelViewMatrix(), &matrixMgr->getProjectionMatrix());

    
    unsigned int n = 0;
    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
        
        mesh->distance = nbd::sphere_distance_in_frustum(frustum,
                                                       &mesh->location,
                                                       mesh->radius);
        if( mesh->distance ) {
            matrixMgr->pushMatrix();
            
            matrixMgr->translate(mesh->location.x,
                                 mesh->location.y,
                                 mesh->location.z );
            
            glUniformMatrix4fv( program->getUniformLocation((char *)"MODELVIEWPROJECTIONMATRIX"),
                               1,
                               GL_FALSE,
                               ( float * )&matrixMgr->getModelViewProjectionMatrix() );
            
            mesh->draw();
            
            matrixMgr->popMatrix();
            n++;
        }
    }
    
    console_print( "Visible Objects: %d from total : %d\n", n, obj->meshList.size());
}

void nbManager::toucheBegan( float x, float y, unsigned int tap_count )
{
    if( y < ( screen_size * 0.5f ) )
    {
        move_location.x = x;
        move_location.y = y;
    }
    else
    {
        view_location.x = x;
        view_location.y = y;
    }
}

void nbManager::toucheMoved( float x, float y, unsigned int tap_count )
{
    if (y > ( ( screen_size * 0.5f ) - ( screen_size * 0.05f ) ) &&
       y < ( ( screen_size * 0.5f ) + ( screen_size * 0.05f ) ) )
    {
        move_delta.z =
        view_delta.x =
        view_delta.y = 0.0f;
        
        move_location.x = x;
        move_location.y = y;
        
        view_location.x = x;
        view_location.y = y;
    }
    else if ( y < ( screen_size * 0.5f ) ) {
        
        nbd::vec3 touche = {x, y, 0.0f};
        
        move_delta = touche.diff(move_location);
        move_delta.normalize();
        
        move_delta.z = CLAMP( move_location.dist(touche) / 128.0f,
                             0.0f,
                             1.0f );
    }
    else {
        
        view_delta.x = view_delta.x * 0.75f + ( x - view_location.x ) * 0.25f;
        view_delta.y = view_delta.y * 0.75f + ( y - view_location.y ) * 0.25f;
        
        view_location.x = x;
        view_location.y = y;
    }
}

void nbManager::toucheEnded( float x, float y, unsigned int tap_count )
{
	move_delta.z = 0.0f;
}

void nbManager::error(void)
{
    unsigned int error;
    
    glLog("glGetError");
    while( ( error = glGetError() ) != GL_NO_ERROR )
    {
        char str[ MAX_CHAR ] = {""};
        
        switch( error ) {
            case GL_INVALID_ENUM: {
                strcpy( str, "GL_INVALID_ENUM" );
                break;
            }
                
            case GL_INVALID_VALUE:
            {
                strcpy( str, "GL_INVALID_VALUE" );
                break;
            }
                
            case GL_INVALID_OPERATION:
            {
                strcpy( str, "GL_INVALID_OPERATION" );
                break;
            }
                
            case GL_OUT_OF_MEMORY:
            {
                strcpy( str, "GL_OUT_OF_MEMORY" );
                break;
            }
        }
        
        console_print( "[ GL_ERROR ]\nERROR: %s\n", str );
    }
}

void nbManager::initOpenGL()
{
    //    printf("\nGL_VENDOR:      %s\n", ( char * )glGetString( GL_VENDOR     ) );
    //    printf("GL_RENDERER:    %s\n"  , ( char * )glGetString( GL_RENDERER   ) );
    //    printf("GL_VERSION:     %s\n"  , ( char * )glGetString( GL_VERSION    ) );
    //    printf("GL_EXTENSIONS:  %s\n"  , ( char * )glGetString( GL_EXTENSIONS ) );
    //
    //    glLog("glHint : GL_GENERATE_MIPMAP_HINT");
    //    glHint( GL_GENERATE_MIPMAP_HINT, GL_NICEST );
    //
    //    glLog("glHint : GL_FRAGMENT_SHADER_DERIVATIVE_HINT_OES");
    //    glHint( GL_FRAGMENT_SHADER_DERIVATIVE_HINT_OES, GL_NICEST );
    
    glLog("glEnable : GL_DEPTH_TEST");
    glEnable( GL_DEPTH_TEST );
    glLog("glEnable : GL_CULL_FACE");
    glEnable( GL_CULL_FACE  );
    glLog("glDisable : GL_DITHER");
    glDisable( GL_DITHER );
    //    glLog("glDepthMask : GL_TRUE");
    //    glDepthMask( GL_TRUE );
    //    glLog("glDepthFunc : GL_LESS");
    //    glDepthFunc( GL_LESS );
    //    glLog("glDepthRangef : 0.0f, 1.0f");
    //    glDepthRangef( 0.0f, 1.0f );
    //    glLog("glClearDepthf : 1.0f");
    //    glClearDepthf( 1.0f );
    //    glLog("glCullFace : GL_BACK");
    //    glCullFace ( GL_BACK );
    //    glLog("glFrontFace : GL_CCW");
    //    glFrontFace( GL_CCW  );
    //    glLog("glClearStencil : 0");
    //    glClearStencil( 0 );
    //    glLog("glStencilMask : 0xFFFFFFFF");
    //    glStencilMask( 0xFFFFFFFF );
    
    //    glLog("glClearColor : 0.0f, 0.0f, 0.0f, 1.0f");
    //    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
    
    //    glLog("glClear : GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT");
    //    glClear( GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
    
    //    glLog("glClear : GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT");
    //    glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
    
    //    matrixMgr->setMatrixMode(kTextureMatrix);
    //    matrixMgr->loadIdentity();
    //
    //    matrixMgr->setMatrixMode(kProjectionMatrix);
    //    matrixMgr->loadIdentity();
    //
    //    matrixMgr->setMatrixMode(kModelViewMatrix);
    //    matrixMgr->loadIdentity();
    
    error();
}

void nbManager::programBindAttrLocation(void *ptr)
{
    Program *program = (Program *)ptr;
    
    glLog("glBindAttribLocation : POSITION");
    glBindAttribLocation(program->pid, 0, "POSITION");
    glLog("glBindAttribLocation : NORMAL");
    glBindAttribLocation(program->pid, 1, "NORMAL");
    glLog("glBindAttribLocation : TEXCOORD0");
    glBindAttribLocation(program->pid, 2, "TEXCOORD0");
}

void nbManager::materialDrawCallback(void *ptr)
{
    glLogVerbose("material_draw_callback");
    
    Material *objmaterial = ( Material * )ptr;
    Program *program = objmaterial->program;
    
    nbManager::programDrawCallback(program);
    
    for (int i = 0; i < program->uniformList.size(); i++) {
        UNIFORM uniform = program->uniformList[i];
//        if( !strcmp( uniform.name, "DIFFUSE" ) ) {
//            glLogVerbose("glUniform1i : DIFFUSE");
//            glUniform1i( uniform.location, 1 );
//        }
//        
//        else if( !strcmp( uniform.name, "MODELVIEWPROJECTIONMATRIX" ) ) {
//            glLogVerbose("glUniformMatrix4fv : MODELVIEWPROJECTIONMATRIX");
//            glUniformMatrix4fv( uniform.location,
//                               1,
//                               GL_FALSE,
//                               ( float * )&nbManager::matrixMgr->getModelViewProjectionMatrix());
//        }
        
        if( !strcmp( uniform.name, "DISSOLVE" ) ) {
            glLogVerbose("glUniform1f : DISSOLVE");
            glUniform1f( uniform.location, objmaterial->dissolve );
        }
        
        else if( !strcmp( uniform.name, "AMBIENT_COLOR" ) ) {
            glLogVerbose("glUniform3fv : AMBIENT_COLOR");
            glUniform3fv( uniform.location,
                         1,
                         ( float * )&objmaterial->ambient );
        }
        
        else if( !strcmp( uniform.name, "DIFFUSE_COLOR" ) ) {
            
            //            printf("\ndiffuse, r[%f], g[%f], b[%f]\n", objmaterial->diffuse.x, objmaterial->diffuse.y,objmaterial->diffuse.z);
            glLogVerbose("glUniform3fv : DIFFUSE_COLOR");
            glUniform3fv( uniform.location,
                         1,
                         ( float * )&objmaterial->diffuse );
            
            
        }
        
        else if( !strcmp( uniform.name, "SPECULAR_COLOR" ) ) {
            glLogVerbose("glUniform3fv : SPECULAR_COLOR");
            glUniform3fv( uniform.location,
                         1,
                         ( float * )&objmaterial->specular );
        }
        
        else if( !strcmp( uniform.name, "SHININESS" ) ) {
            glLogVerbose("glUniform1f : SHININESS");
            glUniform1f( uniform.location,
                        objmaterial->specular_exponent * 0.128f );
        }
        
//        else if( !strcmp( uniform.name, "MODELVIEWMATRIX" ) ) {
//            glLogVerbose("glUniformMatrix4fv : MODELVIEWMATRIX");
//            glUniformMatrix4fv( uniform.location,
//                               1,
//                               GL_FALSE,
//                               ( float * )&nbManager::matrixMgr->getModelViewMatrix());
//        }
//        
//        else if( !strcmp( uniform.name, "PROJECTIONMATRIX" ) ) {
//            glLogVerbose("glUniformMatrix4fv : PROJECTIONMATRIX");
//            glUniformMatrix4fv( uniform.location,
//                               1,
//                               GL_FALSE,
//                               ( float * )&nbManager::matrixMgr->getProjectionMatrix());
//        }
//        
//        else if( !strcmp( uniform.name, "NORMALMATRIX" ) ) {
//            glLogVerbose("glUniformMatrix3fv : NORMALMATRIX");
//            glUniformMatrix3fv( uniform.location,
//                               1,
//                               GL_FALSE,
//                               ( float * )&nbManager::matrixMgr->getNormalMatrix());
//        }
//        
//        else if( !strcmp( uniform.name, "LIGHTPOSITION" ) ) {
//            
//            nbd::vec3 position    = { 0.0f, -3.0f, 10.0f };
//            nbd::vec3 eyeposition = { 0.0f,  0.0f,  0.0f };
//            
//            eyeposition = position.multiply(nbManager::matrixMgr->getModelViewMatrix());
//            
//            glLogVerbose("glUniform3fv : LIGHTPOSITION");
//            glUniform3fv( uniform.location,
//                         1,
//                         ( float * )&eyeposition );
//        }
    }
}

void nbManager::programDrawCallback(void *ptr)
{
    glLogVerbose("material_draw_callback");
    
    Program *program = ( Program * )ptr;
    
    for (int i = 0; i < program->uniformList.size(); i++) {
        UNIFORM uniform = program->uniformList[i];
        if( !strcmp( uniform.name, "DIFFUSE" ) ) {
            glLogVerbose("glUniform1i : DIFFUSE");
            glUniform1i( uniform.location, 1 );
        }
        
        else if( !strcmp( uniform.name, "MODELVIEWPROJECTIONMATRIX" ) ) {
            glLogVerbose("glUniformMatrix4fv : MODELVIEWPROJECTIONMATRIX");
            glUniformMatrix4fv( uniform.location,
                               1,
                               GL_FALSE,
                               ( float * )&nbManager::matrixMgr->getModelViewProjectionMatrix());
        }
        
        
        else if( !strcmp( uniform.name, "MODELVIEWMATRIX" ) ) {
            glLogVerbose("glUniformMatrix4fv : MODELVIEWMATRIX");
            glUniformMatrix4fv( uniform.location,
                               1,
                               GL_FALSE,
                               ( float * )&nbManager::matrixMgr->getModelViewMatrix());
        }
        
        else if( !strcmp( uniform.name, "PROJECTIONMATRIX" ) ) {
            glLogVerbose("glUniformMatrix4fv : PROJECTIONMATRIX");
            glUniformMatrix4fv( uniform.location,
                               1,
                               GL_FALSE,
                               ( float * )&nbManager::matrixMgr->getProjectionMatrix());
        }
        
        else if( !strcmp( uniform.name, "NORMALMATRIX" ) ) {
            glLogVerbose("glUniformMatrix3fv : NORMALMATRIX");
            glUniformMatrix3fv( uniform.location,
                               1,
                               GL_FALSE,
                               ( float * )&nbManager::matrixMgr->getNormalMatrix());
        }
        
        else if( !strcmp( uniform.name, "LIGHTPOSITION" ) ) {
            
            nbd::vec3 position    = { 0.0f, -3.0f, 10.0f };
            nbd::vec3 eyeposition = { 0.0f,  0.0f,  0.0f };
            
            eyeposition = position.multiply(nbManager::matrixMgr->getModelViewMatrix());
            
            glLogVerbose("glUniform3fv : LIGHTPOSITION");
            glUniform3fv( uniform.location,
                         1,
                         ( float * )&eyeposition );
        }
    }
}

void nbManager::drawMesh(Mesh *mesh)
{
    mesh->draw();
    
#if 0
    OBJMATERIAL *material = mesh->objmaterial;
    
    glLogVerbose("Program::draw > glUseProgram");
    glUseProgram(material->program->pid);
    
    
    GLuint vertex = material->program->get_vertex_attrib_location((char *)"POSITION");
    GLuint normal = material->program->get_vertex_attrib_location((char *)"NORMAL");
    GLuint uv = material->program->get_vertex_attrib_location((char *)"TEXCOORD0");
    
    glLog("glBindBuffer : GL_ARRAY_BUFFER");
    glBindBuffer( GL_ARRAY_BUFFER, mesh->vboVertex );
    
    
    glLog("glEnableVertexAttribArray : 0");
    glEnableVertexAttribArray( vertex );
    
    glLog("glVertexAttribPointer : 0");
    glVertexAttribPointer(vertex,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          mesh->stride,
                          ( void * )NULL );
    
    
    glLog("glEnableVertexAttribArray : 1");
    glEnableVertexAttribArray( normal );
    
    glLog("glVertexAttribPointer : 1");
    glVertexAttribPointer(normal,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          mesh->stride,
                          BUFFER_OFFSET( mesh->offset[ 1 ] ) );
    
    glLog("glEnableVertexAttribArray : 2");
    glEnableVertexAttribArray( uv );
    
    glLog("glVertexAttribPointer : 2");
    glVertexAttribPointer(uv,
                          2,
                          GL_FLOAT,
                          GL_FALSE,
                          mesh->stride,
                          BUFFER_OFFSET( mesh->offset[ 3 ] ) );
    
    if (material->texture_diffuse) {
        glLogVerbose("glActiveTexture : GL_TEXTURE1");
        glActiveTexture( GL_TEXTURE1 );
        
        glLogVerbose("Texture::draw > glBindTexture");
        glBindTexture( material->texture_diffuse->target, material->texture_diffuse->tid );
    }
    
    GLuint diffuse = material->program->get_uniform_location((char *)"DIFFUSE");
    GLuint dissolve = material->program->get_uniform_location((char *)"DISSOLVE");
    GLuint ambientColor = material->program->get_uniform_location((char *)"AMBIENT_COLOR");
    GLuint diffuseColor = material->program->get_uniform_location((char *)"DIFFUSE_COLOR");
    GLuint specularColor = material->program->get_uniform_location((char *)"SPECULAR_COLOR");
    GLuint shineness = material->program->get_uniform_location((char *)"SHININESS");
    GLuint modelView = material->program->get_uniform_location((char *)"MODELVIEWMATRIX");
    GLuint projection = material->program->get_uniform_location((char *)"PROJECTIONMATRIX");
    GLuint normalMatrix = material->program->get_uniform_location((char *)"NORMALMATRIX");
    GLuint light = material->program->get_uniform_location((char *)"LIGHTPOSITION");
    
    glUniform1i(diffuse, 1);
    glUniform1f(dissolve, material->dissolve);
    glUniform3fv(ambientColor, 1, (float *)&material->ambient);
    glUniform3fv(diffuseColor, 1, (float *)&material->diffuse);
    glUniform3fv(specularColor, 1, (float *)&material->specular);
    glUniform1f(shineness, material->specular_exponent * 0.128f);
    
    glUniformMatrix4fv(modelView, 1, GL_FALSE, (float *)&matrixMgr->getModelViewMatrix());
    glUniformMatrix4fv(projection, 1, GL_FALSE, (float *)&matrixMgr->getProjectionMatrix());
    glUniformMatrix3fv(normalMatrix, 1, GL_FALSE, (float *)&matrixMgr->getNormalMatrix());
    
    vec3 position    = { 0.0f, -3.0f, 10.0f };
    vec3 eyeposition = { 0.0f,  0.0f,  0.0f };
    
    eyeposition = position.multiply(nbManager::matrixMgr->getModelViewMatrix());
    
    
    glUniform3fv(light, 1, (float *)&eyeposition);
    
    glLogVerbose("glBindBuffer : GL_ELEMENT_ARRAY_BUFFER");
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, mesh->vboVertexIndex );
    
    glLogVerbose("glDrawElements");
    glDrawElements(mesh->mode,
                   (int)mesh->countOfVertexIndexList,
                   GL_UNSIGNED_SHORT,
                   ( void * )NULL );
#endif
}
