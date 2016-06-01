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


#define OBJ_FILE                ( char * )"Scene.obj"
#define VERTEX_SHADER_FILE      ( char * )"vertex.glsl"
#define FRAGMENT_SHADER_FILE    ( char * )"fragment.glsl"

btSoftBodyRigidBodyCollisionConfiguration *collisionconfiguration = NULL;

btCollisionDispatcher *collisionDispatcher = NULL;

btBroadphaseInterface *broadphase = NULL;

btConstraintSolver *solver = NULL;

btSoftRigidDynamicsWorld *dynamicWorld = NULL;

MatrixMgr *nbManager::matrixMgr;

void initPhisicalWorld( void )
{
    collisionconfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    
    collisionDispatcher = new btCollisionDispatcher(collisionconfiguration);
    
    broadphase = new btDbvtBroadphase();
    
    solver = new btSequentialImpulseConstraintSolver();
    
    dynamicWorld = new btSoftRigidDynamicsWorld(collisionDispatcher,
                                                 broadphase,
                                                 solver,
                                                 collisionconfiguration);
    
    dynamicWorld->setGravity(btVector3(0.0f, 0.0f, -9.8f));
}

void addRigidBody(Mesh *mesh, float mass)
{
    nbd::mat4 mat;
    
    nbd::vec4 rotx = {1.0f, 0.0f, 0.0f, mesh->rotation.x},
    roty = {0.0f, 1.0f, 0.0f, mesh->rotation.y},
    rotz = {0.0f, 0.0f, 1.0f, mesh->rotation.z};
    
    mat.identity();
    mat.translate(mesh->location);
    mat.rotate(rotz);
    mat.rotate(roty);
    mat.rotate(rotx);
    
    btTransform bttransform;
    
    bttransform.setFromOpenGLMatrix((float *)&mat);
    
    btDefaultMotionState *motionState = new btDefaultMotionState(bttransform);
    
    
    btCollisionShape *collisionShape = new btBoxShape(btVector3(mesh->dimension.x * 0.5f,
                                                                  mesh->dimension.y * 0.5f,
                                                                  mesh->dimension.z * 0.5f));

    btVector3 inertia(0.0f, 0.0f, 0.0f);
    if (mass) {
        collisionShape->calculateLocalInertia(mass, inertia);
    }
    
    mesh->rigidBody = new btRigidBody(mass,
                                      motionState,
                                      collisionShape,
                                      inertia);
    
    mesh->rigidBody->setUserPointer(mesh);
    dynamicWorld->addRigidBody(mesh->rigidBody);
}

void freePhisicalWorld(void)
{
    while (dynamicWorld->getNumCollisionObjects()) {
        btCollisionObject *btcollisionobject = dynamicWorld->getCollisionObjectArray()[ 0 ];
        
        btRigidBody *btrigidbody = btRigidBody::upcast( btcollisionobject );
        
        if (btrigidbody)
        {
            delete btrigidbody->getCollisionShape();
            
            delete btrigidbody->getMotionState();
            
            dynamicWorld->removeRigidBody(btrigidbody);
            
            dynamicWorld->removeCollisionObject(btcollisionobject);
            
            delete btrigidbody;
        }
    }
    
    delete collisionconfiguration; collisionconfiguration = NULL;
    
    delete collisionDispatcher; collisionDispatcher = NULL;
    
    delete broadphase; broadphase = NULL;
    
    delete solver; solver = NULL;
    
    delete dynamicWorld; dynamicWorld = NULL;
}

void nbManager::init()
{
    matrixMgr = new MatrixMgr();
//    _matrixMgr = matrixMgr;
}

void nbManager::freeObj()
{
    freePhisicalWorld();
}

void nbManager::start(int width, int height)
{
    this->width = width;
    this->height = height;
    
    initOpenGL();
    
    initPhisicalWorld();
    
    matrixMgr->setMatrixMode(kProjectionMatrix);
    matrixMgr->loadIdentity();
    matrixMgr->setPerspective(45.0f,
                              (float)width / (float)height,
                              0.1f,
                              100.0f,
                              0.0f );
    
    loadObject();
}

void nbManager::loadObject(void)
{
    nbObj.loadObjFile(OBJ_FILE, 1);
    
    for (int i = 0; i < nbObj.meshList.size(); i++) {
        Mesh *mesh = nbObj.meshList[i];
        
        if (!strcmp(mesh->name, "Cube")) {
            mesh->rotation.x =
            mesh->rotation.y =
            mesh->rotation.z = 35.0f;
            
            addRigidBody(mesh, 1.0f);
        }
        else {
            addRigidBody(mesh, 0.0f);
        }
    }
    
#if 0
    nbObj.buildMaterial(VERTEX_SHADER_FILE, FRAGMENT_SHADER_FILE, nbManager::programBindAttrLocation, nbManager::materialDrawCallback);
#else
    Program *program = Program::create((char *)"default", VERTEX_SHADER_FILE, FRAGMENT_SHADER_FILE, 1, 1, nbManager::programBindAttrLocation, nbManager::programDrawCallback);
    
    for (int i = 0; i < nbObj.meshList.size(); i++) {
        Mesh *mesh = nbObj.meshList[i];
        mesh->program = program;
    }
#endif
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

    
    // mushroom.obj
//    nbd::vec3 e = {  -2.0f, -12.0f, 2.0f },
//    c = {  0.0f, 0.0f, 2.0f },
//    u = {  0.0f,  0.0f, 1.0f  };
    
    nbd::vec3 e = { 10.4f, -9.8f, 5.5f },
    c = { -3.4f,  2.8f, 0.0f },
    u = {  0.0f,  0.0f, 1.0f };
    
    matrixMgr->lookAt(e, c, u);

    for (int i = 0; i < nbObj.meshList.size(); i++) {
        Mesh *mesh = nbObj.meshList[i];
        matrixMgr->pushMatrix();

        nbd::mat4 mat;
        mesh->rigidBody->getWorldTransform().getOpenGLMatrix((float * )&mat);
        
        matrixMgr->multiply(mat);

//        matrixMgr->translate(mesh->location.x,
//                             mesh->location.y,
//                             mesh->location.z);
        
        matrixMgr->rotate(rotAngle.x, 1.0f, 0.0f, 0.0f);
        matrixMgr->rotate(rotAngle.z, 0.0f, 0.0f, 1.0f);
        
        mesh->draw();
        
        matrixMgr->popMatrix();
    }
    
    dynamicWorld->stepSimulation( 1.0f / 60.0f );
    
    unsigned int n_manifolds = dynamicWorld->getDispatcher()->getNumManifolds();
    
    int i = 0;
    while( i != n_manifolds )
    {
        btPersistentManifold *manifold = dynamicWorld->getDispatcher()->getManifoldByIndexInternal( i );
        
        Mesh *objmesh0 = ( Mesh * )( ( btRigidBody * )manifold->getBody0() )->getUserPointer();
        
        Mesh *objmesh1 = ( Mesh * )( ( btRigidBody * )manifold->getBody1() )->getUserPointer();
        
        unsigned int j = 0,
        n_contacts = manifold->getNumContacts();
        
        while( j != n_contacts )
        {
            btManifoldPoint &contact = manifold->getContactPoint( j );
            
            console_print("Manifold : %d\n", i );
            console_print("Contact  : %d\n", j );
            
            console_print("Object #0: %s\n", objmesh0->name );
            console_print("Point  #0: %.3f %.3f %.3f\n",
                          contact.getPositionWorldOnA().x(),
                          contact.getPositionWorldOnA().y(),
                          contact.getPositionWorldOnA().z() );
            
            console_print("Object #1: %s\n", objmesh1->name );
            console_print("Point  #1: %.3f %.3f %.3f\n",
                          contact.getPositionWorldOnB().x(),
                          contact.getPositionWorldOnB().y(),
                          contact.getPositionWorldOnB().z() );
            
            console_print("Distance : %.3f\n", contact.getDistance() );
            console_print("Lifetime : %d\n"  , contact.getLifeTime() );
            
            console_print("Normal   : %.3f %.3f %.3f\n",
                          contact.m_normalWorldOnB.x(),
                          contact.m_normalWorldOnB.y(),
                          contact.m_normalWorldOnB.z() );
            
            console_print( "%d\n\n", get_milli_time() );
            
            ++j;
        }
        
        ++i;
    }
}

void nbManager::toucheBegan( float x, float y, unsigned int tap_count )
{
    touche.x = x;
    touche.y = y;
}

void nbManager::toucheMoved( float x, float y, unsigned int tap_count )
{
    rotAngle.z += -( touche.x - x );
    rotAngle.x += -( touche.y - y );
    
    touche.x = x;
    touche.y = y;
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
                   (int)mesh->vertexIndexList.size(),
                   GL_UNSIGNED_SHORT,
                   ( void * )NULL );
#endif
}
