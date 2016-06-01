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
#include "nbNavigation.h"

#define OBJ_FILE                ( char * )"maze.obj"
#define PHYSIC_FILE             ( char * )"maze.bullet"
#define VERTEX_SHADER_FILE      ( char * )"vertex.glsl"
#define FRAGMENT_SHADER_FILE    ( char * )"fragment.glsl"

MatrixMgr *nbManager::matrixMgr;
Object *obj = NULL;

nbd::vec2 view_location,
view_delta = { 0.0f, 0.0f };

nbd::vec3 eye,
next_eye,
center = { 0.0f, 0.0f, 0.0f },
up = { 0.0f, 0.0f, 1.0f };


float rotx		= 45.0f,
next_rotx = 0.0f,
rotz		= 0.0f,
next_rotz	= -45.0f,
distance	= 30.0f;

Mesh *player = NULL;

Mesh *maze = NULL;

NAVIGATION *navigation = NULL;

unsigned char double_tap = 0;

NAVIGATIONPATH navigationpath_player;

NAVIGATIONPATHDATA navigationpathdata_player;

int viewport_matrix[ 4 ];



btSoftBodyRigidBodyCollisionConfiguration *collisionconfiguration = NULL;

btCollisionDispatcher *dispatcher = NULL;

btBroadphaseInterface *broadphase = NULL;

btConstraintSolver *solver = NULL;

btSoftRigidDynamicsWorld *dynamicsworld = NULL;


void init_physic_world( void )
{
    collisionconfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    
    dispatcher = new btCollisionDispatcher( collisionconfiguration );
    
    broadphase = new btDbvtBroadphase();
    
    solver = new btSequentialImpulseConstraintSolver();
    
    dynamicsworld = new btSoftRigidDynamicsWorld( dispatcher,
                                                 broadphase,
                                                 solver,
                                                 collisionconfiguration );
    
    dynamicsworld->setGravity( btVector3( 0.0f, 0.0f, -9.8f ) );
}


void load_physic_world( void )
{
    btBulletWorldImporter *btbulletworldimporter = new btBulletWorldImporter( dynamicsworld );
    
    Memory *memory = Memory::open(PHYSIC_FILE, 1 );
    
    btbulletworldimporter->loadFileFromMemory( ( char * )memory->buffer, memory->size );
    
    delete memory;
    
    unsigned int i = 0;
    
    while( i != btbulletworldimporter->getNumRigidBodies() ) {
        
        Mesh *mesh = obj->getMesh(btbulletworldimporter->getNameForPointer(
                                                                                 btbulletworldimporter->getRigidBodyByIndex( i ) ), 0 );
        
        if( mesh ) {
            
            mesh->rigidBody = ( btRigidBody * )btbulletworldimporter->getRigidBodyByIndex( i );
            
            mesh->rigidBody->setUserPointer( mesh );
        }
        
        ++i;
    }
    
    delete btbulletworldimporter;
}


void free_physic_world( void )
{
    while( dynamicsworld->getNumCollisionObjects() )
    {
        btCollisionObject *btcollisionobject = dynamicsworld->getCollisionObjectArray()[ 0 ];
        
        btRigidBody *btrigidbody = btRigidBody::upcast( btcollisionobject );
        
        if( btrigidbody )
        {
            delete btrigidbody->getCollisionShape();
            
            delete btrigidbody->getMotionState();
            
            dynamicsworld->removeRigidBody( btrigidbody );
            
            dynamicsworld->removeCollisionObject( btcollisionobject );
            
            delete btrigidbody;
        }
    }
    
    delete collisionconfiguration; collisionconfiguration = NULL;
    
    delete dispatcher; dispatcher = NULL;
    
    delete broadphase; broadphase = NULL;
    
    delete solver; solver = NULL;
    
    delete dynamicsworld; dynamicsworld = NULL;	
}


void nbManager::init()
{
    matrixMgr = new MatrixMgr();
    program = NULL;
    obj = new Object;
}

void nbManager::freeObj()
{
    NAVIGATION_free( navigation );
    free_physic_world();
    
    delete obj;
    delete matrixMgr;
    delete program;
}

void nbManager::start(int width, int height)
{
    this->width = width;
    this->height = height;
    
    initOpenGL();
    
    glViewport(0.0f, 0.0f, width, height);

    
    glGetIntegerv(GL_VIEWPORT, viewport_matrix);
    
    matrixMgr->setMatrixMode(kProjectionMatrix);
    matrixMgr->loadIdentity();
    matrixMgr->setPerspective(80.0f,
                              (float)width / (float)height,
                              0.1f,
                              100.0f,
                              -90.0f);
    loadObject();
}


void nbManager::loadObject(void)
{
    obj->justLoadObjFile(OBJ_FILE, 1);
    
    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
        if( strstr( mesh->name, "maze" ) ) {
            
            navigation = NAVIGATION_init( ( char * )"maze" );
            
            navigation->navigationconfiguration.agent_height = 2.0f;
            
            navigation->navigationconfiguration.agent_radius = 0.4f;
            
            if( NAVIGATION_build( navigation, obj, i ) )
            {
                console_print( "Navigation generated.\n");
            }
            else
            {
                console_print( "Unable to create the navigation mesh." );
            }
        }
        
//        OBJ_optimize_mesh( obj, i, 128 );
        
        mesh->justBuildVbo(&obj->vertexMgr);
        
        mesh->freeVertexData();
    }
    
    obj->freeVertextData();
    
    init_physic_world();
    
    load_physic_world();
    
    
    player = obj->getMesh("player", 0 );
    
    
    player->rigidBody->setAngularFactor( 0.0f );
    
    maze = obj->getMesh("maze", 0 );
    
    ::distance = maze->radius * 2.0f;
    
    
    
    for (int i = 0; i < obj->textureList.size(); i++) {
        Texture *texture = obj->textureList[i];
        texture->build(TEXTURE_MIPMAP, TEXTURE_FILTER_2X, 0.0f);
    }
    
    obj->buildMaterial();
    
    program = Program::create((char *)"default", VERTEX_SHADER_FILE, FRAGMENT_SHADER_FILE, 1, 1, nbManager::programBindAttrLocation, NULL);
    
//    program->draw();
//    
//    glUniform1i( program->getUniformLocation(( char * )"DIFFUSE" ), 1 );
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
        
        if( view_delta.y ) next_rotz -= view_delta.y;
        
        if( view_delta.x ) {
            next_rotx -= view_delta.x;
            next_rotx = CLAMP( next_rotx, 0.0f, 90.0f );
        }
        
        view_delta.x =
        view_delta.y = 0.0f;
    }
    
    rotx = rotx * 0.9f + next_rotx * 0.1f;
    rotz = rotz * 0.9f + next_rotz * 0.1f;
    
    
    
    eye.x = center.x +
    ::distance *
    cosf( rotx * DEG_TO_RAD ) *
    sinf( rotz * DEG_TO_RAD );
    
    eye.y = center.y -
    ::distance *
    cosf( rotx * DEG_TO_RAD ) *
    cosf( rotz * DEG_TO_RAD );
    
    
    eye.z = center.z +
    ::distance *
    sinf( rotx * DEG_TO_RAD );
    
    
    rotx = rotx * 0.9f + next_rotx * 0.1f;
    rotz = rotz * 0.9f + next_rotz * 0.1f;
    
    
    center.x = maze->location.x;
    center.y = maze->location.y;
    center.z = maze->location.z;
    
    matrixMgr->lookAt(eye,
                center,
                up );
    
    
    
    if( double_tap ) {
        
        nbd::vec3 location;
        
        if (matrixMgr->unproject(view_location.x,
                          viewport_matrix[ 3 ] - view_location.y,
                          1.0f,
                             &matrixMgr->getModelViewMatrix(),
                             &matrixMgr->getProjectionMatrix(),
                          viewport_matrix,
                          &location.x,
                          &location.y,
                          &location.z  )) {
            
            btVector3 ray_from( eye.x,
                               eye.y,
                               eye.z ),
            
            ray_to( location.x + eye.x,
                   location.y + eye.y,
                   location.z + eye.z );
            
            btCollisionWorld::ClosestRayResultCallback collision_ray( ray_from,
                                                                     ray_to );
            
            dynamicsworld->rayTest( ray_from,
                                   ray_to,
                                   collision_ray );
            
            if( collision_ray.hasHit() &&
               collision_ray.m_collisionObject == maze->rigidBody ) {
                
                collision_ray.m_hitNormalWorld.normalize();
                
                if( collision_ray.m_hitNormalWorld.z() == 1.0f ) {
                    
                    navigationpath_player.start_location.x = player->location.x;
                    navigationpath_player.start_location.y = player->location.y;
                    navigationpath_player.start_location.z = player->location.z;
                    
                    navigationpath_player.end_location.x = collision_ray.m_hitPointWorld.x();
                    navigationpath_player.end_location.y = collision_ray.m_hitPointWorld.y();
                    navigationpath_player.end_location.z = collision_ray.m_hitPointWorld.z();
                    
                    if( NAVIGATION_get_path( navigation,
                                            &navigationpath_player,
                                            &navigationpathdata_player ) ) {
                        
                        unsigned int i = 0;
                        while( i != navigationpathdata_player.path_point_count + 1 ) { 
                            
                            console_print( "%d: %f %f %f\n",
                                          i,
                                          navigationpathdata_player.path_point_array[ i ].x,
                                          navigationpathdata_player.path_point_array[ i ].y,
                                          navigationpathdata_player.path_point_array[ i ].z );
                            ++i; 
                        }
                        printf( "\n" );
                    }
                }
            }
        }
        
        double_tap = 0;
    }
    
    
    program->draw();
    
//    glUniform1i( PROGRAM_get_uniform_location( program, ( char * )"DIFFUSE" ), 1 );
     glUniform1i( program->getUniformLocation(( char * )"DIFFUSE" ), 1 );
    
    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
        
        matrixMgr->pushMatrix();
        
        nbd::mat4 mat;
        
        mesh->rigidBody->getWorldTransform().getOpenGLMatrix( ( float * )&mat );
        
        memcpy( &mesh->location, ( nbd::vec3 * )&mat.m[ 3 ], sizeof( nbd::vec3 ) );
        
        matrixMgr->multiply(mat);
        
        glUniformMatrix4fv( program->getUniformLocation((char *)"MODELVIEWPROJECTIONMATRIX"),
                           1,
                           GL_FALSE,
                           ( float * )&matrixMgr->getModelViewProjectionMatrix() );
        
        mesh->draw();
        
        matrixMgr->popMatrix();
    }
    
    NAVIGATION_draw( navigation, *matrixMgr );

    
    dynamicsworld->stepSimulation( 1.0f / 60.0f );
    
//    console_print( "Visible Objects: %d from total : %d\n", n, obj->meshList.size());
}

void nbManager::toucheBegan( float x, float y, unsigned int tap_count )
{
    if( tap_count == 2 ) {
        double_tap = 1;
    }
    
    view_location.x = x;
    view_location.y = y;
}

void nbManager::toucheMoved( float x, float y, unsigned int tap_count )
{
    view_delta.x = view_delta.x * 0.75f + ( x - view_location.x ) * 0.25f;
    view_delta.y = view_delta.y * 0.75f + ( y - view_location.y ) * 0.25f;
    
    view_location.x = x;
    view_location.y = y;
}

void nbManager::toucheEnded( float x, float y, unsigned int tap_count )
{

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
