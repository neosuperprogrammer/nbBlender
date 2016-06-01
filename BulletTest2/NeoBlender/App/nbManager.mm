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


#define OBJ_FILE                ( char * )"game.obj"
#define VERTEX_SHADER_FILE      ( char * )"vertex.glsl"
#define FRAGMENT_SHADER_FILE    ( char * )"fragment.glsl"

MatrixMgr *nbManager::matrixMgr;

Program *program = NULL;

nbd::vec2 start_pos = { 0.0f, 0.0f };

nbd::vec3 eye = { 3.5f, -10.8f, 5.3f };
nbd::vec3 center = { 3.5f,  -9.8f, 5.3f };
nbd::vec3 up = { 0.0f,   0.0f, 1.0f };

unsigned int momo_index = 0;

Mesh *momo = NULL;

Mesh *gameover = NULL;

bool restart_game = 0;
bool momo_launch  = 0;
int banana = 0;

nbd::vec2 view_location;
int viewport_matrix[ 4 ];

bool doubleTapped = false;


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


enum
{
    BOX		 = 0,
    SPHERE	 = 1,
    CYLINDER = 2
};

void add_rigid_body( Mesh *objmesh, unsigned char bound, float mass, unsigned char dynamic_only )
{
    btCollisionShape *btcollisionshape = NULL;
    
    switch ( bound )
    {
        case BOX:
        {
            btcollisionshape = new btBoxShape( btVector3( objmesh->dimension.x * 0.5f,
                                                         objmesh->dimension.y * 0.5f,
                                                         objmesh->dimension.z * 0.5f ) );
            break;
        }
            
        case SPHERE:
        {
            btcollisionshape = new btSphereShape( objmesh->radius );
            break;
        }
            
        case CYLINDER:
        {
            btcollisionshape = new btCylinderShapeZ( btVector3( objmesh->dimension.x * 0.5f,
                                                               objmesh->dimension.y * 0.5f,
                                                               objmesh->dimension.z * 0.5f ) );
            break;
        }
    }
    
    btTransform bttransform;
    
    bttransform.setIdentity();
    
    bttransform.setOrigin( btVector3( objmesh->location.x,
                                     objmesh->location.y,
                                     objmesh->location.z ) );
    
    btDefaultMotionState *btdefaultmotionstate = NULL;
    
    btdefaultmotionstate = new btDefaultMotionState( bttransform );
    
    btVector3 localinertia( 0.0f, 0.0f, 0.0f );
    
    if( mass > 0.0f ) {
        btcollisionshape->calculateLocalInertia( mass, localinertia );
    }
    
    objmesh->rigidBody = new btRigidBody( mass,
                                           btdefaultmotionstate,
                                           btcollisionshape,
                                           localinertia );
    
    if( mass > 0.0f ) {
        
        objmesh->rigidBody->setLinearFactor( btVector3( 1.0f, 0.0f, 1.0f ) );
        
        if( !dynamic_only )	objmesh->rigidBody->setAngularFactor( btVector3( 0.0f, 1.0f, 0.0f ) );
        
        else objmesh->rigidBody->setAngularFactor( 0.0f );
    }
    
    
    objmesh->rigidBody->setUserPointer( objmesh );
    
    dynamicsworld->addRigidBody( objmesh->rigidBody );
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


bool contact_added_callback( btManifoldPoint &btmanifoldpoint,
                            const btCollisionObject *btcollisionobject0,
                            int part_0, int index_0,
                            const btCollisionObject *btcollisionobject1,
                            int part_1, int index_1 ) {
    
    Mesh *objmesh0 = ( Mesh * )( ( btRigidBody * )btcollisionobject0 )->getUserPointer();
    
    Mesh *objmesh1 = ( Mesh * )( ( btRigidBody * )btcollisionobject1 )->getUserPointer();
    
    
    /* Check if one of the two object involve in the collision is a momo. */
    if( ( strstr( objmesh0->name, "momo" ) || 
         strstr( objmesh1->name, "momo" ) )
       &&
       ( strstr( objmesh0->name, "banana" ) || 
        strstr( objmesh1->name, "banana" ) ) ) { 
           
           Mesh *bananaMesh = NULL;
           btCollisionObject *btcollisionobject = NULL;
           
           if( strstr( objmesh0->name, "banana" ) ) { 
               bananaMesh = objmesh0;
               btcollisionobject = ( btCollisionObject * )btcollisionobject0;
           } 
           else { 
               bananaMesh = objmesh1;
               btcollisionobject = ( btCollisionObject * )btcollisionobject1;
           }
           
           bananaMesh->visible = 0;
           
           --banana;
           
           btRigidBody *bananaBody = bananaMesh->rigidBody;
           bananaMesh->rigidBody = NULL;
           
           delete bananaBody->getCollisionShape();
           
           delete bananaBody->getMotionState();
           
           dynamicsworld->removeRigidBody( bananaBody );
           
           dynamicsworld->removeCollisionObject( btcollisionobject );
           
           delete bananaBody;
           
       }
    
    return false;
}

void nbManager::get_next_momo( void )
{
    char tmp[ MAX_CHAR ] = {""};
    
    momo = NULL;
    
    ++momo_index;
    
    sprintf( tmp, "momo%d", momo_index );

    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
        if( strstr( mesh->name, tmp ) ) {
            momo = mesh;
            momo_launch = false;
            momo->rigidBody->setActivationState( DISABLE_DEACTIVATION );
            return;
        }
        
    }
}


void nbManager::load_game( void )
{
    init_physic_world();
    
    gContactAddedCallback = contact_added_callback;
    
    obj = new Object();
    
    obj->justLoadObjFile(OBJ_FILE, 1);
    
    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
//        OBJ_optimize_mesh( obj, i, 128 );
        mesh->build(&obj->vertexMgr);
        
        if( strstr( mesh->name, "momo" ) )
            add_rigid_body( mesh, SPHERE, 2.0f, 0 );
        
        else if( strstr( mesh->name, "barrel" ) )
            add_rigid_body( mesh, CYLINDER, 1.0f, 0 );
        
        else if( strstr( mesh->name, "plank" ) )
            add_rigid_body( mesh, BOX, 1.0f, 0 );
        
        else if( strstr( mesh->name, "ground" ) )
            add_rigid_body( mesh, BOX, 0.0f, 0 );
        
        else if( strstr( mesh->name, "steel" ) )
            add_rigid_body( mesh, CYLINDER, 0.0f, 0 );
        
        else if( strstr( mesh->name, "banana" ) ) {
            
            add_rigid_body( mesh, SPHERE, 1.0f, 0 );
            
            mesh->rigidBody->setCollisionFlags( mesh->rigidBody->getCollisionFlags() |
                                                    btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );
            
            mesh->rigidBody->forceActivationState ( ISLAND_SLEEPING );
            
            ++banana;
        }
        
        else if( strstr( mesh->name, "gameover" ) ) {
            
            mesh->visible = 0;
            
            gameover = mesh;
        }
        
        mesh->freeVertexData();
    }
    
    obj->freeVertextData();
    
    for (int i = 0; i < obj->textureList.size(); i++) {
        Texture *texture = obj->textureList[i];
        texture->build(TEXTURE_MIPMAP | TEXTURE_16_BITS, TEXTURE_FILTER_2X, 0.0f);
    }
    
#if 0
    obj->buildMaterial(VERTEX_SHADER_FILE, FRAGMENT_SHADER_FILE, nbManager::programBindAttrLocation, nbManager::materialDrawCallback);
#else
    obj->buildMaterial();
    program = Program::create(( char * )"default", VERTEX_SHADER_FILE, FRAGMENT_SHADER_FILE, 1, 1, nbManager::programBindAttrLocation, NULL);

    program->draw();
    glUniform1i( program->getUniformLocation(( char * )"DIFFUSE" ), 1 );
#endif

    momo_index  = 0;
    
    center.x =
    eye.x    = 3.5f;
	  	
    get_next_momo();
}


void nbManager::init()
{
    matrixMgr = new MatrixMgr();
//    obj = new Object();
}

void nbManager::freeObj()
{
    if( dynamicsworld ) free_physic_world();
    if (obj) {
        delete obj; obj = NULL;
    }
}

void nbManager::start(int width, int height)
{
    this->width = width;
    this->height = height;
    
    initOpenGL();
    
    glViewport(0.0f, 0.0f, width, height);
    glGetIntegerv(GL_VIEWPORT, viewport_matrix);

    
//    init_physic_world();
    
    matrixMgr->setMatrixMode(kProjectionMatrix);
    matrixMgr->loadIdentity();
//    matrixMgr->setPerspective(45.0f,
//                              (float)width / (float)height,
//                              0.1f,
//                              100.0f,
//                              -90.0f );
    
   matrixMgr->set_orthographic( ( float )height / ( float )width,
                         15.0f,
                         ( float )width / ( float )height,
                         1.0f,
                         100.0f,
                         -90.0f );
    load_game();
//    loadObject();
}

void nbManager::loadObject(void)
{
//    obj->loadObjFile(OBJ_FILE, 1);
//    
////    for (int i = 0; i < nbObj.meshList.size(); i++) {
////        Mesh *mesh = nbObj.meshList[i];
////        
////        if (!strcmp(mesh->name, "Cube")) {
////            mesh->rotation.x =
////            mesh->rotation.y =
////            mesh->rotation.z = 35.0f;
////            
////            addRigidBody(mesh, 1.0f);
////        }
////        else {
////            addRigidBody(mesh, 0.0f);
////        }
////    }
//    
//#if 1
//    obj->buildMaterial(VERTEX_SHADER_FILE, FRAGMENT_SHADER_FILE, nbManager::programBindAttrLocation, nbManager::materialDrawCallback);
//#else
//    Program *program = Program::create((char *)"default", VERTEX_SHADER_FILE, FRAGMENT_SHADER_FILE, 1, 1, nbManager::programBindAttrLocation, nbManager::programDrawCallback);
//    
//    for (int i = 0; i < nbObj.meshList.size(); i++) {
//        Mesh *mesh = nbObj.meshList[i];
//        mesh->program = program;
//    }
//#endif
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
    
    if( restart_game )
    {
        freeObj();
        
        load_game();
        
        restart_game = 0;
    }
    
    matrixMgr->setMatrixMode(kModelViewMatrix);
    matrixMgr->loadIdentity();
    
    
    if( momo )
    {
        eye.x = eye.x * 0.98f + momo->location.x * 0.02f;
        center.x =
        eye.x = CLAMP( eye.x, -2.0f, 3.5f );
    }
    
    matrixMgr->lookAt(eye, center, up);

    
    if (doubleTapped) {
        
        nbd::vec3 location;
        
        if (matrixMgr->unproject(view_location.x,
                                 viewport_matrix[ 3 ] - view_location.y,
                                 1.0f,
                                 &matrixMgr->getModelViewMatrix(),
                                 &matrixMgr->getProjectionMatrix(),
                                 viewport_matrix,
                                 &location.x,
                                 &location.y,
                                 &location.z  ))
        {
//            location.y = 1.0;
            btVector3 ray_from( location.x,
                               -10.0,
                               location.z );
            
//            btVector3 ray_from( eye.x,
//                               eye.y,
//                               eye.z ),
            
//            ray_to( location.x + eye.x,
//                   location.y + eye.y,
//                   location.z + eye.z );

            btVector3 ray_to( location.x,
                   location.y,
                   location.z);

            btCollisionWorld::ClosestRayResultCallback collision_ray( ray_from,
                                                                     ray_to );
            
            dynamicsworld->rayTest( ray_from,
                                   ray_to,
                                   collision_ray );

            if( collision_ray.hasHit()) {
                collision_ray.m_hitNormalWorld.normalize();
                
                btRigidBody *collisionBody = (btRigidBody *)collision_ray.m_collisionObject;
                Mesh *mesh = (Mesh *)collisionBody->getUserPointer();
                console_print("collision mesh : %s", mesh->name);
                if( collision_ray.m_hitNormalWorld.z() == 1.0f ) {
                    btVector3 hitPoint = collision_ray.m_hitPointWorld;
                }
                
            }

//            if( collision_ray.hasHit() &&
//               collision_ray.m_collisionObject == maze->rigidBody ) {
//                
//                collision_ray.m_hitNormalWorld.normalize();
//                
//                if( collision_ray.m_hitNormalWorld.z() == 1.0f ) {
//                    
//                    navigationpath_player.start_location.x = player->location.x;
//                    navigationpath_player.start_location.y = player->location.y;
//                    navigationpath_player.start_location.z = player->location.z;
//                    
//                    navigationpath_player.end_location.x = collision_ray.m_hitPointWorld.x();
//                    navigationpath_player.end_location.y = collision_ray.m_hitPointWorld.y();
//                    navigationpath_player.end_location.z = collision_ray.m_hitPointWorld.z();
//                    
//                    if( NAVIGATION_get_path( navigation,
//                                            &navigationpath_player,
//                                            &navigationpathdata_player ) ) {
//                        
//                        unsigned int i = 0;
//                        while( i != navigationpathdata_player.path_point_count + 1 ) {
//                            
//                            console_print( "%d: %f %f %f\n",
//                                          i,
//                                          navigationpathdata_player.path_point_array[ i ].x,
//                                          navigationpathdata_player.path_point_array[ i ].y,
//                                          navigationpathdata_player.path_point_array[ i ].z );
//                            ++i;
//                        }
//                        printf( "\n" );
//                    }
//                }
//            }
        }
        
        doubleTapped = false;
    }
    
    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
        
        matrixMgr->pushMatrix();
        
        if( mesh->rigidBody )
        {
            nbd::mat4 mat;
            
            mesh->rigidBody->getWorldTransform().getOpenGLMatrix( ( float * )&mat );
            
//            mesh->location.x = mat.m[ 3 ].x;
            memcpy( &mesh->location, ( nbd::vec3 * )&mat.m[ 3 ], sizeof( nbd::vec3 ) );

            
            matrixMgr->multiply(mat);
        }
        else {
            matrixMgr->translate(mesh->location.x, mesh->location.y, mesh->location.z );
        }
        
        
        glUniformMatrix4fv(program->getUniformLocation((char *)"MODELVIEWPROJECTIONMATRIX"),
                               1,
                               GL_FALSE,
                               ( float * )&nbManager::matrixMgr->getModelViewProjectionMatrix());

        
        mesh->draw();
        
        matrixMgr->popMatrix();
        
    }
    
//    matrixMgr->pushMatrix();
//    
//    nbd::mat4 mat;
//    
//    momo->rigidBody->getWorldTransform().getOpenGLMatrix( ( float * )&mat );
//    
//    matrixMgr->multiply(mat);
//    
//    for (int i = 0; i < momo->uniqueVertexUVIndexList.size(); i++) {
//        UniqueVertexUVIndex index = momo->uniqueVertexUVIndexList[i];
//        nbd::vec3 vertex = obj->vertexMgr.uniqueVertexList[index.vertexIndex];
//        printf("vertex data : x[%f], y[%f], z[%f]\n", vertex.x, vertex.y, vertex.z);
//        nbd::vec3 newVertex = vertex.multiply(nbManager::matrixMgr->getModelViewProjectionMatrix());
//        printf("new vertex data : x[%f], y[%f], z[%f]\n", newVertex.x * 320, newVertex.y, newVertex.z * 480);
//        
//        
//     
//    }
//    
//    matrixMgr->popMatrix();

    
    
    dynamicsworld->stepSimulation( 1.0f / 60.0f );
    
    
    if(momo &&
       (momo->rigidBody->getLinearVelocity().length() > 20.0f ||
        momo->rigidBody->getActivationState() == ISLAND_SLEEPING))  {
           get_next_momo();
       }
		  
    if( !momo || !banana ) {
        
        gameover->visible = 1;
        gameover->location.x = eye.x;
        gameover->location.z = eye.z;
    }
}

void nbManager::toucheBegan( float x, float y, unsigned int tap_count )
{
    if( tap_count == 2 ) {
        doubleTapped = true;
    }
    
    view_location.x = x;
    view_location.y = y;

    start_pos.x = x;
    start_pos.y = y;
}

void nbManager::toucheMoved( float x, float y, unsigned int tap_count )
{
    rotAngle.z += -( touche.x - x );
    rotAngle.x += -( touche.y - y );
    
    touche.x = x;
    touche.y = y;
}

void nbManager::toucheEnded( float x, float y, unsigned int tap_count )
{
    if( gameover->visible && !restart_game )
    {
        restart_game = 1;
        return;
    }
    
    if( momo && !momo_launch) {
        momo_launch = true;
        
        momo->rigidBody->forceActivationState( ACTIVE_TAG );
        
        momo->rigidBody->setLinearVelocity(
                                             btVector3( CLAMP( ( y - start_pos.y ) * 0.1f, 0.0f, 10.0f ),
                                                       0.0f,
                                                       CLAMP( ( x - start_pos.x ) * 0.1f, 0.0f, 10.0f ) ) );
    } 
    
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
