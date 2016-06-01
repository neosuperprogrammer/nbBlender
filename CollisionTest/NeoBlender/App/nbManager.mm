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
#include "btTriangleMeshShape.h"
#include "btGImpactShape.h"
#include "btGImpactCollisionAlgorithm.h"
#include "btBulletWorldImporter.h"
#include "nbUtils.h"
#include "btSoftBodyHelpers.h"

#define OBJ_FILE                ( char * )"Scene.obj"
#define VERTEX_SHADER_FILE      ( char * )"vertex.glsl"
#define FRAGMENT_SHADER_FILE    ( char * )"fragment.glsl"

MatrixMgr *nbManager::matrixMgr;
Object *obj = NULL;


nbd::vec3 location = { 0.0f, 0.0f, 1.84f };

float rotz = 0.0f, rotx = 90.0f;

nbd::vec4 frustum[ 6 ];

float screen_size = 0.0f;

nbd::vec2 view_location,
view_delta = { 0.0f, 0.0f };

nbd::vec3 move_location = { 0.0f, 0.0f, 0.0f },
move_delta;

nbd::vec2 tapPoint;
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
    CYLINDER = 2,
    MESH     = 3,
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
        case MESH:
        {
//            btTriangleMesh *meshInterface = new btTriangleMesh();
            
//            for (int i = 0; i < objmesh->uniqueVertexUVIndexList.size() / 3; i+=3) {
//                int index1 = objmesh->uniqueVertexUVIndexList[i].vertexIndex;
//                int index2 = objmesh->uniqueVertexUVIndexList[i + 1].vertexIndex;
//                int index3 = objmesh->uniqueVertexUVIndexList[i + 2].vertexIndex;
//                
//                nbd::vec3 pivot = obj->vertexMgr.uniqueVertexList[index1];
//                pivot = pivot.diff(objmesh->location);
//                btVector3 vertex0 = btVector3(pivot.x, pivot.y, pivot.z);
//
//                pivot = obj->vertexMgr.uniqueVertexList[index2];
//                pivot = pivot.diff(objmesh->location);
//                btVector3 vertex1 = btVector3(pivot.x, pivot.y, pivot.z);
//
//                pivot = obj->vertexMgr.uniqueVertexList[index3];
//                pivot = pivot.diff(objmesh->location);
//                btVector3 vertex2 = btVector3(pivot.x, pivot.y, pivot.z);
//
//                meshInterface->addTriangle(vertex0, vertex1, vertex2, false);
//                
//            }
            
//            for(uint i = 0; i < objmesh->vertexIndexList.size(); i++) {
//                meshInterface->addIndex(objmesh->vertexIndexList[i]);
//            }

//            for(uint i = 0; i < objmesh->vertexIndexList.size() / 3; i+=3) {
//                
//                int index1 = objmesh->uniqueVertexUVIndexList[objmesh->vertexIndexList[i]].vertexIndex;
//                int index2 = objmesh->uniqueVertexUVIndexList[objmesh->vertexIndexList[i + 1]].vertexIndex;
//                int index3 = objmesh->uniqueVertexUVIndexList[objmesh->vertexIndexList[i + 2]].vertexIndex;
//                
//                nbd::vec3 pivot = obj->vertexMgr.uniqueVertexList[index1];
//                pivot = pivot.diff(objmesh->location);
//                btVector3 vertex0 = btVector3(pivot.x, pivot.y, pivot.z);
//                
//                pivot = obj->vertexMgr.uniqueVertexList[index2];
//                pivot = pivot.diff(objmesh->location);
//                btVector3 vertex1 = btVector3(pivot.x, pivot.y, pivot.z);
//                
//                pivot = obj->vertexMgr.uniqueVertexList[index3];
//                pivot = pivot.diff(objmesh->location);
//                btVector3 vertex2 = btVector3(pivot.x, pivot.y, pivot.z);
//                
//                meshInterface->addTriangle(vertex0, vertex1, vertex2, true);
//
//            }
//            
//            btcollisionshape = new btBvhTriangleMeshShape(meshInterface, false, true);

            
            
            int buffSize = (int)objmesh->uniqueVertexUVIndexList.size() * sizeof(nbd::vec3);
            
            unsigned char *vertexArray = (unsigned char *)malloc(buffSize);
            unsigned char *vertexStart = vertexArray;
            
            for (int i = 0; i < objmesh->uniqueVertexUVIndexList.size(); i++) {
                int index = objmesh->uniqueVertexUVIndexList[i].vertexIndex;
                
                nbd::vec3 pivot = obj->vertexMgr.uniqueVertexList[index];
                
                pivot = pivot.diff(objmesh->location);
                
                memcpy(vertexArray, &pivot, sizeof(nbd::vec3));
                vertexArray += sizeof(nbd::vec3);
                
            }
            
            btIndexedMesh *indexedMesh = new btIndexedMesh();
//            indexedMesh->m_indexType = PHY_SHORT;
//            indexedMesh->m_vertexType = PHY_FLOAT;
            
            indexedMesh->m_numTriangles = (int)objmesh->vertexIndexList.size() / 3;
            indexedMesh->m_triangleIndexBase = (const unsigned char *)&objmesh->vertexIndexList[0];
            indexedMesh->m_triangleIndexStride = sizeof(short) * 3;
            
            indexedMesh->m_numVertices = (int)objmesh->uniqueVertexUVIndexList.size();
            indexedMesh->m_vertexBase = vertexStart;
            indexedMesh->m_vertexStride = sizeof(nbd::vec3);
            
            btTriangleIndexVertexArray* colonVertexArrays = new btTriangleIndexVertexArray();
            
            colonVertexArrays->addIndexedMesh(*indexedMesh, PHY_SHORT);
            btcollisionshape = new btBvhTriangleMeshShape(colonVertexArrays, false, true);

//            free(vertexStart);
            
            

            
            break;
        }
    }
    
    btTransform bttransform;
    
    bttransform.setIdentity();
    
    
//    if (strstr(objmesh->name, "Sphere")) {
//        objmesh->location.y -= 3.0;
//        objmesh->location.z += 3.0;
//    }
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
        
        objmesh->rigidBody->setLinearFactor( btVector3( 1.0f, 1.0f, 1.0f ) );
        
        if( !dynamic_only )	objmesh->rigidBody->setAngularFactor( btVector3( 1.0f, 1.0f, 1.0f ) );
        
        else objmesh->rigidBody->setAngularFactor( 0.0f );
    }
    
    
    objmesh->rigidBody->setUserPointer( objmesh );
    
    if( !strcmp( objmesh->name, "Sphere" ) )
    {
        objmesh->rigidBody->setCollisionFlags( objmesh->rigidBody->getCollisionFlags() |
                                                btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );
    }	

    
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
    
    console_print("Object #0: %s\n", objmesh0->name );
    console_print("Point  #0: %.3f %.3f %.3f\n",
                  btmanifoldpoint.m_positionWorldOnA.x(),
                  btmanifoldpoint.m_positionWorldOnA.y(),
                  btmanifoldpoint.m_positionWorldOnA.z() );
    
    console_print("Object #1: %s\n", objmesh1->name );
    console_print("Point  #1: %.3f %.3f %.3f\n",
                  btmanifoldpoint.m_positionWorldOnB.x(),
                  btmanifoldpoint.m_positionWorldOnB.y(),
                  btmanifoldpoint.m_positionWorldOnB.z() );
    
    console_print("Normal   : %.3f %.3f %.3f\n",
                  btmanifoldpoint.m_normalWorldOnB.x(),
                  btmanifoldpoint.m_normalWorldOnB.y(),
                  btmanifoldpoint.m_normalWorldOnB.z() );
    
    console_print( "%d\n\n", get_milli_time() );
    
    return false;
}

void near_callback( btBroadphasePair &btbroadphasepair,
                   btCollisionDispatcher  &btdispatcher,
                   const btDispatcherInfo &btdispatcherinfo ) {
    
    Mesh *objmesh0 = ( Mesh * )( ( btRigidBody * )
                                      ( btbroadphasepair.m_pProxy0->m_clientObject ) )->getUserPointer();
    
    Mesh *objmesh1 = ( Mesh * )( ( btRigidBody * )
                                      ( btbroadphasepair.m_pProxy1->m_clientObject ) )->getUserPointer();
    
    console_print("Object #0: %s\n", objmesh0->name );
    console_print("Object #1: %s\n", objmesh1->name );
    console_print("%d\n\n", get_milli_time() );
    
    btdispatcher.defaultNearCallback( btbroadphasepair,
                                     btdispatcher,
                                     btdispatcherinfo );
}

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
    
    glViewport(0.0f, 0.0f, width, height);
    glGetIntegerv(GL_VIEWPORT, viewport_matrix);
    
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
    
    init_physic_world();
    
//    dispatcher->setNearCallback( near_callback );

    gContactAddedCallback = contact_added_callback;
    
//    for (int i = 0; i < obj->meshList.size(); i++) {
//        Mesh *mesh = obj->meshList[i];
//        mesh->build(&obj->vertexMgr);
//        mesh->freeVertexData();
//    }
    
    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
        //        OBJ_optimize_mesh( obj, i, 128 );
        mesh->build(&obj->vertexMgr);
        
        if( strstr( mesh->name, "Sphere" ) ) {
            
            add_rigid_body( mesh, SPHERE, 2.0f, 0 );
//            mesh->rigidBody->forceActivationState(ISLAND_SLEEPING);
        }
        if( strstr( mesh->name, "house" ) ) {
            add_rigid_body( mesh, MESH, 0.0f, 0 );
        }
        
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

Mesh *sphere = NULL;

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

//    if (doubleTapped) {
//        
//        nbd::vec3 loc;
//        
//        if (matrixMgr->unproject(tapPoint.x,
//                                 viewport_matrix[ 3 ] - tapPoint.y,
//                                 1.0f,
//                                 &matrixMgr->getModelViewMatrix(),
//                                 &matrixMgr->getProjectionMatrix(),
//                                 viewport_matrix,
//                                 &loc.x,
//                                 &loc.y,
//                                 &loc.z  ))
//        {
//            //            location.y = 1.0;
//            //            btVector3 ray_from( location.x,
//            //                               -10.0,
//            //                               location.z );
//            
//            btVector3 ray_from( location.x,
//                               location.y,
//                               location.z );
//            
//            btVector3 ray_to( loc.x + location.x,
//                             loc.y + location.y,
//                             loc.z + location.z );
//            
//            //            btVector3 ray_to( location.x,
//            //                             location.y,
//            //                             location.z);
//            
//            btCollisionWorld::ClosestRayResultCallback collision_ray( ray_from,
//                                                                     ray_to );
//            
//            dynamicsworld->rayTest( ray_from,
//                                   ray_to,
//                                   collision_ray );
//            
//            if( collision_ray.hasHit()) {
//                collision_ray.m_hitNormalWorld.normalize();
//                
//                btRigidBody *collisionBody = (btRigidBody *)collision_ray.m_collisionObject;
//                Mesh *mesh = (Mesh *)collisionBody->getUserPointer();
//                console_print("collision mesh : %s\n", mesh->name);
//                if( collision_ray.m_hitNormalWorld.z() == 1.0f ) {
//                    btVector3 hitPoint = collision_ray.m_hitPointWorld;
//                    console_print("!!!! collision mesh : %s\n", mesh->name);
//                }
//                sphere = mesh;
//                
//            }
//            else {
//                sphere = NULL;
//            }
//            
//            //            if( collision_ray.hasHit() &&
//            //               collision_ray.m_collisionObject == maze->rigidBody ) {
//            //
//            //                collision_ray.m_hitNormalWorld.normalize();
//            //
//            //                if( collision_ray.m_hitNormalWorld.z() == 1.0f ) {
//            //
//            //                    navigationpath_player.start_location.x = player->location.x;
//            //                    navigationpath_player.start_location.y = player->location.y;
//            //                    navigationpath_player.start_location.z = player->location.z;
//            //
//            //                    navigationpath_player.end_location.x = collision_ray.m_hitPointWorld.x();
//            //                    navigationpath_player.end_location.y = collision_ray.m_hitPointWorld.y();
//            //                    navigationpath_player.end_location.z = collision_ray.m_hitPointWorld.z();
//            //
//            //                    if( NAVIGATION_get_path( navigation,
//            //                                            &navigationpath_player,
//            //                                            &navigationpathdata_player ) ) {
//            //
//            //                        unsigned int i = 0;
//            //                        while( i != navigationpathdata_player.path_point_count + 1 ) {
//            //
//            //                            console_print( "%d: %f %f %f\n",
//            //                                          i,
//            //                                          navigationpathdata_player.path_point_array[ i ].x,
//            //                                          navigationpathdata_player.path_point_array[ i ].y,
//            //                                          navigationpathdata_player.path_point_array[ i ].z );
//            //                            ++i;
//            //                        }
//            //                        printf( "\n" );
//            //                    }
//            //                }
//            //            }
//        }
//        
//        doubleTapped = false;
//    }
    
    unsigned int n = 0;
    for (int i = 0; i < obj->meshList.size(); i++) {
        Mesh *mesh = obj->meshList[i];
        
        mesh->distance = nbd::sphere_distance_in_frustum(frustum,
                                                       &mesh->location,
                                                       mesh->radius);
//        if( mesh->distance ) {
            matrixMgr->pushMatrix();
            
        
        if (mesh->rigidBody) {
            nbd::mat4 mat;
            
            mesh->rigidBody->getWorldTransform().getOpenGLMatrix( ( float * )&mat );
            
            memcpy( &mesh->location, ( nbd::vec3 * )&mat.m[ 3 ], sizeof( nbd::vec3 ) );
            
            matrixMgr->multiply(mat);
        }
        else {
            
            matrixMgr->translate(mesh->location.x,
                                 mesh->location.y,
                                 mesh->location.z );
        }
            glUniformMatrix4fv( program->getUniformLocation((char *)"MODELVIEWPROJECTIONMATRIX"),
                               1,
                               GL_FALSE,
                               ( float * )&matrixMgr->getModelViewProjectionMatrix() );
            
            mesh->draw();
            
            matrixMgr->popMatrix();
            n++;
//        }
    }
    
    dynamicsworld->stepSimulation( 1.0f / 60.0f );

//    console_print( "Visible Objects: %d from total : %d\n", n, obj->meshList.size());
    
//    unsigned int n_manifolds = dynamicsworld->getDispatcher()->getNumManifolds();
//    
//    int i = 0;
//    while( i != n_manifolds )
//    {
//        btPersistentManifold *manifold = dynamicsworld->getDispatcher()->getManifoldByIndexInternal( i );
//        
//        Mesh *objmesh0 = ( Mesh * )( ( btRigidBody * )manifold->getBody0() )->getUserPointer();
//        
//        Mesh *objmesh1 = ( Mesh * )( ( btRigidBody * )manifold->getBody1() )->getUserPointer();
//        
//        unsigned int j = 0,
//        n_contacts = manifold->getNumContacts();
//        
//        while( j != n_contacts )
//        {
//            btManifoldPoint &contact = manifold->getContactPoint( j );
//            
//            console_print("Manifold : %d\n", i );
//            console_print("Contact  : %d\n", j );
//            
//            console_print("Object #0: %s\n", objmesh0->name );
//            console_print("Point  #0: %.3f %.3f %.3f\n",
//                          contact.getPositionWorldOnA().x(),
//                          contact.getPositionWorldOnA().y(),
//                          contact.getPositionWorldOnA().z() );
//            
//            console_print("Object #1: %s\n", objmesh1->name );
//            console_print("Point  #1: %.3f %.3f %.3f\n",
//                          contact.getPositionWorldOnB().x(),
//                          contact.getPositionWorldOnB().y(),
//                          contact.getPositionWorldOnB().z() );
//            
//            console_print("Distance : %.3f\n", contact.getDistance() );
//            console_print("Lifetime : %d\n"  , contact.getLifeTime() );
//            
//            console_print("Normal   : %.3f %.3f %.3f\n",
//                          contact.m_normalWorldOnB.x(),
//                          contact.m_normalWorldOnB.y(),
//                          contact.m_normalWorldOnB.z() );
//            
//            console_print( "%d\n\n", get_milli_time() );
//            
//            ++j;
//        }
//        
//        ++i;
//    }
}

void nbManager::toucheBegan( float x, float y, unsigned int tap_count )
{
    doubleTapped = true;
    
    tapPoint.x = x;
    tapPoint.y = y;
    
    if (doubleTapped) {
        
        nbd::vec3 loc;
        
        if (matrixMgr->unproject(tapPoint.x,
                                 viewport_matrix[ 3 ] - tapPoint.y,
                                 1.0f,
                                 &matrixMgr->getModelViewMatrix(),
                                 &matrixMgr->getProjectionMatrix(),
                                 viewport_matrix,
                                 &loc.x,
                                 &loc.y,
                                 &loc.z  ))
        {
            //            location.y = 1.0;
            //            btVector3 ray_from( location.x,
            //                               -10.0,
            //                               location.z );
            
            btVector3 ray_from( location.x,
                               location.y,
                               location.z );
            
            btVector3 ray_to( loc.x + location.x,
                             loc.y + location.y,
                             loc.z + location.z );
            
            //            btVector3 ray_to( location.x,
            //                             location.y,
            //                             location.z);
            
            btCollisionWorld::ClosestRayResultCallback collision_ray( ray_from,
                                                                     ray_to );
            
            dynamicsworld->rayTest( ray_from,
                                   ray_to,
                                   collision_ray );
            
            if( collision_ray.hasHit()) {
                collision_ray.m_hitNormalWorld.normalize();
                
                btRigidBody *collisionBody = (btRigidBody *)collision_ray.m_collisionObject;
                Mesh *mesh = (Mesh *)collisionBody->getUserPointer();
                console_print("collision mesh : %s\n", mesh->name);
                if( collision_ray.m_hitNormalWorld.z() == 1.0f ) {
                    btVector3 hitPoint = collision_ray.m_hitPointWorld;
                    console_print("!!!! collision mesh : %s\n", mesh->name);
                }
                if (strstr(mesh->name, "Sphere")) {
                    sphere = mesh;
                }
                else {
                sphere = NULL;
                }
                
            }
            else {
                sphere = NULL;
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
    
    
    if( sphere) {
        
        
        sphere->rigidBody->setLinearVelocity(
                                           btVector3( CLAMP( ( y - tapPoint.y ) * 0.1f, 0.0f, 10.0f ),
                                                     0.0f,
                                                     CLAMP( ( x - tapPoint.x ) * 0.1f, 0.0f, 10.0f ) ) );
        sphere->rigidBody->forceActivationState( ACTIVE_TAG );
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
