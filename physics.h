/*
 * Physics simulation for collision detection 
 */
#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H 

#include <iostream>

#include <osgDB/ReadFile>
#include <osg/CoordinateSystemNode>
#include <osg/PositionAttitudeTransform>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <btBulletDynamicsCommon.h>

btDynamicsWorld *m_dynamicsWorld;
btRigidBody* rigidWall;
btRigidBody* rigidModel;

class RootUpdateCallback : public osg::NodeCallback
{
    public:
        RootUpdateCallback()
        {
            frame_tick = osg::Timer::instance()->tick();
        }
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {

            // Physics update
            osg::Timer_t now_tick = osg::Timer::instance()->tick();
            float dt = osg::Timer::instance()->delta_s(frame_tick, now_tick);
            frame_tick = now_tick;
            /* int numSimSteps = */
            m_dynamicsWorld->stepSimulation(dt); //, 10, 0.01);
            m_dynamicsWorld->updateAabbs();


            traverse(node, nv);
        }

        osg::Timer_t frame_tick; 
};

class ModelUpdateCallback: public osg::NodeCallback
{
    private:
        btRigidBody *_body;
        bool print;

    public:
        ModelUpdateCallback(btRigidBody *body) :
            _body(body)
    {
             std::cout << _body->getLinearVelocity().x() << ", " << _body->getLinearVelocity().y() << ", " << _body->getLinearVelocity().z() << std::endl;
             print = true;
    }

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            btScalar m[16];

            if(print)
            {
             std::cout << "---" << _body->getLinearVelocity().x() << ", " << _body->getLinearVelocity().y() << ", " << _body->getLinearVelocity().z() << std::endl;
             // print = false;
            }

            btDefaultMotionState* myMotionState = (btDefaultMotionState*) _body->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);

            osg::Matrixf mat(m);

            osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *> (node);
            pat->setPosition(mat.getTrans());
            pat->setAttitude(mat.getRotate());

            // std::cout << mat.getTrans().x() << ", " << mat.getTrans().y() << ", " << mat.getTrans().z() << std::endl;
            // std::cout << _body->getLinearVelocity().x() << ", " << _body->getLinearVelocity().y() << ", " << _body->getLinearVelocity().z() << std::endl;
            
            if(print)
            {
             std::cout << "---" << _body->getLinearVelocity().x() << ", " << _body->getLinearVelocity().y() << ", " << _body->getLinearVelocity().z() << std::endl;
             std::cout << mat.getTrans().x() << ", " << mat.getTrans().y() << ", " << mat.getTrans().z() << std::endl;
             // _body->setLinearVelocity(btVector3(0, -1, 0));
               print = false;
            }

            traverse(node, nv);
        }
};


void createPhysicsWorld()
{
    // World co-ordinates
    btVector3 worldAabbMin(-1000, -1000, -1000);
    btVector3 worldAabbMax(1000, 1000, 1000);
    const int maxProxies = 32766;

    // Create world with every physics component default
    btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;
    btAxisSweep3 *broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);
    btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration); 
    
    // No gravity 
    m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
    return;
}


btRigidBody* createRigidBody(btDynamicsWorld *world, float mass, const btTransform& startTransform, btCollisionShape* shape)
{
    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(1, 1, 1);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody* body = new btRigidBody(mass, myMotionState, shape, localInertia);
    // myMotionState->setRigidBody(body);
    world->addRigidBody(body);
    return body;
}


void createRigidWall(osg::ref_ptr<osg::Geode> wall)
{
    // Plane with normal along X axis and half-extent 100
    btCollisionShape *wall_shape = new btStaticPlaneShape(btVector3(0,1, 0), 100);

    // Attach a rigid body 
    btVector3 pos;
    pos.setValue(0,-275,0);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(pos);
    btScalar mass = 0.f;
    rigidWall = createRigidBody(m_dynamicsWorld, mass, trans,wall_shape);
    rigidWall->setUserPointer(wall);
    rigidWall->setCollisionFlags(rigidWall->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT ); 
    rigidWall->setActivationState(DISABLE_DEACTIVATION);
    return;
}

void createRigidModel(osg::ref_ptr<osgCal::Model> model)
{
    // pat for the model is at (0,0,0)
    btCollisionShape *cyl_shape = new btCylinderShapeZ(btVector3(0,0,0));
    btTransform trans; 
    trans.setIdentity();
    trans.setOrigin(btVector3(0,0,0)); 
    
    rigidModel = createRigidBody(m_dynamicsWorld,btScalar(100.f),trans,cyl_shape);
    rigidModel->setUserPointer(model); 
    rigidModel->setCollisionFlags(rigidModel->getCollisionFlags() | btCollisionObject::CF_CHARACTER_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK); 

    rigidModel->setActivationState(DISABLE_DEACTIVATION);

    rigidModel->setLinearVelocity(btVector3(0,-100,0));
    std::cout << rigidModel->getLinearVelocity().x() << ", " << rigidModel->getLinearVelocity().y() << ", " << rigidModel->getLinearVelocity().z() << std::endl;
    return;
}

#endif /* ifndef PHYSICS_WORLD */

