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
#include <stdio.h>

btDynamicsWorld *m_dynamicsWorld;
btRigidBody* rigidWall;
btRigidBody* rigidModel;
bool movementDisallowed=false;

class ModelUpdateCallback: public osg::NodeCallback
{
    private:
        btRigidBody *_body;
        bool print;
        palladinPosition *palPos;
        float currentAngle;

    public:
        ModelUpdateCallback(btRigidBody *body, palladinPosition *palPosPtr) :
            _body(body)
    {
             palPos = palPosPtr;
    }

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            btScalar m[16];

            btDefaultMotionState* myMotionState = (btDefaultMotionState*) _body->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);

            osg::Matrixf mat(m);

            osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *> (node);
            pat->setPosition(mat.getTrans());
            pat->setAttitude(mat.getRotate());

            osg::Vec3 axis(0,0,1);
            osg::Quat att(palPos->currentAngle,axis);
            pat->setAttitude(att);

            if(palPos->advance)
            {
                currentAngle = palPos->currentAngle;
                rigidModel->setLinearVelocity(btVector3(100*sin(currentAngle),-100*cos(currentAngle),0));
            }
            else
                _body->setLinearVelocity(btVector3(0, 0, 0));

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

    // btCollisionShape *wall_shape = new btStaticPlaneShape(btVector3(0,1, 0), 100);
    btCollisionShape *wall_shape = new btBoxShape(btVector3(500,10, 500));

    // Attach a rigid body 
    btVector3 pos;
    pos.setValue(0,0,0);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(pos);
    btScalar mass = 0.f;
    rigidWall = createRigidBody(m_dynamicsWorld, mass, trans,wall_shape);
    rigidWall->setUserPointer(wall);
    rigidWall->setCollisionFlags(rigidWall->getCollisionFlags() ); 
    rigidWall->setActivationState(DISABLE_DEACTIVATION);
    return;
}

void createRigidModel(osg::ref_ptr<osgCal::Model> model)
{
    // pat for the model is at (0,0,0)
    btCollisionShape *cyl_shape = new btCylinderShapeZ(btVector3(10,10,100));
    btTransform trans; 
    trans.setIdentity();
    trans.setOrigin(btVector3(0, 200,0)); 
    
    rigidModel = createRigidBody(m_dynamicsWorld,btScalar(100.f),trans,cyl_shape);
    rigidModel->setUserPointer(model); 
    rigidModel->setCollisionFlags(rigidModel->getCollisionFlags() | btCollisionObject::CF_CHARACTER_OBJECT ); 

    rigidModel->setActivationState(DISABLE_DEACTIVATION);


    return;
}


void detectCollidingObjects()
{
    int numManifolds =  m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	
        if ((obA == rigidWall || obB == rigidWall) && (obA == rigidWall || obB == rigidWall)) 
        {
            movementDisallowed = true;
            std::cout << "Collision between model and wall detected" << std::endl;
            std::cout << "Model coordinates are: " << rigidModel->getCenterOfMassPosition().x() << ","
                << rigidModel->getCenterOfMassPosition().y() << ","
                << rigidModel->getCenterOfMassPosition().z() << std::endl << std::endl;
        }
        else 
        {
            movementDisallowed = false;
        }


		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			glBegin(GL_LINES);
			glColor3f(0, 0, 0);
			
			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();

			glVertex3d(ptA.x(),ptA.y(),ptA.z());
			glVertex3d(ptB.x(),ptB.y(),ptB.z());
			glEnd();
		}
    }



}
#endif /* ifndef PHYSICS_WORLD */

