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
#include <osgCal/CoreModel>
#include <osgCal/Model>

#include <btBulletDynamicsCommon.h>
#include <stdio.h>
#include <stdlib.h>
#include "movement.h"

extern btDynamicsWorld *m_dynamicsWorld;
extern btRigidBody* rigidWall;
extern btRigidBody* rigidModel;

extern osgText::Text* textOne;



void createPhysicsWorld();
void createRigidWall(osg::ref_ptr<osg::Geode> wall);
void detectCollidingObjects();
void createRigidModel(osg::ref_ptr<osgCal::Model> model,palladinPosition* palPosPtr);
void detectCollidingObjects();
void getModelDownCallback(btDynamicsWorld* world,btScalar timestep);
btRigidBody* createRigidBody(btDynamicsWorld *world, float mass, const btTransform& startTransform, btCollisionShape* shape);



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

            osg::Vec3 position = pat->getPosition();
            std::cout << position.x() << " "<< position.y() << " "<< position.z() << " "<<  "\n";


            osg::Vec3 axis(0,0,1);
            osg::Quat att(palPos->currentAngle,axis);
            pat->setAttitude(att);

            currentAngle = palPos->currentAngle;

            std::cout << "Calling detect\n" ;
            detectCollidingObjects();

            btScalar prevZVel = rigidModel->getLinearVelocity().getZ();
            // Set LInear x and y
            if(palPos->advance && movementAllowed)
                rigidModel->setLinearVelocity(btVector3(100*sin(currentAngle),-100*cos(currentAngle),prevZVel));

            else if(!palPos->advance && movementAllowed)
                rigidModel->setLinearVelocity(btVector3(0, 0, prevZVel));

            else if(palPos->advance && !movementAllowed)
                rigidModel->setLinearVelocity(btVector3(0, 100, prevZVel));

            else if(!palPos->advance && !movementAllowed)
                rigidModel->setLinearVelocity(btVector3(0, 100, prevZVel));

            // Else: collision has happened => do nothing, let collision reset
            // the vel. 
            
            /*
            else if(!palPos->advance && !movementAllowed)

            else if(palPos->advance && !movementAllowed)
                _body->setLinearVelocity(btVector3(0, 0, 0));
            */

            traverse(node, nv);
        }
};
















#endif /* ifndef PHYSICS_WORLD */

