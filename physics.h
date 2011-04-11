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

#include "hud.h"

extern HUDElement* hud;

enum collider_type{POWER_UP,WALL};
typedef enum collider_type COLLIDER_TYPE;

enum normal_dirn{NORMAL_X,NORMAL_Y};
typedef enum normal_dirn NORMAL_DIRN;


extern btDynamicsWorld *m_dynamicsWorld;
extern btRigidBody* rigidWall;
extern btRigidBody* rigidModel;

extern osgText::Text* textOne;
extern osg::ref_ptr<osg::Group> root;
extern osg::ref_ptr<osg::Switch> QSwitch;
extern osg::ref_ptr<osg::Projection> QProjection;
extern bool questionDisplayed;



void createPhysicsWorld();
btRigidBody* createRigidWall(btVector3 centerOfMass,btVector3 halfExtents,NORMAL_DIRN direction);
btRigidBody* createRigidPowerUp(btVector3 centerOfMass,btScalar radius,osg::ref_ptr<osg::Switch> puSwitch, osg::ref_ptr<osg::Geode> puGeode); 
btVector3 detectCollidingObjects();

void createRigidModel(osg::ref_ptr<osgCal::Model> model,palladinPosition* palPosPtr);
// void detectCollidingObjects();
void getModelDownCallback(btDynamicsWorld* world,btScalar timestep);
btRigidBody* createRigidBody(btDynamicsWorld *world, float mass, const btTransform& startTransform, btCollisionShape* shape);


class ColliderInfo{
    public:
        // What is the object type - PowerUp or Wall ?
        COLLIDER_TYPE type;
        // Information for the wall 
        btVector3 centerOfMass;
        btVector3 normal;
        // Information for the PowerUp
        osg::ref_ptr<osg::Switch> puSwitch;
        osg::ref_ptr<osg::Geode> puGeode;
        bool visible;

        ColliderInfo(btVector3 com, btVector3 norm) : centerOfMass(com), normal(norm)
    {
        type = WALL;
    }
        ColliderInfo(osg::ref_ptr<osg::Switch> pswitch, osg::ref_ptr<osg::Geode> pgeode): puSwitch(pswitch), puGeode(pgeode)
    {
        type = POWER_UP;
        visible = true;
    }
        ~ColliderInfo();


        // Function to be called if the type is POWER_UP
        // Destroy the osg stuff here
        void destroyPowerUp()
        {

            if(visible)
            {
                // Displaty the Question
                QSwitch->setAllChildrenOn();
                questionDisplayed = true;
                
                
                puSwitch->setChildValue(puGeode.get(), false);

                // Score incrementing now done in the keyboard handler if
                // correct key is pressed
            //    hud->IncreementScore();
                visible = false;
            }
        }


        // Function to be called if the type is WALL 
        btVector3 getEffectiveNormal(btVector3 position)
        {
            std::cout << "Model position: ";
            std::cout << position.getX() << ","  << position.getY() << ","  << position.getZ() << "\n"; 
            std::cout << "Wall's Center of Mass: ";
            std::cout << centerOfMass.getX() << ","  << centerOfMass.getY() << ","  << centerOfMass.getZ() << "\n"; 
            std::cout << "Normal direction: ";
            std::cout << normal.getX() << ","  << normal.getY() << ","  << normal.getZ() << "\n"; 

            btVector3 temp;
            temp.setX( position.getX() - centerOfMass.getX() );
            temp.setY( position.getY() - centerOfMass.getY() );
            temp.setZ( position.getZ() - centerOfMass.getZ() );
            if (normal.dot(temp) > 0)
                return normal;
            else return (-1)*normal;
        }
};

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

            if(palPos->left)
                palPos->increementAngle();
            if(palPos->right)
                palPos->decreementAngle();

            osg::Vec3 axis(0,0,1);
            osg::Quat att(palPos->currentAngle,axis);
            pat->setAttitude(att);

            currentAngle = palPos->currentAngle;

            btVector3 reboundVelocity(detectCollidingObjects());

            btScalar prevZVel = rigidModel->getLinearVelocity().getZ();

            std::cout << "Normal : " << reboundVelocity.getX() << ", " << reboundVelocity.getY() << ", " <<reboundVelocity.getZ() << "\n";

            if(reboundVelocity.isZero()) 
                movementAllowed = true;
            else 
                movementAllowed = false;

            std::cout << "movementAllowed : " << movementAllowed << "\n";

            reboundVelocity *= btScalar(300);
            reboundVelocity.setZ(prevZVel);

            // Set LInear x and y
            if(palPos->advance && movementAllowed)
                rigidModel->setLinearVelocity(btVector3(300*sin(currentAngle),-300*cos(currentAngle),prevZVel));

            else if(!palPos->advance && movementAllowed)
                rigidModel->setLinearVelocity(btVector3(0, 0, prevZVel));

            else if(palPos->advance && !movementAllowed)
                rigidModel->setLinearVelocity(reboundVelocity);

            else if(!palPos->advance && !movementAllowed)
                rigidModel->setLinearVelocity(reboundVelocity);

            /*
            else if(palPos->advance && !movementAllowed)
                rigidModel->setLinearVelocity(btVector3(0, 100, prevZVel));

            else if(!palPos->advance && !movementAllowed)
                rigidModel->setLinearVelocity(btVector3(0, 100, prevZVel));
                */

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

