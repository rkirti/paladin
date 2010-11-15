#include "physics.h"
#include "osgdraw.h"


btDynamicsWorld *m_dynamicsWorld;
btRigidBody* rigidWall;
btRigidBody* rigidBox;
btRigidBody* rigidModel;
bool movementAllowed=true;

extern btRigidBody* tempWall;

void getModelDownCallback(btDynamicsWorld* world,btScalar timestep)
{
    btVector3 curVelocity(rigidModel->getLinearVelocity());
    btVector3 currCoM(rigidModel->getCenterOfMassPosition());


    if ( (curVelocity.getZ() != 0)  | (currCoM.getZ() != 0))
    {
        if((curVelocity.getZ() > 0) | ( currCoM.getZ() > 1))
        {
            rigidModel->setLinearVelocity(btVector3(curVelocity.getX(), curVelocity.getY(),curVelocity.getZ() -10  ));
        }
        else if(currCoM.getZ() <= 1)
        {
           // textOne->setText("In else");
            rigidModel->setLinearVelocity(btVector3(curVelocity.getX(), curVelocity.getY(),0));
            // rigidModel->setCenterOfMassPosition(btVector3(currCoM.getX(), currCoM.getY(),0 ));

            currCoM.setZ(0);
            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(currCoM);
            rigidModel->setWorldTransform(transform);
        }   
    }
}



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

    // Ugly hack to  get the jumping model come down
    m_dynamicsWorld->setInternalTickCallback(getModelDownCallback, m_dynamicsWorld->getWorldUserInfo(),true);
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
    
    std::cout << "*************** From Rigid body *************" ;
    // myMotionState->setRigidBody(body);
    world->addRigidBody(body);
    return body;
}


btRigidBody* createRigidPowerUp(btVector3 centerOfMass,btScalar radius,osg::ref_ptr<osg::Switch> puSwitch,
        osg::ref_ptr<osg::Geode> puGeode)
{
    // Create the spherical shape
    btCollisionShape* sphere_shape = new btSphereShape(radius);
    // Create the rigid body
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(centerOfMass);
    // trans.setOrigin(btVector3(-200, 200, 100));
    btRigidBody* rigidSphere = createRigidBody(m_dynamicsWorld,btScalar(0.f),trans, sphere_shape);
    
    rigidSphere->setUserPointer(new ColliderInfo(puSwitch,puGeode));
    rigidSphere->setCollisionFlags(rigidWall->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT); 
    rigidSphere->setActivationState(DISABLE_DEACTIVATION);
    return rigidSphere;
}


btRigidBody* createRigidWall(btVector3 centerOfMass,btVector3 halfExtents,NORMAL_DIRN direction)
{
    // Create a box shape with the given halfExtents
    btCollisionShape* wall_shape = new btBoxShape(halfExtents);

    // Create the rigid body
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(centerOfMass);
    btRigidBody* rigidWall = createRigidBody(m_dynamicsWorld,btScalar(0.f),trans, wall_shape);

    // Set the wall info for collision
    if (direction == NORMAL_X)
        rigidWall->setUserPointer(new ColliderInfo(centerOfMass,btVector3(1,0, 0)));
    else
        rigidWall->setUserPointer(new ColliderInfo(centerOfMass,btVector3(0,1, 0)));
    
    rigidWall->setCollisionFlags(rigidWall->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT); 
    rigidWall->setActivationState(DISABLE_DEACTIVATION);

    return rigidWall;
}


void createRigidModel(osg::ref_ptr<osgCal::Model> model,palladinPosition* palPosPtr)
{
    // pat for the model is at (0,0,0)
    btCollisionShape *cyl_shape = new btCylinderShapeZ(btVector3(2,2,100));
    btTransform trans; 
    trans.setIdentity();
    trans.setOrigin(btVector3(0, 200,0)); 

    rigidModel = createRigidBody(m_dynamicsWorld,btScalar(100.f),trans,cyl_shape);
    rigidModel->setUserPointer(palPosPtr); 
    rigidModel->setCollisionFlags(rigidModel->getCollisionFlags() | btCollisionObject::CF_CHARACTER_OBJECT ); 

    rigidModel->setActivationState(DISABLE_DEACTIVATION);


    return;
}



btVector3 detectCollidingObjects()
{
    int numManifolds =  m_dynamicsWorld->getDispatcher()->getNumManifolds();

    if(numManifolds == 0) 
        return btVector3(0,0,0);
    else
    {
        for (int i=0;i<numManifolds;i++)
        {
            // get the contact points - the manifold stores pairs of them
            btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

            // get the two colliding bodies
            btRigidBody* obA = static_cast<btRigidBody*>(contactManifold->getBody0());
            btRigidBody* obB = static_cast<btRigidBody*>(contactManifold->getBody1());

            // Get the other colliding body
            if (obA == rigidModel) 
            {
                btBoxShape *box = static_cast<btBoxShape*>((static_cast<btCollisionObject*>(obB))->getCollisionShape());

                if ( ((ColliderInfo*)(obB->getUserPointer()))->type == WALL ) 
                    return   ((ColliderInfo*)(obB->getUserPointer()))->getEffectiveNormal(rigidModel->getCenterOfMassPosition());
                else if ( ((ColliderInfo*)(obB->getUserPointer()))->type == POWER_UP ) 
                {

                    // Call function to destroy the powerup and increment points
                    return btVector3(0,0,0);
                }
            }
            else if (obB == rigidModel)
            {
                btBoxShape *box = static_cast<btBoxShape*>((static_cast<btCollisionObject*>(obA))->getCollisionShape());
                if ( ((ColliderInfo*)(obA->getUserPointer()))->type == WALL ) 
                    return   ((ColliderInfo*)(obA->getUserPointer()))->getEffectiveNormal(rigidModel->getCenterOfMassPosition());
                else if ( ((ColliderInfo*)(obA->getUserPointer()))->type == POWER_UP ) 
                {

                    // Call function to destroy the powerup and increment points
                    return btVector3(0,0,0);
                }
            }
        }
    }

    return btVector3(0,0,0);
}

