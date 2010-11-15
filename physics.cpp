#include "physics.h"
#include "osgdraw.h"

enum collidertype{wall,booster};
typedef enum collidertype colliderType;

btDynamicsWorld *m_dynamicsWorld;
btRigidBody* rigidWall;
btRigidBody* rigidBox;
btRigidBody* rigidModel;
bool movementAllowed=true;


class ColliderObject 
{
public: 
        colliderType type;
        osg::ref_ptr<osg::Switch> boosterCollider;
        btBoxShape* wallShapeCollider;
        //btVector3 getEffectiveNormal(btVector3 position);

};

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
            textOne->setText("In else");
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
    
    // myMotionState->setRigidBody(body);
    world->addRigidBody(body);
    return body;
}


void createRigidWall(osg::ref_ptr<osg::Geode> wall)
{
    // Plane with normal along X axis and half-extent 100

    // btCollisionShape *wall_shape = new btStaticPlaneShape(btVector3(0,1, 0), 100);
    btCollisionShape *wall_shape = new btBoxShape(btVector3(100,5, 100));

    // Attach a rigid body 
    btVector3 pos;
    pos.setValue(0,0,0);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(pos);
    btScalar mass = 0.f;
    rigidWall = createRigidBody(m_dynamicsWorld, mass, trans,wall_shape);
    rigidWall->setUserPointer(wall);
    rigidWall->setCollisionFlags(rigidWall->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT); 
    rigidWall->setActivationState(DISABLE_DEACTIVATION);
    return;
}



void createRigidBox(osg::ref_ptr<osg::Switch> box)
{
    btCollisionShape *box_shape = new btBoxShape(btVector3(25,25,25));

    // Attach a rigid body 
    btVector3 pos;
    pos.setValue(50,50,50);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(pos);
    btScalar mass = 0.f;
    rigidBox = createRigidBody(m_dynamicsWorld, mass, trans,box_shape);
    rigidBox->setUserPointer(box);
    rigidBox->setCollisionFlags(rigidWall->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT); 
    return;
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



void detectCollidingObjects()
{
    int numManifolds =  m_dynamicsWorld->getDispatcher()->getNumManifolds();
    printf("%d\n",numManifolds);
    if(numManifolds == 0) movementAllowed = true;
    else
    {
        for (int i=0;i<numManifolds;i++)
        {
            btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
            btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
            btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
            printf("%p   %p   %p   %p\n",obA,obB,rigidModel, rigidBox);

            if ((obA == rigidWall && obB == rigidModel) || (obA == rigidModel && obB == rigidWall)) 
            {
                if (movementAllowed)
                    std::cout  << "Disallowing movement now" << std::endl; 
                movementAllowed = false;
            }
            else 
            {
                if (!movementAllowed)
                    std::cout  << "Allowing movement now" << std::endl; 
                movementAllowed = true;
            }
        
            if ((obA == rigidBox && obB == rigidModel) || (obA == rigidModel && obB == rigidBox)) 
            {
                std::cout << "BLOWING UP THE BOX" << std::endl;
                disablePowerUp();
                 
            }
        }
    }
}

