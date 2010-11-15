#include "osgdraw.h"
#include "physics.h"

#include <osg/TextureCubeMap>
#include <osg/TexEnvCombine>
#include <osg/TexGen>
#include <osg/Texture>


osg::ref_ptr<osg::Switch> powerUpSwitch; 
osg::ref_ptr<osg::Geode> basicShapesGeode; 

osg::ref_ptr<osg::Geode> createSide(osg::ref_ptr<osg::Vec3Array> corners, osg::ref_ptr<osg::Vec3Array> normal, osg::ref_ptr<osg::Image> image)
{
    // Create an object to store geometry in.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    // Create an array of four vertices.
    // geom->setVertexArray( v.get() );
    geom->setVertexArray( corners.get() );
    // Create an array of four colors.

    // Create an array for the single normal.
    geom->setNormalArray( normal.get() );
    geom->setNormalBinding( osg::Geometry::BIND_OVERALL );

    // Draw a four-vertex quad from the stored data.
    geom->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, 4 ) );

    // Add the Geometry (Drawable) to a Geode and
    // return the Geode.
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom.get() );

    osg::Vec2Array *mTexcoords = new osg::Vec2Array(8);
   (*mTexcoords)[0].set(0.0f,0.0f);
   (*mTexcoords)[1].set(2.0f,0.0f);
   (*mTexcoords)[2].set(2.0f,2.0f);
   (*mTexcoords)[3].set(0.0f,2.0f);

   geom->setTexCoordArray(0,mTexcoords);
    // Colors
   osg::Texture2D* mHUDTexture = new osg::Texture2D;
   mHUDTexture->setDataVariance(osg::Object::DYNAMIC);
   mHUDTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
   mHUDTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT); 
   mHUDTexture->setImage(image.get());

   osg::StateSet* HUDStateSet = new osg::StateSet();
   geode->setStateSet(HUDStateSet);
   HUDStateSet->setTextureAttributeAndModes(0,mHUDTexture,osg::StateAttribute::ON);

    return geode;

}

osg::ref_ptr<osg::Group> createWall(int comX, int comY, int halfWidth, int halfThickness, int height, int isXPointing)
{
    int heX, heY;

    osg::ref_ptr<osg::Group> wall = new osg::Group;

    osg::ref_ptr<osg::Vec3Array> vertices;
    osg::ref_ptr<osg::Vec3Array> normal;

    NORMAL_DIRN direction;

    if(isXPointing)
    {
        heX = halfThickness;
        heY = halfWidth;
        direction = NORMAL_X;
    }
    else
    {
        heY = halfThickness;
        heX = halfWidth;
        direction = NORMAL_Y;
    }

/////////btRigidBody* createRigidWall(btVector3 centerOfMass,btVector3 halfExtents,NORMAL_DIRN direction);
    btVector3 centerOfMass(comX, comY, height/2);
    btVector3 halfExtents(heX, heY, height/2);
    createRigidWall(centerOfMass, halfExtents, direction);

    // Pos Y face
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, 0)); 
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, 0));
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, height)); 
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3( 0.f, 1.f, 0.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/wall_light.TGA")));

    // Neg Y face
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, 0)); 
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, 0));
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, height)); 
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3( 0.f, -1.f, 0.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/wall_light.TGA")));

    // Top
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, height)); 
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, height));
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, height)); 
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3( 0.f, 0.f, 1.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/wall_light.TGA")));

    // left 
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, 0)); 
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, 0));
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, height)); 
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3( 1.f, 0.f, 0.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/wall_light.TGA")));

    // right
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, 0)); 
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, 0));
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, height)); 
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3(-1.f, 0.f, 0.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/wall_light.TGA")));

    return wall;
}

// osg::ref_ptr<osg::Group> createWall(int comX, int comY, int halfWidth, int halfThickness, int height, int isXPointing)

osg::ref_ptr<osg::Group> createWalls()
{
    osg::ref_ptr<osg::Group> grp = new osg::Group;

    grp->addChild(createWall(0,0,50,5,50,0));
    grp->addChild(createWall(50,50,50,5,50,1));
    grp->addChild(createWall(-50,50,50,5,50,1));

    return grp;
}
/*
osg::ref_ptr<osg::Geode> createWall()
{
    // Create an object to store geometry in.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    // Create an array of four vertices.
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
    geom->setVertexArray( v.get() );
    float halfWidth = 100.0;
    v->push_back( osg::Vec3( -halfWidth, 0.f, -halfWidth) );
    v->push_back( osg::Vec3( -halfWidth, 0.f,  halfWidth) );
    v->push_back( osg::Vec3(  halfWidth, 0.f,  halfWidth) );
    v->push_back( osg::Vec3(  halfWidth, 0.f, -halfWidth) );

    // Create an array of four colors.
    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    geom->setColorArray( c.get() );
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    c->push_back( osg::Vec4( 1.f, 0.f, 0.f, 1.f ) );
    c->push_back( osg::Vec4( 0.f, 1.f, 0.f, 1.f ) );
    c->push_back( osg::Vec4( 0.f, 0.f, 1.f, 1.f ) );
    c->push_back( osg::Vec4( 1.f, 1.f, 1.f, 1.f ) );

    // Create an array for the single normal.
    osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
    geom->setNormalArray( n.get() );
    geom->setNormalBinding( osg::Geometry::BIND_OVERALL );
    n->push_back( osg::Vec3( 0.f, -1.f, 0.f ) );

    // Draw a four-vertex quad from the stored data.
    geom->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, 4 ) );

    // Add the Geometry (Drawable) to a Geode and
    // return the Geode.
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom.get() );
    return geode;
}
*/

osg::ref_ptr<osgCal::Model> createModel(const std::string fileName)
{
    // Create and setup core model
    osg::ref_ptr<osgCal::CoreModel > coreModel( new osgCal::CoreModel() );
    osg::ref_ptr<osgCal::BasicMeshAdder > meshAdder( new osgCal::DefaultMeshAdder );
    osg::ref_ptr<osgCal::MeshParameters > p( new osgCal::MeshParameters );
    p->software = true;

    // Load core model from file
    coreModel->load(fileName);

    // Create Model
    osg::ref_ptr<osgCal::Model> model = new osgCal::Model();

    // Load model
    model->load(coreModel.get(), meshAdder.get());   

    return model;
}


void createTestPowerup()
{

    powerUpSwitch = new osg::Switch; 

    // Declare a box class (derived from shape class) instance
    // This constructor takes an osg::Vec3 to define the center
    // and a float to define the height, width and depth.
    // (an overloaded constructor allows you to specify unique
    // height, width and height values.)
    osg::Box* unitCube = new osg::Box( osg::Vec3(50,50,50),50.0f);

    // Declare an instance of the shape drawable class and initialize
    // it with the unitCube shape we created above.
    // This class is derived from 'drawable' so instances of this
    // class can be added to Geode instances.
    osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube);

    // Declare a instance of the geode class:
    basicShapesGeode = new osg::Geode();

    // Add the unit cube drawable to the geode:
    basicShapesGeode->addDrawable(unitCubeDrawable);

    powerUpSwitch->addChild(basicShapesGeode);
}

void disablePowerUp()
{
    powerUpSwitch->setChildValue(basicShapesGeode.get(), false);
}

void followTheModel(osgViewer::Viewer* viewer, osgCal::Model *model)
{
    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;
    viewer->setCameraManipulator( keyswitchManipulator.get() );

    osgGA::NodeTrackerManipulator::TrackerMode trackerMode = osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION;
    osgGA::NodeTrackerManipulator::RotationMode rotationMode = osgGA::NodeTrackerManipulator::TRACKBALL;

    osgGA::NodeTrackerManipulator* tm = new osgGA::NodeTrackerManipulator;
    tm->setTrackerMode( trackerMode );
    tm->setRotationMode( rotationMode );
    tm->setTrackNode(model);

    unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
    keyswitchManipulator->addMatrixManipulator( 's', "sun", tm);
    keyswitchManipulator->selectMatrixManipulator( num );

}

/*
 * The keyboard handler: Every time an event happens, this class's 'handle'
 * function is called. 
 *
 * Action taken: 
 *      Key Press event:
 *
 *          Up arrow:          Start running animation 
 *                             Instruct the rendered to increementing the
 *                             postion of the model 
 *
 *          Left/Right Arrow:  Rotate the model 
 *
 *      Key Release event:
 *
 *          Up arrow:          Stop running animation 
 *                             Instruct the renderer to stop increementing the
 *                             postion of the model
 */

bool AnimationToggleHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
    if (!viewer) return false;

    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYDOWN):
            {
                if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
                {
                    currentAnimation = '5'-'1';

                    // model->blendCycle( currentAnimation, 1.0f, 0.0 );                        
                    model->blendCycle( currentAnimation, 1.0f, 1.0 );                        

                    // rigidModel->setLinearVelocity(btVector3(100*sin(currentAngle),-100*cos(currentAngle),0));
                    // [rkirti] Testing: Allow advance only if no collision
                    // has happened
                    // if (movementAllowed)
                    palPos->startAdvance();
                }
                else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
                {
                    // m_dynamicsWorld->setGravity(btVector3(0,0,-500));
                    // rigidModel->applyCentralImpulse(btVector3(0,0,20000));

                     palPos->increementAngle();
                    // currentAngle += 0.05;
                    // rigidModel->setLinearVelocity(btVector3(100*sin(currentAngle),-100*cos(currentAngle),0));
                }
                else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
                {
                    /// currentAngle -= 0.05;
                    // rigidModel->setLinearVelocity(btVector3(100*sin(currentAngle),-100*cos(currentAngle),0));
                    palPos->decreementAngle();
                }

                break;

            }
        case(osgGA::GUIEventAdapter::KEYUP):
            {
                if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
                {
                    // model->blendCycle( 0, 0.0f, 1.0 );                        
                    model->clearCycle( currentAnimation, 0.0 ); // clear now
                    currentAnimation = -1;

                    // rigidModel->setLinearVelocity(btVector3(0,0,0));
                    palPos->stopAdvance();
                }
            }
        default: break;
    }

    return false;
}


