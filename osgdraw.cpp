#include "osgdraw.h"
#include "physics.h"

#include <osg/TextureCubeMap>
#include <osg/TexEnvCombine>
#include <osg/TexGen>
#include <osg/Texture>

#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <iostream>

#include "hud.h"

// btRigidBody* tempWall;

using std::ifstream;

extern HUDElement* hud;; 
extern osgText::Text* textOne;
extern osg::ref_ptr<osg::Group> root;
extern osg::ref_ptr<osg::Switch> QSwitch;
extern osg::ref_ptr<osg::Projection> QProjection;
extern bool questionDisplayed;



// osg::ref_ptr<osg::Switch> powerUpSwitch; 
osg::ref_ptr<osg::Geode> basicShapesGeode; 

osg::ref_ptr<osg::Geode> createSide(osg::ref_ptr<osg::Vec3Array> corners, osg::ref_ptr<osg::Vec3Array> normal, osg::ref_ptr<osg::Image> image)
{
    // Create an object to store geometry in.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    // Create an array of four vertices.
    // geom->setVertexArray( v.get() );
    geom->setVertexArray( corners.get() );
    // Create an array of four colors.

    int lengthX = (int)((*corners)[1] + (-(*corners)[0])).length();
    int lengthY = (int)((*corners)[1] + (-(*corners)[2])).length();
    // std::cout << "Imageeeeeeeeeee " << image.get()->s() << "," << image.get()->t() << "\n";
    // std::cout << (*corners)[0].x() << ", " << (*corners)[0].y() << ", " <<(*corners)[0].z() << "\n";
    // std::cout << (*corners)[1].x() << ", " << (*corners)[1].y() << ", " <<(*corners)[1].z() << "\n";
    // std::cout << (*corners)[2].x() << ", " << (*corners)[2].y() << ", " <<(*corners)[2].z() << "\n";
    // std::cout << (*corners)[3].x() << ", " << (*corners)[3].y() << ", " <<(*corners)[3].z() << "\n";
    // std::cout << lengthX << ", " << lengthY << "\n";
    float ratioX = ((float)lengthX)/(image.get()->s());
    float ratioY = ((float)lengthY)/(image.get()->t());

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
   (*mTexcoords)[1].set(ratioX,0.0f);
   (*mTexcoords)[2].set(ratioX,ratioY);
   (*mTexcoords)[3].set(0.0f,ratioY);

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
    // tempWall = createRigidWall(centerOfMass, halfExtents, direction);
    createRigidWall(centerOfMass, halfExtents, direction);
    // printf("Just after creation : %p\n", tempWall);
//     std::cout << tempWall->getUserPointer()

    // Pos Y face
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, 0)); 
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, 0));
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, height)); 
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3( 0.f, 1.f, 0.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/texture.tga")));

    // Neg Y face
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, 0)); 
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, 0));
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, height)); 
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3( 0.f, -1.f, 0.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/texture.tga")));

    // Top
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, height)); 
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, height));
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, height)); 
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3( 0.f, 0.f, 1.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/texture.tga")));

    // left 
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, 0)); 
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, 0));
    vertices->push_back(osg::Vec3(comX + heX, comY + heY, height)); 
    vertices->push_back(osg::Vec3(comX + heX, comY - heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3( 1.f, 0.f, 0.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/texture.tga")));

    // right
    vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, 0)); 
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, 0));
    vertices->push_back(osg::Vec3(comX - heX, comY - heY, height)); 
    vertices->push_back(osg::Vec3(comX - heX, comY + heY, height));

    normal = new osg::Vec3Array;
    normal->push_back( osg::Vec3(-1.f, 0.f, 0.f ) );
    wall->addChild(createSide(vertices, normal ,osgDB::readImageFile("./data/texture.tga")));

    return wall;
}

// osg::ref_ptr<osg::Group> createWall(int comX, int comY, int halfWidth, int halfThickness, int height, int isXPointing)

// Maze generation details- the python scripts generates a maze that is a quad
// with vertices (0,0), (10,10) as endpoints. We scale up by a constant factor
// of 400 and halfwidth fixed at 200
osg::ref_ptr<osg::Group> createWalls()
{
    osg::ref_ptr<osg::Group> grp = new osg::Group;

    std::ifstream mazeFile;
    std::ofstream checkFile;
    mazeFile.open("mazeTest3.txt");
    while (mazeFile.good())
    {   
            
        float CoMX,CoMY;
        float acCoMX, acCoMY;
        char dir;
        int isXPointing;
        int halfWidth;

        mazeFile >> CoMX >> CoMY >> dir;
        std::cout << "Read the wall " << CoMX << CoMY << dir << "\n";

        acCoMX = CoMX*400 - 2000;
        acCoMY = (10-CoMY)*400 - 2000;
        
        if (dir == 'X') isXPointing = 1;
        else isXPointing = 0;
        
        std::cout << "Creating the wall " << acCoMX << acCoMY << "  " <<  isXPointing << std::endl;
        grp->addChild(createWall(acCoMX, acCoMY, 200, 10, 500, isXPointing));
    }

    mazeFile.close();
    checkFile.close();
    // Border walls dont seem to work right now, so adding them temporarily this
    // way
    grp->addChild(createWall(2200,0,2200,10,500,1));
    grp->addChild(createWall(-2200,0,2200,10,500,1));
    grp->addChild(createWall(0,2200,2200,10,500,0));
    grp->addChild(createWall(0,-2200,2200,10,500,0));

    return grp;
}

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

osg::ref_ptr<osg::Switch> createPowerUp(int x, int y, int z)
{
    int rad = 10;

    osg::ref_ptr<osg::Sphere> sph = new osg::Sphere(osg::Vec3(x, y, z), rad);
    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(sph);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(drawable);

    osg::ref_ptr<osg::Switch> swt= new osg::Switch;
    swt->addChild(geode.get(), true);


 //   drawable->setColor(osg::Vec4(1,0,0,1));

    // Rigid body
    createRigidPowerUp(btVector3(x,y,z), rad, swt, geode);

    // Add textures, make it look good
    osg::Texture2D* puTexture = new osg::Texture2D;
    puTexture->setDataVariance(osg::Object::DYNAMIC);
    // Read the texture image
    osg::Image* teximage = osgDB::readImageFile("data/puTexture.png");
    if (!teximage)
    {
        std:: cout << "Texture not found, quitting." << std::endl;
        exit(0);
    }

    puTexture->setImage(teximage);

    // Declare a state for the blend texture mode
    osg::StateSet* blendStateSet = new osg::StateSet();

   // Declare a TexEnv instance, set the mode to 'BLEND'
   osg::TexEnv* blendTexEnv = new osg::TexEnv;
   blendTexEnv->setMode(osg::TexEnv::BLEND);
   blendStateSet->setTextureAttributeAndModes(0,puTexture,osg::StateAttribute::ON);
   blendStateSet->setTextureAttribute(0,blendTexEnv);
   geode->setStateSet(blendStateSet);
    return swt;

}




osg::ref_ptr<osg::Group> createPowerUps()
{

    osg::ref_ptr<osg::Group> grp = new osg::Group;

    ifstream mazePUFile;
    // mazeFile.open("./mazeFiles/maze-1.txt");
    mazePUFile.open("./mazeFiles/powerup-map.txt");
    while (mazePUFile.good())
    {   
        int posX, posY, posZ;

        mazePUFile >> posX >> posY >> posZ;

        grp->addChild(createPowerUp(posX, posY, posZ));
    }
    return grp;

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
    if (questionDisplayed) 
    {
        if ((ea.getKey() == '1') || (ea.getKey() == '2') )
        {
            questionDisplayed=false;
            QSwitch->setChildValue(QProjection,false);
            hud->IncreementScore();

        }


        else
        {
            model->clearCycle(currentAnimation,0.0);
            palPos->stopAdvance();
            return false;
        }
    }
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYDOWN):
            {
                if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
                {
                    currentAnimation = '5'-'1';

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

                    //  palPos->increementAngle();
                    palPos->startLeft();
                    // currentAngle += 0.05;
                    // rigidModel->setLinearVelocity(btVector3(100*sin(currentAngle),-100*cos(currentAngle),0));
                }
                else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
                {
                    /// currentAngle -= 0.05;
                    // rigidModel->setLinearVelocity(btVector3(100*sin(currentAngle),-100*cos(currentAngle),0));
                    // palPos->decreementAngle();
                    palPos->startRight();
                }
                else if( ea.getKey() == 'z')
                {
                    if(rigidModel->getCenterOfMassPosition().getZ() == 0)
                        rigidModel->applyCentralImpulse(btVector3(0,0,30000));
                }
                else if(ea.getKey() == 'c')
                {

                    if(cameraPos == 0)
                    {
                        // This is the top view.
                        viewer->getCameraManipulator()->setHomePosition(osg::Vec3(0, 0, 2000), osg::Vec3(0, 0, 0), osg::Vec3(0, -1, 0),  false );
                    }
                    else if(cameraPos == 1)
                    {
                        viewer->getCameraManipulator()->setHomePosition( osg::Vec3(0, 600, 800), osg::Vec3(0, 0, 300), osg::Vec3(0, 0, 1),  false );
                    }
                    else if(cameraPos == 2)
                    {
                        viewer->getCameraManipulator()->setHomePosition(osg::Vec3(0,200, 200),osg::Vec3(0, 0, 200), osg::Vec3(0, 0, 1),  false );
                    }

                    cameraPos = (cameraPos+1)%3;

                    viewer->home(); 

                }

                else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Return)
                {
                    hud->RemoveInitScreen();
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
                else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
                {
                    palPos->stopLeft();
                }
                else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
                {
                    palPos->stopRight();
                }
            }
        default: break;
    }

    return false;
}


