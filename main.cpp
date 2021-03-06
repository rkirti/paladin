#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <math.h>
#include <osg/LightSource>
#include <osg/LightModel>
#include <osg/CullFace>
#include <osgCal/CoreModel>
#include <osgCal/Model>
#include <osg/CameraView>
#include <osg/Quat>
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osg/NodeCallback>

#include <osgGA/MatrixManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgText/Text>
#include <osgGA/NodeTrackerManipulator>

#include "movement.h"
#include "physics.h"
#include "skybox.h"
#include "terrain.h"

#include "hud.h"

#include "osgdraw.h"

HUDElement* hud;; 

osg::ref_ptr<osg::Group> root;
osg::ref_ptr<osg::Switch> QSwitch;
osg::ref_ptr<osg::Projection> QProjection;
bool questionDisplayed=false;
QuestionBank gBank;

int main(int argc, char** argv)
{
    gBank.readQuestions(std::string("questions.txt"));

    if(argc < 2)
    {
        std::cout << "Usage: "<< argv[0]<<" path_to_cal3d.cfg \n";
        exit(0);
    }
 
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc, argv);

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    // Create a root node for the scene graph
    root = new osg::Group;

    // Skybox
    root->addChild(createSkyBox());

    // Terrain
    osg::ref_ptr<osg::Group> theFloor = createTerrain("data/floor.png");
    root->addChild(theFloor);

    // Ceiling
 // osg::ref_ptr<osg::Group> theCeiling = createCeiling("data/ceiling.png");
 // root->addChild(theCeiling);

        // Model
    osg::ref_ptr<osgCal::Model> model = createModel(argv[1]);

    // Create a PAT which helps in moving the model around
    // Place the PAT at 0,0,0
    osg::Vec3 newPos(0,0,0);
    osg::PositionAttitudeTransform* pat =  new osg::PositionAttitudeTransform;
    pat->setPosition(newPos);

    // Add the model under the PAT
    pat->addChild(model);

    // Add pat to the root
    root->addChild(pat);


    // Reading the Chalice node
    osg::ref_ptr<osg::Node> cauldronNode = osgDB::readNodeFile("cauldron.osg");
    osg::PositionAttitudeTransform* cessnaPat = new osg::PositionAttitudeTransform();
    cessnaPat->setPosition(osg::Vec3(800,950,50));
    osg::MatrixTransform* mScale  = new osg::MatrixTransform();
    mScale->setMatrix(osg::Matrixd::scale(10,10,10));
    root->addChild(cessnaPat);
    cessnaPat->addChild(mScale);
    mScale->addChild(cauldronNode);

    // Add the HUD for the question. Keep it under a switch
    // so that its invisible for now
    QSwitch = new osg::Switch();
    QProjection = displayQuestion();
    root->addChild(QSwitch);
    QSwitch->addChild(QProjection, false);

    // init physics
    createPhysicsWorld();

    // interface between the keyboard handler and the update callback
    // The palladinPosition class has all details of the model's motion
    // and position and provides functions to change them.
    palladinPosition palPos;

    // Rigid bodies
    root->addChild(createWalls());
    createRigidModel(model,&palPos);

    // Powerups
    root->addChild(createPowerUps());
    
    // HUD
	hud = new HUDElement(root, 1200, 800);
	hud->DefineHUDQuad( 1200, 150, 600, 75, "data/floor.png", 11);
    
    

    // Keyboard handler
    viewer.addEventHandler( new AnimationToggleHandler( model , &palPos , &viewer));

    // Add the scene graph to the viewer
    viewer.setSceneData(root);

    // Set up the pat updates
    pat->setUpdateCallback(new ModelUpdateCallback(rigidModel, &palPos));

    // set up the camera manipulators.
    {
        // This allows the camera manipulator to be switched at the press of a
        // key
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator());
   //     keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
  //      keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
  //      keyswitchManipulator->addMatrixManipulator('4', "Terrain", new osgGA::TerrainManipulator());

        std::string pathfile;
        char keyForAnimationPath = '5';
        while (arguments.read("-p", pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid())
            {
                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator(keyForAnimationPath, "Path", apm);
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator(keyswitchManipulator.get());
    }

    // add the state manipulator
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    // add the thread model handler
    viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the help handler
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    // add the screen capture handler
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

    // Make the camera follow the model if the user asked for it
    // int followCamera = atoi(argv[2]);
    // if(followCamera)
    followTheModel(&viewer, model);

    // Set initial position of the camera
    viewer.getCameraManipulator()->setHomePosition( osg::Vec3(0, 600, 800), osg::Vec3(0, 0, 300), osg::Vec3(0, 0, 1),  false );
    viewer.home(); 

    // create the windows and run the threads.
    viewer.realize();

    osg::Timer_t frame_tick = osg::Timer::instance()->tick();
    while (!viewer.done())
    {
        // Physics update
        osg::Timer_t now_tick = osg::Timer::instance()->tick();
        float dt = osg::Timer::instance()->delta_s(frame_tick, now_tick);
        frame_tick = now_tick;
        
        m_dynamicsWorld->stepSimulation(dt); //, 10, 0.01);
        m_dynamicsWorld->updateAabbs();
       
        // render
        viewer.frame();
    }
    return 0;
}
