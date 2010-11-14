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

osgText::Text* textOne;

#include "osgdraw.h"
#include "movement.h"
#include "physics.h"
#include "skybox.h"
#include "terrain.h"

#include "hud.h"



int main(int argc, char** argv)
{
    if(argc < 3)
    {
        std::cout << "Usage: "<< argv[0]<<" path_to_cal3d.cfg follow_camera?(1 or 0)\n";
        exit(0);
    }
 
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc, argv);

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    // Create a root node for the scene graph
    osg::ref_ptr<osg::Group> root = new osg::Group;

    root->addChild(createSkyBox());

    // Add the wall (in the y plane)
    // (-100, -200, -100)
    // (-100, -200,  100)
    // ( 100, -200,  100)
    // ( 100, -200, -100)
    osg::ref_ptr<osg::Geode> theWall = createWall();
    root->addChild(theWall);

    // osg::ref_ptr<osg::Geode> theFloor = createTerrain("../../bullet-test/data/grass.png");
    osg::ref_ptr<osg::Group> theFloor = createTerrain("data/floor.png");
    root->addChild(theFloor);

    // root->addChild(osgDB::readNodeFile("skydome.osg"));

    // Load the model
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

    root->addChild(createHUD());

    /***** Scene graph created *********/
    /*
     * root (Group)
     * |
     * L----> wall (Geode, which contains a Geometry)
     * |
     * L----> pat (PositionAttitudeTransform)
     *        |
     *        L----> model (Model)
     */

    // interface between the keyboard handler and the update callback
    palladinPosition palPos;

    // Each time the PAT is rendered, check 'palPos' contents to see the
    // position of the model  
    // Moved to later-  when the rigid body for the paladin is setup 
    // pat->setUpdateCallback(new updatePalPos(&palPos));

    // Keyboard handler
    viewer.addEventHandler( new AnimationToggleHandler( model , &palPos));

    // Add the scene graph to the viewer
    viewer.setSceneData(root);


    // Physics
    createPhysicsWorld();
    createRigidWall(theWall);
    createRigidModel(model,&palPos);

    // Set up the pat updates
    pat->setUpdateCallback(new ModelUpdateCallback(rigidModel, &palPos));

    // root->setUpdateCallback(new RootUpdateCallback);


    // Start the show
    // viewer.run();

    // set up the camera manipulators.
    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator());
        keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
        keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
        keyswitchManipulator->addMatrixManipulator('4', "Terrain", new osgGA::TerrainManipulator());

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
    int followCamera = atoi(argv[2]);
    if(followCamera)
        followTheModel(&viewer, model);

    // Set initial position of the camera
    viewer.getCameraManipulator()->setHomePosition( osg::Vec3(0, 500, 200), osg::Vec3(0, 0, 200), osg::Vec3(0, 0, 1),  false );
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
