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
#include <osgGA/NodeTrackerManipulator>

#include "movement.h"
#include "physics.h"

osg::ref_ptr<osg::Geode> createWall()
{
    // Create an object to store geometry in.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    // Create an array of four vertices.
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
    geom->setVertexArray( v.get() );
    float halfWidth = 100.0;
    v->push_back( osg::Vec3( -halfWidth, -200.f, -halfWidth) );
    v->push_back( osg::Vec3( -halfWidth, -200.f,  halfWidth) );
    v->push_back( osg::Vec3(  halfWidth, -200.f,  halfWidth) );
    v->push_back( osg::Vec3(  halfWidth, -200.f, -halfWidth) );

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

    // Add the wall (in the y plane)
    // (-100, -200, -100)
    // (-100, -200,  100)
    // ( 100, -200,  100)
    // ( 100, -200, -100)
    osg::ref_ptr<osg::Geode> theWall = createWall();
    root->addChild(theWall);

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
    viewer.addEventHandler( new AnimationToggleHandler( model, &palPos ) );

    // Make the camera follow the model if the user asked for it
    int followCamera = atoi(argv[2]);
    if(followCamera)
        followTheModel(&viewer, model);

    // Add the scene graph to the viewer
    viewer.setSceneData(root);


    // Physics
    createPhysicsWorld();
    createRigidWall(theWall);
    createRigidModel(model);

    // Set up the pat updates
    pat->setUpdateCallback(new ModelUpdateCallback(rigidModel));

    // root->setUpdateCallback(new RootUpdateCallback);

    std::cout << rigidModel->getLinearVelocity().x() << ", " << rigidModel->getLinearVelocity().y() << ", " << rigidModel->getLinearVelocity().z() << std::endl;

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

    // create the windows and run the threads.
    viewer.realize();

    osg::Timer_t frame_tick = osg::Timer::instance()->tick();
    while (!viewer.done())
    {
        // Physics update
        osg::Timer_t now_tick = osg::Timer::instance()->tick();
        float dt = osg::Timer::instance()->delta_s(frame_tick, now_tick);
        frame_tick = now_tick;
        /* int numSimSteps = */
        m_dynamicsWorld->stepSimulation(dt); //, 10, 0.01);
        m_dynamicsWorld->updateAabbs();

        // render
        viewer.frame();
    }
    return 0;
}