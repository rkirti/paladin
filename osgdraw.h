#ifndef  OSGDRAW_H
#define  OSGDRAW_H

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

#include <osg/ShapeDrawable>
#include <osg/Group>

#include "movement.h"
#include "physics.h"


// osg::ref_ptr<osg::Geode> createWall();
osg::ref_ptr<osgCal::Model> createModel(const std::string fileName);
void followTheModel(osgViewer::Viewer* viewer, osgCal::Model *model);

osg::ref_ptr<osg::Group> createWalls();

void createTestPowerup();
void disablePowerUp();

class AnimationToggleHandler : public osgGA::GUIEventHandler 
{
    public: 

        AnimationToggleHandler( osgCal::Model* m, palladinPosition *palPosPtr)
            : model( m )
            , currentAnimation( -1 )
        {
            palPos = palPosPtr;
        }
        
        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
        
    
    private:

        osgCal::Model*              model;
        // std::vector< std::string >  animationNames;
        int                         currentAnimation;
        palladinPosition *palPos;
};

#endif 
