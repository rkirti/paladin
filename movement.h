/*
 * This class stores the current position of the Model, acting as an interface
 *      between the keyboard handler and the renderer. 
 * The keyboard handler updates the values of the members
 * The 'update' phase of the renderer reads the values and updates the position
 *      of the model 
 */
class palladinPosition
{
    public:
        palladinPosition()
        {
            currentAngle = 0;
            advance = false;
        }

        void startAdvance(){advance = true;}
        void  stopAdvance(){advance = false;}

        void increementAngle(){ currentAngle+=0.05;}
        void decreementAngle(){ currentAngle-=0.05;}

        float currentAngle; 
        bool advance;
};

/*
 * This class's 'operator()' is called everytime the Model is drawn. It reads
 * the coordinate info from a 'palladinPosition' class and positions the model
 * appropriately. 
 */
class updatePalPos : public osg::NodeCallback {
    public:
        updatePalPos(palladinPosition *posPtr)
        {
            palPos = posPtr;
            currentPosition.set(0.0, 0.0, 0.0);
        }

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osg::PositionAttitudeTransform* pat = dynamic_cast<osg::PositionAttitudeTransform*> (node);
            if(pat)
            {
                if(palPos->advance)
                {
                    float currX = currentPosition.x();
                    float currY = currentPosition.y();

                    currentPosition.set(currX + 1*sin(palPos->currentAngle), currY - 1*cos(palPos->currentAngle), 0);

                    pat->setPosition(currentPosition);
                }

                osg::Vec3 axis(0,0,1);
                osg::Quat att(palPos->currentAngle,axis);
                pat->setAttitude(att);

            }
            traverse(node, nv); 
        }

    protected:
        palladinPosition *palPos;
        osg::Vec3d currentPosition; 
        osg::ref_ptr<osg::Camera> myCamera;
        osg::ref_ptr<osgViewer::Viewer> myViewer; 
};

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
class AnimationToggleHandler : public osgGA::GUIEventHandler 
{
    public: 

        AnimationToggleHandler( osgCal::Model* m, palladinPosition* posPtr)
            : model( m )
            , currentAnimation( -1 ), palPos(posPtr)
        {
        }
        
        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
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

                        palPos->startAdvance();
                    }
                    else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
                        palPos->increementAngle();
                    else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
                        palPos->decreementAngle();

                    break;

                }
                case(osgGA::GUIEventAdapter::KEYUP):
                {
                    if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
                    {
                        // model->blendCycle( 0, 0.0f, 1.0 );                        
                        model->clearCycle( currentAnimation, 0.0 ); // clear now
                        currentAnimation = -1;

                        palPos->stopAdvance();
                    }
                }
                default: break;
            }
        
            return false;
        }
    
    private:

        osgCal::Model*              model;
        // std::vector< std::string >  animationNames;
        int                         currentAnimation;
        palladinPosition *palPos;

};

