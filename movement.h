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

