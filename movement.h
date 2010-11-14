/*
 * This class stores the current position of the Model, acting as an interface
 *      between the keyboard handler and the renderer. 
 * The keyboard handler updates the values of the members
 * The 'update' phase of the renderer reads the values and updates the position
 *      of the model 
 */

#ifndef MOVEMENT_H
#define MOVEMENT_H


extern bool movementAllowed;

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

        void increementAngle(){ currentAngle+=0.02;}
        void decreementAngle(){ currentAngle-=0.02;}

        float currentAngle; 
        bool advance;
};


#endif 
