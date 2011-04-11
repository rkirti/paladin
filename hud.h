#ifndef _HUD_ELEMENT_
#define _HUD_ELEMENT_

#include <osg/Vec3>
#include <osg/Matrix>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile> 
#include <osg/Projection>

#include <osgText/Text>
#include <string>

enum OPTIONS{OPTION_A=1,OPTION_B=2, OPTION_C=3, OPTION_D=4};
osg::Projection* displayQuestion();

class HUDElement
{
    public:
        HUDElement(osg::Group* root, int width=1600, int height=1200);
        void DefineHUDQuad(osg::Vec3 ll_corner, osg::Vec3 ul_corner, osg::Vec3 ur_corner, osg::Vec3 lr_corner, int posx, int posy, const std::string texture_filename, int draw_order=11);
        void DefineHUDQuad(float width, float height, int posx, int posy, const std::string texture_filename, int draw_order=11);
        void Rotate(float degrees);

        void DisplayScore();

        void IncreementScore(){
            score += 10;
            DisplayScore();
        }
        void DecreementScore(){
            if(score > 0) score -= 10;
            DisplayScore();
        }
        void DisplayInitScreen();
        void RemoveInitScreen();


    private:
        osg::ref_ptr<osg::Geode> mHUDGeode;
        osg::ref_ptr<osg::Projection> mHUDProjectionMatrix;
        osg::ref_ptr<osg::MatrixTransform> mHUDModelViewMatrix;
        osg::ref_ptr<osg::Geometry> mHUDBackgroundGeometry;
        osg::ref_ptr<osg::Vec3Array> mHUDBackgroundVertices;
        osg::ref_ptr<osg::DrawElementsUInt> mHUDBackgroundIndices;
        osg::ref_ptr<osg::Vec4Array> mHUDcolors;
        osg::ref_ptr<osg::Vec2Array> mTexcoords;
        osg::ref_ptr<osg::Texture2D> mHUDTexture;
        osg::ref_ptr<osg::Image> mHudImage;
        osg::Matrix mMtx;
        osgText::Text* textOne;
        osg::Switch *swt;
        osg::Geode *HUDGeode;
        int score;
        int mPosX;
        int mPosY;
};



class Question{
    public:
    std::string  qText;
    std::string option1;
    std::string option2;
    std::string option3;
    std::string option4;
    OPTIONS correctAnswer;   
    Question(std::string question, std::string ans1, std::string ans2, std::string ans3, std::string ans4, OPTIONS sol)
    {
        qText = question;
        option1 = ans1;
        option2 = ans2;
        option3 = ans3;
        option4 = ans4;
        correctAnswer = sol;
    }

};

#endif
