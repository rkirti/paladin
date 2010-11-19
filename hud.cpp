#include "hud.h"

#include <iostream>
#include <sstream>

#include <osg/Switch>

HUDElement::HUDElement(osg::Group* root, int width, int height)
{
mHUDProjectionMatrix = new osg::Projection();
mHUDProjectionMatrix->setMatrix(osg::Matrix::ortho2D(0,width,0,height));
mHUDModelViewMatrix = new osg::MatrixTransform();
mHUDModelViewMatrix->setMatrix(osg::Matrix::identity());
mHUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
// mHUDModelViewMatrix->addChild( mHUDGeode );
mHUDProjectionMatrix->addChild(mHUDModelViewMatrix);
root->addChild(mHUDProjectionMatrix);

// mHUDModelViewMatrix->addChild(createGeodeForImage(osgDB::readImageFile("../../initscreen.png")));

score = 0;
textOne = new osgText::Text; 
DisplayInitScreen();
}

void HUDElement::DefineHUDQuad(osg::Vec3 ll_corner, osg::Vec3 ul_corner, osg::Vec3 ur_corner, osg::Vec3 lr_corner, int posx, int posy, const std::string texture_filename, int draw_order)
{

    mHUDGeode = new osg::Geode();

    mHUDBackgroundGeometry = new osg::Geometry();

    mHUDBackgroundVertices = new osg::Vec3Array;

   mHUDBackgroundVertices->clear();
   mHUDBackgroundVertices->push_back( ll_corner );
   mHUDBackgroundVertices->push_back( lr_corner );
   mHUDBackgroundVertices->push_back( ur_corner );
   mHUDBackgroundVertices->push_back( ul_corner );	

   mHUDBackgroundIndices=new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
   mHUDBackgroundIndices->push_back(0);
   mHUDBackgroundIndices->push_back(1);
   mHUDBackgroundIndices->push_back(2);
   mHUDBackgroundIndices->push_back(3);

   mHUDcolors = new osg::Vec4Array;
   mHUDcolors->push_back(osg::Vec4(0.8f,0.8f,0.8f,1.0f));

   mTexcoords = new osg::Vec2Array(4);
   (*mTexcoords)[0].set(0.0f,0.0f);
   (*mTexcoords)[1].set(1.0f,0.0f);
   (*mTexcoords)[2].set(1.0f,1.0f);
   (*mTexcoords)[3].set(0.0f,1.0f);

   mHUDBackgroundGeometry->setTexCoordArray(0,mTexcoords);

   mHUDTexture = new osg::Texture2D;
   mHUDTexture->setDataVariance(osg::Object::DYNAMIC);
   // mHudImage = osgDB::readImageFile(texture_filename);
   mHudImage = osgDB::readImageFile("./data/HUD.png");
   mHUDTexture->setImage(mHudImage);

   osg::Vec3Array* HUDnormals = new osg::Vec3Array;
   
   HUDnormals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
   mHUDBackgroundGeometry->setNormalArray(HUDnormals);
   mHUDBackgroundGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
   mHUDBackgroundGeometry->addPrimitiveSet(mHUDBackgroundIndices);
   mHUDBackgroundGeometry->setVertexArray(mHUDBackgroundVertices);
   // mHUDBackgroundGeometry->setColorArray(mHUDcolors);
   mHUDBackgroundGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

   mHUDGeode->addDrawable(mHUDBackgroundGeometry);

   osg::StateSet* HUDStateSet = new osg::StateSet();
   mHUDGeode->setStateSet(HUDStateSet);
   HUDStateSet->setTextureAttributeAndModes(0,mHUDTexture,osg::StateAttribute::ON);

   // For this state set, turn blending on (so alpha texture looks right)
   HUDStateSet->setMode(GL_BLEND,osg::StateAttribute::ON);

   // Disable depth testing so geometry is draw regardless of depth values
   // of geometry already draw.
   HUDStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
   HUDStateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

   // Need to make sure this geometry is draw last. RenderBins are handled
   // in numerical order so set bin number to 11
   HUDStateSet->setRenderBinDetails( draw_order, "RenderBin");
   
   mHUDModelViewMatrix->addChild( mHUDGeode );

   // Add the text (Text class is derived from drawable) to the geode:
   mHUDGeode->addDrawable( textOne );

   // Set up the parameters for the text we'll add to the HUD:
   textOne->setCharacterSize(25);
   textOne->setFont("data/impact.ttf");

   DisplayScore();

   textOne->setAxisAlignment(osgText::Text::SCREEN);
   textOne->setPosition(osg::Vec3(-500,0,-1.5) );
   //textOne->setColor(osg::Vec4(199, 77, 15, 1) );
   textOne->setColor(osg::Vec4(199, 77, 15, 1) );

   // Help contents 
   osgText::Text *helpText = new osgText::Text;
   mHUDGeode->addDrawable(helpText);
   helpText->setAxisAlignment(osgText::Text::SCREEN);
   helpText->setPosition(osg::Vec3(100, 50, -1.5));
   helpText->setColor(osg::Vec4(199, 77, 15, 1) );
   helpText->setCharacterSize(20);
   helpText->setFont("data/impact.ttf");

   helpText->setText("'h' : Display On Screen Help \n'c' : Toggle Camera Position \n'z' : Jump \nUp : Move forward \nLeft,Right : Turn \nEsc : Quit");


   mPosX=posx;
   mPosY=posy;
   Rotate(0.0);
}

void HUDElement::DefineHUDQuad(float width, float height, int posx, int posy, const std::string texture_filename, int draw_order)
{
	osg::Vec3 ll_corner, ul_corner, ur_corner, lr_corner;
	ll_corner.set(-width/2, -height/2, 0);
	ul_corner.set(-width/2, height/2, 0);
	ur_corner.set(width/2, height/2, 0);
	lr_corner.set(width/2, -height/2, 0);
	DefineHUDQuad(ll_corner,ul_corner,ur_corner,lr_corner,posx,posy, texture_filename,draw_order);
}

void HUDElement::Rotate(float degrees)
{
   // mMtx.makeRotate( -degrees/180.0f*M_PI, osg::Vec3(0.0,0.0,1.0) );
   mMtx.makeRotate( -degrees, osg::Vec3(0.0,0.0,1.0) );
   mMtx.setTrans( osg::Vec3(mPosX,mPosY,0) );
   mHUDModelViewMatrix->setMatrix( mMtx );
}

void HUDElement::DisplayScore()
{
   std::string pointsData;
   std::stringstream out;
   out << "Points : " << score;
   pointsData = out.str();
   // textOne->setText("Points : 0");
   textOne->setText(osgText::String(pointsData));

}

void HUDElement::DisplayInitScreen()
{
    HUDGeode = new osg::Geode;

    // Set up geometry for the HUD and add it to the HUD
       osg::Geometry* HUDBackgroundGeometry = new osg::Geometry();

       osg::Vec3Array* HUDBackgroundVertices = new osg::Vec3Array;
       HUDBackgroundVertices->push_back( osg::Vec3( -600,-75,-1) );
       HUDBackgroundVertices->push_back( osg::Vec3(600, -75,-1) );
       HUDBackgroundVertices->push_back( osg::Vec3(600,725,-1) );
       HUDBackgroundVertices->push_back( osg::Vec3( -600  ,725,-1) );

       osg::DrawElementsUInt* HUDBackgroundIndices = new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
       HUDBackgroundIndices->push_back(0);
       HUDBackgroundIndices->push_back(1);
       HUDBackgroundIndices->push_back(2);
       HUDBackgroundIndices->push_back(3);

       // osg::Vec4Array* HUDcolors = new osg::Vec4Array;
       // HUDcolors->push_back(osg::Vec4(0.8f,0.8f,0.8f,0.8f));

       osg::Vec2Array* texcoords = new osg::Vec2Array(4);
       (*texcoords)[0].set(0.0f,0.0f);
       (*texcoords)[1].set(1.0f,0.0f);
       (*texcoords)[2].set(1.0f,1.0f);
       (*texcoords)[3].set(0.0f,1.0f);

       HUDBackgroundGeometry->setTexCoordArray(0,texcoords);
       osg::Texture2D* HUDTexture = new osg::Texture2D;
       HUDTexture->setDataVariance(osg::Object::DYNAMIC);
       osg::Image* hudImage;
       hudImage = osgDB::readImageFile("../../initscreen.png");
       HUDTexture->setImage(hudImage);
       osg::Vec3Array* HUDnormals = new osg::Vec3Array;
       HUDnormals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
       HUDBackgroundGeometry->setNormalArray(HUDnormals);
       HUDBackgroundGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
       HUDBackgroundGeometry->addPrimitiveSet(HUDBackgroundIndices);
       HUDBackgroundGeometry->setVertexArray(HUDBackgroundVertices);
       // HUDBackgroundGeometry->setColorArray(HUDcolors);
       // HUDBackgroundGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

       HUDGeode->addDrawable(HUDBackgroundGeometry);
// Create and set up a state set using the texture from above:
       osg::StateSet* HUDStateSet = new osg::StateSet();
       HUDGeode->setStateSet(HUDStateSet);
       HUDStateSet->
          setTextureAttributeAndModes(0,HUDTexture,osg::StateAttribute::ON);

       // For this state set, turn blending on (so alpha texture looks right)
       HUDStateSet->setMode(GL_BLEND,osg::StateAttribute::ON);

       // Disable depth testing so geometry is draw regardless of depth values
       // of geometry already draw.
       HUDStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
       HUDStateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

       // Need to make sure this geometry is draw last. RenderBins are handled
       // in numerical order so set bin number to 11
       HUDStateSet->setRenderBinDetails( 13, "RenderBin");

       swt = new osg::Switch;
       mHUDModelViewMatrix->addChild(swt);
       swt->addChild(HUDGeode, true);
       // swt
}


void HUDElement::RemoveInitScreen()
{
    swt->setChildValue(HUDGeode, false);
}
