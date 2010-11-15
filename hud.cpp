#include "hud.h"

#include <iostream>

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
   mHudImage = osgDB::readImageFile(texture_filename);
   mHUDTexture->setImage(mHudImage);

   osg::Vec3Array* HUDnormals = new osg::Vec3Array;
   
   HUDnormals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
   mHUDBackgroundGeometry->setNormalArray(HUDnormals);
   mHUDBackgroundGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
   mHUDBackgroundGeometry->addPrimitiveSet(mHUDBackgroundIndices);
   mHUDBackgroundGeometry->setVertexArray(mHUDBackgroundVertices);
   mHUDBackgroundGeometry->setColorArray(mHUDcolors);
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

