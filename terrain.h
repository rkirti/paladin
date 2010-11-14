osg::ref_ptr<osg::Group> createTerrain(const std::string filename)
{
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;

    texture->setImage(osgDB::readImageFile(filename));

    texture->setDataVariance(osg::Object::DYNAMIC); 

    // Create a new StateSet with default settings:
    osg::StateSet* stateOne = new osg::StateSet();
    stateOne->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

    // Create an object to store geometry in.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    // Create an array of four vertices.
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
    geom->setVertexArray( v.get() );
    float halfWidth = 100.0;
    v->push_back( osg::Vec3( -halfWidth, -halfWidth, 0.f) );
    v->push_back( osg::Vec3( -halfWidth,  halfWidth, 0.f) );
    v->push_back( osg::Vec3(  halfWidth,  halfWidth, 0.f) );
    v->push_back( osg::Vec3(  halfWidth, -halfWidth, 0.f) );

    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.00f,0.0f); // tex coord for vertex 0 
    (*texcoords)[1].set(0.00f,1.0f); // tex coord for vertex 1 
    (*texcoords)[2].set(1.00f,1.0f); // ""
    (*texcoords)[3].set(1.00f,0.0f); // "" 
    geom->setTexCoordArray(0,texcoords);

    // Create an array for the single normal.
    osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
    geom->setNormalArray( n.get() );
    geom->setNormalBinding( osg::Geometry::BIND_OVERALL );
    n->push_back( osg::Vec3( 0.f, 0.f, 1.f ) );

    // Draw a four-vertex quad from the stored data.
    geom->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, 4 ) );

    // Add the Geometry (Drawable) to a Geode and
    // return the Geode.
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom.get() );
    geode->setStateSet(stateOne);

    // return geode; 
    osg::ref_ptr<osg::Group> grp = new osg::Group;

    int i, j;
    int dim = 10;

    for(i=-dim; i<=dim; i++)
    {
        for(j=-dim; j<=dim; j++)
        {
            osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
            osg::Vec3 newPos(2*i*halfWidth,2*j*halfWidth,0);
            pat->setPosition(newPos);

            pat->addChild(geode);

            grp->addChild(pat);
        }
    }

    return grp;

}
