#include "Visualization.hpp"
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/TextureRectangle>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <iostream>
#include <boost/thread/pthread/mutex.hpp>



using namespace osg;
boost::mutex cameraLock;

ref_ptr<Geode> getBoxGeode(const ::Box &box)
{
    const osg::Vec3 position(box.position.x(), box.position.y(), box.position.z());
    ref_ptr<osg::Box> boxShape = new osg::Box(position, box.width, box.height, box.depth);
    ref_ptr<ShapeDrawable> boxDrawable = new ShapeDrawable(boxShape);
    boxDrawable->setColor(osg::Vec4(box.color.x(), box.color.y(), box.color.z(), 1));
    
    ref_ptr<Geode> boxGeode = new osg::Geode();
    boxGeode->addDrawable(boxDrawable);
    
    return boxGeode;
}

ref_ptr<osg::PositionAttitudeTransform> getRobotGeode()
{
    ref_ptr<osg::PositionAttitudeTransform> ret = new osg::PositionAttitudeTransform();
    const osg::Vec3 position(0,0,0.01);
    ref_ptr<osg::Box> boxShape = new osg::Box(position, 0.2, 0.10, 0.15);
    ref_ptr<ShapeDrawable> boxDrawable = new ShapeDrawable(boxShape);
    boxDrawable->setColor(osg::Vec4(1, 0 , 0, 1));
    
    ref_ptr<Geode> boxGeode = new osg::Geode();
    boxGeode->addDrawable(boxDrawable);

    ret->addChild(boxGeode);
    
    return ret;
}

ref_ptr<Geode> getCylinderGeode(const ::Cylinder &cylinder)
{
    const osg::Vec3 position(cylinder.position.x(), cylinder.position.y(), cylinder.position.z());
    ref_ptr<osg::Cylinder> cylinderShape = new osg::Cylinder(position, cylinder.radius, cylinder.height);
    ref_ptr<ShapeDrawable> cylinderDrawable = new ShapeDrawable(cylinderShape);
    cylinderDrawable->setColor(osg::Vec4(cylinder.color.x(), cylinder.color.y(), cylinder.color.z(), 1));
    
    ref_ptr<Geode> cylinderGeode = new osg::Geode();
    cylinderGeode->addDrawable(cylinderDrawable);
    
    return cylinderGeode;
}
osg::Camera* createHUD();
Visualization::Visualization(Parcour& parcour) : parcour(parcour)
{
    // construct the viewer.
    viewer = new osgViewer::Viewer();
        
    rootNode = new osg::Group;
    
//     rootNode->addChild(new Plane());
    
    robot = getRobotGeode();
    
    rootNode->addChild(robot);
    
    width = 640;
    height = 480;
    
    textureWidth = 128;
    textureHeight = 96;
    
    viewer->setSceneData(rootNode.get());
    viewer->setUpViewInWindow(0, 0, width, height);
    Camera *cam = viewer->getCamera();
    cam->setViewMatrix(Matrixf::lookAt(Vec3(10,0,0), Vec3(50,2,0), Vec3(0,0,1)));
    
    osgGA::TrackballManipulator* manipulator = new osgGA::TrackballManipulator();
    viewer->setCameraManipulator(manipulator);

    osg::Matrixd viewMatrix;
    viewMatrix.set(-0.999, 0.018, 0.011, 0,
                   0.016, -0.980, 0.196, 0,
                   0.014, 0.196, 0.980, 0,
                   1.993, 4.063, 9.466, 1);
    manipulator->setByMatrix(viewMatrix);
    
    setupScene();
    addCamera();
    createHUD();
    viewer->realize();
}

void Visualization::setRobotPose(const Eigen::Vector3d position, const Eigen::Quaterniond& orientation)
{
    Eigen::Vector3d lookAt = position + orientation * Eigen::Vector3d::UnitX();
    Vec3 pos(position.x(), position.y(), position.z());
 
    osg::Quat q(orientation.x(), orientation.y(), orientation.z(), orientation.w());
    
    robot->setPosition(pos + q * Vec3(-0.1, 0, 0));
    robot->setAttitude(q);
    
    viewer->getCamera()->setViewMatrix(Matrixf::lookAt(pos, Vec3(lookAt.x(), lookAt.y(), lookAt.z()), Vec3(0,0,1)));
    rttCamera->setViewMatrix(Matrixf::lookAt(pos, Vec3(lookAt.x(), lookAt.y(), lookAt.z()), Vec3(0,0,1)));
}

void Visualization::setupScene()
{
    
    
    for(std::vector< ::Box >::const_iterator it = parcour.getBoxes().begin(); it != parcour.getBoxes().end();it++)
    {
        rootNode->addChild(getBoxGeode(*it));
//      std::cout << "Drawing box" << std::endl;
    }
    for(std::vector< ::Cylinder >::const_iterator it = parcour.getCylinders().begin(); it != parcour.getCylinders().end();it++)
    {
        rootNode->addChild(getCylinderGeode(*it));
//      std::cout << "Drawing goal" << std::endl;
    }
    
    // create a local light.
    osg::Light* myLight2 = new osg::Light;
    myLight2->setLightNum(0);
    myLight2->setPosition(osg::Vec4(2.0,2.0,1.0,1.0f));
    myLight2->setAmbient(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    myLight2->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    myLight2->setConstantAttenuation(1.0f);
    myLight2->setLinearAttenuation(2.0f / 5);
    myLight2->setQuadraticAttenuation(2.0f/osg::square(5));

    osg::LightSource* lightS2 = new osg::LightSource;    
    lightS2->setLight(myLight2);
    lightS2->setLocalStateSetModes(osg::StateAttribute::ON); 

    StateSet *rootStateSet = rootNode->getOrCreateStateSet();
    lightS2->setStateSetModes(*rootStateSet ,osg::StateAttribute::ON);
    rootNode->addChild(lightS2);
    
//     // create a spot light.
//     osg::Light* myLight1 = new osg::Light;
//     myLight1->setLightNum(0);
//     myLight1->setPosition(osg::Vec4(4, 4, 3 ,1.0f));
//     myLight1->setAmbient(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
//     myLight1->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
//     myLight1->setSpotCutoff(20.0f);
//     myLight1->setSpotExponent(50.0f);
//     myLight1->setDirection(osg::Vec3(1.0f,1.0f,-1.0f));
// 
//     osg::LightSource* lightS1 = new osg::LightSource;    
//     lightS1->setLight(myLight1);
//     lightS1->setLocalStateSetModes(osg::StateAttribute::ON); 
// 
//     lightS1->setStateSetModes(*rootStateSet,osg::StateAttribute::ON);
//     rootNode->addChild(lightS1);
    
}

void Visualization::copyCameraImage(uint8_t* data)
{
    cameraLock.lock();
    memcpy(data, imageData.data(), textureWidth * textureHeight * 2);
    cameraLock.unlock();
}

void Visualization::operator()(const Camera&) const
{
    cameraLock.lock();
    
    //everything should be drawn now, time to copy the image out
    memcpy(imageData.data(), rttImage->data(), textureWidth * textureHeight * 2);
    
    cameraLock.unlock();
}


void Visualization::createHUD()
{
    // create a camera to set up the projection and model view matrices, and the subgraph to draw in the HUD
    osg::Camera* camera = new osg::Camera;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1,0,1));

    // set the view matrix    
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::POST_RENDER);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    camera->setAllowEventFocus(false);
    

//     // add to this camera a subgraph to render
    {

        osg::Geode* geode = new osg::Geode();

        // turn lighting off for the text and disable depth test to ensure it's always ontop.
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

            {
                const double qwidth = 0.2;
                const double qheight = qwidth * textureWidth / textureHeight;
                
            // left hand side of bounding box.
            osg::Vec3 top_left(0, qheight, 0);
            osg::Vec3 bottom_left(0, 0, 0);
            osg::Vec3 bottom_right(qwidth, 0, 0);
            osg::Vec3 top_right(qwidth, qheight, 0);
            
            // create the geometry for the wall.
            osg::Geometry* geom = new osg::Geometry;
            
            osg::Vec3Array* vertices = new osg::Vec3Array(4);
            (*vertices)[0] = top_left;
            (*vertices)[1] = bottom_left;
            (*vertices)[2] = bottom_right;
            (*vertices)[3] = top_right;
            geom->setVertexArray(vertices);
            
            osg::Vec2Array* texcoords = new osg::Vec2Array(4);
            (*texcoords)[0].set(0.0f,0);
            (*texcoords)[1].set(0.0f,textureHeight);
            (*texcoords)[2].set(textureWidth,textureHeight);
            (*texcoords)[3].set(textureWidth,00);
            geom->setTexCoordArray(0,texcoords);

            osg::Vec3Array* normals = new osg::Vec3Array(1);
            (*normals)[0].set(1.0f,0.0f,0.0f);
            geom->setNormalArray(normals);
            geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
            
            osg::Vec4Array* colors = new osg::Vec4Array(1);
            (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
            geom->setColorArray(colors);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);

            geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));
            
            geode->addDrawable(geom);
            
            osg::StateSet* stateset = geom->getOrCreateStateSet();
            stateset->setTextureAttributeAndModes(0,rttTexture,osg::StateAttribute::ON);
        }
        camera->addChild(geode);
    }

    osgViewer::Viewer::Windows windows;
    viewer->getWindows(windows);
    
//     if (windows.empty()) return 1;
    
    
    // set up cameras to render on the first window available.
    camera->setGraphicsContext(windows[0]);
    camera->setViewport(0,0,windows[0]->getTraits()->width, windows[0]->getTraits()->height);

    viewer->addSlave(camera, false);

    
//     return camera;
}

void Visualization::addCamera()
{
    rttTexture = new osg::TextureRectangle();
    rttTexture->setTextureSize(textureWidth, textureHeight);
    rttTexture->setInternalFormat(GL_RGB);
    rttTexture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    rttTexture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    rttCamera = new osg::Camera;

    // set up the background color and clear mask.
    rttCamera->setClearColor(osg::Vec4(0.1f,0.1f,0.1f,1.0f));
    rttCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // the camera is going to look at our input quad
//     rttCamera->setProjectionMatrix(viewer->getCamera()->getProjectionMatrix());

    rttCamera->setProjectionMatrixAsPerspective(-30.0f, -static_cast<double>(textureWidth)/static_cast<double>(textureHeight), 1.0f, 10000.0f);
    
    rttCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    rttCamera->setViewMatrix(Matrixf::lookAt(Vec3(10,0,0), Vec3(50,2,0), Vec3(0,0,1)));

    // set viewport
    rttCamera->setViewport(0, 0, textureWidth, textureHeight);

    // set the camera to render before the main camera.
    rttCamera->setRenderOrder(osg::Camera::POST_RENDER);

    // tell the camera to use OpenGL frame buffer objects
    rttCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

    // attach the textures to use
    rttCamera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0), rttTexture);

    rttImage = new osg::Image();
    rttImage->allocateImage(textureWidth, textureHeight,
                                1, GL_RGB, GL_UNSIGNED_SHORT_5_6_5);
    
    // attach the image so its copied on each frame.
    rttCamera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0), rttImage.get());

    rttCamera->addChild(viewer->getSceneData());
    
    rttTexture->setImage(rttImage);

    imageData.resize(textureWidth * textureHeight * 2, 0);
    
//     viewer->getSceneData()->asGroup()->addChild(rttCamera);
    viewer->getCamera()->addChild(rttCamera);

//     viewer->addSlave(rttCamera, false);
    
    //we want to get called after the image was rendered in
    //order to be able to copy it
    rttCamera->setPostDrawCallback(this);
}

