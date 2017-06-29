#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include "Parcour.hpp"
#include <osg/ref_ptr>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osg/Camera>

namespace osgViewer {
class Viewer;
}

namespace osg {
class Group;
}

class Visualization: public osg::Camera::DrawCallback
{
    osg::ref_ptr<osg::Group> rootNode;
    osgViewer::Viewer *viewer;
    osg::ref_ptr<osg::TextureRectangle> rttTexture;
    osg::ref_ptr<osg::Image> rttImage;
    osg::ref_ptr<osg::Camera> rttCamera;
    osg::ref_ptr<osg::PositionAttitudeTransform> robot;
    mutable std::vector<uint8_t> imageData;
    
    Parcour &parcour;
    
    size_t width;
    size_t height;

    size_t textureWidth;
    size_t textureHeight;

    virtual void operator () (const osg::Camera& /*camera*/) const;

public:
    Visualization(Parcour &parcour);
    
    void setupScene();    
    void setRobotPose(const Eigen::Vector3d postion, const Eigen::Quaterniond &orientation);
    void draw(const Eigen::Vector3d postion, const Eigen::Quaterniond &orientation);
    void copyCameraImage(uint8_t *imageData);
    
    void createHUD();
    
    void addCamera();
    
    osgViewer::Viewer *getViewer()
    {
        return viewer;
    };
    
};

#endif // VISUALIZATION_HPP
