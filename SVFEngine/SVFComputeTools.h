#pragma once
#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif
#include "osg/Camera"
#include "osg/Texture2D"
#include "osg/Geode"
#include <osg/MatrixTransform>
#include <osg/TextureCubeMap>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osg/GLExtensions>
#include <osgViewer/Renderer>
#include <osgGA/TrackballManipulator>
#include <osgDB/WriteFile>
#include <osgViewer/ViewerEventHandlers>
#include <osg/ClampColor>
#include <osgDB/ReadFile>
#include "osg/LineWidth"
#include <osgSim/LineOfSight>
#include "PointRenderer.h"
#include <osgText/Font>
#include <osgText/Text>
#include <iomanip>
#include "ScreenOverlay.h"
#include "GrassSolar.h"

typedef void(*OnResultsUpdated)(float, SolarRadiation);

class CubemapSurface : public RenderSurface, public osg::NodeCallback
{
public:
	osg::Vec3d _pos;//position of camera
	osg::Vec3d _dir;//viewing direction of camera
	osg::Vec3d _up;//the _up vector for use in building a view matrix
	osg::Matrixd _viewProjMatrix;
  CubemapSurface(int width, int height, GLenum internalFormat, GLenum sourceFormat, GLenum sourceType, bool allocateImage,
	 osg::Vec3d dir, osg::Vec3d up, std::string name) :
		RenderSurface(width, height, internalFormat, sourceFormat, sourceType, allocateImage),
		_dir(dir), _up(up)
	{
		osg::Camera::setName(name);
		addCullCallback(this);
	}

	void update()
	{
		osg::Matrix localOffset;
		localOffset.makeLookAt(_pos, _pos + _dir * 100, _up);
		osg::Matrix viewMatrix = localOffset;
		setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		float nearDist = 0.1;
		setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
		//setProjectionMatrixAsFrustum(-nearDist, nearDist, -nearDist, nearDist, 0.01, 1000.0);
		setProjectionMatrixAsPerspective(90, 1.0, 0.01, 1000000);
		setViewMatrix(viewMatrix);
		setClearColor(osg::Vec4(0, 0, 0, 0));
		_viewProjMatrix = viewMatrix * getProjectionMatrix();
	}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		if (nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
		{
			update();
		}
		osg::NodeCallback::traverse(node, nv);
	}
};

class SVFComputeTools
{
public:
	SVFComputeTools();
	~SVFComputeTools();
	//create a group of cameras to render a cubemap set from a 3D position
	//it contains six cameras looking respectively in the six cubemap directions
	static osg::Group* createSVFCameras(osg::Node* city);
	//create node to convert a cubemap set into a fisheye view and then render onto an off-screen _image
	static RenderSurface* cubemap2hemispherical(osg::Group* _cubemapCameras);
	//create node to convert a cubemap set into a fisheye view and then render onto the screen
	static osg::Node* createTextureRect(std::string texfile);
	//calculate SVF from a fisheye _image
	//Lambert's cosine law will be applied when applyLambert = true
	static double calSVF(osg::Image* img, bool applyLambert = false);
	static SolarRadiation calSolar(osg::Image* img, SolarParam* solarParam, osg::Vec3d pos, osg::Vec3d normal, osg::Node* sceneNode);
};

//class for handling interactive 3D picking, SVF calculation and result displaying
class SkyViewFactorEventHandler : public osgGA::GUIEventHandler
{
public:
	//_cubemapCameras: a group of cubemap cameras created from SVFComputeTools
	//_root: child nodes for showing the point lable and the fisheye HUD are inserted into this parent node
	SkyViewFactorEventHandler(osg::Node* threeDModel, osg::Group* root, 
		osg::ref_ptr<osgGA::CameraManipulator> manip, 
		osgViewer::Viewer* viewer, SolarParam* solarParam,
		OnResultsUpdated resultsCallback);
	~SkyViewFactorEventHandler();
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	osg::ref_ptr<RenderSurface> _cubemap2fisheyeCamera;
private:
	//compute the mouse-model intersection point; compute SVF at this point; update the fisheye HUD and _text labels
	void computeMouseIntersection(osgUtil::LineSegmentIntersector* ray);
	SolarRadiation calSolar(osg::Vec3d pos, osg::Vec3d normal);
	osg::ref_ptr<osg::Group> _cubemapCameras;
	osg::ref_ptr<osgGA::CameraManipulator> _manip;
	osg::Group* _root;
	osg::Node* _sceneNode;
	osg::ref_ptr<PointRenderer> _pointRenderer;
	osg::ref_ptr<osg::Image> _screenshotTextImg;
	osgViewer::Viewer* _viewer;
	void printfVec3d(osg::Vec3d v);
	void printfVec3(osg::Vec3 v);
	osg::ref_ptr<osg::Group> _renderGroup;
	OnResultsUpdated _resultsCallback;
	SolarParam* _solarParam;
	void Test(SunVector sunVec, osg::Vec3d pos);
};

