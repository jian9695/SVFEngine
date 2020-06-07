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
#include <osgEarth/MapNode>

typedef void(*OnResultsUpdated)(float, SolarRadiation);

class CubemapSurface : public RenderSurface
{
public:
	osg::Vec3d _pos;//position of camera
	osg::Vec3d _dir;//viewing direction of camera
	osg::Vec3d _up;//the _up vector for use in building a view matrix
	osg::Matrixd _local2World;
	osg::Matrixd _viewProjMatrix;
  CubemapSurface(int width, int height, GLenum internalFormat, GLenum sourceFormat, GLenum sourceType, bool allocateImage,
	 osg::Vec3d dir, osg::Vec3d up, std::string name) :
		RenderSurface(width, height, internalFormat, sourceFormat, sourceType, allocateImage),
		_dir(dir), _up(up)
	{
		_local2World = osg::Matrixd::identity();
		osg::Camera::setName(name);
		setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
		setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		setClearColor(osg::Vec4(0, 0, 0, 0));
	}

	void update()
	{
		osg::Vec3d dir = _dir * _local2World;
		dir.normalize();
		osg::Vec3d up = _up * _local2World;
		_viewMatrix.makeLookAt(_pos, _pos + dir * 100, up);
		_projectionMatrix.makePerspective(90, 1.0, 0.01, 1000000);
	   dirtyBound();
	}
};

//class for handling interactive 3D picking, SVF calculation and result displaying
class SolarInteractiveHandler : public osgGA::GUIEventHandler
{
public:
	//_cubemapCameras: a group of cubemap cameras created from SolarInteractiveHandler
	//_root: child nodes for showing the point lable and the fisheye HUD are inserted into this parent node
	SolarInteractiveHandler(
		osg::Node* sceneNode,
		osg::Group* root,
		osgEarth::MapNode* mapNode,
		osgGA::CameraManipulator* manip,
		osgViewer::Viewer* viewer,
		SolarParam* solarParam,
		OnResultsUpdated resultsCallback);
	~SolarInteractiveHandler();
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	RenderSurface* _cubemap2fisheyeCamera;
private:
	//compute the mouse-model intersection point; compute SVF at this point; update the fisheye HUD and _text labels
	void computeMouseIntersection(osgUtil::LineSegmentIntersector* ray);
	SolarRadiation calSolar(osg::Vec3d geoPos, osg::Vec3d normal);
	bool isEarth();
	std::tuple<bool, osg::Vec3d, osg::Matrixd> getGeoTransform(osg::Vec3d worldPos);
	static osg::Group* createSVFCameras();
	//create node to convert a cubemap set into a fisheye view and then render onto an off-screen _image
	static RenderSurface* cubemap2hemispherical(osg::Group* _cubemapCameras);
	//calculate SVF from a fisheye _image
	//Lambert's cosine law will be applied when applyLambert = true
	static double calSVF(osg::Image* img, bool applyLambert = false);
	static SolarRadiation calSolar(osg::Image* img, SolarParam* solarParam, osg::Vec3d pos, osg::Vec3d normal, osg::Node* sceneNode);
private:
	osg::Group* _cubemapCameras;
	osgGA::CameraManipulator* _manip;
	osg::Group* _root;
	osg::Node* _sceneNode;
	osgEarth::MapNode* _mapNode;
	PointRenderer* _pointRenderer;
	osgViewer::Viewer* _viewer;
	void printfVec3d(osg::Vec3d v);
	void printfVec3(osg::Vec3 v);
	OnResultsUpdated _resultsCallback;
	SolarParam* _solarParam;
	void Test(SunVector sunVec, osg::Vec3d pos);
};

