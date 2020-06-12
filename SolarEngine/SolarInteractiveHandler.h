#pragma once
#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgEarth/MapNode>
#include <iomanip>
#include "PointsRenderer.h"
#include "RenderSurface.h"
#include "GrassSolar.h"
#include "Cubemap.h"

typedef void(*OnResultsUpdated)(float, SolarRadiation);

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
	RenderSurface* fisheyeSurface() { return _cubemap2fisheyeCamera; }
	void postDrawUpdate();
	std::tuple<bool, SolarRadiationPoint> queryPoint(const float& mouseX, const float& mouseY);
private:
	//compute the mouse-model intersection point; compute SVF at this point; update the fisheye HUD and _text labels
	void computeMouseIntersection(osgUtil::LineSegmentIntersector* ray);
	SolarRadiation calSolar(SolarParam& solarParam);
	bool isEarth();
	std::tuple<bool, osg::Vec3d, osg::Matrixd> getGeoTransform(osg::Vec3d worldPos);
private:
	Cubemap* _cubemap;
	osgGA::CameraManipulator* _manip;
	osg::Group* _root;
	osg::Node* _sceneNode;
	osgEarth::MapNode* _mapNode;
	PointsRenderer* _pointRenderer;
	osgViewer::Viewer* _viewer;
	void printfVec3d(osg::Vec3d v);
	void printfVec3(osg::Vec3 v);
	OnResultsUpdated _resultsCallback;
	SolarParam* _solarParam;
	RenderSurface* _cubemap2fisheyeCamera;
};

