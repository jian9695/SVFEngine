#include "SolarInteractiveHandler.h"
#include <osg/ComputeBoundsVisitor>
#include <osgDB/WriteFile>
#include <ctime>

SolarInteractiveHandler::SolarInteractiveHandler(
	osg::Node* sceneNode,
	osg::Group* root,
	osgEarth::MapNode* mapNode,
	osgGA::CameraManipulator* manip,
	osgViewer::Viewer* viewer,
	SolarParam* solarParam,
	OnResultsUpdated resultsCallback)
{
	_viewer = viewer;
	_manip = manip;
	_root = root;
	_solarParam = solarParam;
	_resultsCallback = resultsCallback;
	_sceneNode = sceneNode;
	_mapNode = mapNode;

  if (_mapNode)
		_cubemap = Cubemap::create(512, mapNode);
	else
		_cubemap = Cubemap::create(512, sceneNode);
	_cubemap->setNodeMask(false);
	root->addChild(_cubemap);

	for (int i = 0; i < _cubemap->getNumChildren(); i++)
	{
		_cubemap->getFace(i)->setRenderOrder(osg::Camera::PRE_RENDER, i);
	}

	//create a node to transform a cubemap into a fisheye view and then render onto an off-screen image for svf calculation
	_cubemap2fisheyeCamera = _cubemap->toHemisphericalSurface();
	_cubemap2fisheyeCamera->setRenderOrder(osg::Camera::PRE_RENDER, _cubemap->getNumChildren());
	root->addChild(_cubemap2fisheyeCamera);

	//create point labels
	_pointRenderer = new PointsRenderer;

	osg::ref_ptr<osg::Image> depthImage = new osg::Image;
	depthImage->allocateImage(512, 512, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
	_viewer->getCamera()->attach(osg::Camera::DEPTH_BUFFER, depthImage.get());
	_pointRenderer->setSceneDepthImage(depthImage.get());
	_pointRenderer->setSceneCamera(_viewer->getCamera());
	_root->addChild(_pointRenderer);
}

SolarInteractiveHandler::~SolarInteractiveHandler()
{

}

void SolarInteractiveHandler::printfVec3d(osg::Vec3d v)
{
	printf("%f,%f,%f\n", v.x(), v.y(), v.z());
}

void SolarInteractiveHandler::printfVec3(osg::Vec3 v)
{
	printf("%f,%f,%f\n", v.x(), v.y(), v.z());
}

void SolarInteractiveHandler::computeMouseIntersection(osgUtil::LineSegmentIntersector* ray)
{
	osg::Vec3d orieye, oricenter, oriup;
	_viewer->getCamera()->getViewMatrixAsLookAt(orieye, oricenter, oriup);
	if (ray->getIntersections().size() == 0)
		return;
	osg::Vec3d _dir = orieye - oricenter;
	_dir.normalize();
	osg::Vec3d curcenter = ray->getFirstIntersection().getWorldIntersectPoint();
	osg::Vec3d cureye = ray->getFirstIntersection().getWorldIntersectPoint() + _dir * 50;

	osg::Vec3d worldPos = ray->getFirstIntersection().getWorldIntersectPoint();
	osg::Vec3d surfaceNormal = ray->getFirstIntersection().getWorldIntersectNormal();
	surfaceNormal.normalize();
	worldPos = worldPos + surfaceNormal * 0.1; //offset from the surface

	bool hasSpatialRef = isEarth();
	osg::Vec3d geoPos = osg::Vec3d(_solarParam->lon, _solarParam->lat, _solarParam->elev + worldPos.z());
	osg::Matrixd local2world = osg::Matrixd::identity();
	if (hasSpatialRef)
	{
		std::tie(hasSpatialRef, geoPos, local2world) = getGeoTransform(worldPos);
		osg::Matrixd world2local = osg::Matrixd::inverse(local2world);
		surfaceNormal = surfaceNormal * world2local;
		surfaceNormal.normalize();
	}

	for (int i = 0; i < _cubemap->getNumChildren(); i++)
	{
		CubemapSurface* cameraBuffer = (CubemapSurface*)_cubemap->getChild(i);
		cameraBuffer->_pos = worldPos;
		cameraBuffer->_local2World = local2world;
		cameraBuffer->update();
	}
	_cubemap->setNodeMask(true);
	//_viewer->setCameraManipulator(nullptr, false);
	//_viewer->getCamera()->setNodeMask(false);
	_viewer->frame();
	for (size_t i = 0; i < 2; i++)
	{
		_viewer->frame();
	}
	_cubemap->setNodeMask(false);
	//_viewer->setCameraManipulator(nullptr, false);
	//_viewer->getCamera()->setNodeMask(true);

	//Write out fisheye images
	//for (size_t i = 0; i < _cubemap->getNumChildren(); i++)
	//{
	//	CubemapSurface* face = _cubemap->getFace(i);
	//	osgDB::writeImageFile(*face->Image(), face->getName() + ".png");
	//}
	//osg::ref_ptr<osg::Image> fisheye = _cubemap->toHemisphericalImage(512, 512);
	//osgDB::writeImageFile(*fisheye, "fisheye2.png");
	//osgDB::writeImageFile(*_cubemap2fisheyeCamera->Image(), "fisheye.png");
	double svf = Utils::calSVF(_cubemap2fisheyeCamera->Image().get(), false);

	SolarParam param = *_solarParam;
	param.lon = geoPos.x();
	param.lat = geoPos.y();
	param.elev = param.elev + max(geoPos.z(), 0);
	param.slope = Utils::calculateSlope(surfaceNormal);
	param.aspect = Utils::calculateAspect(surfaceNormal);
	SolarRadiation solarRad = calSolar(param);
	solarRad.svf = svf;
	_resultsCallback(svf, solarRad);
	_pointRenderer->pushPoint(worldPos, param, solarRad);
}

bool SolarInteractiveHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME)
		return false;
	osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
	if (!viewer)
		return false;
	if ((ea.getModKeyMask() & ea.MODKEY_CTRL) && !(ea.getModKeyMask() & ea.MODKEY_LEFT_SHIFT))
	{
		if (ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON && ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
		{
			osg::ref_ptr<osgUtil::LineSegmentIntersector> ray = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, ea.getXnormalized(), ea.getYnormalized());
			osgUtil::IntersectionVisitor visitor(ray);
			viewer->getCamera()->accept(visitor);
			computeMouseIntersection(ray.get());
			return true;
		}
		else if (osgGA::GUIEventAdapter::KEYDOWN)
		{
			int key = ea.getUnmodifiedKey();
			if (key == osgGA::GUIEventAdapter::KEY_Delete || key == osgGA::GUIEventAdapter::KEY_KP_Delete || key == osgGA::GUIEventAdapter::KEY_BackSpace || key == 65454)
			{
				_pointRenderer->popPoint();
				return true;
			}
			else if (key == osgGA::GUIEventAdapter::KEY_Z)
			{
				_pointRenderer->undo();
				return true;
			}
			else if (key == osgGA::GUIEventAdapter::KEY_Y)
			{
				_pointRenderer->redo();
				return true;
			}
			else if (key == osgGA::GUIEventAdapter::KEY_E)
			{
				std::time_t t = std::time(0);   // get time now
				std::tm* now = std::localtime(&t);
				std::stringstream filenameSS;
				filenameSS << 1900 + now->tm_year << "-" << now->tm_mon + 1 << "-" << now->tm_mday
					<< "-" << now->tm_hour << "-" << now->tm_min << "-" << now->tm_sec << ".csv";
				_pointRenderer->exportPoints(filenameSS.str());
				return true;
			}
			//printf("%d,%d,%d\n", key, osgGA::GUIEventAdapter::KEY_Z, osgGA::GUIEventAdapter::KEY_Y);
		}
	}
	return false;
}

SolarRadiation SolarInteractiveHandler::calSolar(SolarParam& solarParam)
{
	osg::Image* img = _cubemap2fisheyeCamera->Image().get();
	SolarRadiation annualRad;
	annualRad.Zero();
	std::vector<SolarRadiation> dailyRads;
	
	int startDay = solarParam.startDay;
	int endDay = solarParam.endDay;
	if (solarParam.isSingleDay && endDay > startDay)
		endDay = startDay;
	int lastSteps = 0;
	GrassSolar rsun;
	for (int day = startDay; day <= endDay; day++)
	{
		solarParam.day = day;
		std::vector<SunVector> sunVecs = rsun.getSunVectors(solarParam);
		if (lastSteps != sunVecs.size())
		{
			if (solarParam.shadowInfo)
				delete[] solarParam.shadowInfo;
			solarParam.shadowInfo = new bool[sunVecs.size()];
		}
		std::string shadowInfo = "";
		for (int n = 0; n < sunVecs.size(); n++)
		{
			SunVector sunVec = sunVecs[n];
			solarParam.shadowInfo[n] = _cubemap->isShadowed((double)sunVec.alt, (double)sunVec.azimuth);
			shadowInfo += (solarParam.shadowInfo[n] ? "1" : "0");
			if (n < sunVecs.size() - 1)
				shadowInfo += ",";
		}
		//printf("%s\n", shadowInfo.data());
		solarParam.day = day;
		SolarRadiation dailyRad = rsun.calculateSolarRadiation(solarParam);
		dailyRads.push_back(dailyRad);
		annualRad = annualRad + dailyRad;
	}
	if (solarParam.shadowInfo)
		delete[] solarParam.shadowInfo;
	return annualRad;
}

std::tuple<bool, osg::Vec3d, osg::Matrixd> SolarInteractiveHandler::getGeoTransform(osg::Vec3d worldPos)
{
	osg::Matrixd local2World = osg::Matrixd::identity();
	if (!_mapNode)
		return std::make_tuple(false, worldPos, local2World);
	osg::Vec3d geoPos;
	_mapNode->getMapSRS()->transformFromWorld(worldPos, geoPos);
	_mapNode->getMapSRS()->createLocalToWorld(geoPos, local2World);
	local2World = local2World * osg::Matrixd::translate(-local2World.getTrans());
	return std::make_tuple(true, geoPos, local2World);
}

bool SolarInteractiveHandler::isEarth() { return _mapNode; }

void SolarInteractiveHandler::postDrawUpdate()
{
	_pointRenderer->postDrawUpdate();
}

std::tuple<bool, SolarRadiationPoint> SolarInteractiveHandler::queryPoint(const float& mouseX, const float& mouseY)
{
	SolarRadiationPoint solarPoint;
	osg::ref_ptr<osgUtil::LineSegmentIntersector> ray = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, mouseX, mouseY);
	osgUtil::IntersectionVisitor visitor(ray);
	_viewer->getCamera()->accept(visitor);
	if (ray->getIntersections().size() == 0)
		return std::make_tuple(false, solarPoint);
	//computeMouseIntersection(ray.get());

	osg::Vec3d curcenter = ray->getFirstIntersection().getWorldIntersectPoint();
	osg::Vec3d cureye = ray->getFirstIntersection().getWorldIntersectPoint();

	osg::Vec3d worldPos = ray->getFirstIntersection().getWorldIntersectPoint();
	osg::Vec3d surfaceNormal = ray->getFirstIntersection().getWorldIntersectNormal();
	surfaceNormal.normalize();
	worldPos = worldPos + surfaceNormal * 0.1; //offset from the surface
	return std::make_tuple(true, solarPoint);
}