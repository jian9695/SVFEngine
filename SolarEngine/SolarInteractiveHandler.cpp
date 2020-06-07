#include "SolarInteractiveHandler.h"
#include <osg/ComputeBoundsVisitor>

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

	//create a node to render a cubemap from a 3D position picked by mouse double-click
	_cubemap = Cubemap::create(512, sceneNode);
	_cubemap->setNodeMask(false);
	root->addChild(_cubemap);

	for (int i = 0; i < _cubemap->getNumChildren(); i++)
	{
		_cubemap->getFace(i)->setRenderOrder(osg::Camera::PRE_RENDER, i);
	}

	//create a node to transform a cubemap into a fisheye view and then render onto an off-screen image for svf calculation
	//_cubemap2fisheyeCamera = CameraBuffer::createSlave(512, 512, contexts[0]);
	_cubemap2fisheyeCamera = _cubemap->toHemisphericalSurface();

	root->addChild(_cubemap2fisheyeCamera);

	_cubemap2fisheyeCamera->setRenderOrder(osg::Camera::PRE_RENDER, _cubemap->getNumChildren());

	//create point label for highlighting intersection point
	_pointRenderer = new PointsRenderer;
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
	//_viewer->getCamera()->setNodeMask(false);
	_viewer->frame();
	for (size_t i = 0; i < 2; i++)
	{
		_viewer->frame();
	}
	_cubemap->setNodeMask(false);
	//_viewer->getCamera()->setNodeMask(true);
	if (_cubemap2fisheyeCamera)
	{
		//for (size_t i = 0; i < _cubemap->getNumChildren(); i++)
		//{
				//RenderSurface* cameraBuffer = (RenderSurface*)_cubemap->getChild(i);
				//osgDB::writeImageFile(*cameraBuffer->Image(), cameraBuffer->getName() + ".png");
		//}
	 //osg::ref_ptr<osg::Image> fisheye =	cubemap2fisheye(512, 512, getCubemapCameras(_cubemapCameras.get()));
	 //osgDB::writeImageFile(*fisheye, "fisheye2.png");
	 //osgDB::writeImageFile(*_cubemap2fisheyeCamera->Image(), "fisheye.png");
		double svf = Utils::calSVF(_cubemap2fisheyeCamera->Image().get(), false);
		//SolarRadiation solarRad = SolarInteractiveHandler::calSolar(_cubemap2fisheyeCamera->Image().get(), _solarParam, observer, observerNormal, _sceneNode);
		//printf("SVF=%f\n", svf);
		SolarRadiation solarRad = calSolar(geoPos, surfaceNormal);
		_resultsCallback(svf, solarRad);
		_pointRenderer->pushPoint(worldPos, Utils::value2String(solarRad.global, 2));
	}
}

bool SolarInteractiveHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME)
		return false;
	osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
	if (!viewer)
		return false;
	osg::Vec3d orieye, oricenter, oriup;
	viewer->getCamera()->getViewMatrixAsLookAt(orieye, oricenter, oriup);
	if (ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON && (ea.getModKeyMask() & ea.MODKEY_CTRL) && ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
		//if ((ea.getModKeyMask() & ea.MODKEY_CTRL) != 0 && ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
	{
		osg::ref_ptr<osgUtil::LineSegmentIntersector> ray = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, ea.getXnormalized(), ea.getYnormalized());
		osgUtil::IntersectionVisitor visitor(ray);
		viewer->getCamera()->accept(visitor);
		computeMouseIntersection(ray.get());
	}

	return false;
}

SolarRadiation SolarInteractiveHandler::calSolar(osg::Vec3d geoPos, osg::Vec3d normal)
{
	osg::Image* img = _cubemap2fisheyeCamera->Image().get();
	SolarRadiation annualRad;
	annualRad.Zero();
	std::vector<SolarRadiation> dailyRads;
	SolarParam param = *_solarParam;
	param.lon = geoPos.x();
	param.lat = geoPos.y();
	param.elev = max(geoPos.z(), 0);
	param.slope = Utils::calculateSlope(normal);
	param.aspect = Utils::calculateAspect(normal);
	int startDay = param.startDay;
	int endDay = param.endDay;
	if (param.isSingleDay && endDay > startDay)
		endDay = startDay;
	int lastSteps = 0;
	GrassSolar rsun;
	for (int day = startDay; day <= endDay; day++)
	{
		param.startDay = day;
		std::vector<SunVector> sunVecs = rsun.getSunVectors(param);
		if (lastSteps != sunVecs.size())
		{
			if (param.shadowInfo)
				delete[] param.shadowInfo;
			param.shadowInfo = new bool[sunVecs.size()];
		}
		for (int n = 0; n < sunVecs.size(); n++)
		{
			SunVector sunVec = sunVecs[n];
			param.shadowInfo[n] = _cubemap->isShadowed(sunVec.alt, sunVec.azimuth);
		}

		param.day = day;
		SolarRadiation dailyRad = rsun.calculateSolarRadiation(param);
		dailyRads.push_back(dailyRad);
		annualRad = annualRad + dailyRad;
	}
	if (param.shadowInfo)
		delete[] param.shadowInfo;
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