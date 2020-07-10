#pragma once
#include <Windows.h>
#include <string>
#include <osg/Node>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/Bounds>
#include <cstdlib>
#include <random>
#include "GrassSolar.h"

class OSGRayCaster : public RayCasterBase
{
public:
	osg::Node* m_scene;
	OSGRayCaster(osg::Node* scene)
	{
		m_scene = scene;
	};

	bool isShadowed(const double& alt, const double& azimuth, const osg::Vec3d& pos)
	{
		osg::Vec3d dir = Utils::solarAngle2Vector(alt, azimuth);
		osg::Vec3d start = pos;
		osg::Vec3d end = pos + dir * 10000000;
		osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
		osgUtil::IntersectionVisitor intersectVisitor(intersector.get());
		m_scene->accept(intersectVisitor);
		if (intersector->containsIntersections())
		{
			return true;
		}
		return false;
	}
};

class DSMRayCaster : public RayCasterBase
{
public:
	DSMRayCaster()
	{
		m_slopeData = nullptr;
		m_aspectData = nullptr;
	};
	~DSMRayCaster();
	void Initialize(std::vector<std::string> rasterFiles, std::vector<osg::BoundingBoxd> bounds, std::string slopeFile, std::string aspectFile);
	static bool Intersects(const Ray& r, const osg::BoundingBoxd& aabb, double& t);
	std::tuple<int, int, int, int, double, osg::BoundingBoxd> GetCellIndex(double x, double y);
	bool Intersects(const Ray& ray, double& t, double& slope, double& aspect);
	std::tuple<SolarRadiationPoint, SolarRadiationPoint> calculateSolarRadiation(SolarParam& solar_param, osg::Vec3d pos);
	bool isShadowed(const double& alt, const double& azimuth, const osg::Vec3d& pos);

public:
	std::vector<void*> m_rasters;
	std::vector<osg::BoundingBoxd> m_bounds;
	float* m_slopeData;
	float* m_aspectData;
private:
	double m_slope;
	double m_aspect;
};
