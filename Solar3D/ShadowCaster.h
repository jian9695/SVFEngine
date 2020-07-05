#pragma once
#include <Windows.h>
#include <string>
#include <osg/Node>
#include <osgEarth/Bounds>
#include <cstdlib>
#include <random>
#include "GrassSolar.h"

class ShadowCaster : public RayCasterBase
{
public:
	ShadowCaster()
	{
		m_slopeData = nullptr;
		m_aspectData = nullptr;
	};
	~ShadowCaster();
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
