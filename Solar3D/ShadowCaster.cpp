#include "ShadowCaster.h"
#include <osgDB/ReadFile>
#include <vector>
#include <sstream>
#include <qdir.h>
#include <qfile.h>
#include "osg/ComputeBoundsVisitor"
#include "osgUtil/SmoothingVisitor"
#include "osgDB/writeFile"
#include "osgDB/readFile"
//#include "GrassSolar.h"
#include "GDAL_DS.h"

ShadowCaster::~ShadowCaster()
{
	for each (GDAL_DS<float> * ds in m_rasters)
	{
		delete ds;
	}
	m_rasters.clear();

	delete[] m_slopeData;
	delete[] m_aspectData;
	m_slopeData = nullptr;
	m_aspectData = nullptr;
}

bool ShadowCaster::Intersects(const Ray& r, const osg::BoundingBoxd& aabb, double& t)
{
	double tmin, tmax, tymin, tymax, tzmin, tzmax;
	osg::Vec3d bounds[2];
	bounds[0] = aabb._min;
	bounds[1] = aabb._max;
	tmin = (bounds[r.sign[0]].x() - r.orig.x()) * r.invdir.x();
	tmax = (bounds[1 - r.sign[0]].x() - r.orig.x()) * r.invdir.x();
	tymin = (bounds[r.sign[1]].y() - r.orig.y()) * r.invdir.y();
	tymax = (bounds[1 - r.sign[1]].y() - r.orig.y()) * r.invdir.y();

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;

	tzmin = (bounds[r.sign[2]].z() - r.orig.z()) * r.invdir.z();
	tzmax = (bounds[1 - r.sign[2]].z() - r.orig.z()) * r.invdir.z();

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;

	t = tmin;

	if (t < 0) {
		t = tmax;
		if (t < 0) return false;
	}

	return true;
}

void ShadowCaster::Initialize(std::vector<std::string> rasterFiles, std::vector<osg::BoundingBoxd> bounds, std::string slopeFile, std::string aspectFile)
{
	if (m_slopeData)
		delete[] m_slopeData;
	if (m_aspectData)
		delete[] m_aspectData;
	for each (GDAL_DS<float> * ds in m_rasters)
	{
		delete ds;
	}
	m_rasters.clear();
	m_bounds.clear();
	if (rasterFiles.size() != bounds.size())
		return;

	GDAL_DS<float>* slopeDS = new GDAL_DS<float>();
	slopeDS->open(slopeFile);
	m_slopeData = slopeDS->readData(1);
	delete slopeDS;
	GDAL_DS<float>* aspectDS = new GDAL_DS<float>();
	aspectDS->open(aspectFile);
	m_aspectData = aspectDS->readData(1);
	delete aspectDS;

	for (size_t i = 0; i < rasterFiles.size(); i++)
	{
		GDAL_DS<float>* ds = new 	GDAL_DS<float>();
		ds->open(rasterFiles[i]);
		osg::BoundingBoxd bb = bounds[i];
		ds->readCache(bb);
		m_rasters.push_back(ds);
		m_bounds.push_back(bb);
	}
}

struct Color3
{
	unsigned char m_r, m_g, m_b;
	Color3()
	{
		m_r = 0; m_g = 0; m_b = 0;
	}

	Color3(unsigned char r, unsigned char g, unsigned char b)
	{
		m_r = r; m_g = g; m_b = b;
	}
};

std::tuple<int, int, int, int, double, osg::BoundingBoxd> ShadowCaster::GetCellIndex(double x, double y)
{
	for (long i = 0; i < m_rasters.size(); i++)
	{
		GDAL_DS<float>* ds = (GDAL_DS<float>*)m_rasters[i];
		int col, row, index, dsIndex;
		std::tie(col, row, index) = ds->getCellIndexCached(x, y);
		if (index >= 0)
		{
			double z = ds->m_cache[index];
			if (z == ds->m_nodata)
				continue;
			double xmin = x - ds->adfGeoTransform[1];
			double xmax = x + ds->adfGeoTransform[1];
			double ymin = y - abs(ds->adfGeoTransform[5]);
			double ymax = y + abs(ds->adfGeoTransform[5]);
			double zmin = 0;
			double zmax = max(z, 0);
			osg::BoundingBoxd bb(xmin, ymin, zmin, xmax, ymax, zmax);
			return std::make_tuple(col, row, index, i, ds->adfGeoTransform[1], bb);
		}
	}
	return std::make_tuple(-1, -1, -1, -1, -1, osg::BoundingBoxd());
}

bool ShadowCaster::Intersects(const Ray& ray, double& t, double& slope, double& aspect)
{
	slope = 0;
	aspect = 0;
	//Ray r = Ray(r.orig,r.dir);
	Ray groundRay = Ray(osg::Vec2d(ray.orig.x(), ray.orig.y()), osg::Vec2d(ray.dir.x(), ray.dir.y()));
	osg::Vec3d pos = groundRay.orig;
	GDAL_DS<float>* ds = (GDAL_DS<float>*)m_rasters[0];
	double cellSize = ds->adfGeoTransform[1];
	pos = groundRay.orig + groundRay.dir * (cellSize * 0.5);
	int step = 0;
	while (true)
	{
		int col, row, index, dsIndex;
		osg::BoundingBoxd bb;
		t = 0;
		std::tie(col, row, index, dsIndex, cellSize, bb) = GetCellIndex(pos.x(), pos.y());
		if (col == -1)
			return false;
		double t2;
		AABBox aabb(bb._min, bb._max);
		aabb.intersect(ray, t2);
		if (Intersects(ray, bb, t))
		{
			osg::Vec3d hit = ray.orig + ray.dir * t;
			if (dsIndex == 0)
			{
				slope = m_slopeData[index];
				aspect = m_aspectData[index];
			}
			return true;
		}
		step++;
		pos = pos + groundRay.dir * (cellSize * 0.5);
	}
	return false;
}

std::tuple<SolarRadiationPoint, SolarRadiationPoint> ShadowCaster::calculateSolarRadiation(SolarParam& solar_param, osg::Vec3d pos)
{
	GrassSolar grass;
	return grass.calculateSolarRadiation(solar_param, pos, this);
	/*GrassSolar grass;
	std::vector<SunVector> sunVectors = grass.getSunVectors(solar_param);
	std::vector<osg::Vec3d> lightDirs = Utils::sunVector2LightDir(sunVectors);

	solar_param.m_shadowInfo = new bool[lightDirs.size()];
	std::string shadowMasks = "";
	for (long i = 0; i < lightDirs.size(); i++)
	{
		osg::Vec3d start = pos;
		osg::Vec3d end = pos + lightDirs[i] * 100000;

		Ray r(start, end - start);
		double t, aspect, slope;
		if (Intersects(r, t, slope, aspect))
		{
			solar_param.m_shadowInfo[i] = true;
			shadowMasks += "1";
		}
		else
		{
			solar_param.m_shadowInfo[i] = false;
			shadowMasks += "0";
		}
		if (i < lightDirs.size() - 1)
		{
			shadowMasks += ",";
		}
	}

	SolarRadiation solarRad = grass.calculateSolarRadiation(solar_param);
	solarRad.m_shadowMasks = shadowMasks;
	SolarRadiationPoint radPoint(pos, solar_param, solarRad);

	solar_param.m_slope = 0;
	solar_param.m_aspect = 0;
	SolarRadiation solarRad2 = grass.calculateSolarRadiation(solar_param);
	solarRad2.m_shadowMasks = shadowMasks;
	delete[] solar_param.m_shadowInfo;
	SolarRadiationPoint radPoint2(pos, solar_param, solarRad2);
	return std::make_tuple(radPoint, radPoint2);*/
}

bool ShadowCaster::isShadowed(const double& alt, const double& azimuth, const osg::Vec3d& pos)
{
	osg::Vec3d dir = Utils::solarAngle2Vector(alt, azimuth);
	double t,slope,aspect;
	osg::Vec3d start = pos;
	osg::Vec3d end = pos + dir * 100000;
	Ray r(start, end - start);
	bool isShadowed = Intersects(r, t, slope, aspect);
	return isShadowed;
}