#pragma once

#ifdef _WIN32 || WIN32
#include<Windows.h>
#endif

#include <string>
#include <osg/Node>
#include <osg/BoundingBox>
#include <cstdlib>
#include <random>
//#include "GDAL_DS.h"

class BBWrapper : public osg::BoundingBoxd
{
public:
		BBWrapper() :
				osg::BoundingBoxd()
		{

		}

		BBWrapper(const osg::BoundingBoxd& bb) :
				osg::BoundingBoxd(bb)
		{
				bounds[0] = _min;
				bounds[1] = _max;
		}

		BBWrapper(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax) :
				osg::BoundingBoxd(xmin, ymin, zmin, xmax, ymax, zmax)
		{
				bounds[0] = _min;
				bounds[1] = _max;
		}

		BBWrapper(osg::Vec3d min, osg::Vec3d max) :
				osg::BoundingBoxd(min, max)
		{
				bounds[0] = _min;
				bounds[1] = _max;
		}

		double xsize() { return xMax() - xMin(); }
		double ysize() { return yMax() - yMin(); }
		double zsize() { return zMax() - zMin(); }
		double xhalfsize() { return xsize() * 0.5; }
		double yhalfsize() { return ysize() * 0.5; }
		double zhalfsize() { return zsize() * 0.5; }
		osg::Vec3d size() { return _max - _min; }
		osg::Vec3d halfsize() { return size() * 0.5; }
		osg::Vec3d bounds[2];
};
//https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes&src=1
class Ray
{
public:


		Ray(const osg::Vec3d &orig, const osg::Vec3d &dir)
		{
				this->orig = orig; 
				this->dir = dir;
				this->dir.normalize();
				invdir = osg::Vec3d(1.0 / this->dir.x(), 1.0 / this->dir.y(), 1.0 / this->dir.z());
				sign[0] = (invdir.x() < 0);
				sign[1] = (invdir.y() < 0);
				sign[2] = (invdir.z() < 0);
		}


		Ray(const osg::Vec2d &orig, const osg::Vec2d &dir)
				:Ray(osg::Vec3d(orig.x(), orig.y(), 0.0), osg::Vec3d(dir.x(), dir.y(), 0.0))
		{
		}

		//bool intersect(const AABBox &bb, double &t) const
		//{
		//		double tmin, tmax, tymin, tymax, tzmin, tzmax;

		//		tmin = (bb.bounds[sign[0]].x() - orig.x()) * invdir.x();
		//		tmax = (bb.bounds[1 - sign[0]].x() - orig.x()) * invdir.x();
		//		tymin = (bb.bounds[sign[1]].y() - orig.y()) * invdir.y();
		//		tymax = (bb.bounds[1 - sign[1]].y() - orig.y()) * invdir.y();

		//		if ((tmin > tymax) || (tymin > tmax))
		//				return false;

		//		if (tymin > tmin)
		//				tmin = tymin;
		//		if (tymax < tmax)
		//				tmax = tymax;

		//		tzmin = (bb.bounds[sign[2]].z() - orig.z()) * invdir.z();
		//		tzmax = (bb.bounds[1 - sign[2]].z() - orig.z()) * invdir.z();

		//		if ((tmin > tzmax) || (tzmin > tmax))
		//				return false;

		//		if (tzmin > tmin)
		//				tmin = tzmin;
		//		if (tzmax < tmax)
		//				tmax = tzmax;

		//		t = tmin;

		//		if (t < 0) {
		//				t = tmax;
		//				if (t < 0) return false;
		//		}

		//		return true;
		//}

		osg::Vec3d orig, dir; // ray orig and dir 
		osg::Vec3d invdir;
		int sign[3];
};

class AABBox
{
public:
		AABBox(const osg::Vec3d &b0, const osg::Vec3d &b1) { bounds[0] = b0, bounds[1] = b1; }

		AABBox(const osg::Vec2d &b0, const osg::Vec2d &b1) { bounds[0] = osg::Vec3d(b0.x(), b0.y(), 0.0), bounds[1] = osg::Vec3d(b1.x(), b1.y(), 0.0); }

		bool intersect(Ray r) const
		{
				double t;
				return	intersect(r, t);
		}

		bool intersect(const Ray &r, double &t) const
		{
				double tmin, tmax, tymin, tymax, tzmin, tzmax;

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
		osg::Vec3d bounds[2];
};

class ModelLoader
{
public:
	ModelLoader();
	~ModelLoader();
	static osg::Node* LoadModel(std::string path, bool& isIntegratedMesh);
	static osg::Node* Load3DTiles(std::string indir);
	static osg::Node* Load3DTiles(std::string indir, osg::BoundingBox mask, bool intersects);
	static osg::Node* Load3DTiles(std::string indir, std::vector<std::string> maskTiles, bool include);
	static void CopyLeafTiles(std::string indir, std::string outdir, osg::BoundingBox bb);
	static void CopyLeafTiles(std::string indir, std::vector<std::string> tilenames, std::string outfile);
	static void Test();
	static void TileBoundary2Shapefile(std::string indir, std::string outfile);
	static osg::BoundingBoxd CalBound(std::string indir, const std::vector<std::string>& tiles);
};