#pragma once
#include <osg/Geode>
#include <osg/Vec3d>
#include <osg/PagedLOD>

//Render a point at a 3D position picked by mouse double-click 

class TextLOD : public osg::PagedLOD
{
public:
	TextLOD(osg::Vec3d center) { _center = center; };
	osg::BoundingSphere computeBound() const
	{
		return osg::BoundingSphere(_center, 100);
	}
private:
	osg::Vec3d _center;
};

class PointsRenderer : public osg::Group
{
private:
	PointsRenderer() {};
public:
	static PointsRenderer* create();
	void pushPoint(const osg::Vec3d& point, const std::string& label);
	void popPoint();
};

