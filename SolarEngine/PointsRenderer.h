#pragma once
#include <osg/Geode>
#include <osg/Vec3d>
#include <osg/PagedLOD>
#include "Utils.h"

enum ActionTypeEnum { PUSH, POP };
struct Action : SolarRadiationPoint
{
	ActionTypeEnum actionType;
	Action() {}
	Action(ActionTypeEnum type, SolarRadiationPoint point) { actionType = type; *((SolarRadiationPoint*)this) = point; }
};

class TextLOD : public osg::PagedLOD
{
public:
	TextLOD(const SolarRadiationPoint& point) { _point = point; };
	osg::BoundingSphere computeBound() const
	{
		return osg::BoundingSphere(_point.pos, 100);
	}
public:
	SolarRadiationPoint _point;
};

class PointsRenderer : public osg::Group
{
public:
	PointsRenderer() {};
	void pushPoint(const osg::Vec3d& point, const SolarParam& param, const SolarRadiation& rad);
	void pushPoint(const SolarRadiationPoint& point);
	void popPoint();
	void undo();
	void redo();
private:
	std::vector<Action> m_doStack;
	std::vector<Action> m_undoStack;
private:
	void doAction(const Action& action);
	void pushPointInternal(const SolarRadiationPoint& point);
	void popPointInternal();
};

