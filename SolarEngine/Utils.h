#pragma once
#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif

#include <osg/Vec3>
#include <osg/Vec3d>
#include <vector>
#include <osg/Image>
#include "TypeDef.h"

class Utils
{
public:
	static double calculateAspect(osg::Vec3d normal);
	static double calculateSlope(osg::Vec3d normal);
	static double calAzimuthAngle(double x, double y);
	static osg::Vec3d solarAngle2Vector(double alt, double azimuth);
	static std::vector<osg::Vec3d> sunVector2LightDir(std::vector<SunVector>& sunvectors);
	static double calSVF(osg::Image* img, bool applyLambert);
	static SolarRadiation calSolar(osg::Image* img, SolarParam* solarParam, osg::Vec3d pos, osg::Vec3d normal, osg::Node* sceneNode);
	static std::string value2String(float value, int precision);
	static osg::Vec3 WorldPosFromDepth(float depth, osg::Matrixd& projMatrixInv, osg::Matrixd& viewMatrixInv, float u, float v);
};

