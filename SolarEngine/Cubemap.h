#pragma once

#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif

#include <osg/Texture2D>
#include <osg/Geometry>
#include <osg/Camera>
#include <osg/Geode>
#include <osg/ClampColor>
#include <osg/TextureCubeMap>
#include "RenderSurface.h"

class CubemapSurface : public RenderSurface
{
public:
	osg::Vec3d m_pos;//position of camera
	osg::Vec3d m_dir;//viewing direction of camera
	osg::Vec3d m_up;//the up vector
	osg::Matrixd m_local2World;
	osg::Matrixd m_viewProjMatrix;

	CubemapSurface(int width, int height, GLenum internalFormat, GLenum sourceFormat, GLenum sourceType, bool allocateImage,
		osg::Vec3d dir, osg::Vec3d up, std::string name);

	void update();
};

class Cubemap : public osg::Group
{
private:
	Cubemap() : osg::Group() {  }
public:
	static Cubemap* create(int imageSize, osg::Node* scene = nullptr);
	CubemapSurface* getFace(int face);
	CubemapSurface* getFace(osg::TextureCubeMap::Face face);
	osg::TextureCubeMap::Face getFaceNum(float alt, float azimuth);
	CubemapSurface* getFace(float alt, float azimuth);
	RenderSurface* toHemisphericalSurface();
	osg::Image* toHemisphericalImage(int width, int height);
	bool isShadowed(double alt, double azimuth);
	void resize(int size);
};