#include "SVFComputeTools.h"
#include <osg/ComputeBoundsVisitor>
#include <iomanip>

osg::TextureCubeMap::Face getCubemapFace(float alt, float azimuth)
{
	if (alt > 45)
		return osg::TextureCubeMap::POSITIVE_Z;
	else if (alt < -45)
		return osg::TextureCubeMap::NEGATIVE_Z;
	if (azimuth < 45)
		return  osg::TextureCubeMap::POSITIVE_Y;
	if (azimuth < 135)
		return  osg::TextureCubeMap::POSITIVE_X;
	if (azimuth < 225)
		return  osg::TextureCubeMap::NEGATIVE_Y;
	if (azimuth < 315)
		return  osg::TextureCubeMap::NEGATIVE_X;
	return  osg::TextureCubeMap::POSITIVE_Y;
}

std::vector<CubemapSurface*> getCubemapCameras(osg::Group* cubemapCameras)
{
	std::vector<CubemapSurface*> cameras;
	for (size_t i = 0; i < 6; i++)
	{
		cameras.push_back((CubemapSurface*)cubemapCameras->getChild(i));
	}
	return cameras;
}

osg::Image* cubemap2fisheye(int width, int height, const std::vector<CubemapSurface*>& cubemap)
{
	osg::Image* fisheye = new osg::Image;
	fisheye->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
	double yresol = 1.0 / height;
	double xresol = 1.0 / width;
	ColorUB4* data = (ColorUB4*)fisheye->data();
	for (int row = 0; row < height; row++)
	{
		double y = (row / (height - 1.0) - 0.5) * 2.0;
		for (int col = 0; col < width; col++)
		{
			double x = (col / (width - 1.0) - 0.5) * 2.0;
			double radius = sqrt(x * x + y * y);
			if (radius > 1.0)
			{
				*data++ = ColorUB4(osg::Vec4(0, 0, 1, 0.3));
				continue;
			}
			double azimuth = GrassSolar::calAzimuthAngle(x, y);
			double alt = (1.0 - radius) * 90.0;
			osg::TextureCubeMap::Face face = getCubemapFace(alt, azimuth);
			osg::Image* faceImage = cubemap[face]->Image();
			osg::Vec3d sundir = GrassSolar::solarAngle2Vector(alt, azimuth);
			osg::Vec3d targetPos = cubemap[face]->_pos + sundir * 10000;
			osg::Vec3d screenPos = targetPos * cubemap[face]->_viewProjMatrix;
			double u = (screenPos.x() + 1.0) * 0.5;
			double v = (screenPos.y() + 1.0) * 0.5;
			osg::Vec4 color = faceImage->getColor(osg::Vec2(u, v));

			float scaleX = (float)col / (width - 1.0);
			scaleX = max(scaleX, 0.2);
			float scaleY = (float)row / (height - 1.0);
			scaleY = max(scaleY, 0.2);
			color = osg::Vec4(color.r() * scaleX, color.g() * scaleY, color.b(), 1.0);
			*data++ = ColorUB4(color);
		}
	}
	return fisheye;
}

bool isShadowed(double alt, double azimuth, const std::vector<CubemapSurface*>& cubemap)
{
	osg::TextureCubeMap::Face face = getCubemapFace(alt, azimuth);
	osg::Image* faceImage = cubemap[face]->Image();
	osg::Vec3d sundir = GrassSolar::solarAngle2Vector(alt, azimuth);
	osg::Vec3d targetPos = cubemap[face]->_pos + sundir * 10000;
	osg::Vec3d screenPos = targetPos * cubemap[face]->_viewProjMatrix;
	double u = (screenPos.x() + 1.0) * 0.5;
	double v = (screenPos.y() + 1.0) * 0.5;
	osg::Vec4 color = faceImage->getColor(osg::Vec2(u, v));
	return color.a() > 0.65;
}
SVFComputeTools::SVFComputeTools()
{
}

SVFComputeTools::~SVFComputeTools()
{
}

osg::Group * SVFComputeTools::createSVFCameras(osg::Node * city)
{
	osg::Group* cameras = new osg::Group;
	int w = 512;
	int h = 512;
	cameras->addChild(new CubemapSurface(w, h, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, true, osg::Vec3d(1, 0, 0), osg::Vec3d(0, 0, 1), "POS_X"));
	cameras->addChild(new CubemapSurface(w, h, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, true, osg::Vec3(-1, 0, 0), osg::Vec3(0, 0, 1), "NEG_X"));
	cameras->addChild(new CubemapSurface(w, h, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, true, osg::Vec3(0, 1, 0), osg::Vec3(0, 0, 1), "POS_Y"));
	cameras->addChild(new CubemapSurface(w, h, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, true, osg::Vec3(0, -1, 0), osg::Vec3(0, 0, 1), "NEG_Y"));
	cameras->addChild(new CubemapSurface(w, h, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, true, osg::Vec3(0, 0, 1), osg::Vec3(0, -1, 0), "POS_Z"));
	cameras->addChild(new CubemapSurface(w, h, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, true, osg::Vec3(0, 0, -1), osg::Vec3(0, 1, 0), "NEG_Z"));
	for (size_t i = 0; i < cameras->getNumChildren(); i++)
	{
		((CubemapSurface*)cameras->getChild(i))->addChild(city);
	}
	return cameras;
}

RenderSurface* SVFComputeTools::cubemap2hemispherical(osg::Group * svfCameraBuffers)
{
	//osg::TextureCubeMap* cubemap = SkyDome::loadCubeMapTextures("E:/OpenSceneGraphSVF/OpenSceneGraphSVF/images_WEIHAI", ".png");
	enum { POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z };

	osg::ref_ptr<osg::TextureCubeMap> cubeMap = new osg::TextureCubeMap;
	cubeMap->setInternalFormat(GL_RGBA);

	cubeMap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
	cubeMap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	cubeMap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
	cubeMap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

	cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_X, ((RenderSurface*)svfCameraBuffers->getChild(NEG_X))->Image());
	cubeMap->setImage(osg::TextureCubeMap::POSITIVE_X, ((RenderSurface*)svfCameraBuffers->getChild(POS_X))->Image());
	cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Y, ((RenderSurface*)svfCameraBuffers->getChild(POS_Z))->Image());
	cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Y, ((RenderSurface*)svfCameraBuffers->getChild(NEG_Z))->Image());
	cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Z, ((RenderSurface*)svfCameraBuffers->getChild(NEG_Y))->Image());
	cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Z, ((RenderSurface*)svfCameraBuffers->getChild(POS_Y))->Image());
	//cubeMap->setImage(osg::TextureCubeMap::POSITIVE_X, ((RenderSurface*)svfCameraBuffers->getChild(osg::TextureCubeMap::POSITIVE_X))->Image());
	//cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_X, ((RenderSurface*)svfCameraBuffers->getChild(osg::TextureCubeMap::NEGATIVE_X))->Image());
	//cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Y, ((RenderSurface*)svfCameraBuffers->getChild(osg::TextureCubeMap::POSITIVE_Y))->Image());
	//cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Y, ((RenderSurface*)svfCameraBuffers->getChild(osg::TextureCubeMap::NEGATIVE_Y))->Image());
	//cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Z, ((RenderSurface*)svfCameraBuffers->getChild(osg::TextureCubeMap::POSITIVE_Z))->Image());
	//cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Z, ((RenderSurface*)svfCameraBuffers->getChild(osg::TextureCubeMap::NEGATIVE_Z))->Image());
	char vertexSource[] =
		"void main(void)\n"
		"{\n"
		"   gl_TexCoord[0] = vec4(gl_Vertex.x,gl_Vertex.y,0,1.0);\n"
		"   gl_Position   = vec4(-gl_Vertex.x,gl_Vertex.y,0,1);\n"
		"}\n";

	char fragmentSource[] =
		"uniform vec4 color;\n"
		"uniform float alpha;\n"
		"uniform samplerCube uEnvironmentMap;\n"
		"uniform float rotateAngle;\n"
		"\n"
		"vec3 spherical2Cartisian(float lon, float lat)\n"
		"{\n"
		"float theta = lon * 0.0174533;\n"
		"float phi =   lat* 0.0174533;\n"
		"return vec3(cos(phi)*cos(theta), cos(phi)*sin(theta), sin(phi));\n"
		"}\n"

		"void main(void) \n"
		"{\n"
		"    float radius = length(gl_TexCoord[0].xy);\n"
		"    vec3 tex = vec3(-gl_TexCoord[0].x, gl_TexCoord[0].y, -(1-radius));\n"
		"    vec4 rgba = textureCube( uEnvironmentMap, tex.xzy);\n"
		"    if(radius > 1)\n"
		"    {\n"
		"        rgba = vec4(0,0,1,0.3);\n"
		"    }\n"
		"    else if(rgba.a < 0.5)\n"
		"    {\n"
		"        rgba = vec4(1, 0, 0, 0.65);\n"
		"    }\n"
		"    else\n"
		"    {\n"
		"        rgba = vec4(rgba.rgb,0.85);\n"
		"    }\n"
		"    gl_FragColor = rgba;\n"
		//"    if(radius > 1)\n"
		//"    {\n"
		//"        gl_FragColor = vec4(0,0,1,0.3);\n"
		//"    }\n"
		//"    else if(gl_FragColor.a < 0.5)\n"
		//"    {\n"
		//"        gl_FragColor = vec4(0.207843137, 0.317647059, 0.360784314,0.5);\n"
		//"    }\n"
		//"    else\n"
		//"    {\n"
		//"        gl_FragColor = vec4(gl_FragColor.rgb, 0.75);\n"
		//"    }\n"
		"}\n";

	OverlayRenderSurface* fisheye = new OverlayRenderSurface(512, 512, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, true);
	fisheye->Overlay()->setProgramName("fisheye");
	//memcpy(updateParticlesSurfaceStorage->Image()->data(), particlesMap->data(), imageSize);
	//Store the updated particles
	fisheye->Overlay()->SetTextureLayer(cubeMap.get(), 0);
	fisheye->Overlay()->SetVertexShader(vertexSource);
	fisheye->Overlay()->SetFragmentShader(fragmentSource);
	fisheye->getOrCreateStateSet()->addUniform(new osg::Uniform("uEnvironmentMap", 0));
	fisheye->getOrCreateStateSet()->addUniform(new osg::Uniform("alpha", 0.0f));
  return fisheye;
}

osg::Node * SVFComputeTools::createTextureRect(std::string texfile)
{
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
	tex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::CLAMP_TO_BORDER);
	tex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::CLAMP_TO_BORDER);
	tex->setImage(osgDB::readImageFile(texfile));

	osg::Geode* geode = new osg::Geode;
	//createGeometry( );
	//SetupStateSet(cubemap);
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec3Array* normals = new osg::Vec3Array();
	vertices->push_back(osg::Vec3(-1, -1, 1));
	vertices->push_back(osg::Vec3(-1, 1, 1));
	vertices->push_back(osg::Vec3(1, -1, 1));
	vertices->push_back(osg::Vec3(1, -1, 1));
	vertices->push_back(osg::Vec3(-1, 1, 1));
	vertices->push_back(osg::Vec3(1, 1, 1));
	normals->push_back(osg::Vec3(0, 0, 1));
	geom->setVertexArray(vertices);
	geom->setNormalArray(normals);
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geom->setCullingActive(false);
	geode->setCullingActive(false);
	geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,
		osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));
	geode->addDrawable(geom.get());
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	osg::Program* program = new osg::Program;

	char vertexSource[] =
		"void main(void)\n"
		"{\n"
		"   gl_TexCoord[0] = vec4(gl_Vertex.x,gl_Vertex.y,0,1.0);\n"
		"   //gl_Position   = vec4(gl_Vertex.x*0.5-0.5,gl_Vertex.y*0.5+0.5,0,1);\n"
		"   gl_Position   = vec4(gl_Vertex.x*0.5+0.5,gl_Vertex.y*0.5+0.5,0,1);\n"
		"}\n";
	char fragmentSource[] =
		"uniform sampler2D texture0;\n"
		"uniform float rotateAngle;\n"
		"vec2 rotate(vec2 uv,float angle)\n"
		"{\n"
		"    angle = angle * 0.0174533;\n"
		"    float sin_factor = sin(angle);\n"
		"    float cos_factor = cos(angle);\n"
		"    uv = (uv - 0.5) * mat2(cos_factor, sin_factor, -sin_factor, cos_factor);\n"
		"    uv += 0.5;\n"
		"    return uv;\n"
		"}\n"
		"void main(void) \n"
		"{\n"
		"    vec2 uv = gl_TexCoord[0].xy; \n"
		"    uv = uv * 0.5 + 0.5;\n"
		"    uv = rotate(uv,rotateAngle);\n"
		"    vec4 color =   texture2D( texture0, uv );\n"
		"    gl_FragColor = vec4(color.rgb,0.5);\n"
		"}\n";
	program->setName("sky_dome_shader");
	program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexSource));
	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));
	geode->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->addUniform(new osg::Uniform("texture0", 0));
	geode->getOrCreateStateSet()->addUniform(new osg::Uniform("rotateAngle", 0.0f));

	//geode->getOrCreateStateSet()->setRenderBinDetails(100000, "RenderBin");
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	//g_pPanoState = geode->getOrCreateStateSet();
	return geode;
}

double SVFComputeTools::calSVF(osg::Image * img, bool applyLambert)
{
	unsigned int skypixels = 0;
	unsigned int nonskypixels = 0;
	unsigned char* data = img->data();
	unsigned int numpixels = img->s() * img->t();
	unsigned int ncols = img->s();
	unsigned int nrows = img->t();
	if (ncols != nrows)
		return 0;
	double resol = 1.0 / nrows;
	double totalarea = 0;
	double skyarea = 0;
	double y = resol * 0.5 - 0.5;
	for (unsigned int row = 0; row < nrows; row++)
	{
		y += resol;
		double x = resol * 0.5 - 0.5;
		for (unsigned int col = 0; col < ncols; col++)
		{
			unsigned char a = data[3];
			if (a == 0) {
				x += resol;
				data += 4;
				continue;//outside
			}
			double zenithD = sqrt(x*x + y*y) * 90.0;//in degrees
			if (zenithD <= 0.000000001)
				zenithD = 0.000000001;
			double zenithR = zenithD * 3.1415926 / 180.0;
			double wproj = sin(zenithR) / (zenithD / 90);//weight for equal-areal projection
			if (applyLambert)
			{
				wproj = wproj * cos(zenithR);
			}
			totalarea += wproj;
			if (a < 130)
			{
				skypixels++;
				skyarea += wproj;
			}
			else
			{
				nonskypixels++;
			}
			x += resol;
			data += 4;
		}

	}
	double svf = skyarea / totalarea;
	return svf;
}

SolarRadiation SVFComputeTools::calSolar(osg::Image* img, SolarParam* solarParam, osg::Vec3d pos, osg::Vec3d normal, osg::Node* sceneNode)
{
	SolarRadiation annualRad;
	annualRad.Zero();
	std::vector<SolarRadiation> dailyRads;
	SolarParam param = *solarParam;
	param.slope = GrassSolar::calculateSlope(normal);
	param.aspect = GrassSolar::calculateAspect(normal);

	int startDay = param.startDay;
	int endDay = param.endDay;
	if (endDay > startDay)
		endDay = startDay;
	int lastSteps = 0;
	GrassSolar rsun;
	for (int day = startDay; day <= endDay; day++)
	{
		param.startDay = day;
		std::vector<bool> shadowMasks;
		std::vector<bool> shadowMasksRayCaster;
		std::vector<SunVector> sunVecs = rsun.getSunVectors(param);
		shadowMasks.resize(sunVecs.size());
		if (lastSteps != sunVecs.size())
		{
			if (param.shadowInfo)
				delete[] param.shadowInfo;
			param.shadowInfo = new bool[sunVecs.size()];
		}
		for (int n = 0; n < sunVecs.size(); n++)
		{
			SunVector sunVec = sunVecs[n];
			osg::Vec3d sundir = GrassSolar::solarAngle2Vector(sunVec.alt, sunVec.azimuth);
			osg::Vec3d start = pos;
			osg::Vec3d end = pos + sundir * 10000000;
			osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
			osgUtil::IntersectionVisitor intersectVisitor(intersector.get());
			sceneNode->accept(intersectVisitor);
			if (intersector->containsIntersections())
			{
				shadowMasksRayCaster.push_back(true);
			}
			else
			{
				shadowMasksRayCaster.push_back(false);
			}
			double radius = (90.0 - sunVec.alt) / 90.0 * 0.5;
			//double radius = sin(osg::DegreesToRadians(90.0 - sunVec.alt)) * 0.5;
			double theta = sunVec.azimuth - 90;
			if (theta < 0)
				theta += 360;
			theta = osg::DegreesToRadians(theta);
			double x = radius * cos(theta);
			double y = radius * sin(theta);
			x += 0.5;
			y += 0.5;
			osg::Vec4 color = img->getColor(osg::Vec2(x, y));
			param.shadowInfo[n] = color.a() > 0.7 ? true : false;
			shadowMasks[n] = param.shadowInfo[n];
		}

		for (int n = 0; n < sunVecs.size(); n++)
		{
			printf("%d", shadowMasks[n] ? 1 : 0);
			if(n == sunVecs.size() - 1)
				printf("\n");
			else
				printf(",");
		}

		for (int n = 0; n < sunVecs.size(); n++)
		{
			printf("%d", shadowMasksRayCaster[n] ? 1 : 0);
			if (n == sunVecs.size() - 1)
				printf("\n");
			else
				printf(",");
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

SkyViewFactorEventHandler::SkyViewFactorEventHandler(osg::Node* threeDModel, osg::Group * root, osg::ref_ptr<osgGA::CameraManipulator> manip, 
	osgViewer::Viewer * viewer,
	SolarParam* solarParam,
	OnResultsUpdated resultsCallback)
{
	_viewer = viewer;
	_manip = manip;
	_root = root;
	_solarParam = solarParam;
	_resultsCallback = resultsCallback;
	_sceneNode = threeDModel;
	_renderGroup = new osg::Group;
	_root->addChild(_renderGroup.get());

	//create point label for highlighting intersection point
	_pointRenderer = new PointRenderer;
	_pointRenderer->create();
	_renderGroup->addChild(_pointRenderer.get());

	std::vector<osg::GraphicsContext*> contexts;
	_viewer->getContexts(contexts);
	//create a node to render a cubemap from a 3D position picked by mouse double-click
	_cubemapCameras = SVFComputeTools::createSVFCameras(threeDModel);
	_cubemapCameras->setNodeMask(false);
	root->addChild(_cubemapCameras.get());

	//create a node to transform a cubemap into a fisheye view and then render onto an off-screen image for svf calculation
	//_cubemap2fisheyeCamera = CameraBuffer::createSlave(512, 512, contexts[0]);
	_cubemap2fisheyeCamera = SVFComputeTools::cubemap2hemispherical(_cubemapCameras);

	root->addChild(_cubemap2fisheyeCamera.get());

	for (size_t i = 0; i < _cubemapCameras->getNumChildren(); i++)
	{
		((osg::Camera*)_cubemapCameras->getChild(i))->setRenderOrder(osg::Camera::PRE_RENDER, i);
	}
	_cubemap2fisheyeCamera->setRenderOrder(osg::Camera::PRE_RENDER, _cubemapCameras->getNumChildren());
}

SkyViewFactorEventHandler::~SkyViewFactorEventHandler()
{
	_root->removeChild(_renderGroup);
}
void SkyViewFactorEventHandler::printfVec3d(osg::Vec3d v)
{
	printf("%f,%f,%f\n", v.x(), v.y(), v.z());
}

void SkyViewFactorEventHandler::printfVec3(osg::Vec3 v)
{
	printf("%f,%f,%f\n", v.x(), v.y(), v.z());
}

void SkyViewFactorEventHandler::computeMouseIntersection(osgUtil::LineSegmentIntersector * ray)
{

	osg::Vec3d orieye, oricenter, oriup;
	_viewer->getCamera()->getViewMatrixAsLookAt(orieye, oricenter, oriup);
	if (ray->getIntersections().size() == 0)
		return;
	osg::Vec3d _dir = orieye - oricenter;
	_dir.normalize();
	osg::Vec3d curcenter = ray->getFirstIntersection().getWorldIntersectPoint();

	osg::Vec3d cureye = ray->getFirstIntersection().getWorldIntersectPoint() + _dir * 50;

	//_viewer->setCameraManipulator(NULL, false);
	_viewer->frame();
	//while (_viewer->getDatabasePager()->getRequestsInProgress())
	//{
	//		_viewer->frame();
	//}
	for (size_t i = 0; i < 6; i++)
	{
			_viewer->frame();
	}
	osgUtil::IntersectionVisitor visitor(ray);
	_viewer->getCamera()->accept(visitor);
	//printfVec3(ray->getFirstIntersection().getWorldIntersectPoint());
	osg::Vec3d observer = ray->getFirstIntersection().getWorldIntersectPoint();
	osg::Vec3d observerNormal = ray->getFirstIntersection().getWorldIntersectNormal();
	observerNormal.normalize();
	observer = observer + observerNormal * 0.1; //distance of camera from the surface
	//printfVec3(observerNormal);
	_pointRenderer->setPoint(observer);

	for (size_t i = 0; i < _cubemapCameras->getNumChildren(); i++)
	{
		CubemapSurface* cameraBuffer = (CubemapSurface*)_cubemapCameras->getChild(i);
		cameraBuffer->_pos = observer;
	}
	_cubemapCameras->setNodeMask(true);
	_viewer->frame();
	//while (_viewer->getDatabasePager()->getRequestsInProgress())
	//{
	//		_viewer->frame();
	//}
	for (size_t i = 0; i < 6; i++)
	{
			_viewer->frame();
	}
	_cubemapCameras->setNodeMask(false);
	_viewer->getCamera()->setViewMatrixAsLookAt(orieye, oricenter, oriup);

	osg::Matrix viewmat;
	viewmat.makeLookAt(orieye, oricenter, oriup);
	//_viewer->setCameraManipulator(_manip.get(), false);
	_manip->setByInverseMatrix(viewmat);
	_viewer->getCamera()->getViewMatrixAsLookAt(orieye, oricenter, oriup);
	if (_cubemap2fisheyeCamera && _cubemap2fisheyeCamera.valid())
	{
			for (size_t i = 0; i < _cubemapCameras->getNumChildren(); i++)
			{
					RenderSurface* cameraBuffer = (RenderSurface*)_cubemapCameras->getChild(i);
					osgDB::writeImageFile(*cameraBuffer->Image(), cameraBuffer->getName() + ".png");
			}
		 osg::ref_ptr<osg::Image> fisheye =	cubemap2fisheye(512, 512, getCubemapCameras(_cubemapCameras.get()));
		 osgDB::writeImageFile(*fisheye, "fisheye2.png");
		 osgDB::writeImageFile(*_cubemap2fisheyeCamera->Image(), "fisheye.png");
		 double svf = SVFComputeTools::calSVF(_cubemap2fisheyeCamera->Image().get(), false);
		 //SolarRadiation solarRad = SVFComputeTools::calSolar(_cubemap2fisheyeCamera->Image().get(), _solarParam, observer, observerNormal, _sceneNode);
		 //printf("SVF=%f\n", svf);
		 SolarRadiation solarRad = calSolar(observer, observerNormal);
		 _resultsCallback(svf, solarRad);
	}
}

bool SkyViewFactorEventHandler::handle(const osgGA::GUIEventAdapter & ea, osgGA::GUIActionAdapter & aa)
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
	osgViewer::Renderer *render = dynamic_cast<osgViewer::Renderer *>(aa.asView()->getCamera()->getRenderer());
	osgUtil::SceneView *sceneView = render->getSceneView(0);
	sceneView->getRenderInfo().getState()->setCheckForGLErrors(osg::State::NEVER_CHECK_GL_ERRORS);
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
//cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_X, ((RenderSurface*)svfCameraBuffers->getChild(NEG_X))->Image());
//cubeMap->setImage(osg::TextureCubeMap::POSITIVE_X, ((RenderSurface*)svfCameraBuffers->getChild(POS_X))->Image());
//cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Y, ((RenderSurface*)svfCameraBuffers->getChild(POS_Z))->Image());
//cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Y, ((RenderSurface*)svfCameraBuffers->getChild(NEG_Z))->Image());
//cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Z, ((RenderSurface*)svfCameraBuffers->getChild(NEG_Y))->Image());
//cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Z, ((RenderSurface*)svfCameraBuffers->getChild(POS_Y))->Image());

//static const char* FaceNames[] = { "POSITIVE_X","NEGATIVE_X","POSITIVE_Y","NEGATIVE_Y","POSITIVE_Z","NEGATIVE_Z" };
//
//void SkyViewFactorEventHandler::Test(SunVector sunVec, osg::Vec3d pos)
//{
//	osg::Vec3d sundir = GrassSolar::solarAngle2Vector(sunVec.alt, sunVec.azimuth);
//	osg::Vec3d end = pos + sundir * 10000000;
//	osg::TextureCubeMap::Face face = getFace(sunVec.azimuth, sunVec.alt);
//	CubemapSurface* camera = (CubemapSurface*)_cubemapCameras->getChild(face);
//	osg::Image* faceImage = camera->Image();
//	osg::Vec3d screenPos = end * camera->getViewMatrix() * camera->getProjectionMatrix();
//	printf("(%f,%f),(%s,%f,%f)\n", sunVec.azimuth, sunVec.alt, FaceNames[face], screenPos.x(), screenPos.y());
//}

SolarRadiation SkyViewFactorEventHandler::calSolar(osg::Vec3d pos, osg::Vec3d normal)
{
	auto cubemapCameras = getCubemapCameras(_cubemapCameras.get());
	osg::Image* img = _cubemap2fisheyeCamera->Image().get();
	SolarRadiation annualRad;
	annualRad.Zero();
	std::vector<SolarRadiation> dailyRads;
	SolarParam param = *_solarParam;
	param.slope = GrassSolar::calculateSlope(normal);
	param.aspect = GrassSolar::calculateAspect(normal);
	int startDay = param.startDay;
	int endDay = param.endDay;
	if (param.isSingleDay && endDay > startDay)
		endDay = startDay;
	int lastSteps = 0;
	GrassSolar rsun;
	for (int day = startDay; day <= endDay; day++)
	{
		param.startDay = day;
		std::vector<bool> shadowMasks;
		std::vector<bool> shadowMasksRayCaster;
		std::vector<SunVector> sunVecs = rsun.getSunVectors(param);
		shadowMasks.resize(sunVecs.size());
		if (lastSteps != sunVecs.size())
		{
			if (param.shadowInfo)
				delete[] param.shadowInfo;
			param.shadowInfo = new bool[sunVecs.size()];
		}
		for (int n = 0; n < sunVecs.size(); n++)
		{
			SunVector sunVec = sunVecs[n];
			osg::Vec3d sundir = GrassSolar::solarAngle2Vector(sunVec.alt, sunVec.azimuth);
			osg::Vec3d start = pos;
			osg::Vec3d end = pos + sundir * 10000000;
			//int face = getFace(sunVec.azimuth, sunVec.alt);
			//CubemapSurface* camera = (CubemapSurface*)_cubemapCameras->getChild(face);
			//osg::Image* faceImage = camera->Image();
			//osg::Vec3d screenPos = end * camera->getViewMatrix() * camera->getProjectionMatrix();

			//Test(SunVector(0, 44.999), pos);
			//Test(SunVector(0, 45.1), pos);
			//Test(SunVector(45.1, 0), pos);
			//Test(SunVector(135.1, 0), pos);
			//Test(SunVector(225.1, 0), pos);
			//Test(SunVector(315.1, 0), pos);
			//Test(SunVector(45.1, 44.999), pos);
			//Test(SunVector(135.1, 44.999), pos);
			//Test(SunVector(225.1, 44.999), pos);
			//Test(SunVector(315.1, 44.999), pos);
			osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
			osgUtil::IntersectionVisitor intersectVisitor(intersector.get());
			_sceneNode->accept(intersectVisitor);
			if (intersector->containsIntersections())
			{
				shadowMasksRayCaster.push_back(true);
			}
			else
			{
				shadowMasksRayCaster.push_back(false);
			}
			//double radius = (90.0 - sunVec.alt) / 90.0 * 0.5;
			////double radius = sin(osg::DegreesToRadians(90.0 - sunVec.alt)) * 0.5;
			//double theta = sunVec.azimuth - 90;
			//if (theta < 0)
			//	theta += 360;
			//theta = osg::DegreesToRadians(theta);
			//double x = radius * cos(theta);
			//double y = radius * sin(theta);
			//x += 0.5;
			//y += 0.5;
			//osg::Vec4 color = img->getColor(osg::Vec2(x, y));
			//param.shadowInfo[n] = color.a() > 0.7 ? true : false;
			param.shadowInfo[n] = isShadowed(sunVec.alt, sunVec.azimuth, cubemapCameras);
			shadowMasks[n] = param.shadowInfo[n];
		}

		for (int n = 0; n < sunVecs.size(); n++)
		{
			printf("%d", shadowMasks[n] ? 1 : 0);
			if (n == sunVecs.size() - 1)
				printf("\n");
			else
				printf(",");
		}

		for (int n = 0; n < sunVecs.size(); n++)
		{
			printf("%d", shadowMasksRayCaster[n] ? 1 : 0);
			if (n == sunVecs.size() - 1)
				printf("\n");
			else
				printf(",");
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
