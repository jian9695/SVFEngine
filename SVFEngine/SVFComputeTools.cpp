#include "SVFComputeTools.h"
#include <osg/ComputeBoundsVisitor>
#include <iomanip>

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

	char vertexSource[] =
		"void main(void)\n"
		"{\n"
		"   gl_TexCoord[0] = vec4(gl_Vertex.x,gl_Vertex.y,0,1.0);\n"
		"   gl_Position   = vec4(gl_Vertex.x,gl_Vertex.y,0,1);\n"
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
		"    float radius = length(gl_TexCoord[0].xy);\n"
		"    vec3 tex = vec3(-gl_TexCoord[0].x, gl_TexCoord[0].y, -(1-radius));\n"
		"    vec2 uv = gl_TexCoord[0].xy; \n"
		"    uv = uv * 0.5 + 0.5;\n"
		"    uv = rotate(uv,rotateAngle);\n"
		"    gl_FragColor = textureCube( uEnvironmentMap, tex.xzy );\n"
		"    if(radius > 1)\n"
		"    {\n"
		"        gl_FragColor = vec4(0,0,1,0.3);\n"
		"    }\n"
		"    else if(gl_FragColor.a < 0.5)\n"
		"    {\n"
		"        gl_FragColor = vec4(0.207843137, 0.317647059, 0.360784314,0.5);\n"
		"    }\n"
		"    else\n"
		"    {\n"
		"        gl_FragColor = vec4(gl_FragColor.rgb, 0.75);\n"
		"    }\n"
		"}\n";

	OverlayRenderSurface* fisheye = new OverlayRenderSurface(512, 512, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, true);
	fisheye->Overlay()->setProgramName("fisheye");
	//memcpy(updateParticlesSurfaceStorage->Image()->data(), particlesMap->data(), imageSize);
	//Store the updated particles
	fisheye->Overlay()->SetTextureLayer(cubeMap.get(), 0);
	fisheye->Overlay()->SetVertexShader(vertexSource);
	fisheye->Overlay()->SetFragmentShader(fragmentSource);
	fisheye->getOrCreateStateSet()->addUniform(new osg::Uniform("uEnvironmentMap", 0));
	fisheye->getOrCreateStateSet()->addUniform(new osg::Uniform("rotateAngle", 0.0f));
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

SolarRadiation SVFComputeTools::calSolar(osg::Image* img, SolarParam* solarParam)
{
	SolarRadiation annualRad;
	annualRad.Zero();
	std::vector<SolarRadiation> dailyRads;
	SolarParam param = *solarParam;
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
			double radius = (90.0 - sunVec.alt) / 90.0 * 0.5;
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
	printfVec3(ray->getFirstIntersection().getWorldIntersectPoint());
	osg::Vec3d observer = ray->getFirstIntersection().getWorldIntersectPoint();
	osg::Vec3d observerNormal = ray->getFirstIntersection().getWorldIntersectNormal();
	observerNormal.normalize();
	observer = observer + observerNormal * 0.1; //distance of camera from the surface
	printfVec3(observerNormal);
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
			osgDB::writeImageFile(*_cubemap2fisheyeCamera->Image(), "fisheye.png");
		double svf = SVFComputeTools::calSVF(_cubemap2fisheyeCamera->Image().get(), false);
		SolarRadiation solarRad = SVFComputeTools::calSolar(_cubemap2fisheyeCamera->Image().get(), _solarParam);
		printf("SVF=%f\n", svf);
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
