#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif

#include "PointsRenderer.h"
#include <osg/Point>
#include <osg/BlendFunc>
#include <osg/Texture2D>
#include <osg/PointSprite>
#include <osg/PolygonMode>
#include <osg/Geometry>
#include <osgText/Font>
#include <osgText/Text>
#include <osg/PolygonOffset>
#include <osgDB/WriteFile>
#include <fstream>
#include "RenderSurface.h"

osg::Vec3 WorldPosFromDepth(float depth, osg::Matrixd& projMatrixInv, osg::Matrixd& viewMatrixInv, float u, float v)
{
	float z = depth * 2.0 - 1.0;

	osg::Vec4 clipSpacePosition(u * 2.0 - 1.0, v * 2.0 - 1.0, z, 1.0);
	osg::Vec4 viewSpacePosition = clipSpacePosition * projMatrixInv;

	// Perspective division
	viewSpacePosition /= viewSpacePosition.w();
	osg::Vec4 worldSpacePosition = viewSpacePosition * viewMatrixInv;

	return osg::Vec3(worldSpacePosition.x(), worldSpacePosition.y(), worldSpacePosition.z());
}

void WriteDepthImage(osg::Image* depthImage, const std::string& outname)
{
	float* pDepth = (float*)depthImage->data();
	osg::ref_ptr<osg::Image> depthMap = new osg::Image;
	depthMap->allocateImage(depthImage->s(), depthImage->t(), 1, GL_RGB, GL_UNSIGNED_BYTE);
	ColorUB3* data = (ColorUB3*)depthMap->data();
	int idx = 0;
	for (int row = 0; row < depthImage->t(); row++)
	{
		for (int col = 0; col < depthImage->s(); col++)
		{
			float d = pDepth[idx];
			*data++ = ColorUB3(osg::Vec4(d, d, d, d));
			idx++;
		}
	}
	osgDB::writeImageFile(*depthMap, outname);
}

class DrawableDrawCallback : public osg::Drawable::DrawCallback
{
public:
	osg::Image* m_sceneDepthImage;
	//osg::Camera* m_sceneCamera;
	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
	{
		//WriteDepthImage(m_sceneDepthImage, "depth2.jpg");
		const TextLOD* lod = dynamic_cast<const TextLOD*>(drawable->getParent(0));
		osg::Vec3d pos = lod->_point.pos;
		osg::Vec3d eye, center, up;
		renderInfo.getCurrentCamera()->getViewMatrixAsLookAt(eye, center, up);
		double dist = (eye - pos).length();
		osg::Drawable* text = const_cast <osg::Drawable*>(drawable);
		osg::Vec4d vertex = osg::Vec4d(pos, 1.0) * renderInfo.getCurrentCamera()->getViewMatrix() * renderInfo.getCurrentCamera()->getProjectionMatrix();
		osg::Vec3d screenCoords = osg::Vec3d(vertex.x() / vertex.w(), vertex.y() / vertex.w(), vertex.z() / vertex.w());
		//double fov, aspect, znear, zfar;
		//renderInfo.getCurrentCamera()->getProjectionMatrixAsPerspective(fov, aspect, znear, zfar);
		screenCoords.z() = (screenCoords.z() + 1.0) * 0.5;
		osg::Vec2d uv = osg::Vec2d(screenCoords.x() * 0.5 + 0.5, screenCoords.y() * 0.5 + 0.5);
		double xresol = 1.0 / m_sceneDepthImage->s();
		double yresol = 1.0 / m_sceneDepthImage->t();
		text->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		int imgx = (int)(uv.x() * m_sceneDepthImage->s());
		int imgy = (int)(uv.y() * m_sceneDepthImage->t());
		bool isVisible = false;
		osg::Matrixd projInverse = osg::Matrixd::inverse(renderInfo.getCurrentCamera()->getProjectionMatrix());
		osg::Matrixd viewInverse = osg::Matrixd::inverse(renderInfo.getCurrentCamera()->getViewMatrix());
		osg::Vec3d p1 = WorldPosFromDepth(screenCoords.z(), projInverse, viewInverse, uv.x(), uv.y());
		double distTocamera1 = (p1 - eye).length();
		int windowSize = 5;
		osg::ref_ptr<osg::Image> depthImage = new osg::Image;
		depthImage->allocateImage(m_sceneDepthImage->s(), m_sceneDepthImage->t(), 1, GL_RGB, GL_UNSIGNED_BYTE);
		ColorUB3* pColor = (ColorUB3*)depthImage->data();
		float* pDepth = (float*)m_sceneDepthImage->data();
		for (int n = 0; n < m_sceneDepthImage->s() * m_sceneDepthImage->t(); n++)
		{
			unsigned char grayscale = (unsigned char)(pDepth[n] * 255.0);
			pColor[n] = ColorUB3(grayscale, grayscale, grayscale);
		}

		for (int xoffset = -windowSize; xoffset < windowSize; xoffset++)
		{
			float x = (imgx + xoffset) / (float)m_sceneDepthImage->s();
			if (x < 0 || x > 1.0)
				continue;
			for (int yoffset = -windowSize; yoffset < windowSize; yoffset++)
			{
				float y = (imgy + yoffset) / (float)m_sceneDepthImage->t();
				if (y < 0 || y > 1.0)
					continue;
				float sceneDepth = m_sceneDepthImage->getColor(osg::Vec2(x, y)).r();
				osg::Vec3d p2 = WorldPosFromDepth(screenCoords.z(), projInverse, viewInverse, x, y);
				double distTocamera2 = (p2 - eye).length();
				//if (abs(screenCoords.z() - sceneDepth) < 0.001)
				if (distTocamera1 < distTocamera2)
				{
					isVisible = true;
					int index = (imgx + xoffset) + (imgy + yoffset) * m_sceneDepthImage->s();
					pColor[index] = ColorUB3(255, 0, 0);
				}
			}
		}
		//osgDB::writeImageFile(*depthImage, "depth1.jpg");

		if (!isVisible)
			return;

		//if (dist < 50)
		//{
		//	text->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		//}
		//else
		//{
		//	text->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
		//}
		drawable->drawImplementation(renderInfo);
	}
};

void PointsRenderer::pushPointInternal(const SolarRadiationPoint& point)
{
	WriteDepthImage(m_sceneDepthImage, "depth.jpg");
	//printf("depth: %f, %f\n", screenCoords.z(), color.r());
	static osg::ref_ptr<osg::Program> pointsShader = nullptr;
	if (!pointsShader)
	{
		pointsShader = new osg::Program;
		char vertexShaderSource[] =
			"uniform float pointSize;\n"
			"void main(void)\n"
			"{\n"
			"   gl_PointSize = pointSize;\n"
			"   gl_Position = gl_ModelViewProjectionMatrix *  gl_Vertex;\n"
			"}\n";
		char fragmentShaderSource[] =
			"void main(void) \n"
			"{\n"
			"     gl_FragColor = vec4(1,0,0,1);\n"
			"}\n";
		pointsShader->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource));
		pointsShader->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShaderSource));
		getOrCreateStateSet()->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		getOrCreateStateSet()->setRenderBinDetails(6000, "RenderBin");
	}

	static osg::ref_ptr<osg::Program> textsShader = nullptr;
	if (!textsShader)
	{
		textsShader = new osg::Program;
		char vertexShaderSource[] =
			"void main(void)\n"
			"{\n"
			"   gl_Position = gl_ModelViewProjectionMatrix *  gl_Vertex;\n"
			"   gl_Position.z += 0.00001;\n"
			"}\n";
		textsShader->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShaderSource));
	}

	//m_solarPoints.push_back(radPoint);
	std::string label = Utils::value2String(point.global / 1000, 3);
	m_doStack.push_back(Action(ActionTypeEnum::PUSH, point));

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Geometry> polyGeom = new osg::Geometry;
	vertices->push_back(point.pos);
	polyGeom->setVertexArray(vertices.get());
	polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
	polyGeom->getOrCreateStateSet()->setAttribute(pointsShader.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::Geode> pointFar = new osg::Geode;
	pointFar->addDrawable(polyGeom.get());
	pointFar->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
	pointFar->setCullingActive(false);
	osg::ref_ptr<osg::Uniform> pointSizeFar = new osg::Uniform("pointSize", 5.0f);
	pointFar->getOrCreateStateSet()->addUniform(pointSizeFar.get());

	osg::ref_ptr<osg::Geode> pointNear = new osg::Geode;
	pointNear->addDrawable(polyGeom.get());
	pointNear->setCullingActive(false);
	osg::ref_ptr<osg::Uniform> pointSizeNear = new osg::Uniform("pointSize", 10.0f);
	pointNear->getOrCreateStateSet()->addUniform(pointSizeNear.get());

	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setColor(osg::Vec4(0, 0, 0, 1));
	text->setFont("fonts/arial.ttf");
	text->setCharacterSize(20);
	text->setPosition(point.pos);
	text->setAxisAlignment(osgText::Text::SCREEN);
	text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	text->setText(label);
	text->setBackdropColor(osg::Vec4(1, 1, 0, 1));
	text->setBackdropType(osgText::Text::BackdropType::OUTLINE);
	static osg::ref_ptr<DrawableDrawCallback> drawCallback = new DrawableDrawCallback;
	drawCallback->m_sceneDepthImage = m_sceneDepthImage;
	//text->setDrawCallback(drawCallback.get());
	//osg::ref_ptr<osg::PolygonOffset> polyoffset = new osg::PolygonOffset;
	//polyoffset->setFactor(-1.0f);
	//polyoffset->setUnits(-1.0f);
	//text->getOrCreateStateSet()->setAttributeAndModes(polyoffset.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);

	//text->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
	//text->getOrCreateStateSet()->setAttribute(textsShader.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr <TextLOD> lod = new TextLOD(point);
	lod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
	lod->addChild(pointFar.get(), 0, 500);
	lod->addChild(pointNear.get(), 500, 10000000000);
	lod->addChild(text.get(), 1000, 10000000000);
	addChild(lod.get());
}

void PointsRenderer::pushPoint(const SolarRadiationPoint& point)
{
	m_doStack.push_back(Action(ActionTypeEnum::PUSH, point));
	pushPointInternal(point);
}

void PointsRenderer::pushPoint(const osg::Vec3d& point, const SolarParam& param, const SolarRadiation& rad)
{
	SolarRadiationPoint radPoint(point, param, rad);
	pushPointInternal(radPoint);
}

void PointsRenderer::popPointInternal()
{
	int numChildren = getNumChildren();
	if (numChildren < 1)
		return;
	TextLOD* lod = dynamic_cast<TextLOD*>(getChild(numChildren - 1));
	removeChild(numChildren - 1, 1);
}

void PointsRenderer::popPoint()
{
	int numChildren = getNumChildren();
	if (numChildren < 1)
		return;
	TextLOD* lod = dynamic_cast<TextLOD*>(getChild(numChildren - 1));
	m_undoStack.push_back(Action(ActionTypeEnum::POP, lod->_point));
	removeChild(numChildren - 1, 1);
}

void PointsRenderer::undo()
{
	if (m_doStack.size() == 0)
		return;
	Action oldAction = m_doStack.back();
	m_doStack.pop_back();
	Action newAction = oldAction;
	if (oldAction.actionType == ActionTypeEnum::POP)
	{
		newAction.actionType = ActionTypeEnum::PUSH;
	}
	else
	{
		newAction.actionType = ActionTypeEnum::POP;
	}
	m_undoStack.push_back(newAction);

	popPoint();
}

void PointsRenderer::redo()
{
	if (m_undoStack.size() == 0)
		return;
	Action oldAction = m_undoStack.back();
	Action newAction = oldAction;
	if (oldAction.actionType == ActionTypeEnum::POP)
	{
		newAction.actionType = ActionTypeEnum::PUSH;
	}
	else
	{
		newAction.actionType = ActionTypeEnum::POP;
	}
	m_undoStack.pop_back();
	m_doStack.push_back(newAction);
	pushPoint(oldAction);
}

void PointsRenderer::performAction(const Action& action)
{
	if (action.actionType == POP) 
	{
		popPointInternal();
	}
	else
	{
		pushPointInternal(action);
	}
}

void PointsRenderer::exportPoints(const std::string& filename)
{
	int numChildren = getNumChildren();
	if (numChildren < 1)
		return;

	std::ofstream ofs(filename);
	ofs << "lat,lon,elev,3D position,start day,end day,time step,linke,slope,aspect,global[kWh/m2],beam[kWh/m2],diffuse[kWh/m2],reflected[kWh/m2],SVF\n";
	for (int i = 0; i < numChildren; i++)
	{
		TextLOD* lod = dynamic_cast<TextLOD*>(getChild(i));
		if (!lod)
			continue;
		const SolarRadiationPoint& point = lod->_point;
		std::vector<OuputVariable> outputVariables;
		outputVariables.push_back(OuputVariable(point.lat));
		outputVariables.push_back(OuputVariable(point.lon));
		outputVariables.push_back(OuputVariable(point.elev));
		outputVariables.push_back(OuputVariable(point.pos));
		outputVariables.push_back(OuputVariable(point.startDay));
		outputVariables.push_back(OuputVariable(point.endDay));
		outputVariables.push_back(OuputVariable(point.time_step));
		outputVariables.push_back(OuputVariable(point.linke));
		outputVariables.push_back(OuputVariable(point.slope));
		outputVariables.push_back(OuputVariable(point.aspect));
		outputVariables.push_back(OuputVariable(point.global / 1000));
		outputVariables.push_back(OuputVariable(point.beam / 1000));
		outputVariables.push_back(OuputVariable(point.diffuse / 1000));
		outputVariables.push_back(OuputVariable(point.reflected / 1000));
		outputVariables.push_back(OuputVariable(point.svf));
		for (int v = 0; v < outputVariables.size(); v++)
		{
			outputVariables[v].Out(ofs);
			if (v != outputVariables.size())
			{
				ofs << ",";
			}
		}
		ofs << "\n";
		//ofs << std::setprecision(5);
	}
	ofs.close();
}

void PointsRenderer::postDrawUpdate()
{
	int numChildren = getNumChildren();
	if (numChildren < 1)
		return;

	for (int i = 0; i < numChildren; i++)
	{
		TextLOD* lod = dynamic_cast<TextLOD*>(getChild(i));
		if (!lod)
			continue;
		osgText::Text* text = (osgText::Text*)lod->getChild(2);
		const SolarRadiationPoint& point = lod->_point;
		//WriteDepthImage(m_sceneDepthImage, "depth2.jpg");
		osg::Vec3d pos = lod->_point.pos;
		osg::Vec3d eye, center, up;
		m_sceneCamera->getViewMatrixAsLookAt(eye, center, up);
		double dist = (eye - pos).length();
		osg::Vec4d vertex = osg::Vec4d(pos, 1.0) * m_sceneCamera->getViewMatrix() * m_sceneCamera->getProjectionMatrix();
		osg::Vec3d screenCoords = osg::Vec3d(vertex.x() / vertex.w(), vertex.y() / vertex.w(), vertex.z() / vertex.w());
		//double fov, aspect, znear, zfar;
		//renderInfo.getCurrentCamera()->getProjectionMatrixAsPerspective(fov, aspect, znear, zfar);
		screenCoords.z() = (screenCoords.z() + 1.0) * 0.5;
		osg::Vec2d uv = osg::Vec2d(screenCoords.x() * 0.5 + 0.5, screenCoords.y() * 0.5 + 0.5);
		double xresol = 1.0 / m_sceneDepthImage->s();
		double yresol = 1.0 / m_sceneDepthImage->t();
		text->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		int imgx = (int)(uv.x() * m_sceneDepthImage->s());
		int imgy = (int)(uv.y() * m_sceneDepthImage->t());
		bool isVisible = false;
		osg::Matrixd projInverse = osg::Matrixd::inverse(m_sceneCamera->getProjectionMatrix());
		osg::Matrixd viewInverse = osg::Matrixd::inverse(m_sceneCamera->getViewMatrix());
		osg::Vec3d p1 = WorldPosFromDepth(screenCoords.z(), projInverse, viewInverse, uv.x(), uv.y());
		double distTocamera1 = (p1 - eye).length();
		float sceneDepth = m_sceneDepthImage->getColor(osg::Vec2(uv.x(), uv.y())).r();
		osg::Vec3d p2 = WorldPosFromDepth(sceneDepth, projInverse, viewInverse, uv.x(), uv.y());
		//printf("[%f,%f,%f],[%f,%f,%f]\n", pos.x(), pos.y(), pos.z(), p2.x(), p2.y(), p2.z());
		printf("center:[%f,%f,%f],eye:[%f,%f,%f],dist:[%f]\n", pos.x(), pos.y(), pos.z(), eye.x(), eye.y(), eye.z(), distTocamera1);
		int windowSize = 5;
		bool writeImage = false;
		//WriteDepthImage(m_sceneDepthImage, "depth1.jpg");

		//osg::ref_ptr<osg::Image> depthImage = new osg::Image;
		//depthImage = new osg::Image;
		//depthImage->allocateImage(m_sceneDepthImage->s(), m_sceneDepthImage->t(), 1, GL_RGB, GL_UNSIGNED_BYTE);
		//ColorUB3* pColor = (ColorUB3*)depthImage->data();
		//float* pDepth = (float*)m_sceneDepthImage->data();
		//for (int n = 0; n < m_sceneDepthImage->s() * m_sceneDepthImage->t(); n++)
		//{

		//	unsigned char grayscale = (unsigned char)(pDepth[n] * 255.0);
		//	pColor[n] = ColorUB3(grayscale, grayscale, grayscale);
		//}
		//int idx = 0;
		//for (int row = 0; row < depthImage->t(); row++)
		//{
		//	float y = row / (float)(depthImage->t() - 1);
		//	//y = 1.0 - y;
		//	for (int col = 0; col < depthImage->s(); col++)
		//	{
		//		float x = col / (float)(depthImage->s() - 1);
		//		float d = pDepth[idx];
		//		osg::Vec3d p2 = WorldPosFromDepth(d, projInverse, viewInverse, x, y);
		//		dist = (eye - p2).length();
		//		dist / 1000;
		//		p2.normalize();
		//		p2 = p2 * 0.5 + osg::Vec3d(0.5, 0.5, 0.5);
		//		pColor[idx] = ColorUB3(p2);
		//		idx++;
		//	}
		//}
		printf("Neighbors:\n");
		for (int xoffset = -windowSize; xoffset < windowSize; xoffset++)
		{
			float x = (imgx + xoffset) / (float)m_sceneDepthImage->s();
			if (x < 0 || x > 1.0)
				continue;
			for (int yoffset = -windowSize; yoffset < windowSize; yoffset++)
			{
				float y = (imgy + yoffset) / (float)m_sceneDepthImage->t();
				if (y < 0 || y > 1.0)
					continue;
				float sceneDepth = m_sceneDepthImage->getColor(osg::Vec2(x, y)).r();
				osg::Vec3d p2 = WorldPosFromDepth(sceneDepth, projInverse, viewInverse, x, y);
				double distTocamera2 = (p2 - eye).length();
				printf("[%f,%f],[%f,%f,%f],dist:[%f]\n", x, y, p2.x(), p2.y(), p2.z(), distTocamera2);
				//if (abs(screenCoords.z() - sceneDepth) < 0.001)
				if (distTocamera1 < distTocamera2)
				{
					isVisible = true;
					//if (pColor)
					//{
					//	int index = (imgx + xoffset) + (imgy + yoffset) * m_sceneDepthImage->s();
					//	pColor[index] = ColorUB3(255, 0, 0);
					//}

				}
			}
		}
		printf("\n");
		printf("\n");
		text->setNodeMask(isVisible);
		//if (pColor)
		//	osgDB::writeImageFile(*depthImage, "depth1.jpg");
	}
}
