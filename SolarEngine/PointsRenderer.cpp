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

void PointsRenderer::pushPointInternal(const SolarRadiationPoint& point)
{
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
	//pointFar->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
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
	text->setFont("data/arial.ttf");
	text->setCharacterSize(20);
	text->setPosition(point.pos);
	text->setAxisAlignment(osgText::Text::SCREEN);
	text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	text->setText(label);
	text->setBackdropColor(osg::Vec4(1, 1, 0, 1));
	text->setBackdropType(osgText::Text::BackdropType::OUTLINE);

	osg::ref_ptr <TextLOD> lod = new TextLOD(point);
	lod->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
	lod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
	lod->addChild(pointFar.get(), 0, 1000);
	lod->addChild(pointNear.get(), 1000, 10000000000);
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
		osgText::Text* textNode = (osgText::Text*)lod->getChild(2);
		osg::Geode* pointNode = (osg::Geode*)lod->getChild(1);
		const SolarRadiationPoint& point = lod->_point;
		osg::Vec3d pos = lod->_point.pos;
		osg::Vec3d eye, center, up;
		m_sceneCamera->getViewMatrixAsLookAt(eye, center, up);
		double distTocamera1 = (eye - pos).length();
		osg::Vec4d vertex = osg::Vec4d(pos, 1.0) * m_sceneCamera->getViewMatrix() * m_sceneCamera->getProjectionMatrix();
		osg::Vec3d screenCoords = osg::Vec3d(vertex.x() / vertex.w(), vertex.y() / vertex.w(), vertex.z() / vertex.w());
		//double fov, aspect, znear, zfar;
		//m_sceneCamera->getProjectionMatrixAsPerspective(fov, aspect, znear, zfar);
		screenCoords.z() = (screenCoords.z() + 1.0) * 0.5;
		osg::Vec2d uv = osg::Vec2d(screenCoords.x() * 0.5 + 0.5, screenCoords.y() * 0.5 + 0.5);
		osg::Matrixd projInverse = osg::Matrixd::inverse(m_sceneCamera->getProjectionMatrix());
		osg::Matrixd viewInverse = osg::Matrixd::inverse(m_sceneCamera->getViewMatrix());
		//osg::Vec3d p1 = Utils::WorldPosFromDepth(screenCoords.z(), projInverse, viewInverse, uv.x(), uv.y());
		//float sceneDepth = m_sceneDepthImage->getColor(osg::Vec2(uv.x(), uv.y())).r();
		int imgx = (int)(uv.x() * m_sceneDepthImage->s());
		int imgy = (int)(uv.y() * m_sceneDepthImage->t());
		bool isVisible = false;
		int windowSize = 5;
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
				osg::Vec3d p2 = Utils::WorldPosFromDepth(sceneDepth, projInverse, viewInverse, x, y);
				double distTocamera2 = (p2 - eye).length();
				if (distTocamera1 < distTocamera2)
				{
					isVisible = true;
					break;
				}
			}
		}
		textNode->setNodeMask(isVisible);
		pointNode->setNodeMask(isVisible);
	}
}
