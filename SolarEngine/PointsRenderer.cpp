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

void PointsRenderer::pushPointInternal(const SolarRadiationPoint& point)
{
	static osg::ref_ptr<osg::Program> m_pointShader = nullptr;
	if (!m_pointShader)
	{
		m_pointShader = new osg::Program;
		char vertexShaderSource[] =
			"void main(void)\n"
			"{\n"
			"   gl_PointSize = 5;\n"
			"   gl_Position = gl_ModelViewProjectionMatrix *  gl_Vertex;\n"
			"}\n";
		char fragmentShaderSource[] =
			"void main(void) \n"
			"{\n"
			"     gl_FragColor = vec4(0,1,0,1);\n"
			"}\n";
		m_pointShader->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource));
		m_pointShader->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShaderSource));
		getOrCreateStateSet()->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		getOrCreateStateSet()->setRenderBinDetails(6000, "RenderBin");
		getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
	}
	//m_solarPoints.push_back(radPoint);
	std::string label = Utils::value2String(point.global / 1000, 3);
	m_doStack.push_back(Action(ActionTypeEnum::PUSH, point));

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Geometry> polyGeom = new osg::Geometry;
	vertices->push_back(point.pos);
	polyGeom->setVertexArray(vertices.get());
	polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
	polyGeom->getOrCreateStateSet()->setAttribute(m_pointShader.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setColor(osg::Vec4(1, 1, 0, 1));
	text->setFont("fonts/times.ttf");
	text->setCharacterSize(20);
	text->setPosition(point.pos);
	text->setAxisAlignment(osgText::Text::SCREEN);
	text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	text->setText(label);
	text->setBackdropColor(osg::Vec4(0, 0, 0, 1));
	text->setBackdropType(osgText::Text::BackdropType::OUTLINE);
	osg::ref_ptr <TextLOD> lod = new TextLOD(point);
	lod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
	lod->addChild(polyGeom.get(), 0, 1000);
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

void PointsRenderer::doAction(const Action& action)
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
